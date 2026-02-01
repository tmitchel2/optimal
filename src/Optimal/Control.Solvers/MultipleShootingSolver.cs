/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Threading;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers
{
    /// <summary>
    /// Multiple shooting solver for optimal control problems.
    /// Breaks trajectory into segments, solves each independently, then enforces continuity.
    /// Better for unstable or stiff dynamics than direct collocation.
    /// </summary>
    public sealed class MultipleShootingSolver
    {
        private int _segments = 10;
        private double _tolerance = 1e-3;
        private int _maxIterations = 100;
        private int _shootingIntervals = 4;  // Number of shooting intervals
        private IOptimizer? _innerOptimizer;

        /// <summary>
        /// Sets the number of collocation segments per shooting interval.
        /// </summary>
        public MultipleShootingSolver WithSegments(int segments)
        {
            if (segments <= 0)
            {
                throw new ArgumentException("Segments must be positive.", nameof(segments));
            }
            _segments = segments;
            return this;
        }

        /// <summary>
        /// Sets the number of shooting intervals.
        /// Each interval is solved independently, then linked via continuity.
        /// </summary>
        public MultipleShootingSolver WithShootingIntervals(int intervals)
        {
            if (intervals <= 0)
            {
                throw new ArgumentException("Shooting intervals must be positive.", nameof(intervals));
            }
            _shootingIntervals = intervals;
            return this;
        }

        /// <summary>
        /// Sets the convergence tolerance.
        /// </summary>
        public MultipleShootingSolver WithTolerance(double tolerance)
        {
            _tolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the maximum number of iterations.
        /// </summary>
        public MultipleShootingSolver WithMaxIterations(int maxIterations)
        {
            _maxIterations = maxIterations;
            return this;
        }

        /// <summary>
        /// Sets the inner NLP optimizer.
        /// </summary>
        public MultipleShootingSolver WithInnerOptimizer(IOptimizer optimizer)
        {
            _innerOptimizer = optimizer ?? throw new ArgumentNullException(nameof(optimizer));
            return this;
        }

        /// <summary>
        /// Solves the optimal control problem using multiple shooting.
        /// </summary>
        public CollocationResult Solve(ControlProblem problem, CancellationToken cancellationToken)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (problem.Dynamics == null)
            {
                throw new InvalidOperationException("Problem must have dynamics defined.");
            }

            // Divide time horizon into shooting intervals
            var t0 = problem.InitialTime;
            var tf = problem.FinalTime;
            var totalDuration = tf - t0;
            var intervalDuration = totalDuration / _shootingIntervals;

            // Create list of shooting interval problems
            var intervalProblems = new List<ShootingInterval>();

            for (var i = 0; i < _shootingIntervals; i++)
            {
                var intervalStart = t0 + i * intervalDuration;
                var intervalEnd = t0 + (i + 1) * intervalDuration;

                var interval = new ShootingInterval
                {
                    Index = i,
                    StartTime = intervalStart,
                    EndTime = intervalEnd,
                    Segments = _segments
                };

                intervalProblems.Add(interval);
            }

            // Initial guess: Solve each interval independently
            Console.WriteLine("Multiple Shooting: Initial phase - solving intervals independently...");

            var intervalSolutions = new List<CollocationResult>();

            for (var i = 0; i < _shootingIntervals; i++)
            {
                var interval = intervalProblems[i];

                // Create problem for this interval
                var intervalProblem = CreateIntervalProblem(problem, interval, i == 0, i == _shootingIntervals - 1);

                // Solve interval
                var solver = new HermiteSimpsonSolver(
                    new HermiteSimpsonSolverOptions
                    {
                        Segments = _segments,
                        Tolerance = _tolerance,
                        MaxIterations = _maxIterations
                    },
                    _innerOptimizer ?? new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

                var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(intervalProblem, _segments);
                var result = solver.Solve(intervalProblem, initialGuess, cancellationToken);
                intervalSolutions.Add(result);

                Console.WriteLine($"  Interval {i + 1}/{_shootingIntervals}: Success={result.Success}, Cost={result.OptimalCost:E3}");

                if (!result.Success)
                {
                    return new CollocationResult
                    {
                        Success = false,
                        Message = $"Failed to solve interval {i + 1}",
                        Times = Array.Empty<double>(),
                        States = Array.Empty<double[]>(),
                        Controls = Array.Empty<double[]>()
                    };
                }
            }

            // Refinement phase: Enforce continuity between intervals
            Console.WriteLine("Multiple Shooting: Refinement phase - enforcing continuity...");

            for (var iteration = 0; iteration < 3; iteration++)
            {
                var maxDiscontinuity = 0.0;

                for (var i = 0; i < _shootingIntervals - 1; i++)
                {
                    var currentInterval = intervalSolutions[i];
                    var nextInterval = intervalSolutions[i + 1];

                    // Check discontinuity
                    var endState = currentInterval.States[currentInterval.States.Length - 1];
                    var startState = nextInterval.States[0];

                    var discontinuity = 0.0;
                    for (var j = 0; j < endState.Length; j++)
                    {
                        discontinuity += Math.Abs(endState[j] - startState[j]);
                    }

                    if (discontinuity > maxDiscontinuity)
                    {
                        maxDiscontinuity = discontinuity;
                    }

                    // If discontinuity is large, re-solve next interval with corrected initial condition
                    if (discontinuity > _tolerance * 10)
                    {
                        var intervalProblem = CreateIntervalProblem(
                            problem,
                            intervalProblems[i + 1],
                            false,
                            i + 1 == _shootingIntervals - 1);

                        // Use end state of previous interval as initial condition
                        intervalProblem = new ControlProblem()
                            .WithStateSize(problem.StateDim)
                            .WithControlSize(problem.ControlDim)
                            .WithTimeHorizon(intervalProblem.InitialTime, intervalProblem.FinalTime)
                            .WithInitialCondition(endState)
                            .WithDynamics(problem.Dynamics);

                        if (problem.RunningCost != null)
                        {
                            intervalProblem = intervalProblem.WithRunningCost(problem.RunningCost);
                        }

                        if (i + 1 == _shootingIntervals - 1 && problem.FinalState != null)
                        {
                            intervalProblem = intervalProblem.WithFinalCondition(problem.FinalState);
                        }

                        if (problem.TerminalCost != null && i + 1 == _shootingIntervals - 1)
                        {
                            intervalProblem = intervalProblem.WithTerminalCost(problem.TerminalCost);
                        }

                        if (problem.ControlLowerBounds != null && problem.ControlUpperBounds != null)
                        {
                            intervalProblem = intervalProblem.WithControlBounds(
                                problem.ControlLowerBounds,
                                problem.ControlUpperBounds);
                        }

                        var solver = new HermiteSimpsonSolver(
                            new HermiteSimpsonSolverOptions
                            {
                                Segments = _segments,
                                Tolerance = _tolerance,
                                MaxIterations = _maxIterations
                            },
                            _innerOptimizer ?? new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

                        var refinementGuess = InitialGuessFactory.CreateWithControlHeuristics(intervalProblem, _segments);
                        var refinedResult = solver.Solve(intervalProblem, refinementGuess, cancellationToken);

                        if (refinedResult.Success)
                        {
                            intervalSolutions[i + 1] = refinedResult;
                        }
                    }
                }

                Console.WriteLine($"  Iteration {iteration + 1}: Max discontinuity = {maxDiscontinuity:E3}");

                if (maxDiscontinuity < _tolerance * 10)
                {
                    break;
                }
            }

            // Stitch together the solution
            var allTimes = new List<double>();
            var allStates = new List<double[]>();
            var allControls = new List<double[]>();
            var totalCost = 0.0;

            for (var i = 0; i < _shootingIntervals; i++)
            {
                var intervalResult = intervalSolutions[i];

                for (var j = 0; j < intervalResult.Times.Length; j++)
                {
                    // Skip duplicate points at interval boundaries (except first interval)
                    if (i > 0 && j == 0)
                    {
                        continue;
                    }

                    allTimes.Add(intervalResult.Times[j]);
                    allStates.Add(intervalResult.States[j]);
                    allControls.Add(intervalResult.Controls[j]);
                }

                totalCost += intervalResult.OptimalCost;
            }

            return new CollocationResult
            {
                Success = true,
                Message = "Multiple shooting converged",
                Times = allTimes.ToArray(),
                States = allStates.ToArray(),
                Controls = allControls.ToArray(),
                OptimalCost = totalCost,
                Iterations = _maxIterations,
                MaxDefect = ComputeMaxDefect(problem.Dynamics!, allTimes.ToArray(), allStates.ToArray(), allControls.ToArray()),
                GradientNorm = 0.0
            };
        }

        /// <summary>
        /// Creates a problem for a specific shooting interval.
        /// </summary>
        private static ControlProblem CreateIntervalProblem(
            ControlProblem originalProblem,
            ShootingInterval interval,
            bool isFirst,
            bool isLast)
        {
            var intervalProblem = new ControlProblem()
                .WithStateSize(originalProblem.StateDim)
                .WithControlSize(originalProblem.ControlDim)
                .WithTimeHorizon(interval.StartTime, interval.EndTime)
                .WithDynamics(originalProblem.Dynamics!);

            // Initial condition for first interval only
            if (isFirst && originalProblem.InitialState != null)
            {
                intervalProblem = intervalProblem.WithInitialCondition(originalProblem.InitialState);
            }
            else if (!isFirst)
            {
                // For non-first intervals, use a reasonable guess (will be updated during refinement)
                // Linear interpolation between initial and final states if available
                if (originalProblem.InitialState != null && originalProblem.FinalState != null)
                {
                    var alpha = interval.StartTime / (originalProblem.FinalTime - originalProblem.InitialTime);
                    var guessState = new double[originalProblem.StateDim];
                    for (var i = 0; i < guessState.Length; i++)
                    {
                        guessState[i] = (1 - alpha) * originalProblem.InitialState[i] +
                                       alpha * (originalProblem.FinalState[i]);
                    }
                    intervalProblem = intervalProblem.WithInitialCondition(guessState);
                }
                else if (originalProblem.InitialState != null)
                {
                    // Just use initial state as guess
                    intervalProblem = intervalProblem.WithInitialCondition(originalProblem.InitialState);
                }
            }

            // Final condition for last interval only
            if (isLast && originalProblem.FinalState != null)
            {
                intervalProblem = intervalProblem.WithFinalCondition(originalProblem.FinalState);
            }

            // Add costs
            if (originalProblem.RunningCost != null)
            {
                intervalProblem = intervalProblem.WithRunningCost(originalProblem.RunningCost);
            }

            if (isLast && originalProblem.TerminalCost != null)
            {
                intervalProblem = intervalProblem.WithTerminalCost(originalProblem.TerminalCost);
            }

            // Add bounds
            if (originalProblem.ControlLowerBounds != null && originalProblem.ControlUpperBounds != null)
            {
                intervalProblem = intervalProblem.WithControlBounds(
                    originalProblem.ControlLowerBounds,
                    originalProblem.ControlUpperBounds);
            }

            if (originalProblem.StateLowerBounds != null && originalProblem.StateUpperBounds != null)
            {
                intervalProblem = intervalProblem.WithStateBounds(
                    originalProblem.StateLowerBounds,
                    originalProblem.StateUpperBounds);
            }

            return intervalProblem;
        }

        /// <summary>
        /// Computes maximum defect in the solution.
        /// </summary>
        private static double ComputeMaxDefect(
            Func<DynamicsInput, DynamicsResult> dynamics,
            double[] times,
            double[][] states,
            double[][] controls)
        {
            var maxDefect = 0.0;

            for (var i = 0; i < times.Length - 1; i++)
            {
                var h = times[i + 1] - times[i];
                var f0 = dynamics(new DynamicsInput(states[i], controls[i], times[i], i, times.Length - 1)).Value;
                var f1 = dynamics(new DynamicsInput(states[i + 1], controls[i + 1], times[i + 1], i, times.Length - 1)).Value;

                // Simple forward Euler check
                for (var j = 0; j < states[i].Length; j++)
                {
                    var predicted = states[i][j] + h * f0[j];
                    var actual = states[i + 1][j];
                    var defect = Math.Abs(predicted - actual);

                    if (defect > maxDefect)
                    {
                        maxDefect = defect;
                    }
                }
            }

            return maxDefect;
        }

        /// <summary>
        /// Represents a shooting interval.
        /// </summary>
        private sealed class ShootingInterval
        {
            public int Index { get; set; }
            public double StartTime { get; set; }
            public double EndTime { get; set; }
            public int Segments { get; set; }
        }
    }
}
