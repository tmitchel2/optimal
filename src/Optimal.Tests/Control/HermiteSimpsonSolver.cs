/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear;

namespace Optimal.Control
{
    /// <summary>
    /// Solves optimal control problems using Hermite-Simpson collocation.
    /// Converts the continuous problem into a nonlinear programming (NLP) problem
    /// and solves it using existing nonlinear optimizers.
    /// </summary>
    public sealed class HermiteSimpsonSolver
    {
        private int _segments = 20;
        private double _tolerance = 1e-6;
        private int _maxIterations = 100;
        private bool _verbose;
        private IOptimizer? _innerOptimizer;

        /// <summary>
        /// Sets the number of collocation segments.
        /// </summary>
        /// <param name="segments">Number of segments.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithSegments(int segments)
        {
            if (segments <= 0)
            {
                throw new ArgumentException("Segments must be positive.", nameof(segments));
            }

            _segments = segments;
            return this;
        }

        /// <summary>
        /// Sets the convergence tolerance.
        /// </summary>
        /// <param name="tolerance">Tolerance for defects and optimality.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithTolerance(double tolerance)
        {
            _tolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the maximum number of iterations.
        /// </summary>
        /// <param name="maxIterations">Maximum iterations.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithMaxIterations(int maxIterations)
        {
            _maxIterations = maxIterations;
            return this;
        }

        /// <summary>
        /// Enables or disables verbose output.
        /// </summary>
        /// <param name="verbose">True to enable verbose output.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithVerbose(bool verbose = true)
        {
            _verbose = verbose;
            return this;
        }

        /// <summary>
        /// Sets the inner NLP optimizer.
        /// </summary>
        /// <param name="optimizer">The optimizer to use (L-BFGS, CG, etc.).</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithInnerOptimizer(IOptimizer optimizer)
        {
            _innerOptimizer = optimizer ?? throw new ArgumentNullException(nameof(optimizer));
            return this;
        }

        /// <summary>
        /// Solves the optimal control problem.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <returns>The optimal control solution.</returns>
        public CollocationResult Solve(ControlProblem problem)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (problem.Dynamics == null)
            {
                throw new InvalidOperationException("Problem must have dynamics defined.");
            }

            // Create collocation grid
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, _segments);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            // Create initial guess
            var x0 = problem.InitialState ?? new double[problem.StateDim];
            var xf = problem.FinalState ?? new double[problem.StateDim];
            var u0 = new double[problem.ControlDim]; // Zero control

            var z0 = transcription.CreateInitialGuess(x0, xf, u0);

            // Extract dynamics evaluator (without gradients for now)
            double[] DynamicsValue(double[] x, double[] u, double t)
            {
                var result = problem.Dynamics!(x, u, t);
                return result.value;
            }

            // Build objective function for NLP
            Func<double[], (double value, double[] gradient)> nlpObjective = z =>
            {
                var cost = 0.0;

                // Add running cost if defined
                if (problem.RunningCost != null)
                {
                    double RunningCostValue(double[] x, double[] u, double t)
                    {
                        var result = problem.RunningCost(x, u, t);
                        return result.value;
                    }

                    cost += transcription.ComputeRunningCost(z, RunningCostValue);
                }

                // Add terminal cost if defined
                if (problem.TerminalCost != null)
                {
                    double TerminalCostValue(double[] x, double t)
                    {
                        var result = problem.TerminalCost(x, t);
                        return result.value;
                    }

                    cost += transcription.ComputeTerminalCost(z, TerminalCostValue);
                }

                // For now, return zero gradients (will compute numerically via optimizer)
                var gradient = new double[z.Length];
                return (cost, gradient);
            };

            // Create defect constraints
            Func<double[], (double value, double[] gradient)> DefectConstraint(int defectIndex)
            {
                return z =>
                {
                    var allDefects = transcription.ComputeAllDefects(z, DynamicsValue);
                    var gradient = new double[z.Length];
                    return (allDefects[defectIndex], gradient);
                };
            }

            // Set up constrained optimizer
            var constrainedOptimizer = new AugmentedLagrangianOptimizer()
                .WithUnconstrainedOptimizer(_innerOptimizer ?? new LBFGSOptimizer())
                .WithConstraintTolerance(_tolerance);

            constrainedOptimizer
                .WithInitialPoint(z0)
                .WithTolerance(_tolerance)
                .WithMaxIterations(_maxIterations)
                .WithVerbose(_verbose);

            // Add all defect constraints as equality constraints
            var totalDefects = _segments * problem.StateDim;
            for (var i = 0; i < totalDefects; i++)
            {
                constrainedOptimizer.WithEqualityConstraint(DefectConstraint(i));
            }

            // Add boundary condition constraints
            if (problem.InitialState != null)
            {
                for (var i = 0; i < problem.StateDim; i++)
                {
                    var stateIndex = i;
                    var targetValue = problem.InitialState[i];
                    constrainedOptimizer.WithEqualityConstraint(z =>
                    {
                        var x = transcription.GetState(z, 0);
                        var gradient = new double[z.Length];
                        return (x[stateIndex] - targetValue, gradient);
                    });
                }
            }

            if (problem.FinalState != null)
            {
                for (var i = 0; i < problem.StateDim; i++)
                {
                    var stateIndex = i;
                    var targetValue = problem.FinalState[i];
                    constrainedOptimizer.WithEqualityConstraint(z =>
                    {
                        var x = transcription.GetState(z, _segments);
                        var gradient = new double[z.Length];
                        return (x[stateIndex] - targetValue, gradient);
                    });
                }
            }

            // Add control bounds if specified
            if (problem.ControlLowerBounds != null && problem.ControlUpperBounds != null)
            {
                // Build flat bounds for decision vector
                var lowerBounds = new double[transcription.DecisionVectorSize];
                var upperBounds = new double[transcription.DecisionVectorSize];

                for (var k = 0; k <= _segments; k++)
                {
                    var offset = k * (problem.StateDim + problem.ControlDim);

                    // State bounds (if specified)
                    for (var i = 0; i < problem.StateDim; i++)
                    {
                        lowerBounds[offset + i] = problem.StateLowerBounds?[i] ?? double.NegativeInfinity;
                        upperBounds[offset + i] = problem.StateUpperBounds?[i] ?? double.PositiveInfinity;
                    }

                    // Control bounds
                    for (var i = 0; i < problem.ControlDim; i++)
                    {
                        lowerBounds[offset + problem.StateDim + i] = problem.ControlLowerBounds[i];
                        upperBounds[offset + problem.StateDim + i] = problem.ControlUpperBounds[i];
                    }
                }

                constrainedOptimizer.WithBoxConstraints(lowerBounds, upperBounds);
            }

            // Solve the NLP
            var nlpResult = constrainedOptimizer.Minimize(nlpObjective);

            // Extract solution
            var zOpt = nlpResult.OptimalPoint;

            var times = grid.TimePoints;
            var states = new double[_segments + 1][];
            var controls = new double[_segments + 1][];

            for (var k = 0; k <= _segments; k++)
            {
                states[k] = transcription.GetState(zOpt, k);
                controls[k] = transcription.GetControl(zOpt, k);
            }

            // Compute final defects
            var finalDefects = transcription.ComputeAllDefects(zOpt, DynamicsValue);
            var maxDefect = HermiteSimpsonTranscription.MaxDefect(finalDefects);

            return new CollocationResult
            {
                Success = nlpResult.Success,
                Message = nlpResult.Message,
                Times = times,
                States = states,
                Controls = controls,
                OptimalCost = nlpResult.OptimalValue,
                Iterations = nlpResult.Iterations,
                MaxDefect = maxDefect,
                GradientNorm = nlpResult.GradientNorm
            };
        }
    }
}
