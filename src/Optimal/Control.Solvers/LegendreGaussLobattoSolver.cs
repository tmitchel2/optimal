/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.Control.Optimization;
using Optimal.NonLinear.Constrained;
using Optimal.NonLinear.Constraints;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Monitoring;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers
{
    /// <summary>
    /// Solves optimal control problems using Legendre-Gauss-Lobatto (LGL) collocation.
    /// Provides high-order accurate solutions with configurable collocation order.
    /// </summary>
    public sealed class LegendreGaussLobattoSolver : ISolver
    {
        private readonly IOptimizer _innerOptimizer;
        private readonly OptimisationMonitor? _monitor;

        /// <summary>
        /// Gets the solver options.
        /// </summary>
        public LegendreGaussLobattoSolverOptions Options { get; }

        /// <summary>
        /// Initializes a new instance of the <see cref="LegendreGaussLobattoSolver"/> class with the specified options.
        /// </summary>
        /// <param name="options">The solver options.</param>
        /// <param name="innerOptimizer">Inner NLP optimizer.</param>
        /// <param name="monitor">Optional optimization monitor for gradient verification.</param>
        public LegendreGaussLobattoSolver(
            LegendreGaussLobattoSolverOptions options,
            IOptimizer innerOptimizer,
            OptimisationMonitor? monitor = null)
        {
            Options = options ?? throw new ArgumentNullException(nameof(options));
            _innerOptimizer = innerOptimizer ?? throw new ArgumentNullException(nameof(innerOptimizer));
            _monitor = monitor;
        }

        /// <summary>
        /// Gets the optimization monitor report after solving.
        /// Returns null if no monitor was configured.
        /// </summary>
        /// <returns>The monitoring report, or null if monitoring was not enabled.</returns>
        public OptimisationMonitorReport? GetMonitorReport()
        {
            return _monitor?.GenerateReport();
        }

        /// <summary>
        /// Solves the optimal control problem.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <param name="initialGuess">Initial guess for state and control trajectories.</param>
        /// <returns>The collocation result containing the optimal trajectory.</returns>
        public CollocationResult Solve(ControlProblem problem, InitialGuess initialGuess)
        {
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(initialGuess);

            if (problem.Dynamics == null)
            {
                throw new InvalidOperationException("Problem must have dynamics defined.");
            }

            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, Options.Segments);
            ICollocationTranscription transcription = Options.EnableParallelization
                ? new ParallelLGLTranscription(problem, grid, Options.Order, Options.EnableParallelization)
                : new LegendreGaussLobattoTranscription(problem, grid, Options.Order);
            var z0 = transcription.ToDecisionVector(initialGuess);

            if (Options.EnableMeshRefinement)
            {
                return SolveWithMeshRefinement(problem, z0);
            }

            return SolveOnFixedGrid(problem, Options.Segments, z0);
        }

        /// <summary>
        /// Solves the optimal control problem on a fixed grid with given number of segments.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <param name="segments">Number of segments to use.</param>
        /// <param name="initialGuess">Initial guess decision vector.</param>
        /// <returns>The collocation result containing the optimal trajectory.</returns>
        private CollocationResult SolveOnFixedGrid(ControlProblem problem, int segments, double[] initialGuess)
        {
            if (Options.Verbose)
            {
                Console.WriteLine($"Legendre-Gauss-Lobatto Solver");
                Console.WriteLine($"  Segments: {segments}");
                Console.WriteLine($"  Order: {Options.Order}");
                Console.WriteLine($"  State dimension: {problem.StateDim}");
                Console.WriteLine($"  Control dimension: {problem.ControlDim}");
            }

            // Create collocation grid
            var grid = new CollocationGrid(
                problem.InitialTime,
                problem.FinalTime,
                segments);

            // Create transcription (parallel or sequential based on settings)
            ICollocationTranscription transcription = Options.EnableParallelization
                ? new ParallelLGLTranscription(problem, grid, Options.Order, Options.EnableParallelization)
                : new LegendreGaussLobattoTranscription(problem, grid, Options.Order);

            if (Options.Verbose)
            {
                Console.WriteLine($"  Total collocation points: {transcription.TotalPoints}");
                Console.WriteLine($"  Decision vector size: {transcription.DecisionVectorSize}");
                Console.WriteLine($"  Defects per segment: {Options.Order - 2}");
                Console.WriteLine($"  Total defect constraints: {segments * (Options.Order - 2) * problem.StateDim}");
            }

            var z0 = initialGuess;
            var dynamics = problem.Dynamics!;

            if (Options.Verbose)
            {
                Console.WriteLine($"Initial guess:");

                // Check first 10 points
                Console.WriteLine($"  First 10 states: {string.Join(", ", Enumerable.Range(0, Math.Min(10, transcription.TotalPoints)).Select(i => transcription.GetState(z0, i)[0].ToString("F6", System.Globalization.CultureInfo.InvariantCulture)))}");
                Console.WriteLine($"  First 10 controls: {string.Join(", ", Enumerable.Range(0, Math.Min(10, transcription.TotalPoints)).Select(i => transcription.GetControl(z0, i)[0].ToString("F6", System.Globalization.CultureInfo.InvariantCulture)))}");

                // Compute defects at initial guess
                var initialDefects = transcription.ComputeAllDefects(z0, dynamics);
                var maxInitialDefect = initialDefects.Length > 0 ? initialDefects.Max(d => Math.Abs(d)) : 0.0;
                Console.WriteLine($"  Max defect at initial guess: {maxInitialDefect:E6}");

                // Compute cost at initial guess
                var initialCost = 0.0;
                if (problem.RunningCost != null)
                {
                    initialCost += transcription.ComputeRunningCost(z0, (x, u, t) => problem.RunningCost(new RunningCostInput(x, u, t)).Value);
                }
                if (problem.TerminalCost != null)
                {
                    initialCost += transcription.ComputeTerminalCost(z0, (x, t) => problem.TerminalCost(new TerminalCostInput(x, t)).Value);
                }
                Console.WriteLine($"  Cost at initial guess: {initialCost:E6}");
            }

            // Check if dynamics provides analytical gradients
            var hasAnalyticalDynamicsGradients = false;
            if (!Options.ForceNumericalGradients)
            {
                try
                {
                    var testX = new double[problem.StateDim];
                    var testU = new double[problem.ControlDim];
                    var testResult = problem.Dynamics!(new DynamicsInput(testX, testU, 0.0, -1, -1));

                    if (testResult.Gradients != null && testResult.Gradients.Length >= 2)
                    {
                        var gradWrtState = testResult.Gradients[0];
                        var gradWrtControl = testResult.Gradients[1];
                        var expectedStateGradSize = problem.StateDim * problem.StateDim;
                        var expectedControlGradSize = problem.StateDim * problem.ControlDim;

                        hasAnalyticalDynamicsGradients =
                            gradWrtState != null && gradWrtState.Length == expectedStateGradSize &&
                            gradWrtControl != null && gradWrtControl.Length == expectedControlGradSize;
                    }
                }
                catch
                {
                    // If evaluation fails, fall back to numerical gradients
                }
            }

            if (Options.Verbose && hasAnalyticalDynamicsGradients)
            {
                Console.WriteLine("Dynamics provides analytical gradients");
            }

            // Check if we can use analytical gradients for costs
            var useAnalyticalGradients = !Options.ForceNumericalGradients;

            if (problem.RunningCost != null)
            {
                try
                {
                    var testX = new double[problem.StateDim];
                    var testU = new double[problem.ControlDim];
                    var testResult = problem.RunningCost(new RunningCostInput(testX, testU, 0.0));
                    var expectedSize = problem.StateDim + problem.ControlDim + 1;

                    if (Options.Verbose)
                    {
                        Console.WriteLine($"Testing running cost gradients: hasGrads={(testResult.Gradients != null)}, length={testResult.Gradients?.Length ?? 0}, expected={expectedSize}");
                    }

                    useAnalyticalGradients = useAnalyticalGradients &&
                        testResult.Gradients != null &&
                        testResult.Gradients.Length >= expectedSize;
                }
                catch (Exception ex)
                {
                    if (Options.Verbose)
                    {
                        Console.WriteLine($"Running cost gradient test failed: {ex.Message}");
                    }
                    useAnalyticalGradients = false;
                }
            }

            if (problem.TerminalCost != null)
            {
                try
                {
                    var testX = new double[problem.StateDim];
                    var testResult = problem.TerminalCost(new TerminalCostInput(testX, 0.0));
                    var expectedSize = problem.StateDim + 1;
                    useAnalyticalGradients = useAnalyticalGradients &&
                        testResult.Gradients != null &&
                        testResult.Gradients.Length >= expectedSize;
                }
                catch
                {
                    useAnalyticalGradients = false;
                }
            }

            if (Options.Verbose && useAnalyticalGradients)
            {
                Console.WriteLine("Using analytical gradients from AutoDiff");
            }

            // Progress callback state
            var iterationCount = 0;

            // Build objective function for NLP
            Func<double[], (double value, double[] gradient)> nlpObjective = z =>
            {
                var cost = 0.0;

                // Add running cost if defined
                if (problem.RunningCost != null)
                {
                    cost += transcription.ComputeRunningCost(z, (x, u, t) => problem.RunningCost(new RunningCostInput(x, u, t)).Value);
                }

                // Add terminal cost if defined
                if (problem.TerminalCost != null)
                {
                    cost += transcription.ComputeTerminalCost(z, (x, t) => problem.TerminalCost(new TerminalCostInput(x, t)).Value);
                }

                double[] gradient;

                if (useAnalyticalGradients)
                {
                    // Use analytical gradients via AutoDiffLGLGradientHelper
                    gradient = new double[z.Length];

                    if (problem.RunningCost != null)
                    {
                        var runningGrad = AutoDiffLGLGradientHelper.ComputeRunningCostGradient(
                            problem, grid, z, Options.Order, transcription.GetState, transcription.GetControl,
                            (x, u, t) =>
                            {
                                var res = problem.RunningCost!(new RunningCostInput(x, u, t));
                                return (res.Value, res.Gradients!);
                            });

                        for (var i = 0; i < gradient.Length; i++)
                        {
                            gradient[i] += runningGrad[i];
                        }
                    }

                    if (problem.TerminalCost != null)
                    {
                        var terminalGrad = AutoDiffLGLGradientHelper.ComputeTerminalCostGradient(
                            problem, z, transcription.TotalPoints, grid.FinalTime, transcription.GetState,
                            (x, t) =>
                            {
                                var res = problem.TerminalCost!(new TerminalCostInput(x, t));
                                return (res.Value, res.Gradients!);
                            });

                        for (var i = 0; i < gradient.Length; i++)
                        {
                            gradient[i] += terminalGrad[i];
                        }
                    }
                }
                else
                {
                    // Compute gradient numerically
                    gradient = NumericalGradients.ComputeGradient(zz =>
                    {
                        var c = 0.0;
                        if (problem.RunningCost != null)
                        {
                            c += transcription.ComputeRunningCost(zz, (x, u, t) => problem.RunningCost(new RunningCostInput(x, u, t)).Value);
                        }
                        if (problem.TerminalCost != null)
                        {
                            c += transcription.ComputeTerminalCost(zz, (x, t) => problem.TerminalCost(new TerminalCostInput(x, t)).Value);
                        }
                        return c;
                    }, z);
                }

                // Invoke progress callback
                if (Options.ProgressCallback != null)
                {
                    iterationCount++;
                    var states = new double[transcription.TotalPoints][];
                    var controls = new double[transcription.TotalPoints][];

                    for (var i = 0; i < transcription.TotalPoints; i++)
                    {
                        states[i] = transcription.GetState(z, i);
                        controls[i] = transcription.GetControl(z, i);
                    }

                    // Compute max constraint violation for defects
                    var defects = transcription.ComputeAllDefects(z, dynamics);
                    var maxViolation = defects.Length > 0 ? defects.Max(d => Math.Abs(d)) : 0.0;

                    // Get time points from grid
                    var times = new double[transcription.TotalPoints];
                    var pointIndex = 0;
                    for (var k = 0; k < segments; k++)
                    {
                        var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(Options.Order);
                        var h = grid.GetTimeStep(k);
                        var tk = grid.TimePoints[k];

                        for (var ii = 0; ii < Options.Order; ii++)
                        {
                            if (k > 0 && ii == 0)
                                continue; // Skip shared endpoint

                            var tau = lglPoints[ii];
                            times[pointIndex++] = tk + (tau + 1.0) * h / 2.0;
                        }
                    }

                    Options.ProgressCallback(iterationCount, cost, states, controls, times, maxViolation, Options.Tolerance);
                }

                return (cost, gradient);
            };

            // Get LGL differentiation matrix for analytical gradients
            var lglDiffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(Options.Order);

            var analyticalDefectCount = 0;
            var numericalDefectFallbackCount = 0;

            // Create defect constraint functions (one for each defect component)
            Func<int, Func<double[], (double, double[])>> DefectConstraint = defectIndex => z =>
            {
                var allDefects = transcription.ComputeAllDefects(z, dynamics);

                double[] gradient;

                // Try to use analytical gradients if dynamics provides them
                if (hasAnalyticalDynamicsGradients)
                {
                    try
                    {
                        // Map flat defectIndex to (segmentIndex, interiorPointIndex, stateComponentIndex)
                        var numInteriorPointsPerSegment = Options.Order - 2;
                        var defectsPerSegment = numInteriorPointsPerSegment * problem.StateDim;

                        var segmentIndex = defectIndex / defectsPerSegment;
                        var remainder = defectIndex % defectsPerSegment;
                        var interiorPointIndex = 1 + (remainder / problem.StateDim); // +1 because interior points start at index 1
                        var stateComponentIndex = remainder % problem.StateDim;

                        gradient = AutoDiffLGLGradientHelper.ComputeDefectGradient(
                            problem, grid, z, Options.Order, lglDiffMatrix,
                            transcription.GetState, transcription.GetControl,
                            segmentIndex, interiorPointIndex, stateComponentIndex,
                            (x, u, t) =>
                            {
                                var res = problem.Dynamics!(new DynamicsInput(x, u, t, segmentIndex, grid.Segments));
                                return (res.Value, res.Gradients);
                            });
                        analyticalDefectCount++;
                    }
                    catch
                    {
                        // Fall back to numerical if AutoDiff fails
                        gradient = NumericalGradients.ComputeConstraintGradient(zz =>
                        {
                            var defects = transcription.ComputeAllDefects(zz, dynamics);
                            return defects[defectIndex];
                        }, z);
                        numericalDefectFallbackCount++;
                    }
                }
                else
                {
                    // Compute gradient numerically
                    gradient = NumericalGradients.ComputeConstraintGradient(zz =>
                    {
                        var defects = transcription.ComputeAllDefects(zz, dynamics);
                        return defects[defectIndex];
                    }, z);
                    numericalDefectFallbackCount++;
                }

                return (allDefects[defectIndex], gradient);
            };

            // Collect all constraints
            var totalDefects = segments * (Options.Order - 2) * problem.StateDim;

            // Verify gradients at initial guess for a few defects
            if (Options.Verbose && hasAnalyticalDynamicsGradients)
            {
                Console.WriteLine("Verifying defect gradients at initial guess...");
                var defectIndicesToCheck = new[] { 0, totalDefects / 2, totalDefects - 1 };
                var maxGradError = 0.0;
                var worstDefect = -1;

                foreach (var defectIdx in defectIndicesToCheck)
                {
                    if (defectIdx >= totalDefects)
                    {
                        continue;
                    }

                    var (defVal, analyticalGrad) = DefectConstraint(defectIdx)(z0);
                    var numericalGrad = NumericalGradients.ComputeConstraintGradient(zz =>
                    {
                        var defects = transcription.ComputeAllDefects(zz, dynamics);
                        return defects[defectIdx];
                    }, z0);

                    for (var idx = 0; idx < z0.Length; idx++)
                    {
                        var diff = Math.Abs(analyticalGrad[idx] - numericalGrad[idx]);
                        var scale = Math.Max(1.0, Math.Max(Math.Abs(analyticalGrad[idx]), Math.Abs(numericalGrad[idx])));
                        var relError = diff / scale;
                        if (relError > maxGradError)
                        {
                            maxGradError = relError;
                            worstDefect = defectIdx;
                        }
                    }
                }

                Console.WriteLine($"  Max relative gradient error: {maxGradError:E6} (defect {worstDefect})");
                if (maxGradError > 0.01)
                {
                    Console.WriteLine($"  WARNING: Large gradient error detected!");
                }
            }

            if (Options.Verbose)
            {
                Console.WriteLine($"Adding {totalDefects} defect constraints");
            }

            // Build equality constraints list
            var equalityConstraints = new List<Func<double[], (double value, double[] gradient)>>();

            for (var i = 0; i < totalDefects; i++)
            {
                equalityConstraints.Add(DefectConstraint(i));
            }

            // Add boundary condition constraints
            if (problem.InitialState != null)
            {
                for (var i = 0; i < problem.StateDim; i++)
                {
                    var stateIndex = i;
                    var targetValue = problem.InitialState[i];
                    equalityConstraints.Add(z =>
                    {
                        var x = transcription.GetState(z, 0);

                        double BoundaryValue(double[] zz)
                        {
                            var xx = transcription.GetState(zz, 0);
                            return xx[stateIndex] - targetValue;
                        }

                        var gradient = NumericalGradients.ComputeConstraintGradient(BoundaryValue, z);
                        return (x[stateIndex] - targetValue, gradient);
                    });
                }
            }

            // Add final state constraints
            if (problem.FinalState != null)
            {
                for (var i = 0; i < problem.StateDim; i++)
                {
                    var stateIndex = i;
                    var targetValue = problem.FinalState[i];

                    // Skip NaN values (free terminal conditions)
                    if (double.IsNaN(targetValue))
                    {
                        continue;
                    }

                    var finalPointIndex = transcription.TotalPoints - 1;
                    equalityConstraints.Add(z =>
                    {
                        var x = transcription.GetState(z, finalPointIndex);

                        double BoundaryValue(double[] zz)
                        {
                            var xx = transcription.GetState(zz, finalPointIndex);
                            return xx[stateIndex] - targetValue;
                        }

                        var gradient = NumericalGradients.ComputeConstraintGradient(BoundaryValue, z);
                        return (x[stateIndex] - targetValue, gradient);
                    });
                }
            }

            // Build box constraints if specified
            BoxConstraints? boxConstraints = null;
            if (problem.ControlLowerBounds != null && problem.ControlUpperBounds != null)
            {
                // Build flat bounds for decision vector
                var lowerBounds = new double[transcription.DecisionVectorSize];
                var upperBounds = new double[transcription.DecisionVectorSize];

                for (var i = 0; i < transcription.TotalPoints; i++)
                {
                    var offset = i * (problem.StateDim + problem.ControlDim);

                    // State bounds (if specified)
                    for (var j = 0; j < problem.StateDim; j++)
                    {
                        lowerBounds[offset + j] = problem.StateLowerBounds?[j] ?? double.NegativeInfinity;
                        upperBounds[offset + j] = problem.StateUpperBounds?[j] ?? double.PositiveInfinity;
                    }

                    // Control bounds
                    for (var j = 0; j < problem.ControlDim; j++)
                    {
                        lowerBounds[offset + problem.StateDim + j] = problem.ControlLowerBounds[j];
                        upperBounds[offset + problem.StateDim + j] = problem.ControlUpperBounds[j];
                    }
                }

                boxConstraints = new BoxConstraints(lowerBounds, upperBounds);
            }

            // Build inequality constraints list
            var inequalityConstraints = new List<Func<double[], (double value, double[] gradient)>>();

            // Add path constraints if specified
            if (problem.PathConstraints != null && problem.PathConstraints.Count > 0)
            {
                // Apply each path constraint at all collocation points
                for (var constraintIndex = 0; constraintIndex < problem.PathConstraints.Count; constraintIndex++)
                {
                    var pathConstraint = problem.PathConstraints[constraintIndex];

                    for (var k = 0; k < segments; k++)
                    {
                        var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(Options.Order);

                        for (var localIdx = 0; localIdx < Options.Order; localIdx++)
                        {
                            var globalIdx = k * (Options.Order - 1) + localIdx;

                            // Skip shared endpoint (already constrained by previous segment)
                            if (k > 0 && localIdx == 0)
                            {
                                continue;
                            }

                            var tk = grid.TimePoints[k];
                            var h = grid.GetTimeStep(k);
                            var tau = lglPoints[localIdx];
                            var timePoint = tk + (tau + 1.0) * h / 2.0;

                            inequalityConstraints.Add(z =>
                            {
                                var x = transcription.GetState(z, globalIdx);
                                var u = transcription.GetControl(z, globalIdx);
                                var input = new PathConstraintInput(x, u, timePoint);
                                var result = pathConstraint(input);

                                // Compute gradient numerically
                                double ConstraintValue(double[] zz)
                                {
                                    var xx = transcription.GetState(zz, globalIdx);
                                    var uu = transcription.GetControl(zz, globalIdx);
                                    var inp = new PathConstraintInput(xx, uu, timePoint);
                                    return pathConstraint(inp).Value;
                                }

                                var gradient = NumericalGradients.ComputeConstraintGradient(ConstraintValue, z);
                                return (result.Value, gradient);
                            });
                        }
                    }
                }
            }

            // Create options with collected constraints
            // Use higher initial penalty for LGL since defects are typically larger
            var options = new AugmentedLagrangianOptions
            {
                Tolerance = Options.Tolerance,
                MaxIterations = Options.MaxIterations,
                Verbose = Options.Verbose,
                PenaltyParameter = 100.0,
                ConstraintTolerance = Options.Tolerance,
                EqualityConstraints = equalityConstraints,
                InequalityConstraints = inequalityConstraints,
                BoxConstraints = boxConstraints
            };

            // Create optimizer with constructor DI
            var constrainedOptimizer = new AugmentedLagrangianOptimizer(
                options,
                _innerOptimizer ?? new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()),
                _monitor);

            // Solve the NLP
            var nlpResult = constrainedOptimizer.Minimize(nlpObjective, z0);

            // Extract solution
            var zOpt = nlpResult.OptimalPoint;

            // Build time, state, and control arrays
            var times = new double[transcription.TotalPoints];
            var states = new double[transcription.TotalPoints][];
            var controls = new double[transcription.TotalPoints][];

            for (var k = 0; k < segments; k++)
            {
                var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(Options.Order);

                for (var localIdx = 0; localIdx < Options.Order; localIdx++)
                {
                    var globalIdx = k * (Options.Order - 1) + localIdx;

                    // Skip shared endpoint (already set by previous segment)
                    if (k > 0 && localIdx == 0)
                    {
                        continue;
                    }

                    var tk = grid.TimePoints[k];
                    var h = grid.GetTimeStep(k);
                    var tau = lglPoints[localIdx];
                    times[globalIdx] = tk + (tau + 1.0) * h / 2.0;

                    states[globalIdx] = transcription.GetState(zOpt, globalIdx);
                    controls[globalIdx] = transcription.GetControl(zOpt, globalIdx);
                }
            }

            // Compute final defects
            var finalDefects = transcription.ComputeAllDefects(zOpt, dynamics);
            var maxDefect = LegendreGaussLobattoTranscription.MaxDefect(finalDefects);

            if (Options.Verbose)
            {
                Console.WriteLine($"\nSolver completed:");
                Console.WriteLine($"  Objective: {nlpResult.OptimalValue:E6}");
                Console.WriteLine($"  Iterations: {nlpResult.Iterations}");
                Console.WriteLine($"  Converged: {nlpResult.Success}");
                Console.WriteLine($"  Max defect: {maxDefect:E6}");
                Console.WriteLine($"  Defect gradient calls: {analyticalDefectCount} analytical, {numericalDefectFallbackCount} numerical");
            }

            return new CollocationResult
            {
                Times = times,
                States = states,
                Controls = controls,
                OptimalCost = nlpResult.OptimalValue,
                Iterations = nlpResult.Iterations,
                MaxDefect = maxDefect,
                Success = nlpResult.Success
            };
        }

        /// <summary>
        /// Solves the optimal control problem with adaptive mesh refinement.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <param name="initialGuess">Initial guess decision vector.</param>
        /// <returns>The collocation result containing the optimal trajectory.</returns>
        private CollocationResult SolveWithMeshRefinement(ControlProblem problem, double[] initialGuess)
        {
            var currentSegments = Options.Segments;
            CollocationResult? result = null;
            var previousSolution = initialGuess;

            for (var iteration = 0; iteration < Options.MaxRefinementIterations; iteration++)
            {
                if (Options.Verbose)
                {
                    Console.WriteLine($"Mesh refinement iteration {iteration + 1}, segments = {currentSegments}");
                }

                // Solve on current grid
                result = SolveOnFixedGrid(problem, currentSegments, previousSolution);

                if (!result.Success)
                {
                    if (Options.Verbose)
                    {
                        Console.WriteLine("Failed to converge on current mesh");
                    }
                    break;
                }

                if (Options.Verbose)
                {
                    Console.WriteLine($"  Max defect: {result.MaxDefect:E2}");
                }

                // Check if we should stop
                if (result.MaxDefect < Options.RefinementDefectThreshold)
                {
                    if (Options.Verbose)
                    {
                        Console.WriteLine($"Converged with max defect {result.MaxDefect:E2}");
                    }
                    break;
                }

                // Analyze defects and refine mesh
                var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, currentSegments);
                ICollocationTranscription transcription = Options.EnableParallelization
                    ? new ParallelLGLTranscription(problem, grid, Options.Order, Options.EnableParallelization)
                    : new LegendreGaussLobattoTranscription(problem, grid, Options.Order);

                // Rebuild solution vector for defect computation
                var z = new double[transcription.DecisionVectorSize];
                for (var k = 0; k < transcription.TotalPoints; k++)
                {
                    transcription.SetState(z, k, result.States[k]);
                    transcription.SetControl(z, k, result.Controls[k]);
                }

                var defects = transcription.ComputeAllDefects(z, problem.Dynamics!);

                // Identify segments for refinement
                var numInteriorPointsPerSegment = Options.Order - 2;
                var meshRefinement = new MeshRefinement(Options.RefinementDefectThreshold, maxSegments: 200);
                var shouldRefine = meshRefinement.IdentifySegmentsForRefinement(defects, problem.StateDim, numInteriorPointsPerSegment);

                var refinementPct = MeshRefinement.ComputeRefinementPercentage(shouldRefine);

                if (Options.Verbose)
                {
                    Console.WriteLine($"  Refining {refinementPct:F1}% of segments");
                }

                // If no segments need refinement, we're done
                if (refinementPct < 0.1)
                {
                    break;
                }

                // Create refined grid
                var newGrid = meshRefinement.RefineGrid(grid, shouldRefine);
                var newSegments = newGrid.Segments;

                if (newSegments == currentSegments)
                {
                    // Can't refine further (hit limit)
                    break;
                }

                // Interpolate solution to new grid for warm start
                ICollocationTranscription newTranscription = Options.EnableParallelization
                    ? new ParallelLGLTranscription(problem, newGrid, Options.Order, Options.EnableParallelization)
                    : new LegendreGaussLobattoTranscription(problem, newGrid, Options.Order);

                previousSolution = MeshRefinement.InterpolateLGLSolution(
                    grid, newGrid, z, Options.Order, problem.StateDim, problem.ControlDim);

                currentSegments = newSegments;
            }

            return result ?? throw new InvalidOperationException("Mesh refinement failed to produce a result.");
        }
    }
}
