/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;
using Optimal.NonLinear;

namespace Optimal.Control
{
    /// <summary>
    /// Solves optimal control problems using Legendre-Gauss-Lobatto (LGL) collocation.
    /// Provides high-order accurate solutions with configurable collocation order.
    /// </summary>
    public sealed class LegendreGaussLobattoSolver
    {
        private int _segments = 20;
        private int _order = 4;
        private double _tolerance = 1e-6;
        private int _maxIterations = 100;
        private bool _verbose;
        private IOptimizer? _innerOptimizer;

        /// <summary>
        /// Sets the number of collocation segments.
        /// </summary>
        /// <param name="segments">Number of segments (must be > 0).</param>
        /// <returns>This solver for method chaining.</returns>
        public LegendreGaussLobattoSolver WithSegments(int segments)
        {
            if (segments <= 0)
            {
                throw new ArgumentException("Number of segments must be positive.", nameof(segments));
            }

            _segments = segments;
            return this;
        }

        /// <summary>
        /// Sets the LGL collocation order (number of collocation points per segment).
        /// Higher orders provide more accurate solutions but increase computational cost.
        /// Common values: 3, 4, 5, 7, 9.
        /// </summary>
        /// <param name="order">LGL order (must be >= 2).</param>
        /// <returns>This solver for method chaining.</returns>
        public LegendreGaussLobattoSolver WithOrder(int order)
        {
            if (order < 2)
            {
                throw new ArgumentException("LGL order must be at least 2.", nameof(order));
            }

            _order = order;
            return this;
        }

        /// <summary>
        /// Sets the convergence tolerance.
        /// </summary>
        /// <param name="tolerance">Tolerance for convergence (must be > 0).</param>
        /// <returns>This solver for method chaining.</returns>
        public LegendreGaussLobattoSolver WithTolerance(double tolerance)
        {
            if (tolerance <= 0)
            {
                throw new ArgumentException("Tolerance must be positive.", nameof(tolerance));
            }

            _tolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the maximum number of iterations.
        /// </summary>
        /// <param name="maxIterations">Maximum iterations (must be > 0).</param>
        /// <returns>This solver for method chaining.</returns>
        public LegendreGaussLobattoSolver WithMaxIterations(int maxIterations)
        {
            if (maxIterations <= 0)
            {
                throw new ArgumentException("Max iterations must be positive.", nameof(maxIterations));
            }

            _maxIterations = maxIterations;
            return this;
        }

        /// <summary>
        /// Sets verbose output mode.
        /// </summary>
        /// <param name="verbose">True to enable verbose output.</param>
        /// <returns>This solver for method chaining.</returns>
        public LegendreGaussLobattoSolver WithVerbose(bool verbose = true)
        {
            _verbose = verbose;
            return this;
        }

        /// <summary>
        /// Sets a custom inner optimizer for the unconstrained NLP subproblem.
        /// If not set, uses LBFGSOptimizer by default.
        /// </summary>
        /// <param name="optimizer">The optimizer to use.</param>
        /// <returns>This solver for method chaining.</returns>
        public LegendreGaussLobattoSolver WithInnerOptimizer(IOptimizer optimizer)
        {
            _innerOptimizer = optimizer;
            return this;
        }

        /// <summary>
        /// Solves the optimal control problem.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <returns>The collocation result containing the optimal trajectory.</returns>
        public CollocationResult Solve(ControlProblem problem)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (problem.Dynamics == null)
            {
                throw new InvalidOperationException("Problem must have dynamics defined.");
            }

            if (_verbose)
            {
                Console.WriteLine($"Legendre-Gauss-Lobatto Solver");
                Console.WriteLine($"  Segments: {_segments}");
                Console.WriteLine($"  Order: {_order}");
                Console.WriteLine($"  State dimension: {problem.StateDim}");
                Console.WriteLine($"  Control dimension: {problem.ControlDim}");
            }

            // Create collocation grid
            var grid = new CollocationGrid(
                problem.InitialTime,
                problem.FinalTime,
                _segments);

            // Create transcription
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, _order);

            if (_verbose)
            {
                Console.WriteLine($"  Total collocation points: {transcription.TotalPoints}");
                Console.WriteLine($"  Decision vector size: {transcription.DecisionVectorSize}");
                Console.WriteLine($"  Defects per segment: {_order - 2}");
                Console.WriteLine($"  Total defect constraints: {_segments * (_order - 2) * problem.StateDim}");
            }

            // Create initial guess
            var initialState = problem.InitialState ?? new double[problem.StateDim];
            var finalState = problem.FinalState ?? Enumerable.Repeat(double.NaN, problem.StateDim).ToArray();

            // For simple dynamics like ẋ = u, estimate required control from state change
            var constantControl = new double[problem.ControlDim];
            var duration = grid.FinalTime - grid.InitialTime;
            for (var i = 0; i < Math.Min(problem.StateDim, problem.ControlDim); i++)
            {
                if (!double.IsNaN(finalState[i]) && duration > 0)
                {
                    // Estimate average control needed: (final - initial) / duration
                    constantControl[i] = (finalState[i] - initialState[i]) / duration;
                }
            }

            var z0 = transcription.CreateInitialGuess(initialState, finalState, constantControl);

            // Extract dynamics evaluator (without gradients for now)
            double[] DynamicsValue(double[] x, double[] u, double t)
            {
                var result = problem.Dynamics(x, u, t);
                return result.value;
            }

            if (_verbose)
            {
                Console.WriteLine($"Initial guess:");
                Console.WriteLine($"  Constant control: {string.Join(", ", constantControl.Select(c => c.ToString("F6", System.Globalization.CultureInfo.InvariantCulture)))}");

                // Check first 10 points
                Console.WriteLine($"  First 10 states: {string.Join(", ", Enumerable.Range(0, Math.Min(10, transcription.TotalPoints)).Select(i => transcription.GetState(z0, i)[0].ToString("F6", System.Globalization.CultureInfo.InvariantCulture)))}");
                Console.WriteLine($"  First 10 controls: {string.Join(", ", Enumerable.Range(0, Math.Min(10, transcription.TotalPoints)).Select(i => transcription.GetControl(z0, i)[0].ToString("F6", System.Globalization.CultureInfo.InvariantCulture)))}");

                // Compute defects at initial guess
                var initialDefects = transcription.ComputeAllDefects(z0, DynamicsValue);
                var maxInitialDefect = initialDefects.Length > 0 ? initialDefects.Max(d => Math.Abs(d)) : 0.0;
                Console.WriteLine($"  Max defect at initial guess: {maxInitialDefect:E6}");

                // Compute cost at initial guess
                var initialCost = 0.0;
                if (problem.RunningCost != null)
                {
                    initialCost += transcription.ComputeRunningCost(z0, (x, u, t) => problem.RunningCost(x, u, t).value);
                }
                if (problem.TerminalCost != null)
                {
                    initialCost += transcription.ComputeTerminalCost(z0, (x, t) => problem.TerminalCost(x, t).value);
                }
                Console.WriteLine($"  Cost at initial guess: {initialCost:E6} (should be ~0.2)");
            }

            // Check if dynamics provides analytical gradients
            var hasAnalyticalDynamicsGradients = false;
            try
            {
                var testX = new double[problem.StateDim];
                var testU = new double[problem.ControlDim];
                var testResult = problem.Dynamics(testX, testU, 0.0);

                if (testResult.gradients != null && testResult.gradients.Length >= 2)
                {
                    var gradWrtState = testResult.gradients[0];
                    var gradWrtControl = testResult.gradients[1];
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

            if (_verbose && hasAnalyticalDynamicsGradients)
            {
                Console.WriteLine("Dynamics provides analytical gradients");
            }

            // Check if we can use analytical gradients for costs
            var useAnalyticalGradients = true;

            if (problem.RunningCost != null)
            {
                try
                {
                    var testX = new double[problem.StateDim];
                    var testU = new double[problem.ControlDim];
                    var testResult = problem.RunningCost(testX, testU, 0.0);
                    var expectedSize = problem.StateDim + problem.ControlDim + 1;

                    if (_verbose)
                    {
                        Console.WriteLine($"Testing running cost gradients: hasGrads={(testResult.gradients != null)}, length={testResult.gradients?.Length ?? 0}, expected={expectedSize}");
                    }

                    useAnalyticalGradients = useAnalyticalGradients &&
                        testResult.gradients != null &&
                        testResult.gradients.Length >= expectedSize;
                }
                catch (Exception ex)
                {
                    if (_verbose)
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
                    var testResult = problem.TerminalCost(testX, 0.0);
                    var expectedSize = problem.StateDim + 1;
                    useAnalyticalGradients = useAnalyticalGradients &&
                        testResult.gradients != null &&
                        testResult.gradients.Length >= expectedSize;
                }
                catch
                {
                    useAnalyticalGradients = false;
                }
            }

            if (_verbose && useAnalyticalGradients)
            {
                Console.WriteLine("Using analytical gradients from AutoDiff");
            }

            // Build objective function for NLP
            Func<double[], (double value, double[] gradient)> nlpObjective = z =>
            {
                var cost = 0.0;

                // Add running cost if defined
                if (problem.RunningCost != null)
                {
                    cost += transcription.ComputeRunningCost(z, (x, u, t) => problem.RunningCost(x, u, t).value);
                }

                // Add terminal cost if defined
                if (problem.TerminalCost != null)
                {
                    cost += transcription.ComputeTerminalCost(z, (x, t) => problem.TerminalCost(x, t).value);
                }

                double[] gradient;

                if (useAnalyticalGradients)
                {
                    // Use analytical gradients via AutoDiffLGLGradientHelper
                    gradient = new double[z.Length];

                    if (problem.RunningCost != null)
                    {
                        var runningGrad = AutoDiffLGLGradientHelper.ComputeRunningCostGradient(
                            problem, grid, z, _order, transcription.GetState, transcription.GetControl,
                            (x, u, t) =>
                            {
                                var res = problem.RunningCost!(x, u, t);
                                return (res.value, res.gradients!);
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
                                var res = problem.TerminalCost!(x, t);
                                return (res.value, res.gradients!);
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
                            c += transcription.ComputeRunningCost(zz, (x, u, t) => problem.RunningCost(x, u, t).value);
                        }
                        if (problem.TerminalCost != null)
                        {
                            c += transcription.ComputeTerminalCost(zz, (x, t) => problem.TerminalCost(x, t).value);
                        }
                        return c;
                    }, z);
                }

                return (cost, gradient);
            };

            // Get LGL differentiation matrix for analytical gradients
            var lglDiffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(_order);

            var analyticalDefectCount = 0;
            var numericalDefectFallbackCount = 0;

            // Create defect constraint functions (one for each defect component)
            Func<int, Func<double[], (double, double[])>> DefectConstraint = defectIndex => z =>
            {
                var allDefects = transcription.ComputeAllDefects(z, DynamicsValue);

                double[] gradient;

                // Try to use analytical gradients if dynamics provides them
                if (hasAnalyticalDynamicsGradients)
                {
                    try
                    {
                        // Map flat defectIndex to (segmentIndex, interiorPointIndex, stateComponentIndex)
                        var numInteriorPointsPerSegment = _order - 2;
                        var defectsPerSegment = numInteriorPointsPerSegment * problem.StateDim;

                        var segmentIndex = defectIndex / defectsPerSegment;
                        var remainder = defectIndex % defectsPerSegment;
                        var interiorPointIndex = 1 + (remainder / problem.StateDim); // +1 because interior points start at index 1
                        var stateComponentIndex = remainder % problem.StateDim;

                        gradient = AutoDiffLGLGradientHelper.ComputeDefectGradient(
                            problem, grid, z, _order, lglDiffMatrix,
                            transcription.GetState, transcription.GetControl,
                            segmentIndex, interiorPointIndex, stateComponentIndex,
                            (x, u, t) =>
                            {
                                var res = problem.Dynamics!(x, u, t);
                                return (res.value, res.gradients);
                            });
                        analyticalDefectCount++;
                    }
                    catch
                    {
                        // Fall back to numerical if AutoDiff fails
                        gradient = NumericalGradients.ComputeConstraintGradient(zz =>
                        {
                            var defects = transcription.ComputeAllDefects(zz, DynamicsValue);
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
                        var defects = transcription.ComputeAllDefects(zz, DynamicsValue);
                        return defects[defectIndex];
                    }, z);
                    numericalDefectFallbackCount++;
                }

                return (allDefects[defectIndex], gradient);
            };

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
            var totalDefects = _segments * (_order - 2) * problem.StateDim;

            if (_verbose)
            {
                Console.WriteLine($"Adding {totalDefects} defect constraints");
            }

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
                    constrainedOptimizer.WithEqualityConstraint(z =>
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

            // Add control bounds if specified
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

                constrainedOptimizer.WithBoxConstraints(lowerBounds, upperBounds);
            }

            // Add path constraints if specified
            if (problem.PathConstraints != null && problem.PathConstraints.Count > 0)
            {
                // Apply each path constraint at all collocation points
                for (var constraintIndex = 0; constraintIndex < problem.PathConstraints.Count; constraintIndex++)
                {
                    var pathConstraint = problem.PathConstraints[constraintIndex];

                    for (var k = 0; k < _segments; k++)
                    {
                        var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(_order);

                        for (var localIdx = 0; localIdx < _order; localIdx++)
                        {
                            var globalIdx = k * (_order - 1) + localIdx;

                            // Skip shared endpoint (already constrained by previous segment)
                            if (k > 0 && localIdx == 0)
                            {
                                continue;
                            }

                            var tk = grid.TimePoints[k];
                            var h = grid.GetTimeStep(k);
                            var tau = lglPoints[localIdx];
                            var timePoint = tk + (tau + 1.0) * h / 2.0;

                            constrainedOptimizer.WithInequalityConstraint(z =>
                            {
                                var x = transcription.GetState(z, globalIdx);
                                var u = transcription.GetControl(z, globalIdx);
                                var result = pathConstraint(x, u, timePoint);

                                // Compute gradient numerically
                                double ConstraintValue(double[] zz)
                                {
                                    var xx = transcription.GetState(zz, globalIdx);
                                    var uu = transcription.GetControl(zz, globalIdx);
                                    var res = pathConstraint(xx, uu, timePoint);
                                    return res.value;
                                }

                                var gradient = NumericalGradients.ComputeConstraintGradient(ConstraintValue, z);
                                return (result.value, gradient);
                            });
                        }
                    }
                }
            }

            // Solve the NLP
            var nlpResult = constrainedOptimizer.Minimize(nlpObjective);

            // Extract solution
            var zOpt = nlpResult.OptimalPoint;

            // Build time, state, and control arrays
            var times = new double[transcription.TotalPoints];
            var states = new double[transcription.TotalPoints][];
            var controls = new double[transcription.TotalPoints][];

            for (var k = 0; k < _segments; k++)
            {
                var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(_order);

                for (var localIdx = 0; localIdx < _order; localIdx++)
                {
                    var globalIdx = k * (_order - 1) + localIdx;

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
            var finalDefects = transcription.ComputeAllDefects(zOpt, DynamicsValue);
            var maxDefect = LegendreGaussLobattoTranscription.MaxDefect(finalDefects);

            if (_verbose)
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
    }
}
