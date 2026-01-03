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
    /// Callback invoked during optimization to report progress.
    /// </summary>
    /// <param name="iteration">Current iteration number.</param>
    /// <param name="cost">Current cost value.</param>
    /// <param name="states">Current state trajectory.</param>
    /// <param name="controls">Current control trajectory.</param>
    /// <param name="times">Time points.</param>
    /// <param name="maxViolation">Maximum constraint violation.</param>
    /// <param name="constraintTolerance">Constraint tolerance for convergence.</param>
    public delegate void ProgressCallback(int iteration, double cost, double[][] states, double[][] controls, double[] times, double maxViolation, double constraintTolerance);

    /// <summary>
    /// Solves optimal control problems using Hermite-Simpson collocation.
    /// Converts the continuous problem into a nonlinear programming (NLP) problem
    /// and solves it using existing nonlinear optimizers.
    /// </summary>
    public sealed class HermiteSimpsonSolver : ISolver
    {
        private int _segments = 20;
        private double _tolerance = 1e-6;
        private int _maxIterations = 100;
        private bool _verbose;
        private IOptimizer? _innerOptimizer;
        private bool _enableMeshRefinement;
        private int _maxRefinementIterations = 5;
        private double _refinementDefectThreshold = 1e-4;
        private bool _enableParallelization = true;
        private ProgressCallback? _progressCallback;
        private double _initialPenalty = 1.0;

        /// <inheritdoc/>
        ISolver ISolver.WithSegments(int segments) => WithSegments(segments);

        /// <inheritdoc/>
        ISolver ISolver.WithTolerance(double tolerance) => WithTolerance(tolerance);

        /// <inheritdoc/>
        ISolver ISolver.WithMaxIterations(int maxIterations) => WithMaxIterations(maxIterations);

        /// <inheritdoc/>
        ISolver ISolver.WithVerbose(bool verbose) => WithVerbose(verbose);

        /// <inheritdoc/>
        ISolver ISolver.WithInnerOptimizer(IOptimizer optimizer) => WithInnerOptimizer(optimizer);

        /// <inheritdoc/>
        ISolver ISolver.WithMeshRefinement(bool enable, int maxRefinementIterations, double defectThreshold) =>
            WithMeshRefinement(enable, maxRefinementIterations, defectThreshold);

        /// <inheritdoc/>
        ISolver ISolver.WithProgressCallback(ProgressCallback? callback) => WithProgressCallback(callback);

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
        /// Enables adaptive mesh refinement.
        /// </summary>
        /// <param name="enable">True to enable mesh refinement.</param>
        /// <param name="maxRefinementIterations">Maximum refinement iterations.</param>
        /// <param name="defectThreshold">Defect threshold for refinement.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithMeshRefinement(bool enable = true, int maxRefinementIterations = 5, double defectThreshold = 1e-4)
        {
            _enableMeshRefinement = enable;
            _maxRefinementIterations = maxRefinementIterations;
            _refinementDefectThreshold = defectThreshold;
            return this;
        }

        /// <summary>
        /// Enables or disables parallel computation for transcription.
        /// </summary>
        /// <param name="enable">True to enable parallelization.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithParallelization(bool enable = true)
        {
            _enableParallelization = enable;
            return this;
        }

        /// <summary>
        /// Sets a callback to be invoked during optimization progress.
        /// </summary>
        /// <param name="callback">Progress callback function.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithProgressCallback(ProgressCallback? callback)
        {
            _progressCallback = callback;
            return this;
        }

        /// <summary>
        /// Sets the initial penalty parameter for the augmented Lagrangian optimizer.
        /// Higher values enforce constraints more strongly from the start.
        /// </summary>
        /// <param name="penalty">Initial penalty parameter (default: 1.0).</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithInitialPenalty(double penalty)
        {
            if (penalty <= 0)
            {
                throw new ArgumentException("Penalty must be positive.", nameof(penalty));
            }

            _initialPenalty = penalty;
            return this;
        }

        /// <summary>
        /// Solves the optimal control problem.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <param name="initialGuess">Optional initial guess for decision variables.</param>
        /// <returns>The optimal control solution.</returns>
        public CollocationResult Solve(ControlProblem problem, double[]? initialGuess = null)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (problem.Dynamics == null)
            {
                throw new InvalidOperationException("Problem must have dynamics defined.");
            }

            if (_enableMeshRefinement)
            {
                return SolveWithMeshRefinement(problem, initialGuess);
            }

            return SolveOnFixedGrid(problem, _segments, initialGuess);
        }

        /// <summary>
        /// Solves with adaptive mesh refinement.
        /// </summary>
        private CollocationResult SolveWithMeshRefinement(ControlProblem problem, double[]? initialGuess)
        {
            var currentSegments = _segments;
            CollocationResult? result = null;
            double[]? previousSolution = initialGuess;

            for (var iteration = 0; iteration < _maxRefinementIterations; iteration++)
            {
                if (_verbose)
                {
                    Console.WriteLine($"Mesh refinement iteration {iteration + 1}, segments = {currentSegments}");
                }

                // Solve on current grid
                result = SolveOnFixedGrid(problem, currentSegments, previousSolution);

                if (!result.Success)
                {
                    if (_verbose)
                    {
                        Console.WriteLine("Failed to converge on current mesh");
                    }
                    break;
                }

                if (_verbose)
                {
                    Console.WriteLine($"  Max defect: {result.MaxDefect:E2}");
                }

                // Check if we should stop
                if (result.MaxDefect < _refinementDefectThreshold)
                {
                    if (_verbose)
                    {
                        Console.WriteLine($"Converged with max defect {result.MaxDefect:E2}");
                    }
                    break;
                }

                // Analyze defects and refine mesh
                var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, currentSegments);
                var transcription = new ParallelTranscription(problem, grid, _enableParallelization);

                // Extract dynamics evaluator
                double[] DynamicsValue(double[] x, double[] u, double t)
                {
                    var res = problem.Dynamics!(x, u, t);
                    return res.value;
                }

                // Rebuild solution vector for defect computation
                var z = new double[transcription.DecisionVectorSize];
                for (var k = 0; k <= currentSegments; k++)
                {
                    transcription.SetState(z, k, result.States[k]);
                    transcription.SetControl(z, k, result.Controls[k]);
                }

                var defects = transcription.ComputeAllDefects(z, DynamicsValue);

                // Identify segments for refinement
                var meshRefinement = new MeshRefinement(_refinementDefectThreshold, maxSegments: 200);
                var shouldRefine = meshRefinement.IdentifySegmentsForRefinement(defects, problem.StateDim);

                var refinementPct = MeshRefinement.ComputeRefinementPercentage(shouldRefine);

                if (_verbose)
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
                var newTranscription = new ParallelTranscription(problem, newGrid, _enableParallelization);

                // Convert to HermiteSimpsonTranscription for interpolation (shares same layout)
                var oldTranscriptionCompat = new HermiteSimpsonTranscription(problem, grid);
                var newTranscriptionCompat = new HermiteSimpsonTranscription(problem, newGrid);
                previousSolution = MeshRefinement.InterpolateSolution(
                    oldTranscriptionCompat, newTranscriptionCompat, z, grid, newGrid);

                currentSegments = newSegments;
            }

            return result ?? new CollocationResult { Success = false, Message = "Mesh refinement failed to converge" };
        }

        /// <summary>
        /// Solves on a fixed grid (no refinement).
        /// </summary>
        private CollocationResult SolveOnFixedGrid(ControlProblem problem, int segments, double[]? initialGuess)
        {
            // Create collocation grid
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, segments);
            var transcription = new ParallelTranscription(problem, grid, _enableParallelization);

            // Create initial guess
            double[] z0;
            if (initialGuess != null && initialGuess.Length == transcription.DecisionVectorSize)
            {
                z0 = initialGuess;
            }
            else
            {
                var x0 = problem.InitialState ?? new double[problem.StateDim];
                var xf = problem.FinalState ?? new double[problem.StateDim];

                // Compute reasonable initial control guess based on start/end states
                var u0 = new double[problem.ControlDim];
                if (problem.ControlDim == 1 && problem.StateDim >= 2 &&
                    problem.InitialState != null && problem.FinalState != null)
                {
                    // For Brachistochrone-like problems, initialize control to point toward target
                    var dx = xf[0] - x0[0];
                    var dy = xf[1] - x0[1];  // Negative if descending
                    if (Math.Abs(dx) > 1e-10 || Math.Abs(dy) > 1e-10)
                    {
                        // For Brachistochrone, angle is from horizontal, positive for descent
                        // ẏ = -v·sin(θ), so when descending (dy < 0), we need θ > 0
                        // Use absolute value of descent angle and clamp to [0, π/2]
                        var angle = Math.Atan2(-dy, dx);  // Negate dy since we measure descent angle
                        u0[0] = Math.Clamp(angle, 0.0, Math.PI / 2.0);
                    }
                }

                z0 = transcription.CreateInitialGuess(x0, xf, u0);

                if (_verbose)
                {
                    var hasNaN = false;
                    for (var i = 0; i < z0.Length; i++)
                    {
                        if (double.IsNaN(z0[i]))
                        {
                            hasNaN = true;
                            break;
                        }
                    }
                    Console.WriteLine($"Initial guess created: size={z0.Length}, hasNaN={hasNaN}");
                }
            }

            // Extract dynamics evaluator (without gradients for now)
            double[] DynamicsValue(double[] x, double[] u, double t)
            {
                var result = problem.Dynamics!(x, u, t);
                return result.value;
            }

            // Check if dynamics provides gradients
            var hasAnalyticalGradients = false;
            try
            {
                var testX = new double[problem.StateDim];
                var testU = new double[problem.ControlDim];
                var testResult = problem.Dynamics!(testX, testU, 0.0);
                // Check if gradients are not only present but also properly populated
                if (testResult.gradients != null && testResult.gradients.Length >= 2)
                {
                    var gradWrtState = testResult.gradients[0];
                    var gradWrtControl = testResult.gradients[1];
                    // Verify gradients are the right size and not all zeros
                    var expectedStateGradSize = problem.StateDim * problem.StateDim;
                    var expectedControlGradSize = problem.StateDim * problem.ControlDim;
                    
                    hasAnalyticalGradients = 
                        gradWrtState != null && gradWrtState.Length == expectedStateGradSize &&
                        gradWrtControl != null && gradWrtControl.Length == expectedControlGradSize;
                }
            }
            catch
            {
                // If evaluation fails, fall back to numerical gradients
            }

            if (_verbose && hasAnalyticalGradients)
            {
                Console.WriteLine("Using analytical gradients from AutoDiff");
            }
            else if (_verbose)
            {
                Console.WriteLine("Using numerical finite-difference gradients");
            }

            // Iteration counter for progress callback
            var iterationCount = 0;

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

                // Compute gradient using AutoDiff if available, otherwise numerically
                double[] gradient;

                // Check if we can use analytical gradients for costs
                var useAnalytical = true;
                
                if (problem.RunningCost != null)
                {
                    try
                    {
                        var testX = new double[problem.StateDim];
                        var testU = new double[problem.ControlDim];
                        var testResult = problem.RunningCost(testX, testU, 0.0);
                        // gradients array for running cost should be: [∂L/∂x (StateDim), ∂L/∂u (ControlDim), ∂L/∂t]
                        var expectedSize = problem.StateDim + problem.ControlDim + 1;
                        useAnalytical = useAnalytical && 
                            testResult.gradients != null && 
                            testResult.gradients.Length >= expectedSize;
                    }
                    catch
                    {
                        useAnalytical = false;
                    }
                }

                if (problem.TerminalCost != null)
                {
                    try
                    {
                        var testX = new double[problem.StateDim];
                        var testResult = problem.TerminalCost(testX, 0.0);
                        // gradients array for terminal cost should be: [∂Φ/∂x (StateDim), ∂Φ/∂t]
                        var expectedSize = problem.StateDim + 1;
                        useAnalytical = useAnalytical && 
                            testResult.gradients != null && 
                            testResult.gradients.Length >= expectedSize;
                    }
                    catch
                    {
                        useAnalytical = false;
                    }
                }

                if (useAnalytical)
                {
                    // Use analytical gradients via AutoDiffGradientHelper
                    gradient = new double[z.Length];

                    if (problem.RunningCost != null)
                    {
                        var runningGrad = AutoDiffGradientHelper.ComputeRunningCostGradient(
                            problem, grid, z, transcription.GetState, transcription.GetControl,
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
                        var terminalGrad = AutoDiffGradientHelper.ComputeTerminalCostGradient(
                            problem, grid, z, transcription.GetState,
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
                    // Fall back to numerical gradients
                    double ObjectiveValue(double[] zz)
                    {
                        var obj = 0.0;

                        if (problem.RunningCost != null)
                        {
                            double RunningCostValue(double[] x, double[] u, double t)
                            {
                                var result = problem.RunningCost(x, u, t);
                                return result.value;
                            }

                            obj += transcription.ComputeRunningCost(zz, RunningCostValue);
                        }

                        if (problem.TerminalCost != null)
                        {
                            double TerminalCostValue(double[] x, double t)
                            {
                                var result = problem.TerminalCost(x, t);
                                return result.value;
                            }

                            obj += transcription.ComputeTerminalCost(zz, TerminalCostValue);
                        }

                        return obj;
                    }

                    gradient = NumericalGradients.ComputeGradient(ObjectiveValue, z);
                }

                if (_verbose && double.IsNaN(cost))
                {
                    Console.WriteLine($"WARNING: Objective cost is NaN!");
                }
                if (_verbose && gradient.Length > 0 && double.IsNaN(gradient[0]))
                {
                    Console.WriteLine($"WARNING: Gradient is NaN!");
                }

                // Invoke progress callback
                if (_progressCallback != null)
                {
                    iterationCount++;
                    var states = new double[segments + 1][];
                    var controls = new double[segments + 1][];
                    for (var k = 0; k <= segments; k++)
                    {
                        states[k] = transcription.GetState(z, k);
                        controls[k] = transcription.GetControl(z, k);
                    }

                    // Compute maximum constraint violation from defects
                    var allDefects = transcription.ComputeAllDefects(z, DynamicsValue);
                    var maxViolation = 0.0;
                    foreach (var d in allDefects)
                    {
                        var abs = Math.Abs(d);
                        if (abs > maxViolation)
                        {
                            maxViolation = abs;
                        }
                    }

                    _progressCallback(iterationCount, cost, states, controls, grid.TimePoints, maxViolation, _tolerance);
                }

                return (cost, gradient);
            };

            // Create defect constraints
            Func<double[], (double value, double[] gradient)> DefectConstraint(int defectIndex)
            {
                return z =>
                {
                    var allDefects = transcription.ComputeAllDefects(z, DynamicsValue);

                    // Compute which segment and state component this defect corresponds to
                    var segmentIndex = defectIndex / problem.StateDim;
                    var stateComponentIndex = defectIndex % problem.StateDim;

                    double[] gradient;

                    // Try to use analytical gradients if dynamics provides them
                    if (hasAnalyticalGradients)
                    {
                        try
                        {
                            gradient = AutoDiffGradientHelper.ComputeDefectGradient(
                                problem, grid, z, transcription.GetState, transcription.GetControl,
                                segmentIndex, stateComponentIndex,
                                (x, u, t) =>
                                {
                                    var res = problem.Dynamics!(x, u, t);
                                    return (res.value, res.gradients);
                                });
                        }
                        catch
                        {
                            // Fall back to numerical if AutoDiff fails
                            double DefectValue(double[] zz)
                            {
                                var defects = transcription.ComputeAllDefects(zz, DynamicsValue);
                                return defects[defectIndex];
                            }

                            gradient = NumericalGradients.ComputeConstraintGradient(DefectValue, z);
                        }
                    }
                    else
                    {
                        // Compute gradient numerically
                        double DefectValue(double[] zz)
                        {
                            var defects = transcription.ComputeAllDefects(zz, DynamicsValue);
                            return defects[defectIndex];
                        }

                        gradient = NumericalGradients.ComputeConstraintGradient(DefectValue, z);
                    }

                    return (allDefects[defectIndex], gradient);
                };
            }

            // Set up constrained optimizer
            var constrainedOptimizer = new AugmentedLagrangianOptimizer()
                .WithUnconstrainedOptimizer(_innerOptimizer ?? new LBFGSOptimizer())
                .WithConstraintTolerance(_tolerance)
                .WithPenaltyParameter(_initialPenalty);

            constrainedOptimizer
                .WithInitialPoint(z0)
                .WithTolerance(_tolerance)
                .WithMaxIterations(_maxIterations)
                .WithVerbose(_verbose);

            // Add all defect constraints as equality constraints
            var totalDefects = _segments * problem.StateDim;

            if (_verbose)
            {
                // Test objective evaluation on initial guess
                var (testCost, testGrad) = nlpObjective(z0);
                Console.WriteLine($"Test objective on initial guess: cost={testCost}, grad[0]={testGrad[0]}");

                // Test defect evaluation on initial guess
                var testDefects = transcription.ComputeAllDefects(z0, DynamicsValue);
                var defectHasNaN = false;
                for (var i = 0; i < testDefects.Length; i++)
                {
                    if (double.IsNaN(testDefects[i]))
                    {
                        Console.WriteLine($"WARNING: Defect {i} is NaN on initial guess!");
                        defectHasNaN = true;
                        break;
                    }
                }
                if (!defectHasNaN)
                {
                    Console.WriteLine($"All {testDefects.Length} defects are finite on initial guess");
                }

                // Report max defect per state component
                var maxDefectPerState = new double[problem.StateDim];
                for (var seg = 0; seg < segments; seg++)
                {
                    for (var state = 0; state < problem.StateDim; state++)
                    {
                        var defectIdx = seg * problem.StateDim + state;
                        var absDefect = Math.Abs(testDefects[defectIdx]);
                        if (absDefect > maxDefectPerState[state])
                        {
                            maxDefectPerState[state] = absDefect;
                        }
                    }
                }
                Console.WriteLine($"Max defect per state component (initial guess): [{string.Join(", ", maxDefectPerState.Select(d => d.ToString("E3", System.Globalization.CultureInfo.InvariantCulture)))}]");
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

                    constrainedOptimizer.WithEqualityConstraint(z =>
                    {
                        var x = transcription.GetState(z, _segments);

                        double BoundaryValue(double[] zz)
                        {
                            var xx = transcription.GetState(zz, _segments);
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

            // Add path constraints if specified
            if (problem.PathConstraints.Count > 0)
            {
                // Apply each path constraint at all collocation nodes
                for (var constraintIndex = 0; constraintIndex < problem.PathConstraints.Count; constraintIndex++)
                {
                    var pathConstraint = problem.PathConstraints[constraintIndex];

                    for (var k = 0; k <= _segments; k++)
                    {
                        var nodeIndex = k;
                        var timePoint = grid.TimePoints[k];

                        constrainedOptimizer.WithInequalityConstraint(z =>
                        {
                            var x = transcription.GetState(z, nodeIndex);
                            var u = transcription.GetControl(z, nodeIndex);
                            var result = pathConstraint(x, u, timePoint);

                            // Compute gradient numerically
                            double ConstraintValue(double[] zz)
                            {
                                var xx = transcription.GetState(zz, nodeIndex);
                                var uu = transcription.GetControl(zz, nodeIndex);
                                var res = pathConstraint(xx, uu, timePoint);
                                return res.value;
                            }

                            var gradient = NumericalGradients.ComputeConstraintGradient(ConstraintValue, z);
                            return (result.value, gradient);
                        });
                    }
                }
            }

            // Solve the NLP
            var nlpResult = constrainedOptimizer.Minimize(nlpObjective);

            // Extract solution
            var zOpt = nlpResult.OptimalPoint;

            var times = grid.TimePoints;
            var states = new double[segments + 1][];
            var controls = new double[segments + 1][];

            for (var k = 0; k <= segments; k++)
            {
                states[k] = transcription.GetState(zOpt, k);
                controls[k] = transcription.GetControl(zOpt, k);
            }

            // Compute final defects
            var finalDefects = transcription.ComputeAllDefects(zOpt, DynamicsValue);
            var maxDefect = 0.0;
            foreach (var d in finalDefects)
            {
                var abs = Math.Abs(d);
                if (abs > maxDefect)
                {
                    maxDefect = abs;
                }
            }

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
