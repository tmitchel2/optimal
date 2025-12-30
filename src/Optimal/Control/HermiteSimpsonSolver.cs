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
    public sealed class HermiteSimpsonSolver
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

            if (_enableMeshRefinement)
            {
                return SolveWithMeshRefinement(problem);
            }

            return SolveOnFixedGrid(problem, _segments, initialGuess: null);
        }

        /// <summary>
        /// Solves with adaptive mesh refinement.
        /// </summary>
        private CollocationResult SolveWithMeshRefinement(ControlProblem problem)
        {
            var currentSegments = _segments;
            CollocationResult? result = null;
            double[]? previousSolution = null;

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
                var u0 = new double[problem.ControlDim];
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

                // Compute gradient numerically
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

                var gradient = NumericalGradients.ComputeGradient(ObjectiveValue, z);

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

                    // Compute gradient numerically
                    double DefectValue(double[] zz)
                    {
                        var defects = transcription.ComputeAllDefects(zz, DynamicsValue);
                        return defects[defectIndex];
                    }

                    var gradient = NumericalGradients.ComputeConstraintGradient(DefectValue, z);
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
