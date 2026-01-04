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
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, segments);
            var transcription = new ParallelTranscription(problem, grid, _enableParallelization);

            var z0 = HermiteSimpsonInitialGuessGenerator.Generate(problem, transcription, initialGuess, _verbose);

            var dynamicsEvaluator = CreateDynamicsEvaluator(problem);
            var gradientCapabilities = DetectGradientCapabilities(problem);

            LogGradientCapabilities(gradientCapabilities);

            var iterationCounter = new HermiteSimpsonObjectiveBuilder.IterationCounter();
            var nlpObjective = HermiteSimpsonObjectiveBuilder.Build(
                problem, grid, transcription, dynamicsEvaluator,
                gradientCapabilities.HasCostGradients, segments, _verbose,
                _progressCallback, _tolerance, iterationCounter);

            var optimizer = ConfigureOptimizer(z0);

            LogInitialState(nlpObjective, z0);

            AddAllConstraints(optimizer, problem, grid, transcription, dynamicsEvaluator,
                gradientCapabilities.HasDynamicsGradients, segments, z0);

            var nlpResult = optimizer.Minimize(nlpObjective);

            return HermiteSimpsonSolutionExtractor.Extract(
                nlpResult, grid, transcription, dynamicsEvaluator, segments);
        }

        /// <summary>
        /// Creates a dynamics evaluator function.
        /// </summary>
        private static Func<double[], double[], double, double[]> CreateDynamicsEvaluator(ControlProblem problem)
        {
            return (x, u, t) =>
            {
                var result = problem.Dynamics!(x, u, t);
                return result.value;
            };
        }

        /// <summary>
        /// Detects gradient capabilities of the problem.
        /// </summary>
        private static GradientCapabilities DetectGradientCapabilities(ControlProblem problem)
        {
            var hasDynamicsGradients = GradientCapabilityDetector.HasAnalyticalDynamicsGradients(
                problem.Dynamics!, problem.StateDim, problem.ControlDim);

            var hasCostGradients = GradientCapabilityDetector.HasAnalyticalCostGradients(
                problem.RunningCost, problem.TerminalCost, problem.StateDim, problem.ControlDim);

            return new GradientCapabilities(hasDynamicsGradients, hasCostGradients);
        }

        /// <summary>
        /// Logs gradient capability information.
        /// </summary>
        private void LogGradientCapabilities(GradientCapabilities capabilities)
        {
            if (!_verbose)
            {
                return;
            }

            if (capabilities.HasDynamicsGradients)
            {
                Console.WriteLine("Using analytical gradients from AutoDiff");
            }
            else
            {
                Console.WriteLine("Using numerical finite-difference gradients");
            }
        }

        /// <summary>
        /// Configures the NLP optimizer with settings.
        /// </summary>
        private IOptimizer ConfigureOptimizer(double[] initialPoint)
        {
            var optimizer = new AugmentedLagrangianOptimizer()
                .WithUnconstrainedOptimizer(_innerOptimizer ?? new LBFGSOptimizer())
                .WithConstraintTolerance(_tolerance)
                .WithPenaltyParameter(_initialPenalty)
                .WithInitialPoint(initialPoint)
                .WithTolerance(_tolerance)
                .WithMaxIterations(_maxIterations)
                .WithVerbose(_verbose);

            return optimizer;
        }

        /// <summary>
        /// Logs initial state diagnostics.
        /// </summary>
        private void LogInitialState(
            Func<double[], (double value, double[] gradient)> objective,
            double[] initialGuess)
        {
            if (!_verbose)
            {
                return;
            }

            var (testCost, testGrad) = objective(initialGuess);
            Console.WriteLine($"Test objective on initial guess: cost={testCost}, grad[0]={testGrad[0]}");
        }

        /// <summary>
        /// Adds all constraints to the optimizer.
        /// </summary>
        private void AddAllConstraints(
            IOptimizer optimizer,
            ControlProblem problem,
            CollocationGrid grid,
            ParallelTranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            bool hasAnalyticalGradients,
            int segments,
            double[] initialGuess)
        {
            var augmentedOptimizer = (AugmentedLagrangianOptimizer)optimizer;

            HermiteSimpsonConstraintBuilder.AddDefectConstraints(
                augmentedOptimizer, problem, grid, transcription, dynamicsEvaluator,
                hasAnalyticalGradients, segments, _verbose, initialGuess);

            HermiteSimpsonConstraintBuilder.AddBoundaryConstraints(
                augmentedOptimizer, problem, transcription, segments);

            HermiteSimpsonConstraintBuilder.AddBoxConstraints(
                augmentedOptimizer, problem, transcription, segments);

            HermiteSimpsonConstraintBuilder.AddPathConstraints(
                augmentedOptimizer, problem, grid, transcription, segments);
        }

        /// <summary>
        /// Holds gradient capability detection results.
        /// </summary>
        private readonly struct GradientCapabilities
        {
            public bool HasDynamicsGradients { get; }
            public bool HasCostGradients { get; }

            public GradientCapabilities(bool hasDynamicsGradients, bool hasCostGradients)
            {
                HasDynamicsGradients = hasDynamicsGradients;
                HasCostGradients = hasCostGradients;
            }
        }
    }
}
