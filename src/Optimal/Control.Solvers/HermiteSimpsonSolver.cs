/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.Control.Optimization;
using Optimal.Control.Scaling;
using Optimal.NonLinear;
using Optimal.NonLinear.Constrained;
using Optimal.NonLinear.Monitoring;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers
{
    /// <summary>
    /// Solves optimal control problems using Hermite-Simpson collocation.
    /// Converts the continuous problem into a nonlinear programming (NLP) problem
    /// and solves it using existing nonlinear optimizers.
    /// </summary>
    public sealed class HermiteSimpsonSolver : ISolver
    {
        private const double MinRefinementPercentage = 0.1;
        private const int MaxMeshSegments = 200;

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
        private bool _autoScaling;
        private VariableScaling? _scaling;
        private OptimisationMonitor? _monitor;

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

        /// <inheritdoc/>
        ISolver ISolver.WithOptimisationMonitor(OptimisationMonitor monitor) => WithOptimisationMonitor(monitor);

        /// <inheritdoc/>
        OptimisationMonitorReport? ISolver.GetMonitorReport() => GetMonitorReport();

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
        /// Enables automatic variable scaling based on problem bounds.
        /// Scales all state and control variables to [-1, 1] for improved conditioning.
        /// </summary>
        /// <param name="enable">True to enable auto-scaling (default: true).</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithAutoScaling(bool enable = true)
        {
            _autoScaling = enable;
            if (enable)
            {
                _scaling = null; // Clear any custom scaling
            }

            return this;
        }

        /// <summary>
        /// Sets custom variable scaling parameters.
        /// </summary>
        /// <param name="scaling">The scaling parameters to use.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithScaling(VariableScaling scaling)
        {
            _scaling = scaling ?? throw new ArgumentNullException(nameof(scaling));
            _autoScaling = false;
            return this;
        }

        /// <summary>
        /// Sets an optimization monitor for gradient verification and smoothness monitoring.
        /// </summary>
        /// <param name="monitor">The monitor to use.</param>
        /// <returns>This solver instance for method chaining.</returns>
        public HermiteSimpsonSolver WithOptimisationMonitor(OptimisationMonitor monitor)
        {
            _monitor = monitor ?? throw new ArgumentNullException(nameof(monitor));
            return this;
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
        /// <returns>The optimal control solution.</returns>
        public CollocationResult Solve(ControlProblem problem, InitialGuess initialGuess)
        {
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(initialGuess);

            if (problem.Dynamics == null)
            {
                throw new InvalidOperationException("Problem must have dynamics defined.");
            }

            // Determine if scaling should be applied
            var scaling = GetScaling(problem);
            if (scaling != null)
            {
                return SolveWithScaling(problem, initialGuess, scaling);
            }

            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, _segments);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid, _enableParallelization);
            var z0 = transcription.ToDecisionVector(initialGuess);

            if (_enableMeshRefinement)
            {
                return SolveWithMeshRefinement(problem, z0);
            }

            return SolveOnFixedGrid(problem, _segments, z0);
        }

        private VariableScaling? GetScaling(ControlProblem problem)
        {
            if (_scaling != null)
            {
                return _scaling;
            }

            if (!_autoScaling)
            {
                return null;
            }

            // Auto-scaling requires bounds to be defined
            if (problem.StateLowerBounds == null || problem.StateUpperBounds == null ||
                problem.ControlLowerBounds == null || problem.ControlUpperBounds == null)
            {
                if (_verbose)
                {
                    Console.WriteLine("Auto-scaling disabled: bounds not fully specified");
                }

                return null;
            }

            return VariableScaling.FromBounds(
                problem.StateLowerBounds,
                problem.StateUpperBounds,
                problem.ControlLowerBounds,
                problem.ControlUpperBounds);
        }

        private CollocationResult SolveWithScaling(ControlProblem problem, InitialGuess initialGuess, VariableScaling scaling)
        {
            if (_verbose)
            {
                Console.WriteLine("Solving with variable scaling enabled");
                LogScalingInfo(scaling);
            }

            // Create scaled problem
            var scaledProblem = new ScaledControlProblem(problem, scaling);
            var scaledControlProblem = scaledProblem.ToControlProblem();

            // Scale initial guess
            var scaledGuess = scaledProblem.ScaleInitialGuess(initialGuess);

            // Create transcription for scaled problem
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, _segments);
            var transcription = new ParallelHermiteSimpsonTranscription(scaledControlProblem, grid, _enableParallelization);
            var z0 = transcription.ToDecisionVector(scaledGuess);

            // Solve in scaled space
            CollocationResult scaledResult;
            if (_enableMeshRefinement)
            {
                scaledResult = SolveWithMeshRefinement(scaledControlProblem, z0);
            }
            else
            {
                scaledResult = SolveOnFixedGrid(scaledControlProblem, _segments, z0);
            }

            // Unscale the solution
            return UnscaleResult(scaledResult, scaledProblem);
        }

        private static CollocationResult UnscaleResult(CollocationResult scaledResult, ScaledControlProblem scaledProblem)
        {
            // Always unscale states and controls, even if solver didn't converge
            // so that the returned trajectories are in original coordinates
            var unscaledStates = scaledProblem.UnscaleStates(scaledResult.States);
            var unscaledControls = scaledProblem.UnscaleControls(scaledResult.Controls);

            return new CollocationResult
            {
                Success = scaledResult.Success,
                Message = scaledResult.Message,
                Times = scaledResult.Times,
                States = unscaledStates,
                Controls = unscaledControls,
                OptimalCost = scaledResult.OptimalCost,
                Iterations = scaledResult.Iterations,
                MaxDefect = scaledResult.MaxDefect,
                GradientNorm = scaledResult.GradientNorm
            };
        }

        private static void LogScalingInfo(VariableScaling scaling)
        {
            Console.WriteLine($"  State scales: [{string.Join(", ", scaling.StateScales.Select(s => s.ToString("G4", System.Globalization.CultureInfo.InvariantCulture)))}]");
            Console.WriteLine($"  State centers: [{string.Join(", ", scaling.StateCenters.Select(c => c.ToString("G4", System.Globalization.CultureInfo.InvariantCulture)))}]");
            Console.WriteLine($"  Control scales: [{string.Join(", ", scaling.ControlScales.Select(s => s.ToString("G4", System.Globalization.CultureInfo.InvariantCulture)))}]");
            Console.WriteLine($"  Control centers: [{string.Join(", ", scaling.ControlCenters.Select(c => c.ToString("G4", System.Globalization.CultureInfo.InvariantCulture)))}]");
        }

        /// <summary>
        /// Solves with adaptive mesh refinement.
        /// </summary>
        private CollocationResult SolveWithMeshRefinement(ControlProblem problem, double[] initialGuess)
        {
            var currentSegments = _segments;
            CollocationResult? result = null;
            var previousSolution = initialGuess;

            for (var iteration = 0; iteration < _maxRefinementIterations; iteration++)
            {
                LogMeshRefinementIteration(iteration, currentSegments);

                result = SolveOnFixedGrid(problem, currentSegments, previousSolution);

                if (!result.Success)
                {
                    LogConvergenceFailure();
                    break;
                }

                LogMaxDefect(result.MaxDefect);

                if (HasConverged(result.MaxDefect))
                {
                    break;
                }

                var (shouldContinue, newSegments, newSolution) = TryRefineMesh(problem, result, currentSegments, previousSolution);

                if (!shouldContinue)
                {
                    break;
                }

                currentSegments = newSegments;
                previousSolution = newSolution;
            }

            return result ?? new CollocationResult { Success = false, Message = "Mesh refinement failed to converge" };
        }

        private void LogMeshRefinementIteration(int iteration, int segments)
        {
            if (_verbose)
            {
                Console.WriteLine($"Mesh refinement iteration {iteration + 1}, segments = {segments}");
            }
        }

        private void LogConvergenceFailure()
        {
            if (_verbose)
            {
                Console.WriteLine("Failed to converge on current mesh");
            }
        }

        private void LogMaxDefect(double maxDefect)
        {
            if (_verbose)
            {
                Console.WriteLine($"  Max defect: {maxDefect:E2}");
            }
        }

        private bool HasConverged(double maxDefect)
        {
            var converged = maxDefect < _refinementDefectThreshold;
            if (converged && _verbose)
            {
                Console.WriteLine($"Converged with max defect {maxDefect:E2}");
            }
            return converged;
        }

        private (bool shouldContinue, int newSegments, double[] newSolution) TryRefineMesh(
            ControlProblem problem, CollocationResult result, int currentSegments, double[] currentSolution)
        {
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, currentSegments);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid, _enableParallelization);

            var z = RebuildSolutionVector(transcription, result, currentSegments);
            var defects = transcription.ComputeAllDefects(z, problem.Dynamics!);

            var meshRefinement = new MeshRefinement(_refinementDefectThreshold, maxSegments: MaxMeshSegments);
            var shouldRefine = meshRefinement.IdentifySegmentsForRefinement(defects, problem.StateDim);
            var refinementPct = MeshRefinement.ComputeRefinementPercentage(shouldRefine);

            if (_verbose)
            {
                Console.WriteLine($"  Refining {refinementPct:F1}% of segments");
            }

            if (refinementPct < MinRefinementPercentage)
            {
                return (false, currentSegments, currentSolution);
            }

            var newGrid = meshRefinement.RefineGrid(grid, shouldRefine);
            var newSegments = newGrid.Segments;

            if (newSegments == currentSegments)
            {
                return (false, currentSegments, currentSolution);
            }

            var oldTranscriptionCompat = new HermiteSimpsonTranscription(problem, grid);
            var newTranscriptionCompat = new HermiteSimpsonTranscription(problem, newGrid);
            var newSolution = MeshRefinement.InterpolateSolution(
                oldTranscriptionCompat, newTranscriptionCompat, z, grid, newGrid);

            return (true, newSegments, newSolution);
        }

        private static double[] RebuildSolutionVector(ParallelHermiteSimpsonTranscription transcription, CollocationResult result, int segments)
        {
            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= segments; k++)
            {
                transcription.SetState(z, k, result.States[k]);
                transcription.SetControl(z, k, result.Controls[k]);
            }
            return z;
        }

        /// <summary>
        /// Solves on a fixed grid (no refinement).
        /// </summary>
        private CollocationResult SolveOnFixedGrid(ControlProblem problem, int segmentCount, double[] initialGuess)
        {
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, segmentCount);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid, _enableParallelization);

            var z0 = initialGuess;
            var hasAnalyticalGradients = CheckAnalyticalGradientCapability(problem, segmentCount);
            var iterationCount = new int[1];

            var nlpObjective = CreateObjectiveFunction(problem, grid, transcription, segmentCount, iterationCount);
            var constrainedOptimizer = ConfigureOptimizer(problem, grid, transcription, segmentCount, z0, hasAnalyticalGradients);

            if (_verbose)
            {
                LogInitialDiagnostics(problem, transcription, segmentCount, z0, nlpObjective);
            }

            var nlpResult = constrainedOptimizer.Minimize(nlpObjective);
            return ExtractSolution(problem, grid, transcription, segmentCount, nlpResult);
        }

        private bool CheckAnalyticalGradientCapability(ControlProblem problem, int segmentCount)
        {
            try
            {
                var hasGradients = ValidateAnalyticalGradients(problem, segmentCount);
                LogGradientMode(hasGradients);
                return hasGradients;
            }
            catch
            {
                LogGradientMode(false);
                return false;
            }
        }

        private static bool ValidateAnalyticalGradients(ControlProblem problem, int segmentCount)
        {
            var testX = new double[problem.StateDim];
            var testU = new double[problem.ControlDim];
            var testResult = problem.Dynamics!(new DynamicsInput(testX, testU, 0.0, 0, segmentCount));

            if (testResult.Gradients == null || testResult.Gradients.Length < 2)
            {
                return false;
            }

            var gradWrtState = testResult.Gradients[0];
            var gradWrtControl = testResult.Gradients[1];
            var expectedStateGradSize = problem.StateDim * problem.StateDim;
            var expectedControlGradSize = problem.StateDim * problem.ControlDim;

            return gradWrtState != null && gradWrtState.Length == expectedStateGradSize &&
                   gradWrtControl != null && gradWrtControl.Length == expectedControlGradSize;
        }

        private void LogGradientMode(bool useAnalytical)
        {
            if (_verbose)
            {
                Console.WriteLine(useAnalytical
                    ? "Using analytical gradients from AutoDiff"
                    : "Using numerical finite-difference gradients");
            }
        }

        private Func<double[], (double value, double[] gradient)> CreateObjectiveFunction(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            int[] iterationCount)
        {
            return z =>
            {
                var cost = ObjectiveFunctionFactory.ComputeTotalCost(problem, transcription, z);
                var gradient = ObjectiveFunctionFactory.ComputeObjectiveGradient(problem, grid, transcription, z);

                if (_verbose)
                {
                    LogObjectiveWarnings(cost, gradient);
                }

                InvokeProgressCallback(grid, transcription, segments, z, cost, problem.Dynamics!, iterationCount);

                return (cost, gradient);
            };
        }

        private static void LogObjectiveWarnings(double cost, double[] gradient)
        {
            if (double.IsNaN(cost))
            {
                Console.WriteLine("WARNING: Objective cost is NaN!");
            }
            if (gradient.Length > 0 && double.IsNaN(gradient[0]))
            {
                Console.WriteLine("WARNING: Gradient is NaN!");
            }
        }

        private void InvokeProgressCallback(
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            double[] z,
            double cost,
            Func<DynamicsInput, DynamicsResult> dynamics,
            int[] iterationCount)
        {
            if (_progressCallback == null)
            {
                return;
            }

            iterationCount[0]++;
            var states = new double[segments + 1][];
            var controls = new double[segments + 1][];
            for (var k = 0; k <= segments; k++)
            {
                states[k] = transcription.GetState(z, k);
                controls[k] = transcription.GetControl(z, k);
            }

            var maxViolation = ComputeMaxViolation(transcription, z, dynamics);
            _progressCallback(iterationCount[0], cost, states, controls, grid.TimePoints, maxViolation, _tolerance);
        }

        private static double ComputeMaxViolation(ParallelHermiteSimpsonTranscription transcription, double[] z, Func<DynamicsInput, DynamicsResult> dynamics)
        {
            var allDefects = transcription.ComputeAllDefects(z, dynamics);
            ThrowIfDefectsContainNaN(allDefects);
            return allDefects.Select(Math.Abs).Max();
        }

        private AugmentedLagrangianOptimizer ConfigureOptimizer(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            double[] z0,
            bool hasAnalyticalGradients)
        {
            var constrainedOptimizer = new AugmentedLagrangianOptimizer()
                .WithUnconstrainedOptimizer(_innerOptimizer ?? new LBFGSOptimizer())
                .WithConstraintTolerance(_tolerance)
                .WithPenaltyParameter(_initialPenalty);

            constrainedOptimizer
                .WithInitialPoint(z0)
                .WithTolerance(_tolerance)
                .WithMaxIterations(_maxIterations)
                .WithVerbose(_verbose);

            // Add optimization monitor if configured
            if (_monitor != null)
            {
                constrainedOptimizer.WithOptimisationMonitor(_monitor);
            }

            ConstraintConfigurator.AddDefectConstraints(problem, grid, transcription, segments, hasAnalyticalGradients, constrainedOptimizer);
            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, segments, constrainedOptimizer);
            ConstraintConfigurator.AddBoxConstraints(problem, transcription, segments, constrainedOptimizer);
            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, segments, constrainedOptimizer);

            return constrainedOptimizer;
        }

        private static void LogInitialDiagnostics(
            ControlProblem problem,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            double[] z0,
            Func<double[], (double value, double[] gradient)> nlpObjective)
        {
            var (testCost, testGrad) = nlpObjective(z0);
            Console.WriteLine($"Test objective on initial guess: cost={testCost}, grad[0]={testGrad[0]}");

            var testDefects = transcription.ComputeAllDefects(z0, problem.Dynamics!);
            ThrowIfDefectsContainNaN(testDefects);
            LogMaxDefectPerState(testDefects, problem.StateDim, segments);
        }

        private static void ThrowIfDefectsContainNaN(double[] defects)
        {
            var firstNaNIndex = Array.FindIndex(defects, double.IsNaN);

            if (firstNaNIndex >= 0)
            {
                throw new InvalidOperationException($"Defect {firstNaNIndex} is NaN. Check dynamics function for numerical issues.");
            }
        }

        private static void LogMaxDefectPerState(double[] defects, int stateDim, int segments)
        {
            var maxDefectPerState = Enumerable.Range(0, stateDim)
                .Select(state => Enumerable.Range(0, segments)
                    .Select(seg => Math.Abs(defects[seg * stateDim + state]))
                    .Max())
                .ToArray();

            var formattedDefects = maxDefectPerState
                .Select(d => d.ToString("E3", System.Globalization.CultureInfo.InvariantCulture));

            Console.WriteLine($"Max defect per state component (initial guess): [{string.Join(", ", formattedDefects)}]");
        }

        private static CollocationResult ExtractSolution(ControlProblem problem, CollocationGrid grid, ParallelHermiteSimpsonTranscription transcription, int segments, OptimizerResult nlpResult)
        {
            var zOpt = nlpResult.OptimalPoint;

            var states = new double[segments + 1][];
            var controls = new double[segments + 1][];
            for (var k = 0; k <= segments; k++)
            {
                states[k] = transcription.GetState(zOpt, k);
                controls[k] = transcription.GetControl(zOpt, k);
            }

            var maxDefect = ComputeMaxViolation(transcription, zOpt, problem.Dynamics!);

            return new CollocationResult
            {
                Success = nlpResult.Success,
                Message = nlpResult.Message,
                Times = grid.TimePoints,
                States = states,
                Controls = controls,
                OptimalCost = nlpResult.OptimalValue,
                Iterations = nlpResult.Iterations,
                MaxDefect = maxDefect,
                GradientNorm = nlpResult.GradientNorm
            };
        }

        public ISolver WithOrder(int order)
        {
            // if (order != 2)
            // {
            //     throw new ArgumentException("Hermite order must be 2.", nameof(order));
            // }

            return this;
        }

        ISolver ISolver.WithParallelization(bool enable)
        {
            return WithParallelization(enable);
        }
    }
}
