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
        private const double MinRefinementPercentage = 0.1;
        private const double PositionTolerance = 1e-10;
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

                var (shouldContinue, newSegments, newSolution) = TryRefineMesh(problem, result, currentSegments);

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

        private (bool shouldContinue, int newSegments, double[]? newSolution) TryRefineMesh(
            ControlProblem problem, CollocationResult result, int currentSegments)
        {
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, currentSegments);
            var transcription = new ParallelTranscription(problem, grid, _enableParallelization);

            double[] DynamicsValue(double[] x, double[] u, double t) => problem.Dynamics!(x, u, t).value;

            var z = RebuildSolutionVector(transcription, result, currentSegments);
            var defects = transcription.ComputeAllDefects(z, DynamicsValue);

            var meshRefinement = new MeshRefinement(_refinementDefectThreshold, maxSegments: MaxMeshSegments);
            var shouldRefine = meshRefinement.IdentifySegmentsForRefinement(defects, problem.StateDim);
            var refinementPct = MeshRefinement.ComputeRefinementPercentage(shouldRefine);

            if (_verbose)
            {
                Console.WriteLine($"  Refining {refinementPct:F1}% of segments");
            }

            if (refinementPct < MinRefinementPercentage)
            {
                return (false, currentSegments, null);
            }

            var newGrid = meshRefinement.RefineGrid(grid, shouldRefine);
            var newSegments = newGrid.Segments;

            if (newSegments == currentSegments)
            {
                return (false, currentSegments, null);
            }

            var oldTranscriptionCompat = new HermiteSimpsonTranscription(problem, grid);
            var newTranscriptionCompat = new HermiteSimpsonTranscription(problem, newGrid);
            var newSolution = MeshRefinement.InterpolateSolution(
                oldTranscriptionCompat, newTranscriptionCompat, z, grid, newGrid);

            return (true, newSegments, newSolution);
        }

        private static double[] RebuildSolutionVector(ParallelTranscription transcription, CollocationResult result, int segments)
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
        private CollocationResult SolveOnFixedGrid(ControlProblem problem, int segments, double[]? initialGuess)
        {
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, segments);
            var transcription = new ParallelTranscription(problem, grid, _enableParallelization);

            var z0 = CreateInitialGuess(problem, transcription, initialGuess);
            var hasAnalyticalGradients = CheckAnalyticalGradientCapability(problem);
            var iterationCount = new int[1];

            var nlpObjective = CreateObjectiveFunction(problem, grid, transcription, segments, hasAnalyticalGradients, iterationCount);
            var constrainedOptimizer = ConfigureOptimizer(problem, grid, transcription, segments, z0, hasAnalyticalGradients, nlpObjective);

            if (_verbose)
            {
                LogInitialDiagnostics(problem, transcription, segments, z0, nlpObjective);
            }

            var nlpResult = constrainedOptimizer.Minimize(nlpObjective);
            return ExtractSolution(problem, grid, transcription, segments, nlpResult);
        }

        private double[] CreateInitialGuess(ControlProblem problem, ParallelTranscription transcription, double[]? initialGuess)
        {
            if (initialGuess != null && initialGuess.Length == transcription.DecisionVectorSize)
            {
                return initialGuess;
            }

            var x0 = problem.InitialState ?? new double[problem.StateDim];
            var xf = problem.FinalState ?? new double[problem.StateDim];
            var u0 = ComputeInitialControlGuess(problem, x0, xf);
            var z0 = transcription.CreateInitialGuess(x0, xf, u0);

            if (_verbose)
            {
                var hasNaN = z0.Any(double.IsNaN);
                Console.WriteLine($"Initial guess created: size={z0.Length}, hasNaN={hasNaN}");
            }

            return z0;
        }

        private static double[] ComputeInitialControlGuess(ControlProblem problem, double[] x0, double[] xf)
        {
            var u0 = new double[problem.ControlDim];

            if (ShouldComputeAngleBasedGuess(problem))
            {
                var dx = xf[0] - x0[0];
                var dy = xf[1] - x0[1];
                if (HasSignificantDisplacement(dx, dy))
                {
                    var angle = Math.Atan2(-dy, dx);
                    u0[0] = Math.Clamp(angle, 0.0, Math.PI / 2.0);
                }
            }

            return u0;
        }

        private static bool ShouldComputeAngleBasedGuess(ControlProblem problem) =>
            problem.ControlDim == 1 &&
            problem.StateDim >= 2 &&
            problem.InitialState != null &&
            problem.FinalState != null;

        private static bool HasSignificantDisplacement(double dx, double dy) =>
            Math.Abs(dx) > PositionTolerance || Math.Abs(dy) > PositionTolerance;

        private bool CheckAnalyticalGradientCapability(ControlProblem problem)
        {
            try
            {
                var hasGradients = ValidateAnalyticalGradients(problem);
                LogGradientMode(hasGradients);
                return hasGradients;
            }
            catch
            {
                LogGradientMode(false);
                return false;
            }
        }

        private static bool ValidateAnalyticalGradients(ControlProblem problem)
        {
            var testX = new double[problem.StateDim];
            var testU = new double[problem.ControlDim];
            var testResult = problem.Dynamics!(testX, testU, 0.0);

            if (testResult.gradients == null || testResult.gradients.Length < 2)
            {
                return false;
            }

            var gradWrtState = testResult.gradients[0];
            var gradWrtControl = testResult.gradients[1];
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
            ParallelTranscription transcription,
            int segments,
            bool hasAnalyticalGradients,
            int[] iterationCount)
        {
            double[] DynamicsValue(double[] x, double[] u, double t) => problem.Dynamics!(x, u, t).value;

            return z =>
            {
                var cost = ComputeTotalCost(problem, transcription, z);
                var gradient = ComputeObjectiveGradient(problem, grid, transcription, z);

                if (_verbose)
                {
                    LogObjectiveWarnings(cost, gradient);
                }

                InvokeProgressCallback(problem, grid, transcription, segments, z, cost, DynamicsValue, iterationCount);

                return (cost, gradient);
            };
        }

        private static double ComputeTotalCost(ControlProblem problem, ParallelTranscription transcription, double[] z)
        {
            var cost = 0.0;

            if (problem.RunningCost != null)
            {
                double RunningCostValue(double[] x, double[] u, double t) => problem.RunningCost(x, u, t).value;
                cost += transcription.ComputeRunningCost(z, RunningCostValue);
            }

            if (problem.TerminalCost != null)
            {
                double TerminalCostValue(double[] x, double t) => problem.TerminalCost(x, t).value;
                cost += transcription.ComputeTerminalCost(z, TerminalCostValue);
            }

            return cost;
        }

        private static double[] ComputeObjectiveGradient(ControlProblem problem, CollocationGrid grid, ParallelTranscription transcription, double[] z)
        {
            if (!CanUseAnalyticalGradientsForCosts(problem))
            {
                return NumericalGradients.ComputeGradient(zz => ComputeTotalCost(problem, transcription, zz), z);
            }

            var gradient = new double[z.Length];

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

            return gradient;
        }

        private static bool CanUseAnalyticalGradientsForCosts(ControlProblem problem)
        {
            var useAnalytical = true;

            if (problem.RunningCost != null)
            {
                try
                {
                    var testResult = problem.RunningCost(new double[problem.StateDim], new double[problem.ControlDim], 0.0);
                    var expectedSize = problem.StateDim + problem.ControlDim + 1;
                    useAnalytical = useAnalytical && testResult.gradients != null && testResult.gradients.Length >= expectedSize;
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
                    var testResult = problem.TerminalCost(new double[problem.StateDim], 0.0);
                    var expectedSize = problem.StateDim + 1;
                    useAnalytical = useAnalytical && testResult.gradients != null && testResult.gradients.Length >= expectedSize;
                }
                catch
                {
                    useAnalytical = false;
                }
            }

            return useAnalytical;
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
            ControlProblem problem,
            CollocationGrid grid,
            ParallelTranscription transcription,
            int segments,
            double[] z,
            double cost,
            Func<double[], double[], double, double[]> dynamicsValue,
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

            var maxViolation = ComputeMaxViolation(transcription, z, dynamicsValue);
            _progressCallback(iterationCount[0], cost, states, controls, grid.TimePoints, maxViolation, _tolerance);
        }

        private static double ComputeMaxViolation(ParallelTranscription transcription, double[] z, Func<double[], double[], double, double[]> dynamicsValue)
        {
            var allDefects = transcription.ComputeAllDefects(z, dynamicsValue);
            return allDefects.Select(Math.Abs).Max();
        }

        private AugmentedLagrangianOptimizer ConfigureOptimizer(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelTranscription transcription,
            int segments,
            double[] z0,
            bool hasAnalyticalGradients,
            Func<double[], (double value, double[] gradient)> nlpObjective)
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

            ConstraintConfigurator.AddDefectConstraints(problem, grid, transcription, segments, hasAnalyticalGradients, constrainedOptimizer);
            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, segments, constrainedOptimizer);
            ConstraintConfigurator.AddBoxConstraints(problem, transcription, segments, constrainedOptimizer);
            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, segments, constrainedOptimizer);

            return constrainedOptimizer;
        }

        private void LogInitialDiagnostics(
            ControlProblem problem,
            ParallelTranscription transcription,
            int segments,
            double[] z0,
            Func<double[], (double value, double[] gradient)> nlpObjective)
        {
            double[] DynamicsValue(double[] x, double[] u, double t) => problem.Dynamics!(x, u, t).value;

            var (testCost, testGrad) = nlpObjective(z0);
            Console.WriteLine($"Test objective on initial guess: cost={testCost}, grad[0]={testGrad[0]}");

            var testDefects = transcription.ComputeAllDefects(z0, DynamicsValue);
            var defectHasNaN = testDefects.Any(double.IsNaN);

            if (defectHasNaN)
            {
                for (var i = 0; i < testDefects.Length; i++)
                {
                    if (double.IsNaN(testDefects[i]))
                    {
                        Console.WriteLine($"WARNING: Defect {i} is NaN on initial guess!");
                        break;
                    }
                }
            }
            else
            {
                Console.WriteLine($"All {testDefects.Length} defects are finite on initial guess");
            }

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

        private static CollocationResult ExtractSolution(ControlProblem problem, CollocationGrid grid, ParallelTranscription transcription, int segments, OptimizerResult nlpResult)
        {
            double[] DynamicsValue(double[] x, double[] u, double t) => problem.Dynamics!(x, u, t).value;
            var zOpt = nlpResult.OptimalPoint;

            var states = new double[segments + 1][];
            var controls = new double[segments + 1][];
            for (var k = 0; k <= segments; k++)
            {
                states[k] = transcription.GetState(zOpt, k);
                controls[k] = transcription.GetControl(zOpt, k);
            }

            var maxDefect = ComputeMaxViolation(transcription, zOpt, DynamicsValue);

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
    }
}
