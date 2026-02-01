/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;
using System.Threading;
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

        private readonly IOptimizer _innerOptimizer;
        private readonly OptimisationMonitor? _monitor;

        /// <summary>
        /// Gets the solver options.
        /// </summary>
        public HermiteSimpsonSolverOptions Options { get; }

        /// <summary>
        /// Gets the inner optimizer used for NLP solving.
        /// </summary>
        public IOptimizer InnerOptimizer => _innerOptimizer;

        /// <summary>
        /// Initializes a new instance of the <see cref="HermiteSimpsonSolver"/> class with the specified options.
        /// </summary>
        /// <param name="options">The solver options.</param>
        /// <param name="innerOptimizer">Inner NLP optimizer.</param>
        /// <param name="monitor">Optional optimization monitor for gradient verification.</param>
        public HermiteSimpsonSolver(
            HermiteSimpsonSolverOptions options,
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
        /// <param name="cancellationToken">Optional cancellation token to stop optimization early.</param>
        /// <returns>The optimal control solution.</returns>
        public CollocationResult Solve(ControlProblem problem, InitialGuess initialGuess, CancellationToken cancellationToken)
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
                return SolveWithScaling(problem, initialGuess, scaling, cancellationToken);
            }

            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, Options.Segments);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid, Options.EnableParallelization);
            var z0 = transcription.ToDecisionVector(initialGuess);

            if (Options.EnableMeshRefinement)
            {
                return SolveWithMeshRefinement(problem, z0, cancellationToken);
            }

            return SolveOnFixedGrid(problem, Options.Segments, z0, cancellationToken);
        }

        private VariableScaling? GetScaling(ControlProblem problem)
        {
            if (Options.Scaling != null)
            {
                return Options.Scaling;
            }

            if (!Options.AutoScaling)
            {
                return null;
            }

            // Auto-scaling requires bounds to be defined
            if (problem.StateLowerBounds == null || problem.StateUpperBounds == null ||
                problem.ControlLowerBounds == null || problem.ControlUpperBounds == null)
            {
                if (Options.Verbose)
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

        private CollocationResult SolveWithScaling(ControlProblem problem, InitialGuess initialGuess, VariableScaling scaling, CancellationToken cancellationToken)
        {
            if (Options.Verbose)
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
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, Options.Segments);
            var transcription = new ParallelHermiteSimpsonTranscription(scaledControlProblem, grid, Options.EnableParallelization);
            var z0 = transcription.ToDecisionVector(scaledGuess);

            // Solve in scaled space
            CollocationResult scaledResult;
            if (Options.EnableMeshRefinement)
            {
                scaledResult = SolveWithMeshRefinement(scaledControlProblem, z0, cancellationToken);
            }
            else
            {
                scaledResult = SolveOnFixedGrid(scaledControlProblem, Options.Segments, z0, cancellationToken);
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
        private CollocationResult SolveWithMeshRefinement(ControlProblem problem, double[] initialGuess, CancellationToken cancellationToken)
        {
            var currentSegments = Options.Segments;
            CollocationResult? result = null;
            var previousSolution = initialGuess;

            for (var iteration = 0; iteration < Options.MaxRefinementIterations; iteration++)
            {
                LogMeshRefinementIteration(iteration, currentSegments);

                result = SolveOnFixedGrid(problem, currentSegments, previousSolution, cancellationToken);

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
            if (Options.Verbose)
            {
                Console.WriteLine($"Mesh refinement iteration {iteration + 1}, segments = {segments}");
            }
        }

        private void LogConvergenceFailure()
        {
            if (Options.Verbose)
            {
                Console.WriteLine("Failed to converge on current mesh");
            }
        }

        private void LogMaxDefect(double maxDefect)
        {
            if (Options.Verbose)
            {
                Console.WriteLine($"  Max defect: {maxDefect:E2}");
            }
        }

        private bool HasConverged(double maxDefect)
        {
            var converged = maxDefect < Options.RefinementDefectThreshold;
            if (converged && Options.Verbose)
            {
                Console.WriteLine($"Converged with max defect {maxDefect:E2}");
            }
            return converged;
        }

        private (bool shouldContinue, int newSegments, double[] newSolution) TryRefineMesh(
            ControlProblem problem, CollocationResult result, int currentSegments, double[] currentSolution)
        {
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, currentSegments);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid, Options.EnableParallelization);

            var z = RebuildSolutionVector(transcription, result, currentSegments);
            var defects = transcription.ComputeAllDefects(z, problem.Dynamics!);

            var meshRefinement = new MeshRefinement(Options.RefinementDefectThreshold, maxSegments: MaxMeshSegments);
            var shouldRefine = meshRefinement.IdentifySegmentsForRefinement(defects, problem.StateDim);
            var refinementPct = MeshRefinement.ComputeRefinementPercentage(shouldRefine);

            if (Options.Verbose)
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
        private CollocationResult SolveOnFixedGrid(ControlProblem problem, int segmentCount, double[] initialGuess, CancellationToken cancellationToken)
        {
            // Reset monitor for new grid size (important for mesh refinement iterations)
            _monitor?.Reset();

            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, segmentCount);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid, Options.EnableParallelization);

            var z0 = initialGuess;
            var hasAnalyticalGradients = CheckAnalyticalGradientCapability(problem, segmentCount);
            var iterationCount = new int[1];

            var nlpObjective = CreateObjectiveFunction(problem, grid, transcription, segmentCount, iterationCount);
            var constrainedOptimizer = ConfigureOptimizer(problem, grid, transcription, segmentCount, hasAnalyticalGradients);

            if (Options.Verbose)
            {
                LogInitialDiagnostics(problem, transcription, segmentCount, z0, nlpObjective);
            }

            var nlpResult = constrainedOptimizer.Minimize(nlpObjective, z0, cancellationToken);
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
            if (Options.Verbose)
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

                if (Options.Verbose)
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
            if (Options.ProgressCallback == null)
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
            Options.ProgressCallback(iterationCount[0], cost, states, controls, grid.TimePoints, maxViolation, Options.Tolerance);
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
            bool hasAnalyticalGradients)
        {
            // Collect all constraints
            var constraints = new ConstraintCollection();
            ConstraintConfigurator.AddDefectConstraints(problem, grid, transcription, segments, hasAnalyticalGradients, constraints);
            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, segments, constraints);
            ConstraintConfigurator.AddBoxConstraints(problem, transcription, segments, constraints);
            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, segments, constraints);

            // Create options with collected constraints
            var options = new AugmentedLagrangianOptions
            {
                Tolerance = Options.Tolerance,
                MaxIterations = Options.MaxIterations,
                Verbose = Options.Verbose,
                PenaltyParameter = Options.InitialPenalty,
                ConstraintTolerance = Options.Tolerance,
                EqualityConstraints = constraints.EqualityConstraints,
                InequalityConstraints = constraints.InequalityConstraints,
                BoxConstraints = constraints.BoxConstraints,
                ProgressCallback = Options.InnerProgressCallback
            };

            // Create optimizer with constructor DI
            return new AugmentedLagrangianOptimizer(
                options,
                _innerOptimizer,
                _monitor);
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
    }
}
