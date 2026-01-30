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
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers
{
    /// <summary>
    /// Solves multi-phase optimal control problems.
    /// Handles multiple phases with linkage constraints between them.
    /// </summary>
    public sealed class MultiPhaseSolver
    {
        private readonly HermiteSimpsonSolver _phaseSolver;
        private bool _verbose;

        /// <summary>
        /// Creates a multi-phase solver with the given single-phase solver.
        /// </summary>
        /// <param name="phaseSolver">Solver to use for individual phases.</param>
        public MultiPhaseSolver(HermiteSimpsonSolver phaseSolver)
        {
            _phaseSolver = phaseSolver ?? throw new ArgumentNullException(nameof(phaseSolver));
        }

        /// <summary>
        /// Enables verbose output.
        /// </summary>
        /// <param name="verbose">True to enable verbose output.</param>
        /// <returns>This solver for method chaining.</returns>
        public MultiPhaseSolver WithVerbose(bool verbose = true)
        {
            _verbose = verbose;
            return this;
        }

        /// <summary>
        /// Solves a multi-phase optimal control problem.
        /// </summary>
        /// <param name="multiPhaseProblem">The multi-phase problem to solve.</param>
        /// <returns>Multi-phase result.</returns>
        public MultiPhaseResult Solve(MultiPhaseControlProblem multiPhaseProblem)
        {
            ArgumentNullException.ThrowIfNull(multiPhaseProblem);

            multiPhaseProblem.Validate();

            // For simplicity, solve each phase sequentially with continuity constraints
            // A more advanced implementation would solve all phases simultaneously in one NLP

            if (_verbose)
            {
                Console.WriteLine($"Solving multi-phase problem with {multiPhaseProblem.Phases.Count} phases...");
            }

            // Solve phases iteratively, enforcing linkage constraints
            var phaseResults = new CollocationResult[multiPhaseProblem.Phases.Count];
            var phaseDurations = new double[multiPhaseProblem.Phases.Count];

            // First pass: solve each phase independently
            for (var i = 0; i < multiPhaseProblem.Phases.Count; i++)
            {
                var phase = multiPhaseProblem.Phases[i];

                if (_verbose)
                {
                    Console.WriteLine($"\nPhase {i + 1}: {phase.Name}");
                }

                var result = SolvePhase(phase, null);
                phaseResults[i] = result;
                phaseDurations[i] = phase.Duration;

                if (!result.Success)
                {
                    if (_verbose)
                    {
                        Console.WriteLine($"  Failed to converge");
                    }

                    return new MultiPhaseResult
                    {
                        Success = false,
                        Message = $"Phase {i + 1} failed to converge",
                        PhaseResults = phaseResults,
                        PhaseDurations = phaseDurations
                    };
                }

                if (_verbose)
                {
                    Console.WriteLine($"  Cost: {result.OptimalCost:E3}, Defect: {result.MaxDefect:E3}");
                }
            }

            // Second pass: refine solutions with linkage constraints
            // Use warm starting to ensure continuity between phases
            for (var iteration = 0; iteration < 3; iteration++) // Limited refinement iterations
            {
                if (_verbose)
                {
                    Console.WriteLine($"\nRefinement iteration {iteration + 1}...");
                }

                var improved = false;

                for (var i = 1; i < multiPhaseProblem.Phases.Count; i++)
                {
                    // Check linkage constraint with previous phase
                    var linkages = multiPhaseProblem.Linkages
                        .Where(l => l.Phase1Index == i - 1 && l.Phase2Index == i)
                        .ToList();

                    if (linkages.Count > 0)
                    {
                        var phase = multiPhaseProblem.Phases[i];
                        var prevResult = phaseResults[i - 1];

                        // Adjust initial condition to match previous phase final state
                        var finalStatePrev = prevResult.States[prevResult.States.Length - 1];

                        // Create modified problem with updated initial condition
                        var modifiedProblem = CreateModifiedProblem(phase.Problem!, finalStatePrev);

                        // Solve with warm start
                        var result = SolvePhaseWithWarmStart(phase, phaseResults[i], modifiedProblem);

                        if (result.Success && result.MaxDefect < phaseResults[i].MaxDefect)
                        {
                            phaseResults[i] = result;
                            improved = true;

                            if (_verbose)
                            {
                                Console.WriteLine($"  Phase {i + 1} improved: defect = {result.MaxDefect:E3}");
                            }
                        }
                    }
                }

                if (!improved)
                {
                    break; // No improvement, stop refinement
                }
            }

            // Compute linkage violations
            var maxLinkageViolation = ComputeMaxLinkageViolation(phaseResults, multiPhaseProblem.Linkages);

            // Compute total cost
            var totalCost = phaseResults.Sum(r => r.OptimalCost);
            var totalIterations = phaseResults.Sum(r => r.Iterations);

            if (_verbose)
            {
                Console.WriteLine($"\nMulti-phase solution complete:");
                Console.WriteLine($"  Total cost: {totalCost:E3}");
                Console.WriteLine($"  Max linkage violation: {maxLinkageViolation:E3}");
            }

            return new MultiPhaseResult
            {
                Success = true,
                Message = "Multi-phase optimization successful",
                PhaseResults = phaseResults,
                PhaseDurations = phaseDurations,
                TotalCost = totalCost,
                MaxLinkageViolation = maxLinkageViolation,
                TotalIterations = totalIterations
            };
        }

        /// <summary>
        /// Solves a single phase.
        /// </summary>
        private CollocationResult SolvePhase(ControlPhase phase, InitialGuess? initialGuess)
        {
            var solver = new HermiteSimpsonSolver()
                .WithSegments(phase.Segments)
                .WithTolerance(_phaseSolver.GetTolerance())
                .WithMaxIterations(_phaseSolver.GetMaxIterations())
                .WithInnerOptimizer(_phaseSolver.GetInnerOptimizer());

            var guess = initialGuess ?? InitialGuessFactory.CreateWithControlHeuristics(phase.Problem!, phase.Segments);
            return solver.Solve(phase.Problem!, guess);
        }

        /// <summary>
        /// Solves a phase with warm start from previous solution.
        /// </summary>
        private CollocationResult SolvePhaseWithWarmStart(
            ControlPhase phase,
            CollocationResult previousSolution,
            ControlProblem modifiedProblem)
        {
            var grid = new CollocationGrid(
                modifiedProblem.InitialTime,
                modifiedProblem.FinalTime,
                phase.Segments);

            var warmStart = WarmStart.InterpolateFromPrevious(previousSolution, grid);

            var solver = new HermiteSimpsonSolver()
                .WithSegments(phase.Segments)
                .WithTolerance(_phaseSolver.GetTolerance())
                .WithMaxIterations(_phaseSolver.GetMaxIterations())
                .WithInnerOptimizer(_phaseSolver.GetInnerOptimizer());

            return solver.Solve(modifiedProblem, warmStart);
        }

        /// <summary>
        /// Creates a modified problem with updated initial condition.
        /// </summary>
        private static ControlProblem CreateModifiedProblem(ControlProblem original, double[] newInitialState)
        {
            var modified = new ControlProblem()
                .WithStateSize(original.StateDim)
                .WithControlSize(original.ControlDim)
                .WithTimeHorizon(original.InitialTime, original.FinalTime)
                .WithInitialCondition(newInitialState);

            if (original.Dynamics != null)
            {
                modified = modified.WithDynamics(original.Dynamics);
            }

            if (original.RunningCost != null)
            {
                modified = modified.WithRunningCost(original.RunningCost);
            }

            if (original.TerminalCost != null)
            {
                modified = modified.WithTerminalCost(original.TerminalCost);
            }

            if (original.FinalState != null)
            {
                modified = modified.WithFinalCondition(original.FinalState);
            }

            if (original.ControlLowerBounds != null && original.ControlUpperBounds != null)
            {
                modified = modified.WithControlBounds(original.ControlLowerBounds, original.ControlUpperBounds);
            }

            if (original.StateLowerBounds != null && original.StateUpperBounds != null)
            {
                modified = modified.WithStateBounds(original.StateLowerBounds, original.StateUpperBounds);
            }

            return modified;
        }

        /// <summary>
        /// Computes the maximum linkage constraint violation.
        /// </summary>
        private static double ComputeMaxLinkageViolation(
            CollocationResult[] phaseResults,
            IReadOnlyList<PhaseLinkage> linkages)
        {
            var maxViolation = 0.0;

            foreach (var linkage in linkages)
            {
                var result1 = phaseResults[linkage.Phase1Index];
                var result2 = phaseResults[linkage.Phase2Index];

                var xf1 = result1.States[result1.States.Length - 1];
                var x02 = result2.States[0];

                if (linkage.Constraint != null)
                {
                    var (value, _) = linkage.Constraint(xf1, x02);
                    var violation = Math.Abs(value);

                    if (violation > maxViolation)
                    {
                        maxViolation = violation;
                    }
                }
            }

            return maxViolation;
        }
    }

    /// <summary>
    /// Extension methods to access private members of HermiteSimpsonSolver.
    /// </summary>
    public static class HermiteSimpsonSolverExtensions
    {
        /// <summary>
        /// Gets the tolerance from the solver.
        /// </summary>
        public static double GetTolerance(this HermiteSimpsonSolver solver)
        {
            ArgumentNullException.ThrowIfNull(solver);

            var field = typeof(HermiteSimpsonSolver).GetField(
                "_tolerance",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            return field != null ? (double)field.GetValue(solver)! : 1e-6;
        }

        /// <summary>
        /// Gets the max iterations from the solver.
        /// </summary>
        public static int GetMaxIterations(this HermiteSimpsonSolver solver)
        {
            ArgumentNullException.ThrowIfNull(solver);

            var field = typeof(HermiteSimpsonSolver).GetField(
                "_maxIterations",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            return field != null ? (int)field.GetValue(solver)! : 100;
        }

        /// <summary>
        /// Gets the inner optimizer from the solver.
        /// </summary>
        public static IOptimizer GetInnerOptimizer(this HermiteSimpsonSolver solver)
        {
            ArgumentNullException.ThrowIfNull(solver);

            var field = typeof(HermiteSimpsonSolver).GetField(
                "_innerOptimizer",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            return (IOptimizer)(field?.GetValue(solver) ?? new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));
        }
    }
}
