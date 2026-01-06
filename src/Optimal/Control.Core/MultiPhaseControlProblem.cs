/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.Control.Core
{
    /// <summary>
    /// Defines a phase in a multi-phase optimal control problem.
    /// Each phase has its own dynamics, cost, and time horizon.
    /// </summary>
    public sealed class ControlPhase
    {
        /// <summary>
        /// Gets or sets the phase name.
        /// </summary>
        public string Name { get; set; } = string.Empty;

        /// <summary>
        /// Gets or sets the phase duration (can be fixed or free).
        /// </summary>
        public double Duration { get; set; }

        /// <summary>
        /// Gets or sets whether the phase duration is fixed.
        /// </summary>
        public bool FixedDuration { get; set; } = true;

        /// <summary>
        /// Gets or sets the control problem for this phase.
        /// </summary>
        public ControlProblem? Problem { get; set; }

        /// <summary>
        /// Gets or sets the number of collocation segments for this phase.
        /// </summary>
        public int Segments { get; set; } = 20;
    }

    /// <summary>
    /// Defines linkage constraints between two phases.
    /// These constraints connect the final state of one phase to the initial state of the next.
    /// </summary>
    public sealed class PhaseLinkage
    {
        /// <summary>
        /// Gets or sets the index of the first phase (source).
        /// </summary>
        public int Phase1Index { get; set; }

        /// <summary>
        /// Gets or sets the index of the second phase (target).
        /// </summary>
        public int Phase2Index { get; set; }

        /// <summary>
        /// Gets or sets the linkage constraint function.
        /// Takes final state of phase 1 and initial state of phase 2, returns constraint value and gradients.
        /// Constraint is satisfied when value = 0.
        /// </summary>
        public Func<double[], double[], (double value, double[] gradients)>? Constraint { get; set; }

        /// <summary>
        /// Gets or sets the linkage type.
        /// </summary>
        public PhaseLinkageType Type { get; set; } = PhaseLinkageType.Equality;
    }

    /// <summary>
    /// Type of phase linkage constraint.
    /// </summary>
    public enum PhaseLinkageType
    {
        /// <summary>
        /// Equality constraint: x2(0) = x1(T1).
        /// </summary>
        Equality,

        /// <summary>
        /// Custom constraint function.
        /// </summary>
        Custom
    }

    /// <summary>
    /// Defines a multi-phase optimal control problem.
    /// Consists of multiple phases with linkage constraints between them.
    /// </summary>
    public sealed class MultiPhaseControlProblem
    {
        private readonly List<ControlPhase> _phases = new();
        private readonly List<PhaseLinkage> _linkages = new();

        /// <summary>
        /// Gets the phases in this multi-phase problem.
        /// </summary>
        public IReadOnlyList<ControlPhase> Phases => _phases;

        /// <summary>
        /// Gets the phase linkage constraints.
        /// </summary>
        public IReadOnlyList<PhaseLinkage> Linkages => _linkages;

        /// <summary>
        /// Adds a phase to the multi-phase problem.
        /// </summary>
        /// <param name="phase">The phase to add.</param>
        /// <returns>This multi-phase problem for method chaining.</returns>
        public MultiPhaseControlProblem AddPhase(ControlPhase phase)
        {
            ArgumentNullException.ThrowIfNull(phase);
            _phases.Add(phase);
            return this;
        }

        /// <summary>
        /// Adds a linkage constraint between two phases.
        /// </summary>
        /// <param name="linkage">The linkage constraint.</param>
        /// <returns>This multi-phase problem for method chaining.</returns>
        public MultiPhaseControlProblem AddLinkage(PhaseLinkage linkage)
        {
            ArgumentNullException.ThrowIfNull(linkage);

            if (linkage.Phase1Index < 0 || linkage.Phase1Index >= _phases.Count)
            {
                throw new ArgumentException("Invalid Phase1Index.", nameof(linkage));
            }

            if (linkage.Phase2Index < 0 || linkage.Phase2Index >= _phases.Count)
            {
                throw new ArgumentException("Invalid Phase2Index.", nameof(linkage));
            }

            _linkages.Add(linkage);
            return this;
        }

        /// <summary>
        /// Adds a simple continuity linkage between consecutive phases.
        /// Requires that the final state of phase i equals the initial state of phase i+1.
        /// </summary>
        /// <param name="phase1Index">Index of first phase.</param>
        /// <param name="phase2Index">Index of second phase.</param>
        /// <returns>This multi-phase problem for method chaining.</returns>
        public MultiPhaseControlProblem AddContinuityLinkage(int phase1Index, int phase2Index)
        {
            if (phase1Index < 0 || phase1Index >= _phases.Count)
            {
                throw new ArgumentException("Invalid phase1Index.", nameof(phase1Index));
            }

            if (phase2Index < 0 || phase2Index >= _phases.Count)
            {
                throw new ArgumentException("Invalid phase2Index.", nameof(phase2Index));
            }

            var linkage = new PhaseLinkage
            {
                Phase1Index = phase1Index,
                Phase2Index = phase2Index,
                Type = PhaseLinkageType.Equality,
                Constraint = (xf1, x02) =>
                {
                    // Simple equality: xf1 - x02 = 0
                    var n = Math.Min(xf1.Length, x02.Length);
                    var value = 0.0;
                    var gradients = new double[xf1.Length + x02.Length];

                    for (var i = 0; i < n; i++)
                    {
                        var diff = xf1[i] - x02[i];
                        value += diff * diff; // Sum of squared differences

                        gradients[i] = 2.0 * diff; // d/dx_f1
                        gradients[xf1.Length + i] = -2.0 * diff; // d/dx_02
                    }

                    return (value, gradients);
                }
            };

            _linkages.Add(linkage);
            return this;
        }

        /// <summary>
        /// Gets the total number of decision variables across all phases.
        /// </summary>
        /// <returns>Total decision vector size.</returns>
        public int GetTotalDecisionVectorSize()
        {
            var totalSize = 0;

            foreach (var phase in _phases)
            {
                if (phase.Problem == null)
                {
                    throw new InvalidOperationException($"Phase '{phase.Name}' has no problem defined.");
                }

                var segments = phase.Segments;
                var stateSize = phase.Problem.StateDim;
                var controlSize = phase.Problem.ControlDim;

                totalSize += (segments + 1) * (stateSize + controlSize);

                if (!phase.FixedDuration)
                {
                    totalSize += 1; // Add decision variable for phase duration
                }
            }

            return totalSize;
        }

        /// <summary>
        /// Validates the multi-phase problem.
        /// </summary>
        public void Validate()
        {
            if (_phases.Count == 0)
            {
                throw new InvalidOperationException("Multi-phase problem must have at least one phase.");
            }

            foreach (var phase in _phases)
            {
                if (phase.Problem == null)
                {
                    throw new InvalidOperationException($"Phase '{phase.Name}' has no problem defined.");
                }

                if (phase.Duration <= 0.0)
                {
                    throw new InvalidOperationException($"Phase '{phase.Name}' has invalid duration.");
                }
            }

            foreach (var linkage in _linkages)
            {
                if (linkage.Constraint == null)
                {
                    throw new InvalidOperationException("Linkage constraint function is not defined.");
                }
            }
        }
    }

    /// <summary>
    /// Result of a multi-phase optimal control problem.
    /// </summary>
    public sealed record MultiPhaseResult
    {
        /// <summary>
        /// Gets a value indicating whether the optimization succeeded.
        /// </summary>
        public bool Success { get; init; }

        /// <summary>
        /// Gets a human-readable message describing the outcome.
        /// </summary>
        public string Message { get; init; } = string.Empty;

        /// <summary>
        /// Gets the results for each phase.
        /// </summary>
        public CollocationResult[] PhaseResults { get; init; } = Array.Empty<CollocationResult>();

        /// <summary>
        /// Gets the optimal phase durations (for phases with free duration).
        /// </summary>
        public double[] PhaseDurations { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the total optimal cost across all phases.
        /// </summary>
        public double TotalCost { get; init; }

        /// <summary>
        /// Gets the maximum linkage constraint violation.
        /// </summary>
        public double MaxLinkageViolation { get; init; }

        /// <summary>
        /// Gets the total number of iterations.
        /// </summary>
        public int TotalIterations { get; init; }
    }
}
