/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Core;
using Optimal.Control.Scaling;

namespace Optimal.Control.Solvers
{
    /// <summary>
    /// Options for the Hermite-Simpson collocation solver.
    /// </summary>
    public record HermiteSimpsonSolverOptions : CollocationSolverOptions
    {
        /// <summary>
        /// Gets the initial penalty parameter for the augmented Lagrangian optimizer.
        /// Higher values enforce constraints more strongly from the start.
        /// </summary>
        public double InitialPenalty { get; init; } = 1.0;

        /// <summary>
        /// Gets whether automatic variable scaling is enabled.
        /// When enabled, scales all state and control variables to [-1, 1] for improved conditioning.
        /// </summary>
        public bool AutoScaling { get; init; }

        /// <summary>
        /// Gets the custom variable scaling parameters.
        /// If set, overrides auto-scaling.
        /// </summary>
        public VariableScaling? Scaling { get; init; }

        /// <summary>
        /// Gets the optional callback invoked after each inner Augmented Lagrangian iteration.
        /// Fires once per AL outer iteration (less frequent than ProgressCallback).
        /// Parameters: (outerIteration, objectiveValue, maxViolation, penaltyParameter, innerIterations, currentPoint).
        /// </summary>
        public Action<int, double, double, double, int, double[]>? InnerProgressCallback { get; init; }
    }
}
