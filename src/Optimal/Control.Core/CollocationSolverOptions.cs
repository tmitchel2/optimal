/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.Control.Core
{
    /// <summary>
    /// Options for collocation-based optimal control solvers.
    /// </summary>
    public record CollocationSolverOptions : SolverOptions
    {
        /// <summary>
        /// Gets whether adaptive mesh refinement is enabled.
        /// </summary>
        public bool EnableMeshRefinement { get; init; }

        /// <summary>
        /// Gets the maximum number of mesh refinement iterations.
        /// </summary>
        public int MaxRefinementIterations { get; init; } = 5;

        /// <summary>
        /// Gets the defect threshold for mesh refinement convergence.
        /// </summary>
        public double RefinementDefectThreshold { get; init; } = 1e-4;
    }
}
