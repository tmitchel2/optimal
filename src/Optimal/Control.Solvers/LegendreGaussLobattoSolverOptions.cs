/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal.Control.Core;

namespace Optimal.Control.Solvers
{
    /// <summary>
    /// Options for the Legendre-Gauss-Lobatto collocation solver.
    /// </summary>
    public record LegendreGaussLobattoSolverOptions : CollocationSolverOptions
    {
        /// <summary>
        /// Gets the LGL collocation order (number of collocation points per segment).
        /// Higher orders provide more accurate solutions but increase computational cost.
        /// Common values: 3, 4, 5, 7, 9.
        /// </summary>
        public int Order { get; init; } = 4;

        /// <summary>
        /// Gets whether numerical gradients are forced instead of analytical gradients.
        /// Useful for debugging gradient issues.
        /// </summary>
        public bool ForceNumericalGradients { get; init; }
    }
}
