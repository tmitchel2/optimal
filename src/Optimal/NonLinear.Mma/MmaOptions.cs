/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// Options for <see cref="MmaOptimizer"/>.
    /// </summary>
    public record MmaOptions : MovingAsymptotesOptions
    {
        /// <summary>
        /// Gets Svanberg's raa0: the term raa0/(upper_j - lower_j) added to every p_ij and q_ij so that the subproblem
        /// is strictly convex. For a sparse constraint row it is added on the row's pattern only.
        /// </summary>
        public double Regularization { get; init; } = 1e-5;
    }
}
