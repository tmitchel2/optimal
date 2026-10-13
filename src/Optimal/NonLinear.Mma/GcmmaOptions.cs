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
    /// Options for <see cref="GcmmaOptimizer"/>.
    /// </summary>
    public record GcmmaOptions : MovingAsymptotesOptions
    {
        /// <summary>
        /// Gets the cap on inner iterations per outer iteration (Svanberg: 15). An outer iteration that reaches it
        /// accepts a point whose approximations were not shown conservative; the result counts such iterations.
        /// </summary>
        public int MaxInnerIterations { get; init; } = 15;

        /// <summary>
        /// Gets the smallest ρ_0 and ρ_i, the curvature terms that the inner iterations raise until the approximations
        /// are conservative (Svanberg's raa0eps and raaeps).
        /// </summary>
        public double MinimumRho { get; init; } = 1e-6;

        /// <summary>
        /// Gets the slack in the conservativeness test f_i(x̂) &lt;= f̃_i(x̂) + ConservativeTolerance (Svanberg: epsimin).
        /// </summary>
        public double ConservativeTolerance { get; init; } = 1e-7;
    }
}
