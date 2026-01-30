/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear
{
    /// <summary>
    /// Base options shared by all optimizers.
    /// </summary>
    public record OptimizerOptions
    {
        /// <summary>
        /// Gradient norm tolerance for convergence.
        /// </summary>
        public double Tolerance { get; init; } = 1e-6;

        /// <summary>
        /// Maximum number of iterations.
        /// </summary>
        public int MaxIterations { get; init; } = 1000;

        /// <summary>
        /// Enables verbose console output.
        /// </summary>
        public bool Verbose { get; init; }
    }
}
