/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear.Unconstrained
{
    /// <summary>
    /// Options for the L-BFGS optimizer.
    /// </summary>
    public record LBFGSOptions : ExtendedToleranceOptions
    {
        /// <summary>
        /// Number of correction pairs to store (typically 3-20).
        /// </summary>
        public int MemorySize { get; init; } = 10;
    }
}
