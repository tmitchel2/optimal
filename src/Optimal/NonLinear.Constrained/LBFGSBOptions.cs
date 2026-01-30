/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Constrained
{
    /// <summary>
    /// Options for the L-BFGS-B optimizer (bounded optimization).
    /// </summary>
    public record LBFGSBOptions : ExtendedToleranceOptions
    {
        /// <summary>
        /// Number of correction pairs to store (typically 3-20).
        /// </summary>
        public int MemorySize { get; init; } = 10;

        /// <summary>
        /// Lower bounds for each variable (use double.NegativeInfinity for unbounded).
        /// </summary>
        public double[] LowerBounds { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Upper bounds for each variable (use double.PositiveInfinity for unbounded).
        /// </summary>
        public double[] UpperBounds { get; init; } = Array.Empty<double>();
    }
}
