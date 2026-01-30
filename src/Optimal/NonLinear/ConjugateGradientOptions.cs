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
    /// Options for the Conjugate Gradient optimizer.
    /// </summary>
    public record ConjugateGradientOptions : ExtendedToleranceOptions
    {
        /// <summary>
        /// The beta formula to use (Fletcher-Reeves, Polak-Ribiere, or Hestenes-Stiefel).
        /// </summary>
        public ConjugateGradientFormula Formula { get; init; } = ConjugateGradientFormula.FletcherReeves;

        /// <summary>
        /// Restart interval (0 = no periodic restart).
        /// </summary>
        public int RestartInterval { get; init; }
    }
}
