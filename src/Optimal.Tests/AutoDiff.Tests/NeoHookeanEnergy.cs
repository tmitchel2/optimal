/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// The compressible neo-Hookean strain-energy density W(F) = μ/2 (F : F − 3) − μ ln J + λ/2 (ln J)², J = det F: its
    /// gradient is the first Piola–Kirchhoff stress and its Hessian the material tangent ∂P/∂F, both generated.
    /// </summary>
    [OptimalCode]
    public static class NeoHookeanEnergy
    {
        [Vjp(nameof(F))]
        [Hessian(nameof(F))]
        [Hvp(nameof(F))]
        public static void Density([FixedLength(9)] ReadOnlySpan<double> F, double E, double nu, [FixedLength(1)] Span<double> w)
        {
            var f = Tensor3.From(F);
            var mu = E / (2 * (1 + nu));
            var lambda = E * nu / ((1 + nu) * (1 - (2 * nu)));
            var logJ = Math.Log(f.Determinant);
            w[0] = (0.5 * mu * (f.DoubleDot(f) - 3)) - (mu * logJ) + (0.5 * lambda * logJ * logJ);
        }
    }
}
