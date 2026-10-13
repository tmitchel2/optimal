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
    /// The compressible neo-Hookean Tet4 residual, r_ai = V Σ_k P_ik ∂N_a/∂X_k with the first Piola–Kirchhoff stress
    /// P = μ(F − F⁻ᵀ) + λ ln(J) F⁻ᵀ and F = I + ∂u/∂X: a nonlinear kernel, so its tangent depends on u.
    /// </summary>
    [OptimalCode]
    public static class Tet4NeoHookean
    {
        private static ReadOnlySpan<double> ReferenceGradients => [-1, -1, -1, 1, 0, 0, 0, 1, 0, 0, 0, 1];

        [Jacobian(nameof(u))]
        [Jacobian(nameof(X))]
        [Vjp(nameof(X))]
        public static void Residual(
            [FixedLength(12)] ReadOnlySpan<double> u,
            [FixedLength(12)] ReadOnlySpan<double> X,
            double E,
            double nu,
            [FixedLength(12)] Span<double> r)
        {
            Span<double> inv = stackalloc double[9];
            Span<double> g = stackalloc double[12];
            Span<double> f = stackalloc double[9];
            Span<double> fInv = stackalloc double[9];
            var det = ElementGeometry.InverseJacobian(X, ReferenceGradients, 4, inv);
            ElementGeometry.PhysicalGradients(ReferenceGradients, inv, g);

            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    var sum = i == k ? 1.0 : 0.0;
                    for (var a = 0; a < 4; a++)
                    {
                        sum += u[(3 * a) + i] * g[(3 * a) + k];
                    }

                    f[(3 * i) + k] = sum;
                }
            }

            var jacobian = ElementGeometry.Inverse(f, fInv);
            var lambda = ElementGeometry.Lame(E, nu);
            var mu = ElementGeometry.Shear(E, nu);
            var logJ = Math.Log(jacobian);
            for (var a = 0; a < 4; a++)
            {
                for (var i = 0; i < 3; i++)
                {
                    var sum = 0.0;
                    for (var k = 0; k < 3; k++)
                    {
                        var p = (mu * (f[(3 * i) + k] - fInv[(3 * k) + i])) + (lambda * logJ * fInv[(3 * k) + i]);
                        sum += p * g[(3 * a) + k];
                    }

                    r[(3 * a) + i] = det / 6 * sum;
                }
            }
        }
    }
}
