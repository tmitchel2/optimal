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
    /// The linear-elastic Tet4 residual R(u, X, E, ν) = K_e(X, E, ν)·u, written plainly: loops, stackalloc locals and an
    /// inline 3 × 3 inverse, with no helpers.
    /// </summary>
    [OptimalCode]
    public static class Tet4Elastic
    {
        [Jacobian(nameof(u))]
        [Jacobian(nameof(X))]
        [Vjp(nameof(X))]
        [Vjp(nameof(X), nameof(E), nameof(nu))]
        public static void Residual(
            [FixedLength(12)] ReadOnlySpan<double> u,
            [FixedLength(12)] ReadOnlySpan<double> X,
            double E,
            double nu,
            [FixedLength(12)] Span<double> r)
        {
            // Reference gradients of N0 = 1 - ξ - η - ζ, N1 = ξ, N2 = η, N3 = ζ.
            Span<double> dN = stackalloc double[] { -1, -1, -1, 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            Span<double> j = stackalloc double[9];
            Span<double> inv = stackalloc double[9];
            Span<double> g = stackalloc double[12];
            Span<double> h = stackalloc double[9];
            Span<double> s = stackalloc double[9];

            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    var sum = 0.0;
                    for (var a = 0; a < 4; a++)
                    {
                        sum += X[(3 * a) + i] * dN[(3 * a) + k];
                    }

                    j[(3 * i) + k] = sum;
                }
            }

            var det = (j[0] * ((j[4] * j[8]) - (j[5] * j[7])))
                - (j[1] * ((j[3] * j[8]) - (j[5] * j[6])))
                + (j[2] * ((j[3] * j[7]) - (j[4] * j[6])));
            inv[0] = ((j[4] * j[8]) - (j[5] * j[7])) / det;
            inv[1] = ((j[2] * j[7]) - (j[1] * j[8])) / det;
            inv[2] = ((j[1] * j[5]) - (j[2] * j[4])) / det;
            inv[3] = ((j[5] * j[6]) - (j[3] * j[8])) / det;
            inv[4] = ((j[0] * j[8]) - (j[2] * j[6])) / det;
            inv[5] = ((j[2] * j[3]) - (j[0] * j[5])) / det;
            inv[6] = ((j[3] * j[7]) - (j[4] * j[6])) / det;
            inv[7] = ((j[1] * j[6]) - (j[0] * j[7])) / det;
            inv[8] = ((j[0] * j[4]) - (j[1] * j[3])) / det;

            // Physical gradients: ∂N_a/∂x_i = Σ_k ∂N_a/∂ξ_k (J⁻¹)_ki.
            for (var a = 0; a < 4; a++)
            {
                for (var i = 0; i < 3; i++)
                {
                    var sum = 0.0;
                    for (var k = 0; k < 3; k++)
                    {
                        sum += dN[(3 * a) + k] * inv[(3 * k) + i];
                    }

                    g[(3 * a) + i] = sum;
                }
            }

            // Displacement gradient H_ik = Σ_a u_ai ∂N_a/∂x_k.
            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    var sum = 0.0;
                    for (var a = 0; a < 4; a++)
                    {
                        sum += u[(3 * a) + i] * g[(3 * a) + k];
                    }

                    h[(3 * i) + k] = sum;
                }
            }

            var lambda = E * nu / ((1 + nu) * (1 - (2 * nu)));
            var mu = E / (2 * (1 + nu));
            var trace = h[0] + h[4] + h[8];
            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    s[(3 * i) + k] = (mu * (h[(3 * i) + k] + h[(3 * k) + i])) + (i == k ? lambda * trace : 0.0);
                }
            }

            var volume = det / 6;
            for (var a = 0; a < 4; a++)
            {
                for (var i = 0; i < 3; i++)
                {
                    var sum = 0.0;
                    for (var k = 0; k < 3; k++)
                    {
                        sum += s[(3 * i) + k] * g[(3 * a) + k];
                    }

                    r[(3 * a) + i] = volume * sum;
                }
            }
        }
    }
}
