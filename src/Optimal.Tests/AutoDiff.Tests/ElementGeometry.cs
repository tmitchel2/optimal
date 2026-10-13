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
    /// Helpers that array kernels inline: the isoparametric map's Jacobian and its inverse, physical shape-function
    /// gradients, and small-strain linear elasticity. Spans are passed by reference, so a helper writes its caller's
    /// arrays.
    /// </summary>
    [OptimalCode]
    public static class ElementGeometry
    {
        /// <summary>J_ik = Σ_a X_ai ∂N_a/∂ξ_k over <paramref name="nodes"/> nodes; returns det J and writes J⁻¹.</summary>
        public static double InverseJacobian(ReadOnlySpan<double> x, ReadOnlySpan<double> dN, int nodes, Span<double> inverse)
        {
            Span<double> j = stackalloc double[9];
            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    var sum = 0.0;
                    for (var a = 0; a < nodes; a++)
                    {
                        sum += x[(3 * a) + i] * dN[(3 * a) + k];
                    }

                    j[(3 * i) + k] = sum;
                }
            }

            return Inverse(j, inverse);
        }

        /// <summary>Writes the inverse of the 3 × 3 matrix <paramref name="m"/> and returns its determinant.</summary>
        public static double Inverse(ReadOnlySpan<double> m, Span<double> inverse)
        {
            var det = Determinant(m);
            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    // (M⁻¹)_ik is the cofactor of M_ki over det M, with indices taken cyclically.
                    var r1 = (k + 1) % 3;
                    var r2 = (k + 2) % 3;
                    var c1 = (i + 1) % 3;
                    var c2 = (i + 2) % 3;
                    inverse[(3 * i) + k] = ((m[(3 * r1) + c1] * m[(3 * r2) + c2]) - (m[(3 * r1) + c2] * m[(3 * r2) + c1])) / det;
                }
            }

            return det;
        }

        public static double Determinant(ReadOnlySpan<double> j)
        {
            return (j[0] * ((j[4] * j[8]) - (j[5] * j[7])))
                - (j[1] * ((j[3] * j[8]) - (j[5] * j[6])))
                + (j[2] * ((j[3] * j[7]) - (j[4] * j[6])));
        }

        /// <summary>∂N_a/∂x_i = Σ_k ∂N_a/∂ξ_k (J⁻¹)_ki for every node of <paramref name="dN"/>.</summary>
        public static void PhysicalGradients(ReadOnlySpan<double> dN, ReadOnlySpan<double> inverse, Span<double> g)
        {
            for (var a = 0; a < dN.Length / 3; a++)
            {
                for (var i = 0; i < 3; i++)
                {
                    var sum = 0.0;
                    for (var k = 0; k < 3; k++)
                    {
                        sum += dN[(3 * a) + k] * inverse[(3 * k) + i];
                    }

                    g[(3 * a) + i] = sum;
                }
            }
        }

        /// <summary>The small-strain stress σ = λ tr(ε) I + 2με of the displacement gradient h.</summary>
        public static void LinearStress(ReadOnlySpan<double> h, double lambda, double mu, Span<double> s)
        {
            var trace = h[0] + h[4] + h[8];
            for (var i = 0; i < 3; i++)
            {
                for (var k = 0; k < 3; k++)
                {
                    s[(3 * i) + k] = (mu * (h[(3 * i) + k] + h[(3 * k) + i])) + (i == k ? lambda * trace : 0.0);
                }
            }
        }

        public static double Lame(double e, double nu) => e * nu / ((1 + nu) * (1 - (2 * nu)));

        public static double Shear(double e, double nu) => e / (2 * (1 + nu));
    }
}
