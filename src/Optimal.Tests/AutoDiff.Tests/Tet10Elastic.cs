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
    /// The linear-elastic Tet10 residual with the 4-point rule and isoparametric quadratic shape functions, written as an
    /// element library would be: static tables for the edge connectivity and the quadrature points, and helpers from
    /// <see cref="ElementGeometry"/>. Corners 0–3, then edges (0,1), (1,2), (0,2), (0,3), (1,3), (2,3).
    /// </summary>
    [OptimalCode]
    public static class Tet10Elastic
    {
        public const double Qa = 0.5854101966249685;
        public const double Qb = 0.1381966011250105;

        private static readonly int[,] Edges = { { 0, 1 }, { 1, 2 }, { 0, 2 }, { 0, 3 }, { 1, 3 }, { 2, 3 } };

        // Barycentric coordinates (L1, L2, L3) of each quadrature point; L0 = 1 − L1 − L2 − L3.
        private static readonly double[,] Points = { { Qb, Qb, Qb }, { Qa, Qb, Qb }, { Qb, Qa, Qb }, { Qb, Qb, Qa } };

        [Jacobian(nameof(u))]
        [Jacobian(nameof(X))]
        [Vjp(nameof(X))]
        [Vjp(nameof(X), nameof(E), nameof(nu))]
        public static void Residual(
            [FixedLength(30)] ReadOnlySpan<double> u,
            [FixedLength(30)] ReadOnlySpan<double> X,
            double E,
            double nu,
            [FixedLength(30)] Span<double> r)
        {
            const double weight = 1.0 / 24.0;
            Span<double> l = stackalloc double[4];
            Span<double> dN = stackalloc double[30];
            Span<double> inv = stackalloc double[9];
            Span<double> g = stackalloc double[30];
            Span<double> h = stackalloc double[9];
            Span<double> s = stackalloc double[9];
            var lambda = ElementGeometry.Lame(E, nu);
            var mu = ElementGeometry.Shear(E, nu);
            r.Clear();

            for (var q = 0; q < 4; q++)
            {
                l[1] = Points[q, 0];
                l[2] = Points[q, 1];
                l[3] = Points[q, 2];
                l[0] = 1 - l[1] - l[2] - l[3];
                ShapeGradients(l, dN);
                var det = ElementGeometry.InverseJacobian(X, dN, 10, inv);
                ElementGeometry.PhysicalGradients(dN, inv, g);

                for (var i = 0; i < 3; i++)
                {
                    for (var k = 0; k < 3; k++)
                    {
                        var sum = 0.0;
                        for (var a = 0; a < 10; a++)
                        {
                            sum += u[(3 * a) + i] * g[(3 * a) + k];
                        }

                        h[(3 * i) + k] = sum;
                    }
                }

                ElementGeometry.LinearStress(h, lambda, mu, s);
                var scale = weight * det;
                for (var a = 0; a < 10; a++)
                {
                    for (var i = 0; i < 3; i++)
                    {
                        var sum = 0.0;
                        for (var k = 0; k < 3; k++)
                        {
                            sum += s[(3 * i) + k] * g[(3 * a) + k];
                        }

                        r[(3 * a) + i] += scale * sum;
                    }
                }
            }
        }

        /// <summary>
        /// ∂N_a/∂ξ_j at barycentric coordinates <paramref name="l"/>: a corner's N = L(2L − 1), so ∂N/∂ξ = (4L − 1)∇L;
        /// an edge's N = 4 L_a L_b, so ∂N/∂ξ = 4 (L_b ∇L_a + L_a ∇L_b), with ∇L0 = (−1, −1, −1) and ∇L_c = e_c.
        /// </summary>
        private static void ShapeGradients(ReadOnlySpan<double> l, Span<double> dN)
        {
            for (var c = 0; c < 4; c++)
            {
                for (var j = 0; j < 3; j++)
                {
                    dN[(3 * c) + j] = ((4 * l[c]) - 1) * GradL(c, j);
                }
            }

            for (var e = 0; e < 6; e++)
            {
                var a = Edges[e, 0];
                var b = Edges[e, 1];
                for (var j = 0; j < 3; j++)
                {
                    dN[(3 * (4 + e)) + j] = 4 * ((l[b] * GradL(a, j)) + (l[a] * GradL(b, j)));
                }
            }
        }

        private static int GradL(int corner, int j) => corner == 0 ? -1 : (corner - 1 == j ? 1 : 0);
    }
}
