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
    /// Hand-written element stiffness K_e = Σ_q w_q det J_q Bᵀ D B in the textbook dense form: the reference the
    /// generated Jacobians are checked and timed against.
    /// </summary>
    public static class FeaReference
    {
        private const double Qa = Tet10Elastic.Qa;
        private const double Qb = Tet10Elastic.Qb;

        public static void StiffnessTet4(ReadOnlySpan<double> x, double e, double nu, Span<double> k)
        {
            Span<double> dN = stackalloc double[] { -1, -1, -1, 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            Span<double> b = stackalloc double[6 * 12];
            Span<double> d = stackalloc double[36];
            Span<double> db = stackalloc double[6 * 12];
            Elasticity(e, nu, d);
            var det = BuildB(x, dN, 4, b);
            k.Clear();
            AddBtDb(b, d, db, 12, det / 6, k);
        }

        public static void StiffnessTet10(ReadOnlySpan<double> x, double e, double nu, Span<double> k)
        {
            Span<double> dN = stackalloc double[30];
            Span<double> b = stackalloc double[6 * 30];
            Span<double> d = stackalloc double[36];
            Span<double> db = stackalloc double[6 * 30];
            Elasticity(e, nu, d);
            k.Clear();
            for (var q = 0; q < 4; q++)
            {
                var l1 = q == 1 ? Qa : Qb;
                var l2 = q == 2 ? Qa : Qb;
                var l3 = q == 3 ? Qa : Qb;
                Tet10Gradients(1 - l1 - l2 - l3, l1, l2, l3, dN);
                var det = BuildB(x, dN, 10, b);
                AddBtDb(b, d, db, 30, det / 24, k);
            }
        }

        private static void Tet10Gradients(double l0, double l1, double l2, double l3, Span<double> dN)
        {
            dN.Clear();
            dN[0] = dN[1] = dN[2] = 1 - (4 * l0);
            dN[3] = (4 * l1) - 1;
            dN[7] = (4 * l2) - 1;
            dN[11] = (4 * l3) - 1;
            Edge(dN, 4, 0, 1, l0, l1);
            Edge(dN, 5, 1, 2, l1, l2);
            Edge(dN, 6, 0, 2, l0, l2);
            Edge(dN, 7, 0, 3, l0, l3);
            Edge(dN, 8, 1, 3, l1, l3);
            Edge(dN, 9, 2, 3, l2, l3);
        }

        private static void Edge(Span<double> dN, int node, int a, int b, double la, double lb)
        {
            for (var j = 0; j < 3; j++)
            {
                dN[(3 * node) + j] = 4 * ((lb * GradL(a, j)) + (la * GradL(b, j)));
            }
        }

        private static double GradL(int corner, int j) => corner == 0 ? -1 : (corner - 1 == j ? 1 : 0);

        private static void Elasticity(double e, double nu, Span<double> d)
        {
            var lambda = e * nu / ((1 + nu) * (1 - (2 * nu)));
            var mu = e / (2 * (1 + nu));
            d.Clear();
            for (var i = 0; i < 3; i++)
            {
                for (var j = 0; j < 3; j++)
                {
                    d[(6 * i) + j] = lambda;
                }

                d[(6 * i) + i] = lambda + (2 * mu);
                d[(6 * (i + 3)) + i + 3] = mu;
            }
        }

        /// <summary>Builds B (6 × 3n, Voigt xx yy zz yz xz xy, engineering shear) and returns det J.</summary>
        private static double BuildB(ReadOnlySpan<double> x, ReadOnlySpan<double> dN, int nodes, Span<double> b)
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

            var c00 = (j[4] * j[8]) - (j[5] * j[7]);
            var c01 = (j[5] * j[6]) - (j[3] * j[8]);
            var c02 = (j[3] * j[7]) - (j[4] * j[6]);
            var det = (j[0] * c00) + (j[1] * c01) + (j[2] * c02);
            Span<double> inv = stackalloc double[]
            {
                c00 / det, ((j[2] * j[7]) - (j[1] * j[8])) / det, ((j[1] * j[5]) - (j[2] * j[4])) / det,
                c01 / det, ((j[0] * j[8]) - (j[2] * j[6])) / det, ((j[2] * j[3]) - (j[0] * j[5])) / det,
                c02 / det, ((j[1] * j[6]) - (j[0] * j[7])) / det, ((j[0] * j[4]) - (j[1] * j[3])) / det,
            };

            var cols = 3 * nodes;
            b[..(6 * cols)].Clear();
            for (var a = 0; a < nodes; a++)
            {
                var gx = (dN[3 * a] * inv[0]) + (dN[(3 * a) + 1] * inv[3]) + (dN[(3 * a) + 2] * inv[6]);
                var gy = (dN[3 * a] * inv[1]) + (dN[(3 * a) + 1] * inv[4]) + (dN[(3 * a) + 2] * inv[7]);
                var gz = (dN[3 * a] * inv[2]) + (dN[(3 * a) + 1] * inv[5]) + (dN[(3 * a) + 2] * inv[8]);
                var c = 3 * a;
                b[c] = gx;
                b[cols + c + 1] = gy;
                b[(2 * cols) + c + 2] = gz;
                b[(3 * cols) + c + 1] = gz;
                b[(3 * cols) + c + 2] = gy;
                b[(4 * cols) + c] = gz;
                b[(4 * cols) + c + 2] = gx;
                b[(5 * cols) + c] = gy;
                b[(5 * cols) + c + 1] = gx;
            }

            return det;
        }

        private static void AddBtDb(ReadOnlySpan<double> b, ReadOnlySpan<double> d, Span<double> db, int cols, double scale, Span<double> k)
        {
            for (var i = 0; i < 6; i++)
            {
                for (var c = 0; c < cols; c++)
                {
                    var sum = 0.0;
                    for (var m = 0; m < 6; m++)
                    {
                        sum += d[(6 * i) + m] * b[(m * cols) + c];
                    }

                    db[(i * cols) + c] = sum;
                }
            }

            for (var r = 0; r < cols; r++)
            {
                for (var c = 0; c < cols; c++)
                {
                    var sum = 0.0;
                    for (var m = 0; m < 6; m++)
                    {
                        sum += b[(m * cols) + r] * db[(m * cols) + c];
                    }

                    k[(r * cols) + c] += scale * sum;
                }
            }
        }
    }
}
