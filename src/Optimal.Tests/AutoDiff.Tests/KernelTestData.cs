/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>Elements, vectors and error measures shared by the array-kernel tests.</summary>
    public static class KernelTestData
    {
        public const double E = 200e3;
        public const double Nu = 0.3;

        /// <summary>A residual R(u, X, E, ν) with the signature of the element kernels.</summary>
        public delegate void Residual(ReadOnlySpan<double> u, ReadOnlySpan<double> x, double e, double nu, Span<double> r);

        /// <summary>A hand-written element stiffness K_e(X, E, ν).</summary>
        public delegate void Stiffness(ReadOnlySpan<double> x, double e, double nu, Span<double> k);

        /// <summary>A generated Jacobian with the signature of the element kernels.</summary>
        public delegate void Jacobian(ReadOnlySpan<double> u, ReadOnlySpan<double> x, double e, double nu, Span<double> j);

        /// <summary>A generated vector–Jacobian product with the signature of the element kernels.</summary>
        public delegate void Vjp(ReadOnlySpan<double> u, ReadOnlySpan<double> x, double e, double nu, ReadOnlySpan<double> lambda, Span<double> g);

        /// <summary>A distorted, positively oriented Tet4.</summary>
        public static double[] Tet4Nodes() => [0, 0, 0, 1.1, 0.1, -0.05, 0.2, 0.9, 0.1, 0.1, 0.15, 1.2];

        /// <summary>The Tet4 with its edge nodes near, not at, the edge midpoints, so the map is curved.</summary>
        public static double[] Tet10Nodes()
        {
            var corners = Tet4Nodes();
            int[,] edges = { { 0, 1 }, { 1, 2 }, { 0, 2 }, { 0, 3 }, { 1, 3 }, { 2, 3 } };
            var random = new Random(7);
            var x = new double[30];
            Array.Copy(corners, x, 12);
            for (var e = 0; e < 6; e++)
            {
                for (var i = 0; i < 3; i++)
                {
                    x[(3 * (4 + e)) + i] = (0.5 * (corners[(3 * edges[e, 0]) + i] + corners[(3 * edges[e, 1]) + i])) + (0.04 * (random.NextDouble() - 0.5));
                }
            }

            return x;
        }

        public static double[] Nodes(int n) => n == 12 ? Tet4Nodes() : Tet10Nodes();

        public static double[] RandomVector(int n, int seed)
        {
            var random = new Random(seed);
            return Enumerable.Range(0, n).Select(_ => random.NextDouble() - 0.5).ToArray();
        }

        /// <summary>max |actual − expected| over max |expected|.</summary>
        public static double MaxRelative(double[] actual, double[] expected)
        {
            var scale = expected.Max(Math.Abs);
            var difference = actual.Zip(expected, (a, e) => Math.Abs(a - e)).Max();
            return difference / scale;
        }

        /// <summary>Jᵀλ for a row-major n × m Jacobian.</summary>
        public static double[] TransposeTimes(double[] jacobian, double[] lambda, int columns)
        {
            return Enumerable.Range(0, columns)
                .Select(k => Enumerable.Range(0, lambda.Length).Sum(i => lambda[i] * jacobian[(i * columns) + k]))
                .ToArray();
        }

        /// <summary>A central-difference Jacobian of <paramref name="residual"/> with respect to u or X.</summary>
        public static double[] CentralDifferenceJacobian(Residual residual, double[] u, double[] x, bool wrtX, double h)
        {
            var n = u.Length;
            var j = new double[n * n];
            var rp = new double[n];
            var rm = new double[n];
            for (var k = 0; k < n; k++)
            {
                var (up, um, xp, xm) = ((double[])u.Clone(), (double[])u.Clone(), (double[])x.Clone(), (double[])x.Clone());
                (wrtX ? xp : up)[k] += h;
                (wrtX ? xm : um)[k] -= h;
                residual(up, xp, E, Nu, rp);
                residual(um, xm, E, Nu, rm);
                for (var i = 0; i < n; i++)
                {
                    j[(i * n) + k] = (rp[i] - rm[i]) / (2 * h);
                }
            }

            return j;
        }

        /// <summary>
        /// The Taylor remainders |f(X + hd) − f(X) − h ∇f·d| of f = λᵀR at halving h, and the slopes between them,
        /// which are 2 when the gradient is right.
        /// </summary>
        public static (double[] Remainders, double[] Slopes) TaylorTest(Residual residual, Vjp vjp, double[] u, double[] x, int seed)
        {
            var n = u.Length;
            var lambda = RandomVector(n, seed);
            var direction = RandomVector(n, seed + 1);
            var gradient = new double[n];
            vjp(u, x, E, Nu, lambda, gradient);
            double F(double h)
            {
                var r = new double[n];
                residual(u, x.Zip(direction, (xi, di) => xi + (h * di)).ToArray(), E, Nu, r);
                return lambda.Zip(r, (l, ri) => l * ri).Sum();
            }

            var f0 = F(0);
            var slope = gradient.Zip(direction, (g, d) => g * d).Sum();
            var steps = new[] { 1e-2, 5e-3, 2.5e-3, 1.25e-3 };
            var remainders = steps.Select(h => Math.Abs(F(h) - f0 - (h * slope))).ToArray();
            var slopes = Enumerable.Range(0, steps.Length - 1)
                .Select(i => Math.Log(remainders[i] / remainders[i + 1]) / Math.Log(steps[i] / steps[i + 1]))
                .ToArray();
            return (remainders, slopes);
        }

        /// <summary>The bytes the calling thread allocates over 10,000 calls of <paramref name="call"/>, after 1,000 to warm up.</summary>
        public static long AllocatedBytes(Action call)
        {
            for (var i = 0; i < 1000; i++)
            {
                call();
            }

            var before = GC.GetAllocatedBytesForCurrentThread();
            for (var i = 0; i < 10_000; i++)
            {
                call();
            }

            return GC.GetAllocatedBytesForCurrentThread() - before;
        }
    }
}
