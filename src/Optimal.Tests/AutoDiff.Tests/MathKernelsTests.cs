/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>Every supported Math function's derivative rule, in forward and reverse mode, against central differences.</summary>
    [TestClass]
    public class MathKernelsTests
    {
        private const int N = MathKernels.Outputs;

        // Away from every kink: x0 > 0 for Sqrt, Log and Pow; x1 < 0 so Cbrt, Abs and Sign see a negative argument.
        private static readonly double[] Point = [0.7, -0.4, 0.3];

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void JacobianMatchesCentralDifferencesForEveryFunction()
        {
            var generated = new double[N * 3];
            MathKernelsKernels.FunctionsJacobian_x(Point, generated);
            var fd = CentralDifferences(Point);
            var worst = Enumerable.Range(0, N)
                .Select(i => (Row: i, Error: Enumerable.Range(0, 3).Max(k => Math.Abs(generated[(3 * i) + k] - fd[(3 * i) + k]) / Math.Max(1, Math.Abs(fd[(3 * i) + k])))))
                .OrderByDescending(e => e.Error)
                .First();
            TestContext.WriteLine($"Worst row {worst.Row}: error {worst.Error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, worst.Error, $"Output {worst.Row} differs from central differences.");
        }

        [TestMethod]
        public void VjpEqualsForwardModeTransposeLambda()
        {
            var lambda = RandomVector(N, 12);
            var jacobian = new double[N * 3];
            var gradient = new double[3];
            MathKernelsKernels.FunctionsJacobian_x(Point, jacobian);
            MathKernelsKernels.FunctionsVjp_x(Point, lambda, gradient);
            var error = MaxRelative(gradient, TransposeTimes(jacobian, lambda, 3));
            TestContext.WriteLine($"Math VJP vs J^T lambda: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
        }

        [TestMethod]
        public void SelectFollowsTheBranchTaken()
        {
            var jacobian = new double[N * 3];
            MathKernelsKernels.FunctionsJacobian_x([0.2, 0.5, 0.3], jacobian);
            CollectionAssert.AreEqual(new[] { 0.3, 0, 0.2 }, jacobian.Skip(3 * 20).Take(3).ToArray());
        }

        [TestMethod]
        public void KinksTakeTheDocumentedSubgradient()
        {
            var jacobian = new double[6];
            MathKernelsKernels.KinksJacobian_x([0.0, 0.0], jacobian);
            CollectionAssert.AreEqual(new[] { 0.0, 0, 1, 0, 1, 0 }, jacobian);
        }

        [TestMethod]
        public void SmoothHelpersStayWithinTheirWidth()
        {
            var y = new double[N];
            MathKernels.Functions(Point, y);
            Assert.AreEqual(Math.Min(Point[0], Point[1]), y[17], 0.05);
            Assert.AreEqual(Math.Max(Point[0], Point[1]), y[18], 0.05);
            Assert.AreEqual(Math.Abs(Point[1]), y[19], 0.1);
        }

        private static double[] CentralDifferences(double[] x)
        {
            const double h = 1e-6;
            var fd = new double[N * 3];
            var yp = new double[N];
            var ym = new double[N];
            for (var k = 0; k < 3; k++)
            {
                var (xp, xm) = ((double[])x.Clone(), (double[])x.Clone());
                xp[k] += h;
                xm[k] -= h;
                MathKernels.Functions(xp, yp);
                MathKernels.Functions(xm, ym);
                for (var i = 0; i < N; i++)
                {
                    fd[(3 * i) + k] = (yp[i] - ym[i]) / (2 * h);
                }
            }

            return fd;
        }
    }
}
