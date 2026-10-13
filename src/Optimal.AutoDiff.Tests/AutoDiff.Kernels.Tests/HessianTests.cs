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

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>
    /// <c>[Hessian]</c> and <c>[Hvp]</c> on a scalar kernel: the Hessian against the one worked by hand, exactly
    /// symmetric, chunked without changing a bit, and the Hessian–vector product against H v.
    /// </summary>
    [TestClass]
    public class HessianTests
    {
        private const string Source = """
            using System;
            using Optimal;

            namespace Probes
            {
                [OptimalCode]
                public static class Energies
                {
                    // W = x₀²x₁ + sin(x₁x₂) + eˣ⁰ x₂, with a branch that is taken at the test point.
                    [Hessian(nameof(x))]
                    [Hessian(nameof(s))]
                    [Hvp(nameof(x))]
                    public static void W([FixedLength(3)] ReadOnlySpan<double> x, double s, [FixedLength(1)] Span<double> w)
                    {
                        var value = (x[0] * x[0] * x[1]) + Math.Sin(x[1] * x[2]) + (Math.Exp(x[0]) * x[2]);
                        if (x[0] < -10)
                        {
                            value = 0;
                        }

                        w[0] = value * s * s;
                    }

                    [Hessian(nameof(x), ChunkSize = 2)]
                    public static void Chunked([FixedLength(3)] ReadOnlySpan<double> x, double s, [FixedLength(1)] Span<double> w)
                    {
                        W(x, s, w);
                    }
                }

                public static class Run
                {
                    public static void Hessian(double[] x, double s, double[] h) => EnergiesKernels.WHessian_x(x, s, h);

                    public static void ScalarHessian(double[] x, double s, double[] h) => EnergiesKernels.WHessian_s(x, s, h);

                    public static void ChunkedHessian(double[] x, double s, double[] h) => EnergiesKernels.ChunkedHessian_x(x, s, h);

                    public static void Hvp(double[] x, double s, double[] v, double[] hv) => EnergiesKernels.WHvp_x(x, s, v, hv);
                }
            }
            """;

        private static readonly double[] Point = [0.3, -0.7, 1.1];

        [TestMethod]
        public void HessianIsTheOneWorkedByHand()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var h = new double[9];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Hessian", Point, 1.0, h);
            var expected = Expected(Point);
            for (var k = 0; k < 9; k++)
            {
                Assert.AreEqual(expected[k], h[k], 1e-14 * Math.Max(1, Math.Abs(expected[k])), $"element {k}");
            }

            Assert.IsTrue(Enumerable.Range(0, 9).All(k => h[k] == h[(3 * (k % 3)) + (k / 3)]), "exactly symmetric");
        }

        [TestMethod]
        public void HessianOfAScalarInputIsOneByOne()
        {
            // W s² has ∂²/∂s² = 2W.
            var assembly = GeneratorHarness.Compile(Source);
            var h = new double[1];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "ScalarHessian", Point, 0.5, h);
            var (x0, x1, x2) = (Point[0], Point[1], Point[2]);
            var w = (x0 * x0 * x1) + Math.Sin(x1 * x2) + (Math.Exp(x0) * x2);
            Assert.AreEqual(2 * w, h[0], 1e-14);
        }

        [TestMethod]
        public void ChunkingDoesNotChangeAnyBit()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var (h, chunked) = (new double[9], new double[9]);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Hessian", Point, 1.3, h);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "ChunkedHessian", Point, 1.3, chunked);
            CollectionAssert.AreEqual(h, chunked);
            Assert.Contains("ChunkedHessian_xChunk1(", GeneratorHarness.Run(Source).Sources["Probes.EnergiesKernels.g.cs"]);
        }

        [TestMethod]
        public void HvpIsTheHessianTimesTheDirection()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var v = new[] { 0.25, -1.5, 2.0 };
            var (h, hv) = (new double[9], new double[3]);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Hessian", Point, 1.3, h);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Hvp", Point, 1.3, v, hv);
            for (var i = 0; i < 3; i++)
            {
                Assert.AreEqual(Enumerable.Range(0, 3).Sum(j => h[(3 * i) + j] * v[j]), hv[i], 1e-13);
            }
        }

        // H₀₀ = 2x₁ + eˣ⁰x₂, H₀₁ = 2x₀, H₀₂ = eˣ⁰, H₁₁ = −x₂² sin(x₁x₂), H₁₂ = cos(x₁x₂) − x₁x₂ sin(x₁x₂), H₂₂ = −x₁² sin(x₁x₂).
        private static double[] Expected(double[] x)
        {
            var (x0, x1, x2) = (x[0], x[1], x[2]);
            var (sin, cos, exp) = (Math.Sin(x1 * x2), Math.Cos(x1 * x2), Math.Exp(x0));
            var h01 = 2 * x0;
            var h12 = cos - (x1 * x2 * sin);
            return [(2 * x1) + (exp * x2), h01, exp, h01, -x2 * x2 * sin, h12, exp, h12, -x1 * x1 * sin];
        }
    }
}
