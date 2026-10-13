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
    /// <c>KernelMath.ImplicitSolve</c> unrolled by the generator: a system that needs a pivot chosen at run time, and the
    /// largest system, 16 unknowns, whose derivative is the inverse of its matrix.
    /// </summary>
    [TestClass]
    public class ImplicitSolverTests
    {
        private const string Source = """
            using System;
            using Optimal;

            namespace Probes
            {
                [OptimalCode]
                public static class Implicit
                {
                    // (p₂ y₀ + y₁, y₀ + p₂ y₁) = (p₀, p₁): at p₂ = 0 the first pivot is zero.
                    [Jacobian(nameof(p))]
                    [Vjp(nameof(p))]
                    public static void Swap([FixedLength(3)] ReadOnlySpan<double> p, [FixedLength(2)] Span<double> y)
                    {
                        Span<double> unknowns = stackalloc double[2];
                        KernelMath.ImplicitSolve(SwapResidual, p, unknowns, 2);
                        unknowns.CopyTo(y);
                    }

                    // A tridiagonal system (4 + p₁₆) yᵢ − yᵢ₋₁ − yᵢ₊₁ = pᵢ of 16 unknowns.
                    [Jacobian(nameof(p))]
                    public static void Tridiagonal([FixedLength(17)] ReadOnlySpan<double> p, [FixedLength(16)] Span<double> y)
                    {
                        y.Clear();
                        KernelMath.ImplicitSolve(TridiagonalResidual, p, y, 2);
                    }

                    private static void SwapResidual(ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)
                    {
                        g[0] = (p[2] * y[0]) + y[1] - p[0];
                        g[1] = y[0] + (p[2] * y[1]) - p[1];
                    }

                    private static void TridiagonalResidual(ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)
                    {
                        for (var i = 0; i < 16; i++)
                        {
                            var below = i > 0 ? y[i - 1] : 0.0;
                            var above = i < 15 ? y[i + 1] : 0.0;
                            g[i] = ((4 + p[16]) * y[i]) - below - above - p[i];
                        }
                    }
                }

                public static class Run
                {
                    public static void Swap(double[] p, double[] y) => Implicit.Swap(p, y);

                    public static void SwapJacobian(double[] p, double[] j) => ImplicitKernels.SwapJacobian_p(p, j);

                    public static void SwapVjp(double[] p, double[] lambda, double[] g) => ImplicitKernels.SwapVjp_p(p, lambda, g);

                    public static void TridiagonalJacobian(double[] p, double[] j) => ImplicitKernels.TridiagonalJacobian_p(p, j);
                }
            }
            """;

        [TestMethod]
        public void ZeroPivotIsSwappedAtRunTime()
        {
            // y = (p₁, p₀), so ∂y/∂(p₀, p₁) is the swap, and ∂y/∂p₂ = −A⁻¹ y = −(y₁, y₀) = −(p₀, p₁).
            var assembly = GeneratorHarness.Compile(Source);
            var p = new[] { 0.7, -0.4, 0.0 };
            var y = new double[2];
            var j = new double[6];
            var g = new double[3];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Swap", p, y);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "SwapJacobian", p, j);
            var lambda = new[] { 1.0, 2.0 };
            GeneratorHarness.Invoke(assembly, "Probes.Run", "SwapVjp", p, lambda, g);
            Assert.AreEqual(-0.4, y[0], 1e-15);
            Assert.AreEqual(0.7, y[1], 1e-15);
            AssertClose([0, 1, -0.7, 1, 0, 0.4], j);
            AssertClose([2, 1, 0.1], g);
        }

        [TestMethod]
        public void SixteenUnknownsDifferentiateToTheInverse()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var p = Enumerable.Range(0, 17).Select(i => 0.1 * (i - 5)).ToArray();
            var j = new double[16 * 17];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "TridiagonalJacobian", p, j);

            // A ∂y/∂pⱼ = eⱼ for the right-hand sides j < 16.
            var diagonal = 4 + p[16];
            for (var i = 0; i < 16; i++)
            {
                for (var k = 0; k < 16; k++)
                {
                    var below = i > 0 ? j[((i - 1) * 17) + k] : 0.0;
                    var above = i < 15 ? j[((i + 1) * 17) + k] : 0.0;
                    Assert.AreEqual(i == k ? 1.0 : 0.0, (diagonal * j[(i * 17) + k]) - below - above, 1e-14);
                }
            }

            Assert.IsTrue(j.All(double.IsFinite));
            Assert.IsGreaterThan(0.0, Math.Abs(j[16]), "∂y/∂p₁₆ is the derivative through the matrix.");
        }

        private static void AssertClose(double[] expected, double[] actual)
        {
            Assert.HasCount(expected.Length, actual);
            for (var k = 0; k < expected.Length; k++)
            {
                Assert.AreEqual(expected[k], actual[k], 1e-15, $"element {k}");
            }
        }
    }
}
