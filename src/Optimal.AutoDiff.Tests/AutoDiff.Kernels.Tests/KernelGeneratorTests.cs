/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Analyzers.Kernels.Tests.KernelSource;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>
    /// The kernel language end to end: a kernel using every statement form, table and helper the unroller supports is
    /// generated, compiled and run, and its Jacobian and VJP are checked against the derivatives worked by hand.
    /// </summary>
    [TestClass]
    public class KernelGeneratorTests
    {
        private const string Language = """
            using System;
            using Optimal;

            namespace Probes
            {
                [OptimalCode]
                public static class Language
                {
                    private static readonly double[,] Weights = { { 1, 2 }, { 3, 4 } };
                    private static readonly double Half = 0.5;

                    private static ReadOnlySpan<int> Order => [2, 0, 1];

                    [Jacobian(nameof(x))]
                    [Vjp(nameof(x))]
                    public static void Features([FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(8)] Span<double> y)
                    {
                        // while, break and continue: i takes 1, 3 and 4.
                        var i = 0;
                        var acc = 0.0;
                        while (true)
                        {
                            i++;
                            if (i == 2)
                            {
                                continue;
                            }

                            if (i > 4)
                            {
                                break;
                            }

                            acc += x[0] * i;
                        }

                        y[0] = acc;
                        y[1] = Pick(x, 2);

                        // Ranges and slices are views.
                        var tail = x[1..];
                        var middle = x.Slice(1, 1);
                        y[2] = tail[1] * middle[0];

                        // Fill, CopyTo, and a helper writing its caller's span.
                        Span<double> buffer = stackalloc double[4];
                        buffer.Fill(1);
                        x.CopyTo(buffer);
                        y[3] = buffer[3] + buffer[0];
                        Twice(buffer);
                        y[4] = buffer[1];

                        // A rectangular table, an int table and an int helper argument.
                        y[5] = Weights[1, 0] * x[Order[0]];

                        // Compound %= and a compile-time conditional.
                        var k = 7;
                        k %= 3;
                        y[6] = k == 1 ? x[k] * x[k] : 0;

                        // A run-time select inside a helper, and Math on ints.
                        y[7] = Relu(x[0] - x[1]) + x[Math.Abs(-2)];
                    }

                    [Jacobian(nameof(x), ChunkSize = 3)]
                    [Vjp(nameof(x), ChunkSize = 1)]
                    public static void Chunked([FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(8)] Span<double> y)
                    {
                        Features(x, y);
                    }

                    private static double Pick(ReadOnlySpan<double> v, int k)
                    {
                        for (var i = 0; i < v.Length; i++)
                        {
                            if (i == k)
                            {
                                return v[i] * Half;
                            }
                        }

                        return 0;
                    }

                    private static void Twice(Span<double> v)
                    {
                        for (var i = 0; i < v.Length; i++)
                        {
                            v[i] *= 2;
                        }
                    }

                    private static double Relu(double v) => v > 0 ? v : 0;
                }

                public static class Run
                {
                    public static void Jacobian(double[] x, double[] j) => LanguageKernels.FeaturesJacobian_x(x, j);

                    public static void ChunkedJacobian(double[] x, double[] j) => LanguageKernels.ChunkedJacobian_x(x, j);

                    public static void Vjp(double[] x, double[] lambda, double[] g) => LanguageKernels.FeaturesVjp_x(x, lambda, g);

                    public static void ChunkedVjp(double[] x, double[] lambda, double[] g) => LanguageKernels.ChunkedVjp_x(x, lambda, g);
                }
            }
            """;

        private static readonly string[] ExpectedHintNames = ["OptimalAttributes.g.cs", "OptimalKernelAttributes.g.cs", "Probes.LanguageKernels.g.cs"];

        // At x = (0.5, 0.25, 2): row i is ∂y_i/∂x, worked by hand from the kernel above.
        private static readonly double[] Point = [0.5, 0.25, 2.0];

        private static readonly double[] Expected =
        [
            8, 0, 0,
            0, 0, 0.5,
            0, 2, 0.25,
            1, 0, 0,
            0, 2, 0,
            0, 0, 3,
            0, 0.5, 0,
            1, -1, 1,
        ];

        [TestMethod]
        public void JacobianOfEveryLanguageFeatureIsExact()
        {
            var assembly = GeneratorHarness.Compile(Language);
            var j = new double[24];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Jacobian", Point, j);
            CollectionAssert.AreEqual(Expected, j);
        }

        [TestMethod]
        public void ChunkingDoesNotChangeAnyBit()
        {
            var assembly = GeneratorHarness.Compile(Language);
            var (j, chunked, g, chunkedG) = (new double[24], new double[24], new double[3], new double[3]);
            var lambda = new[] { 0.3, -1.1, 0.7, 2.0, -0.5, 0.25, 1.5, -2.0 };
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Jacobian", Point, j);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "ChunkedJacobian", Point, chunked);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Vjp", Point, lambda, g);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "ChunkedVjp", Point, lambda, chunkedG);
            CollectionAssert.AreEqual(j, chunked);
            CollectionAssert.AreEqual(g, chunkedG);
        }

        [TestMethod]
        public void VjpIsTheTransposeOfTheJacobian()
        {
            var assembly = GeneratorHarness.Compile(Language);
            var lambda = new[] { 0.3, -1.1, 0.7, 2.0, -0.5, 0.25, 1.5, -2.0 };
            var g = new double[3];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Vjp", Point, lambda, g);
            var expected = Enumerable.Range(0, 3).Select(k => Enumerable.Range(0, 8).Sum(i => lambda[i] * Expected[(3 * i) + k])).ToArray();
            for (var k = 0; k < 3; k++)
            {
                Assert.AreEqual(expected[k], g[k], 1e-15);
            }
        }

        [TestMethod]
        public void ChunkedMethodSpillsAcrossChunks()
        {
            var source = GeneratorHarness.Run(Language).Sources["Probes.LanguageKernels.g.cs"];
            Assert.Contains("private static void ChunkedJacobian_xChunk1(", source);
            Assert.Contains("Span<double> spill = stackalloc double[", source);
            Assert.DoesNotContain("FeaturesJacobian_xChunk", source);
        }

        [TestMethod]
        public void KernelsGoToTheirOwnFileAndHelpersAreNotScalarDifferentiated()
        {
            var run = GeneratorHarness.Run(Language);
            CollectionAssert.AreEquivalent(ExpectedHintNames, run.Sources.Keys.ToArray());
        }

        [TestMethod]
        public void GeneratedClassFollowsTheKernelClassAccessibility()
        {
            var source = Kernel("y[0] = x[0]; y[1] = x[1] * x[2];").Replace("public static class Probe", "internal static class Probe");
            var generated = GeneratorHarness.Run(source).Sources["Probes.ProbeKernels.g.cs"];
            Assert.Contains("internal static partial class ProbeKernels", generated);
        }

        [TestMethod]
        public void GeneratedNamesAvoidTheKernelsParameters()
        {
            var source = Kernel(
                "y[0] = x[0] * lambda; y[1] = gradient;",
                signature: "[FixedLength(3)] ReadOnlySpan<double> x, double lambda, double gradient, [FixedLength(2)] Span<double> y",
                attributes: "[Vjp(nameof(x))]");
            var generated = GeneratorHarness.Run(source).Sources["Probes.ProbeKernels.g.cs"];
            Assert.Contains("public static void FVjp_x(ReadOnlySpan<double> x, double lambda, double gradient, ReadOnlySpan<double> lambda_, Span<double> gradient_)", generated);
        }
    }
}
