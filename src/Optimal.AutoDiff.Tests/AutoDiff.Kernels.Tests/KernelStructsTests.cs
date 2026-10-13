/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>
    /// Structs of doubles flattened to their fields: as locals, helper parameters (by value and by in) and return
    /// values, built by constructors, initialisers, with and default, read and written field by field and through
    /// nested structs, and used through instance methods, properties and operators.
    /// </summary>
    [TestClass]
    public class KernelStructsTests
    {
        private const string Source = """
            using System;
            using Optimal;

            namespace Probes
            {
                public struct Vec2
                {
                    public double X;
                    public double Y;

                    public Vec2(double x, double y)
                    {
                        X = x;
                        Y = y;
                    }

                    public static Vec2 Zero => default;

                    public readonly double LengthSquared => Dot(this);

                    public readonly double Dot(Vec2 other) => (X * other.X) + (Y * other.Y);

                    public void Scale(double s)
                    {
                        X *= s;
                        this.Y = this.Y * s;
                    }

                    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);

                    public static Vec2 operator -(Vec2 a) => new(-a.X, -a.Y);

                    public static Vec2 operator *(double s, Vec2 a) => new Vec2 { X = s * a.X, Y = s * a.Y };
                }

                public struct Pair
                {
                    public Vec2 First;
                    public Vec2 Second;
                }

                [OptimalCode]
                public static class Structs
                {
                    [Jacobian(nameof(x))]
                    [Vjp(nameof(x))]
                    public static void Features([FixedLength(4)] ReadOnlySpan<double> x, [FixedLength(8)] Span<double> y)
                    {
                        var a = new Vec2(x[0], x[1]);
                        var b = new Vec2 { X = x[2], Y = x[3] };
                        y[0] = a.Dot(b);
                        y[1] = (a + b).LengthSquared;
                        var c = -a;
                        c.Scale(3);
                        y[2] = c.X;
                        var d = 2.0 * Mirror(b);
                        y[3] = d.Y;
                        Pair p;
                        p.First = a;
                        p.Second = b with { Y = 1.0 };
                        p.Second.X += x[0];
                        y[4] = p.Second.X * p.Second.Y * p.First.X;
                        y[5] = Larger(a, b).X;
                        y[6] = Norm2(in a);
                        var e = Vec2.Zero;
                        e.X = x[1];
                        y[7] = e.X + e.Y;
                    }

                    private static Vec2 Mirror(Vec2 v) => new(v.Y, v.X);

                    private static Vec2 Larger(Vec2 a, Vec2 b) => a.X > b.X ? a : b;

                    private static double Norm2(in Vec2 v) => v.LengthSquared;
                }

                public static class Run
                {
                    public static void Primal(double[] x, double[] y) => Structs.Features(x, y);

                    public static void Jacobian(double[] x, double[] j) => StructsKernels.FeaturesJacobian_x(x, j);

                    public static void Vjp(double[] x, double[] lambda, double[] g) => StructsKernels.FeaturesVjp_x(x, lambda, g);
                }
            }
            """;

        // At x = (0.5, −0.25, 0.3, 2), where x₀ > x₂, worked by hand from the kernel above.
        private static readonly double[] Point = [0.5, -0.25, 0.3, 2.0];

        private static readonly double[] Expected =
        [
            0.3, 2, 0.5, -0.25,
            1.6, 3.5, 1.6, 3.5,
            -3, 0, 0, 0,
            0, 0, 2, 0,
            1.3, 0, 0.5, 0,
            1, 0, 0, 0,
            1, -0.5, 0, 0,
            0, 1, 0, 0,
        ];

        [TestMethod]
        public void JacobianThroughStructsIsExact()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var j = new double[32];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Jacobian", Point, j);
            for (var k = 0; k < j.Length; k++)
            {
                Assert.AreEqual(Expected[k], j[k], 1e-15, $"element {k}");
            }
        }

        [TestMethod]
        public void VjpThroughStructsIsTheTransposeOfTheJacobian()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var lambda = new[] { 0.3, -1.1, 0.7, 2.0, -0.5, 0.25, 1.5, -2.0 };
            var g = new double[4];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Vjp", Point, lambda, g);
            for (var k = 0; k < 4; k++)
            {
                Assert.AreEqual(Enumerable.Range(0, 8).Sum(i => lambda[i] * Expected[(4 * i) + k]), g[k], 1e-15);
            }
        }

        [TestMethod]
        public void StructHelpersAreNotScalarDifferentiated()
        {
            var run = GeneratorHarness.Run(Source.Replace("private static Vec2 Mirror", "public static Vec2 Mirror"));
            Assert.IsEmpty(run.Diagnostics);
            Assert.DoesNotContain("Mirror", run.Sources.GetValueOrDefault("Probes.StructsGradients.g.cs") ?? string.Empty);
        }
    }
}
