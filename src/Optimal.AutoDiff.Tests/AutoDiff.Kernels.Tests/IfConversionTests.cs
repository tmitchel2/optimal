/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>
    /// An <c>if</c> on a run-time condition: both branches run and every variable, array element and output either
    /// assigned becomes a select, whose derivative follows the branch taken; a NaN in the branch not taken reaches
    /// neither mode.
    /// </summary>
    [TestClass]
    public class IfConversionTests
    {
        private const string Source = """
            using System;
            using Optimal;

            namespace Probes
            {
                [OptimalCode]
                public static class Branches
                {
                    [Jacobian(nameof(x))]
                    [Vjp(nameof(x))]
                    public static void Features([FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(5)] Span<double> y)
                    {
                        var a = x[0];
                        var b = 0.0;
                        Span<double> buffer = stackalloc double[2];
                        if (x[0] > x[1])
                        {
                            a = a * x[2];
                            b = x[1];
                            buffer[0] = x[2];
                            y[2] = x[0] * x[0];
                            for (var i = 0; i < 2; i++)
                            {
                                if (x[2] > 0)
                                {
                                    b += x[2] * 0.5;
                                }
                            }
                        }
                        else
                        {
                            var local = x[1] * 2;
                            b = local;
                            y[2] = x[1];
                        }

                        y[0] = a;
                        y[1] = b;
                        y[3] = buffer[0];
                        if (!(x[2] >= 0))
                            y[4] = 1;
                        else
                            y[4] = x[2] * x[2];
                    }

                    [Jacobian(nameof(x))]
                    [Vjp(nameof(x))]
                    public static void Guarded([FixedLength(2)] ReadOnlySpan<double> x, [FixedLength(1)] Span<double> y)
                    {
                        var norm = Math.Sqrt((x[0] * x[0]) + (x[1] * x[1]));
                        var value = x[0] + x[1];
                        if (norm > 1)
                        {
                            value = x[0] / norm;
                        }

                        y[0] = value;
                    }
                }

                public static class Run
                {
                    public static void Jacobian(double[] x, double[] j) => BranchesKernels.FeaturesJacobian_x(x, j);

                    public static void Vjp(double[] x, double[] lambda, double[] g) => BranchesKernels.FeaturesVjp_x(x, lambda, g);

                    public static void GuardedJacobian(double[] x, double[] j) => BranchesKernels.GuardedJacobian_x(x, j);

                    public static void GuardedVjp(double[] x, double[] lambda, double[] g) => BranchesKernels.GuardedVjp_x(x, lambda, g);
                }
            }
            """;

        [TestMethod]
        public void TrueBranchDerivativesAreThoseOfTheTrueBranch()
        {
            // x₀ > x₁ and x₂ > 0: a = x₀x₂, b = x₁ + x₂, y₂ = x₀², y₃ = x₂, y₄ = x₂².
            AssertJacobian([2, 1, 3], [3, 0, 2, 0, 1, 1, 4, 0, 0, 0, 0, 1, 0, 0, 6]);
        }

        [TestMethod]
        public void FalseBranchDerivativesAreThoseOfTheFalseBranch()
        {
            // x₀ ≤ x₁ and x₂ < 0: a = x₀, b = 2x₁, y₂ = x₁, y₃ = 0, y₄ = 1.
            AssertJacobian([1, 2, -1], [1, 0, 0, 0, 2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]);
        }

        [TestMethod]
        public void NestedConditionTakenWithinTheTrueBranch()
        {
            // x₀ > x₁ but x₂ < 0: the inner if adds nothing, so b = x₁; y₃ = x₂ still; y₄ = 1.
            AssertJacobian([2, 1, -3], [-3, 0, 2, 0, 1, 0, 4, 0, 0, 0, 0, 1, 0, 0, 0]);
        }

        [TestMethod]
        public void VjpIsTheTransposeOfTheJacobianOnBothBranches()
        {
            var assembly = GeneratorHarness.Compile(Source);
            var lambda = new[] { 0.3, -1.1, 0.7, 2.0, -0.5 };
            foreach (var point in new[] { new[] { 2.0, 1, 3 }, [1.0, 2, -1] })
            {
                var j = new double[15];
                var g = new double[3];
                GeneratorHarness.Invoke(assembly, "Probes.Run", "Jacobian", point, j);
                GeneratorHarness.Invoke(assembly, "Probes.Run", "Vjp", point, lambda, g);
                for (var k = 0; k < 3; k++)
                {
                    Assert.AreEqual(Enumerable.Range(0, 5).Sum(i => lambda[i] * j[(3 * i) + k]), g[k], 1e-15);
                }
            }
        }

        [TestMethod]
        public void NaNInTheBranchNotTakenReachesNeitherMode()
        {
            // At x = 0 the true branch computes 0/0, and ∂(x₀/‖x‖) is infinite, but the false branch is taken.
            var assembly = GeneratorHarness.Compile(Source);
            var origin = new double[2];
            var lambda = new[] { 2.0 };
            var j = new double[2];
            var g = new double[2];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "GuardedJacobian", origin, j);
            GeneratorHarness.Invoke(assembly, "Probes.Run", "GuardedVjp", origin, lambda, g);
            Assert.AreEqual(1.0, j[0]);
            Assert.AreEqual(1.0, j[1]);
            Assert.AreEqual(2.0, g[0]);
            Assert.AreEqual(2.0, g[1]);
        }

        private static void AssertJacobian(double[] point, double[] expected)
        {
            var assembly = GeneratorHarness.Compile(Source);
            var j = new double[15];
            GeneratorHarness.Invoke(assembly, "Probes.Run", "Jacobian", point, j);
            CollectionAssert.AreEqual(expected, j);
        }
    }
}
