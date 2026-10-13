/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Diagnostics;
using System.Reflection;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// The cost of generated tangents and VJPs against hand-written code. Each figure is the best of several repeats, so
    /// another process's load inflates it less; the ratios are the measurement, the absolute times depend on the machine.
    /// A Debug build runs both sides unoptimised, so there the figures are reported and the gate is not judged.
    /// </summary>
    [TestClass]
    [DoNotParallelize]
    public class FeaKernelPerformanceTests
    {
        private const int Calls = 100_000;
        private const int Repeats = 7;

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        [TestCategory("LongRunning")]
        public void Tet10JacobianCostsNoMoreThanHandWrittenBtDb()
        {
            var tet4 = JacobianCost(Tet4ElasticKernels.ResidualJacobian_u, FeaReference.StiffnessTet4, 12);
            var tet10 = JacobianCost(Tet10ElasticKernels.ResidualJacobian_u, FeaReference.StiffnessTet10, 30);
            var vjp4 = VjpCost(Tet4ElasticKernels.ResidualVjp_X, Tet4Elastic.Residual, 12);
            var vjp10 = VjpCost(Tet10ElasticKernels.ResidualVjp_X, Tet10Elastic.Residual, 30);
            TestContext.WriteLine($"Tet4 per call: generated J {tet4.Generated:F3} us, hand K_e {tet4.Hand:F3} us, ratio {tet4.Generated / tet4.Hand:F3}");
            TestContext.WriteLine($"Tet10 per call: generated J {tet10.Generated:F3} us, hand K_e {tet10.Hand:F3} us, ratio {tet10.Generated / tet10.Hand:F3}");
            TestContext.WriteLine($"Tet4 per call: generated VJP {vjp4.Generated:F3} us, primal residual {vjp4.Hand:F3} us, ratio {vjp4.Generated / vjp4.Hand:F3}");
            TestContext.WriteLine($"Tet10 per call: generated VJP {vjp10.Generated:F3} us, primal residual {vjp10.Hand:F3} us, ratio {vjp10.Generated / vjp10.Hand:F3}");
            if (IsJitOptimizerDisabled())
            {
                Assert.Inconclusive("Timed in a Debug build, where the JIT does not optimise; the cost gate is judged in Release.");
            }

            Assert.IsLessThanOrEqualTo(1.0, tet10.Generated / tet10.Hand);
        }

        private static bool IsJitOptimizerDisabled()
        {
            return typeof(FeaKernelPerformanceTests).Assembly.GetCustomAttribute<DebuggableAttribute>()?.IsJITOptimizerDisabled == true;
        }

        private static (double Generated, double Hand) JacobianCost(Jacobian jacobian, Stiffness stiffness, int n)
        {
            var u = RandomVector(n, 10);
            var x = Nodes(n);
            var j = new double[n * n];
            var k = new double[n * n];
            return BestOf(() => jacobian(u, x, E, Nu, j), () => stiffness(x, E, Nu, k));
        }

        private static (double Generated, double Hand) VjpCost(Vjp vjp, Residual residual, int n)
        {
            var u = RandomVector(n, 10);
            var x = Nodes(n);
            var lambda = RandomVector(n, 11);
            var g = new double[n];
            var r = new double[n];
            return BestOf(() => vjp(u, x, E, Nu, lambda, g), () => residual(u, x, E, Nu, r));
        }

        // Microseconds per call of each, the best of the repeats, after a warm-up that brings both to tier-1 code.
        private static (double Generated, double Hand) BestOf(Action generated, Action hand)
        {
            for (var i = 0; i < 20_000; i++)
            {
                generated();
                hand();
            }

            var best = (Generated: double.MaxValue, Hand: double.MaxValue);
            for (var repeat = 0; repeat < Repeats; repeat++)
            {
                best = (Math.Min(best.Generated, Time(generated)), Math.Min(best.Hand, Time(hand)));
            }

            return best;
        }

        private static double Time(Action call)
        {
            var stopwatch = Stopwatch.StartNew();
            for (var i = 0; i < Calls; i++)
            {
                call();
            }

            return stopwatch.Elapsed.TotalMicroseconds / Calls;
        }
    }
}
