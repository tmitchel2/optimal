/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>A nonlinear kernel built from helpers and Math.Log: its tangent, symmetry and shape sensitivity.</summary>
    [TestClass]
    public class Tet4NeoHookeanTests
    {
        public TestContext TestContext { get; set; } = null!;

        // A finite but invertible deformation.
        private static double[] Displacement() => RandomVector(12, 21).Select(v => 0.1 * v).ToArray();

        [TestMethod]
        public void TangentMatchesCentralDifferences()
        {
            var u = Displacement();
            var x = Tet4Nodes();
            var generated = new double[144];
            Tet4NeoHookeanKernels.ResidualJacobian_u(u, x, E, Nu, generated);
            var error = MaxRelative(generated, CentralDifferenceJacobian(Tet4NeoHookean.Residual, u, x, wrtX: false, h: 1e-7));
            TestContext.WriteLine($"Neo-Hookean tangent vs central FD: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
        }

        [TestMethod]
        public void TangentIsSymmetric()
        {
            var k = new double[144];
            Tet4NeoHookeanKernels.ResidualJacobian_u(Displacement(), Tet4Nodes(), E, Nu, k);
            var transposed = Enumerable.Range(0, 144).Select(i => k[(12 * (i % 12)) + (i / 12)]).ToArray();
            Assert.IsLessThanOrEqualTo(1e-13, MaxRelative(k, transposed));
        }

        [TestMethod]
        public void VjpEqualsForwardModeTransposeLambda()
        {
            var u = Displacement();
            var x = Tet4Nodes();
            var lambda = RandomVector(12, 22);
            var jacobian = new double[144];
            var gradient = new double[12];
            Tet4NeoHookeanKernels.ResidualJacobian_X(u, x, E, Nu, jacobian);
            Tet4NeoHookeanKernels.ResidualVjp_X(u, x, E, Nu, lambda, gradient);
            var error = MaxRelative(gradient, TransposeTimes(jacobian, lambda, 12));
            TestContext.WriteLine($"Neo-Hookean VJP vs J^T lambda: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
        }

        [TestMethod]
        public void VjpTaylorSlopeIsTwo()
        {
            var (remainders, slopes) = TaylorTest(Tet4NeoHookean.Residual, Tet4NeoHookeanKernels.ResidualVjp_X, Displacement(), Tet4Nodes(), seed: 23);
            TestContext.WriteLine($"Neo-Hookean Taylor remainders {string.Join(", ", remainders.Select(r => r.ToString("E3")))}; slopes {string.Join(", ", slopes.Select(s => s.ToString("F4")))}");
            foreach (var slope in slopes)
            {
                Assert.AreEqual(2.0, slope, 0.1);
            }
        }
    }
}
