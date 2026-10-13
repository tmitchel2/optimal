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
    /// <summary>
    /// The phase-2 gate for generated element tangents and VJPs: Jacobians against hand-written BᵀDB and central
    /// differences, VJPs against the generated forward mode and by Taylor test, and no allocation.
    /// </summary>
    [TestClass]
    public class FeaKernelTests
    {
        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void Tet4JacobianEqualsBtDb()
        {
            var x = Tet4Nodes();
            var generated = new double[144];
            var reference = new double[144];
            Tet4ElasticKernels.ResidualJacobian_u(RandomVector(12, 1), x, E, Nu, generated);
            FeaReference.StiffnessTet4(x, E, Nu, reference);
            var error = MaxRelative(generated, reference);
            TestContext.WriteLine($"Tet4 Jacobian vs BtDB: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
        }

        [TestMethod]
        public void Tet10JacobianEqualsGaussBtDb()
        {
            var x = Tet10Nodes();
            var generated = new double[900];
            var reference = new double[900];
            Tet10ElasticKernels.ResidualJacobian_u(RandomVector(30, 1), x, E, Nu, generated);
            FeaReference.StiffnessTet10(x, E, Nu, reference);
            var error = MaxRelative(generated, reference);
            TestContext.WriteLine($"Tet10 Jacobian vs Gauss BtDB: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
        }

        [TestMethod]
        public void Tet10JacobianMatchesCentralDifferences()
        {
            var x = Tet10Nodes();
            var u = RandomVector(30, 2);
            var generated = new double[900];
            Tet10ElasticKernels.ResidualJacobian_u(u, x, E, Nu, generated);
            var fd = CentralDifferenceJacobian(Tet10Elastic.Residual, u, x, wrtX: false, h: 1e-6);
            var error = MaxRelative(generated, fd);
            TestContext.WriteLine($"Tet10 Jacobian vs central FD: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
        }

        [TestMethod]
        public void Tet10JacobianWithRespectToXMatchesCentralDifferences()
        {
            var x = Tet10Nodes();
            var u = RandomVector(30, 3);
            var generated = new double[900];
            Tet10ElasticKernels.ResidualJacobian_X(u, x, E, Nu, generated);
            var fd = CentralDifferenceJacobian(Tet10Elastic.Residual, u, x, wrtX: true, h: 1e-6);
            var error = MaxRelative(generated, fd);
            TestContext.WriteLine($"Tet10 dR/dX vs central FD: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
        }

        [TestMethod]
        public void Tet4VjpEqualsForwardModeTransposeLambda()
        {
            VjpEqualsForwardMode("Tet4", Tet4ElasticKernels.ResidualJacobian_X, Tet4ElasticKernels.ResidualVjp_X, 12);
        }

        [TestMethod]
        public void Tet10VjpEqualsForwardModeTransposeLambda()
        {
            VjpEqualsForwardMode("Tet10", Tet10ElasticKernels.ResidualJacobian_X, Tet10ElasticKernels.ResidualVjp_X, 30);
        }

        [TestMethod]
        public void Tet4VjpTaylorSlopeIsTwo()
        {
            TaylorSlopeIsTwo("Tet4", Tet4Elastic.Residual, Tet4ElasticKernels.ResidualVjp_X, 12);
        }

        [TestMethod]
        public void Tet10VjpTaylorSlopeIsTwo()
        {
            TaylorSlopeIsTwo("Tet10", Tet10Elastic.Residual, Tet10ElasticKernels.ResidualVjp_X, 30);
        }

        [TestMethod]
        public void Tet10VjpOfSeveralInputsSharesOneSweep()
        {
            var x = Tet10Nodes();
            var u = RandomVector(30, 4);
            var lambda = RandomVector(30, 5);
            var single = new double[30];
            var gx = new double[30];
            var ge = new double[1];
            var gnu = new double[1];
            Tet10ElasticKernels.ResidualVjp_X(u, x, E, Nu, lambda, single);
            Tet10ElasticKernels.ResidualVjp_X_E_nu(u, x, E, Nu, lambda, gx, ge, gnu);

            double F(double e, double nu)
            {
                var r = new double[30];
                Tet10Elastic.Residual(u, x, e, nu, r);
                return lambda.Zip(r, (l, ri) => l * ri).Sum();
            }

            var dE = (F(E * (1 + 1e-6), Nu) - F(E * (1 - 1e-6), Nu)) / (2e-6 * E);
            var dNu = (F(E, Nu + 1e-6) - F(E, Nu - 1e-6)) / 2e-6;
            TestContext.WriteLine($"Tet10 VJP (X, E, nu): dE {ge[0]:E6} vs FD {dE:E6}; dnu {gnu[0]:E6} vs FD {dNu:E6}");
            Assert.IsLessThanOrEqualTo(1e-15, MaxRelative(gx, single));
            Assert.AreEqual(dE, ge[0], 1e-7 * Math.Abs(dE));
            Assert.AreEqual(dNu, gnu[0], 1e-7 * Math.Abs(dNu));
        }

        [TestMethod]
        public void GeneratedKernelsAllocateNothing()
        {
            var u4 = RandomVector(12, 8);
            var u10 = RandomVector(30, 8);
            var lambda4 = RandomVector(12, 9);
            var lambda10 = RandomVector(30, 9);
            var (x4, x10) = (Tet4Nodes(), Tet10Nodes());
            var (j4, j10, g4, g10, s1, s2) = (new double[144], new double[900], new double[12], new double[30], new double[1], new double[1]);
            var bytes = new[]
            {
                AllocatedBytes(() => Tet4ElasticKernels.ResidualJacobian_u(u4, x4, E, Nu, j4)),
                AllocatedBytes(() => Tet4ElasticKernels.ResidualVjp_X(u4, x4, E, Nu, lambda4, g4)),
                AllocatedBytes(() => Tet10ElasticKernels.ResidualJacobian_u(u10, x10, E, Nu, j10)),
                AllocatedBytes(() => Tet10ElasticKernels.ResidualJacobian_X(u10, x10, E, Nu, j10)),
                AllocatedBytes(() => Tet10ElasticKernels.ResidualVjp_X(u10, x10, E, Nu, lambda10, g10)),
                AllocatedBytes(() => Tet10ElasticKernels.ResidualVjp_X_E_nu(u10, x10, E, Nu, lambda10, g10, s1, s2)),
            };
            TestContext.WriteLine($"Bytes over 10,000 calls after warm-up: {string.Join(", ", bytes)}");
            Assert.AreEqual(0L, bytes.Sum());
        }

        [TestMethod]
        public void GeneratedKernelRejectsWrongSpanLength()
        {
            var jacobian = new double[143];
            var thrown = Assert.ThrowsExactly<ArgumentException>(() => Tet4ElasticKernels.ResidualJacobian_u(new double[12], Tet4Nodes(), E, Nu, jacobian));
            Assert.AreEqual("jacobian", thrown.ParamName);
        }

        private void VjpEqualsForwardMode(string name, KernelTestData.Jacobian jacobianX, Vjp vjp, int n)
        {
            var x = Nodes(n);
            var u = RandomVector(n, 3);
            var lambda = RandomVector(n, 4);
            var jacobian = new double[n * n];
            var gradient = new double[n];
            jacobianX(u, x, E, Nu, jacobian);
            vjp(u, x, E, Nu, lambda, gradient);
            var error = MaxRelative(gradient, TransposeTimes(jacobian, lambda, n));
            TestContext.WriteLine($"{name} VJP vs forward-mode J^T lambda: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
        }

        private void TaylorSlopeIsTwo(string name, Residual residual, Vjp vjp, int n)
        {
            var (remainders, slopes) = TaylorTest(residual, vjp, RandomVector(n, 5), Nodes(n), seed: 6);
            TestContext.WriteLine($"{name} Taylor remainders {string.Join(", ", remainders.Select(r => r.ToString("E3")))}; slopes {string.Join(", ", slopes.Select(s => s.ToString("F4")))}");
            foreach (var slope in slopes)
            {
                Assert.AreEqual(2.0, slope, 0.1);
            }
        }
    }
}
