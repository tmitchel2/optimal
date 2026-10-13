/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>The generated Hessian of a Tet10 element's elastic energy, which is the element stiffness.</summary>
    [TestClass]
    public class Tet10ElasticEnergyTests
    {
        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void HessianMatchesCentralDifferencesOfTheGradient()
        {
            var u = RandomVector(30, 41);
            var x = Tet10Nodes();
            var hessian = new double[900];
            Tet10ElasticEnergyKernels.EnergyHessian_u(u, x, E, Nu, hessian);
            var differenced = CentralDifferenceHessian((v, g) => Tet10ElasticEnergyKernels.EnergyVjp_u(v, x, E, Nu, [1.0], g), u, 1e-6);
            var error = MaxRelative(hessian, differenced);
            TestContext.WriteLine($"Tet10 energy Hessian vs central FD of the VJP: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
        }

        [TestMethod]
        public void HessianIsTheElementStiffness()
        {
            var x = Tet10Nodes();
            var hessian = new double[900];
            var stiffness = new double[900];
            Tet10ElasticEnergyKernels.EnergyHessian_u(RandomVector(30, 42), x, E, Nu, hessian);
            FeaReference.StiffnessTet10(x, E, Nu, stiffness);
            var error = MaxRelative(hessian, stiffness);
            TestContext.WriteLine($"Tet10 energy Hessian vs Gauss BtDB: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
            Assert.AreEqual(0.0, Asymmetry(hessian, 30));
        }
    }
}
