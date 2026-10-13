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
    /// <summary>
    /// The tangent of a Voce-hardening return mapping whose plastic multiplier comes from an implicit solve: against
    /// central differences of the run-time kernel, and, with the saturation switched off, against the closed form.
    /// </summary>
    [TestClass]
    public class VocePlasticityTests
    {
        public const double Alpha = J2PlasticityTests.Alpha;

        public TestContext TestContext { get; set; } = null!;

        // σ₀, σ∞, δ, H.
        public static double[] Hardening => [250, 400, 50, 500];

        [TestMethod]
        public void TangentMatchesCentralDifferences()
        {
            var strain = J2PlasticityTests.PlasticStateStrain;
            var tangent = new double[81];
            VocePlasticityKernels.StressJacobian_strain(strain, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, Hardening, tangent);
            var differenced = CentralDifferenceJacobian(
                (e, s) => VocePlasticity.Stress(e, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, Hardening, s),
                strain,
                9,
                1e-8);
            var error = MaxRelative(tangent, differenced);
            TestContext.WriteLine($"Voce tangent (ImplicitSolve) vs central FD: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
            Assert.IsGreaterThan(1e-3, MaxRelative(tangent, J2PlasticityTests.Elasticity()), "The state must be plastic.");
        }

        [TestMethod]
        public void WithoutSaturationTheTangentIsSimoTaylor()
        {
            // σ∞ = σ₀ leaves linear hardening H, whose consistent tangent has a closed form.
            double[] hardening = [J2PlasticityTests.YieldStress, J2PlasticityTests.YieldStress, 50, J2PlasticityTests.Hardening];
            var strain = J2PlasticityTests.PlasticStateStrain;
            var tangent = new double[81];
            VocePlasticityKernels.StressJacobian_strain(strain, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, hardening, tangent);
            var error = MaxRelative(tangent, J2PlasticityTests.SimoTaylor(strain, J2PlasticityTests.PlasticStrain, Alpha, J2PlasticityTests.Hardening));
            TestContext.WriteLine($"Voce tangent with sigma_inf = sigma_0 vs Simo-Taylor: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-12, error);
        }

        [TestMethod]
        public void ElasticStateGivesTheElasticityTensor()
        {
            double[] strain = [1e-4, 2e-5, 0, 2e-5, -5e-5, 1e-5, 0, 1e-5, 3e-5];
            var lambda = RandomVector(9, 61);
            var tangent = new double[81];
            var gradient = new double[9];
            VocePlasticityKernels.StressJacobian_strain(strain, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, Hardening, tangent);
            VocePlasticityKernels.StressVjp_strain(strain, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, Hardening, lambda, gradient);
            Assert.IsLessThanOrEqualTo(1e-12, MaxRelative(tangent, J2PlasticityTests.Elasticity()));
            Assert.IsTrue(gradient.All(double.IsFinite));
        }

        [TestMethod]
        public void VjpEqualsForwardModeTransposeLambda()
        {
            var strain = J2PlasticityTests.PlasticStateStrain;
            var lambda = RandomVector(9, 62);
            var tangent = new double[81];
            var gradient = new double[9];
            VocePlasticityKernels.StressJacobian_strain(strain, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, Hardening, tangent);
            VocePlasticityKernels.StressVjp_strain(strain, J2PlasticityTests.PlasticStrain, Alpha, E, Nu, Hardening, lambda, gradient);
            Assert.IsLessThanOrEqualTo(1e-13, MaxRelative(gradient, TransposeTimes(tangent, lambda, 9)));
        }
    }
}
