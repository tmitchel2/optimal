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
    /// The generated algorithmic tangent of J2 radial return, written with an if-converted branch: at a plastic state it
    /// is Simo and Taylor's consistent tangent, at an elastic state the elasticity tensor, and at a state where the
    /// plastic branch divides zero by zero, still the elasticity tensor in both modes.
    /// </summary>
    [TestClass]
    public class J2PlasticityTests
    {
        public const double YieldStress = 250;
        public const double Hardening = 1000;
        public const double Alpha = 1e-3;

        public TestContext TestContext { get; set; } = null!;

        public static double[] PlasticStrain => [4e-4, 1e-4, 0, 1e-4, -2e-4, 0, 0, 0, -2e-4];

        public static double[] PlasticStateStrain => [4e-3, 1.5e-3, -5e-4, 1.5e-3, -1e-3, 2e-3, -5e-4, 2e-3, 1.2e-3];

        [TestMethod]
        public void PlasticTangentIsSimoTaylorConsistentTangent()
        {
            var strain = PlasticStateStrain;
            var tangent = new double[81];
            J2PlasticityKernels.StressJacobian_strain(strain, PlasticStrain, Alpha, E, Nu, YieldStress, Hardening, tangent);
            var error = MaxRelative(tangent, SimoTaylor(strain, PlasticStrain, Alpha, Hardening));
            TestContext.WriteLine($"J2 algorithmic tangent vs Simo-Taylor consistent tangent: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-12, error);
        }

        [TestMethod]
        public void ElasticTangentIsTheElasticityTensor()
        {
            double[] strain = [1e-4, 2e-5, 0, 2e-5, -5e-5, 1e-5, 0, 1e-5, 3e-5];
            var tangent = new double[81];
            J2PlasticityKernels.StressJacobian_strain(strain, PlasticStrain, Alpha, E, Nu, YieldStress, Hardening, tangent);
            var error = MaxRelative(tangent, Elasticity());
            TestContext.WriteLine($"J2 elastic tangent vs C: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-12, error);
        }

        [TestMethod]
        public void ZeroDeviatoricStressGivesFiniteElasticDerivatives()
        {
            // s = 0, so the plastic branch, not taken, computes 0/0; neither mode may let its NaN through.
            double[] strain = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3];
            var zero = new double[9];
            var lambda = RandomVector(9, 51);
            var tangent = new double[81];
            var gradient = new double[9];
            J2PlasticityKernels.StressJacobian_strain(strain, zero, 0, E, Nu, YieldStress, Hardening, tangent);
            J2PlasticityKernels.StressVjp_strain(strain, zero, 0, E, Nu, YieldStress, Hardening, lambda, gradient);
            var elasticity = Elasticity();
            Assert.IsTrue(tangent.Concat(gradient).All(double.IsFinite));
            Assert.IsLessThanOrEqualTo(1e-12, MaxRelative(tangent, elasticity));
            Assert.IsLessThanOrEqualTo(1e-12, MaxRelative(gradient, TransposeTimes(elasticity, lambda, 9)));
        }

        [TestMethod]
        public void VjpEqualsForwardModeTransposeLambda()
        {
            var strain = PlasticStateStrain;
            var lambda = RandomVector(9, 52);
            var tangent = new double[81];
            var gradient = new double[9];
            J2PlasticityKernels.StressJacobian_strain(strain, PlasticStrain, Alpha, E, Nu, YieldStress, Hardening, tangent);
            J2PlasticityKernels.StressVjp_strain(strain, PlasticStrain, Alpha, E, Nu, YieldStress, Hardening, lambda, gradient);
            Assert.IsLessThanOrEqualTo(1e-13, MaxRelative(gradient, TransposeTimes(tangent, lambda, 9)));
        }

        /// <summary>
        /// Simo and Taylor's consistent tangent for radial return with linear isotropic hardening H:
        /// C = κ 1⊗1 + 2μθ (I − ⅓ 1⊗1) − 2μθ̄ n⊗n, θ = 1 − 2μΔγ/‖s‖, θ̄ = 1/(1 + H/3μ) − (1 − θ), with I_ijkl = δ_ik δ_jl
        /// because the kernel's nine strain components are independent.
        /// </summary>
        public static double[] SimoTaylor(double[] strain, double[] plasticStrain, double alpha, double hardening)
        {
            var mu = ElementGeometry.Shear(E, Nu);
            var kappa = E / (3 * (1 - (2 * Nu)));
            var elastic = strain.Zip(plasticStrain, (a, b) => a - b).ToArray();
            var trace = elastic[0] + elastic[4] + elastic[8];
            var s = elastic.Select((e, k) => 2 * mu * (e - (k % 4 == 0 ? trace / 3 : 0))).ToArray();
            var norm = Math.Sqrt(s.Sum(v => v * v));
            var f = norm - (Math.Sqrt(2.0 / 3.0) * (YieldStress + (hardening * alpha)));
            Assert.IsGreaterThan(0.0, f, "The state must be plastic.");
            var dGamma = f / ((2 * mu) + (2.0 / 3.0 * hardening));
            var theta = 1 - (2 * mu * dGamma / norm);
            var thetaBar = (1 / (1 + (hardening / (3 * mu)))) - (1 - theta);
            var n = s.Select(v => v / norm).ToArray();
            return Enumerable.Range(0, 81).Select(k =>
            {
                var (row, column) = (k / 9, k % 9);
                var volumetric = row % 4 == 0 && column % 4 == 0 ? 1.0 : 0.0;
                var identity = row == column ? 1.0 : 0.0;
                return (kappa * volumetric) + (2 * mu * theta * (identity - (volumetric / 3))) - (2 * mu * thetaBar * n[row] * n[column]);
            }).ToArray();
        }

        /// <summary>C = κ 1⊗1 + 2μ (I − ⅓ 1⊗1).</summary>
        public static double[] Elasticity()
        {
            var mu = ElementGeometry.Shear(E, Nu);
            var kappa = E / (3 * (1 - (2 * Nu)));
            return Enumerable.Range(0, 81).Select(k =>
            {
                var (row, column) = (k / 9, k % 9);
                var volumetric = row % 4 == 0 && column % 4 == 0 ? 1.0 : 0.0;
                return (kappa * volumetric) + (2 * mu * ((row == column ? 1.0 : 0.0) - (volumetric / 3)));
            }).ToArray();
        }
    }
}
