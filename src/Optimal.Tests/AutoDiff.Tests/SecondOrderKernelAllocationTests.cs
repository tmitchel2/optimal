/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// Hessians, Hessian–vector products, if-converted and implicit-solve kernels allocate nothing per call, and nor does
    /// the run-time <see cref="KernelMath.ImplicitSolve"/> a kernel's primal calls.
    /// </summary>
    [TestClass]
    [DoNotParallelize]
    public class SecondOrderKernelAllocationTests
    {
        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void GeneratedSecondOrderAndImplicitMethodsAllocateNothing()
        {
            var f = RandomVector(9, 81);
            for (var k = 0; k < 9; k += 4)
            {
                f[k] += 1;
            }

            var u = RandomVector(30, 82);
            var x = Tet10Nodes();
            var direction = RandomVector(9, 83);
            var strain = J2PlasticityTests.PlasticStateStrain;
            var plastic = J2PlasticityTests.PlasticStrain;
            var hardening = VocePlasticityTests.Hardening;
            double[] p = [0.3, -0.2, 0.5, 0.0];
            double[] one = [1.0];
            var (h9, h30, v9, t81, s9, y3, j12, h4, g4) = (new double[81], new double[900], new double[9], new double[81], new double[9], new double[3], new double[12], new double[16], new double[4]);
            var calls = new (string Name, Action Call)[]
            {
                ("NeoHookean Hessian", () => NeoHookeanEnergyKernels.DensityHessian_F(f, E, Nu, h9)),
                ("NeoHookean Hvp", () => NeoHookeanEnergyKernels.DensityHvp_F(f, E, Nu, direction, v9)),
                ("Tet10 energy Hessian", () => Tet10ElasticEnergyKernels.EnergyHessian_u(u, x, E, Nu, h30)),
                ("J2 tangent", () => J2PlasticityKernels.StressJacobian_strain(strain, plastic, J2PlasticityTests.Alpha, E, Nu, J2PlasticityTests.YieldStress, J2PlasticityTests.Hardening, t81)),
                ("J2 VJP", () => J2PlasticityKernels.StressVjp_strain(strain, plastic, J2PlasticityTests.Alpha, E, Nu, J2PlasticityTests.YieldStress, J2PlasticityTests.Hardening, direction, s9)),
                ("Voce tangent", () => VocePlasticityKernels.StressJacobian_strain(strain, plastic, J2PlasticityTests.Alpha, E, Nu, hardening, t81)),
                ("Voce run-time stress (ImplicitSolve)", () => VocePlasticity.Stress(strain, plastic, J2PlasticityTests.Alpha, E, Nu, hardening, s9)),
                ("Implicit system Jacobian", () => ImplicitSystemsKernels.SolveJacobian_p(p, j12)),
                ("Implicit system run-time solve", () => ImplicitSystems.Solve(p, y3)),
                ("Implicit energy Hessian", () => ImplicitSystemsKernels.EnergyHessian_p(p, h4)),
                ("Implicit energy VJP", () => ImplicitSystemsKernels.EnergyVjp_p(p, one, g4)),
            };
            foreach (var (name, call) in calls)
            {
                var bytes = AllocatedBytes(call);
                TestContext.WriteLine($"{name}: {bytes} bytes over 10,000 calls");
                Assert.AreEqual(0L, bytes, name);
            }
        }
    }
}
