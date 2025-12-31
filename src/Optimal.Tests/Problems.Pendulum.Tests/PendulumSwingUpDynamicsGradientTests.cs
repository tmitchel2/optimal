/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1707 // Test method names use underscores for readability

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Problems.Pendulum.Tests
{
    /// <summary>
    /// Tests comparing AutoDiff-generated gradients against numerical gradients
    /// and manual analytical gradients for pendulum swing-up dynamics.
    /// </summary>
    [TestClass]
    public sealed class PendulumSwingUpDynamicsGradientTests
    {
        private const double Tolerance = 1e-10;
        private const double FiniteDiffEpsilon = 1e-7;

        // Standard test parameters
        private const double g = 9.81;
        private const double L = 1.0;
        private const double m = 1.0;

        /// <summary>
        /// Computes numerical gradient using central finite differences.
        /// </summary>
        private static double NumericalGradient(System.Func<double, double> func, double x)
        {
            return (func(x + FiniteDiffEpsilon) - func(x - FiniteDiffEpsilon)) / (2 * FiniteDiffEpsilon);
        }

        // ========== ANGLE RATE GRADIENT TESTS ==========

        [TestMethod]
        public void AngleRateGradient_WrtTheta_ShouldBeZero()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 1.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngleRateReverse(theta, thetadot, u, g, L, m);
            var autoDiffGrad = grad[0]; // ∂θ̇/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngleRate(theta_perturbed, thetadot, u, g, L, m),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂θ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̇/∂θ should be exactly 0 (θ̇ = thetadot, independent of θ)");
        }

        [TestMethod]
        public void AngleRateGradient_WrtThetaDot_ShouldBeOne()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 1.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngleRateReverse(theta, thetadot, u, g, L, m);
            var autoDiffGrad = grad[1]; // ∂θ̇/∂thetadot

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngleRate(theta, thetadot_perturbed, u, g, L, m),
                thetadot);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂thetadot: AutoDiff should match numerical gradient");
            Assert.AreEqual(1.0, autoDiffGrad, Tolerance, "∂θ̇/∂thetadot should be exactly 1 (θ̇ = thetadot)");
        }

        [TestMethod]
        public void AngleRateGradient_WrtControl_ShouldBeZero()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 1.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngleRateReverse(theta, thetadot, u, g, L, m);
            var autoDiffGrad = grad[2]; // ∂θ̇/∂u

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                u_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngleRate(theta, thetadot, u_perturbed, g, L, m),
                u);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂u: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̇/∂u should be exactly 0 (θ̇ independent of u)");
        }

        // ========== ANGULAR VELOCITY RATE GRADIENT TESTS ==========
        // θ̈ = -g/L·sin(θ) + u/(m·L²)

        [TestMethod]
        public void AngularVelocityRateGradient_WrtTheta_ShouldMatchNumerical()
        {
            // Expected behavior: ∂θ̈/∂θ = -g/L·cos(θ)
            // This gradient represents how gravity torque changes with angle

            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 1.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(theta, thetadot, u, g, L, m);
            var autoDiffGrad = grad[0]; // ∂θ̈/∂θ

            // Act - Numerical gradient (ground truth)
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta_perturbed, thetadot, u, g, L, m),
                theta);

            // Assert - AutoDiff must match numerical gradient
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂θ̈/∂θ: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");

            // Assert - Analytical formula verification
            var expectedGrad = -g / L * Math.Cos(theta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8,
                $"∂θ̈/∂θ should match analytical formula: -g/L·cos(θ)");
        }

        [TestMethod]
        public void AngularVelocityRateGradient_WrtTheta_AtDifferentAngles()
        {
            // Test gradient behavior at key angles

            var thetadot = 1.0;
            var u = 0.0;

            // At bottom (θ=0): cos(0)=1, gradient should be -g/L (maximum restoring force derivative)
            var (_, grad0) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(0.0, thetadot, u, g, L, m);
            Assert.AreEqual(-g / L, grad0[0], 1e-8, "At bottom, ∂θ̈/∂θ = -g/L");

            // At horizontal (θ=π/2): cos(π/2)=0, gradient should be 0
            var (_, gradPi2) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(Math.PI / 2, thetadot, u, g, L, m);
            Assert.AreEqual(0.0, gradPi2[0], 1e-8, "At horizontal, ∂θ̈/∂θ = 0");

            // At top (θ=π): cos(π)=-1, gradient should be +g/L (unstable equilibrium derivative)
            var (_, gradPi) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(Math.PI, thetadot, u, g, L, m);
            Assert.AreEqual(g / L, gradPi[0], 1e-8, "At top, ∂θ̈/∂θ = +g/L (unstable)");
        }

        [TestMethod]
        public void AngularVelocityRateGradient_WrtThetaDot_ShouldBeZero()
        {
            // θ̈ does not depend on θ̇ for a simple pendulum (no damping)

            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 1.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(theta, thetadot, u, g, L, m);
            var autoDiffGrad = grad[1]; // ∂θ̈/∂thetadot

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot_perturbed, u, g, L, m),
                thetadot);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̈/∂thetadot: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̈/∂thetadot should be exactly 0 (no damping)");
        }

        [TestMethod]
        public void AngularVelocityRateGradient_WrtControl_ShouldMatchNumerical()
        {
            // Expected behavior: ∂θ̈/∂u = 1/(m·L²)

            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 1.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(theta, thetadot, u, g, L, m);
            var autoDiffGrad = grad[2]; // ∂θ̈/∂u

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                u_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u_perturbed, g, L, m),
                u);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̈/∂u: AutoDiff should match numerical gradient");

            // Manual verification: ∂θ̈/∂u = 1/(m·L²)
            var expectedGrad = 1.0 / (m * L * L);
            Assert.AreEqual(expectedGrad, autoDiffGrad, Tolerance, "∂θ̈/∂u should be exactly 1/(m·L²)");
        }

        // ========== RUNNING COST GRADIENT TESTS ==========
        // L = 0.5·u²

        [TestMethod]
        public void RunningCostGradient_WrtTheta_ShouldBeZero()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 3.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.RunningCostReverse(theta, thetadot, u);
            var autoDiffGrad = grad[0]; // ∂L/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta_perturbed, thetadot, u),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂θ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂L/∂θ should be exactly 0 (L independent of θ)");
        }

        [TestMethod]
        public void RunningCostGradient_WrtThetaDot_ShouldBeZero()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 3.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.RunningCostReverse(theta, thetadot, u);
            var autoDiffGrad = grad[1]; // ∂L/∂thetadot

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot_perturbed, u),
                thetadot);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂thetadot: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂L/∂thetadot should be exactly 0 (L independent of thetadot)");
        }

        [TestMethod]
        public void RunningCostGradient_WrtControl_ShouldMatchNumerical()
        {
            // Expected behavior: ∂L/∂u = u (derivative of 0.5·u²)

            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 2.0;
            var u = 3.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.RunningCostReverse(theta, thetadot, u);
            var autoDiffGrad = grad[2]; // ∂L/∂u

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                u_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot, u_perturbed),
                u);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂u: AutoDiff should match numerical gradient");

            // Manual verification: ∂L/∂u = u
            Assert.AreEqual(u, autoDiffGrad, Tolerance, "∂L/∂u should be exactly u (derivative of 0.5·u²)");
        }

        [TestMethod]
        public void RunningCostGradient_WrtControl_AtDifferentControls()
        {
            // Test that gradient is linear in u

            var theta = Math.PI / 4;
            var thetadot = 1.0;
            var u1 = 2.0;
            var u2 = 4.0;

            var (_, grad1) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.RunningCostReverse(theta, thetadot, u1);
            var (_, grad2) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.RunningCostReverse(theta, thetadot, u2);

            Assert.AreEqual(u1, grad1[2], Tolerance, "∂L/∂u should equal u1");
            Assert.AreEqual(u2, grad2[2], Tolerance, "∂L/∂u should equal u2");
            Assert.AreEqual(2.0, grad2[2] / grad1[2], 1e-8, "Doubling control should double gradient");
        }

        // ========== TERMINAL COST GRADIENT TESTS ==========
        // Φ = 0

        [TestMethod]
        public void TerminalCostGradient_WrtTheta_ShouldBeZero()
        {
            // Arrange
            var theta = Math.PI;
            var thetadot = 0.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.TerminalCostReverse(theta, thetadot);
            var autoDiffGrad = grad[0]; // ∂Φ/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.TerminalCost(theta_perturbed, thetadot),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂Φ/∂θ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂Φ/∂θ should be exactly 0 (Φ = 0)");
        }

        [TestMethod]
        public void TerminalCostGradient_WrtThetaDot_ShouldBeZero()
        {
            // Arrange
            var theta = Math.PI;
            var thetadot = 0.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.TerminalCostReverse(theta, thetadot);
            var autoDiffGrad = grad[1]; // ∂Φ/∂thetadot

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.TerminalCost(theta, thetadot_perturbed),
                thetadot);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂Φ/∂thetadot: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂Φ/∂thetadot should be exactly 0 (Φ = 0)");
        }

        // ========== COMPREHENSIVE COMPARISON TEST ==========

        [TestMethod]
        public void AllGradientsShouldMatchNumericalGradientsAtVariousStates()
        {
            // Test at multiple state-control combinations to ensure robustness

            var testCases = new[]
            {
                (theta: 0.0, thetadot: 0.0, u: 0.0),           // Bottom, at rest
                (theta: Math.PI, thetadot: 0.0, u: 0.0),       // Top, at rest
                (theta: Math.PI/2, thetadot: 0.0, u: 0.0),     // Horizontal
                (theta: Math.PI/4, thetadot: 1.0, u: 2.0),     // Generic state
                (theta: -Math.PI/6, thetadot: -0.5, u: -3.0),  // Negative values
                (theta: 3*Math.PI/4, thetadot: 2.5, u: 5.0),   // Large values
            };

            foreach (var (theta, thetadot, u) in testCases)
            {
                // Test AngularVelocityRate gradients (most complex)
                var (value, grad) = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(theta, thetadot, u, g, L, m);

                var numGradTheta = NumericalGradient(
                    t => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(t, thetadot, u, g, L, m), theta);
                var numGradThetaDot = NumericalGradient(
                    td => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, td, u, g, L, m), thetadot);
                var numGradU = NumericalGradient(
                    u_p => OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u_p, g, L, m), u);

                Assert.AreEqual(numGradTheta, grad[0], 1e-5,
                    $"θ̈ gradient w.r.t. θ failed at state ({theta:F2}, {thetadot:F2}, {u:F2})");
                Assert.AreEqual(numGradThetaDot, grad[1], 1e-5,
                    $"θ̈ gradient w.r.t. θ̇ failed at state ({theta:F2}, {thetadot:F2}, {u:F2})");
                Assert.AreEqual(numGradU, grad[2], 1e-5,
                    $"θ̈ gradient w.r.t. u failed at state ({theta:F2}, {thetadot:F2}, {u:F2})");

                // Verify function value is correct
                var expectedValue = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u, g, L, m);
                Assert.AreEqual(expectedValue, value, Tolerance,
                    $"θ̈ function value incorrect at state ({theta:F2}, {thetadot:F2}, {u:F2})");
            }
        }
    }
}
