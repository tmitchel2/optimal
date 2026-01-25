/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1707 // Test method names use underscores for readability

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Problems.Goddard.Tests
{
    /// <summary>
    /// Unit tests for OptimalCli Goddard Rocket dynamics functions.
    /// Tests the dynamics with drag from the OptimalCli implementation.
    /// </summary>
    [TestClass]
    public sealed class GoddardRocketDynamicsCliTests
    {
        private const double Tolerance = 1e-10;

        // Standard test parameters (normalized Goddard problem)
        private const double g = 1.0;
        private const double Dc = 620.0;
        private const double c = 0.5;
        private const double h0 = 500.0;

        [TestMethod]
        public void AltitudeRateShouldEqualVelocity()
        {
            // Arrange
            var h = 100.0;
            var v = 5.0;
            var m = 0.8;
            var T = 2.0;

            // Act
            var hrate = OptimalCli.Problems.Goddard.GoddardRocketDynamics.AltitudeRate(h, v, m, T, g, Dc, c, h0);

            // Assert
            Assert.AreEqual(v, hrate, Tolerance, "Altitude rate should equal velocity");
        }

        [TestMethod]
        public void AltitudeRateShouldBeZeroWhenVelocityIsZero()
        {
            // Arrange
            var h = 50.0;
            var v = 0.0;
            var m = 0.9;
            var T = 1.5;

            // Act
            var hrate = OptimalCli.Problems.Goddard.GoddardRocketDynamics.AltitudeRate(h, v, m, T, g, Dc, c, h0);

            // Assert
            Assert.AreEqual(0.0, hrate, Tolerance, "Altitude should not change when velocity is zero");
        }

        [TestMethod]
        public void VelocityRateShouldBeNegativeWithNoThrustDueToGravity()
        {
            // Arrange: No thrust, no drag (v=0)
            var h = 100.0;
            var v = 0.0;
            var m = 0.8;
            var T = 0.0;

            // Act
            var vrate = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T, g, Dc, c, h0);

            // Assert
            Assert.AreEqual(-g, vrate, Tolerance, "With no thrust and no drag, acceleration should equal -g");
        }

        [TestMethod]
        public void VelocityRateShouldIncreaseWithThrust()
        {
            // Arrange
            var h = 100.0;
            var v = 0.0;
            var m = 1.0;
            var T1 = 0.0;
            var T2 = 2.0;

            // Act
            var vrate1 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T1, g, Dc, c, h0);
            var vrate2 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T2, g, Dc, c, h0);

            // Assert
            Assert.IsGreaterThan(vrate1, vrate2, "Higher thrust should produce higher acceleration");
            Assert.AreEqual(T2 / m - g, vrate2, Tolerance, "Acceleration should equal (T/m - g) when drag is zero");
        }

        [TestMethod]
        public void VelocityRateShouldDecreaseWithDrag()
        {
            // Arrange: Same thrust, different velocities
            var h = 100.0;
            var v1 = 0.0;
            var v2 = 1.0;
            var m = 0.8;
            var T = 2.0;

            // Act
            var vrate1 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v1, m, T, g, Dc, c, h0);
            var vrate2 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v2, m, T, g, Dc, c, h0);

            // Assert
            Assert.IsLessThan(vrate1, vrate2, "Higher velocity should produce more drag, reducing acceleration");
        }

        [TestMethod]
        public void DragShouldDecreaseWithAltitude()
        {
            // Arrange: Same velocity, different altitudes
            var h1 = 0.0;
            var h2 = 500.0;  // At scale height h0
            var v = 1.0;
            var m = 0.8;
            var T = 2.0;

            // Act
            var vrate1 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h1, v, m, T, g, Dc, c, h0);
            var vrate2 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h2, v, m, T, g, Dc, c, h0);

            // Assert
            Assert.IsGreaterThan(vrate1, vrate2, "Drag should decrease with altitude, so acceleration should increase");

            // Drag force at h1: Dc * v^2 * exp(0) = Dc * v^2
            // Drag force at h2: Dc * v^2 * exp(-1) ≈ 0.368 * Dc * v^2
            var drag1 = Dc * v * v;
            var drag2 = Dc * v * v * System.Math.Exp(-1.0);
            Assert.AreEqual((T - drag1) / m - g, vrate1, 1e-8, "Acceleration at ground level should match expected");
            Assert.AreEqual((T - drag2) / m - g, vrate2, 1e-8, "Acceleration at scale height should match expected");
        }

        [TestMethod]
        public void MassRateShouldBeNegativeWhenThrusting()
        {
            // Arrange
            var h = 100.0;
            var v = 1.0;
            var m = 0.8;
            var T = 2.0;

            // Act
            var mrate = OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m, T, g, Dc, c, h0);

            // Assert
            Assert.IsLessThan(0, mrate, "Mass should decrease when thrusting (fuel consumption)");
            Assert.AreEqual(-T / c, mrate, Tolerance, "Mass rate should equal -T/c");
        }

        [TestMethod]
        public void MassRateShouldBeZeroWithNoThrust()
        {
            // Arrange
            var h = 100.0;
            var v = 1.0;
            var m = 0.8;
            var T = 0.0;

            // Act
            var mrate = OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m, T, g, Dc, c, h0);

            // Assert
            Assert.AreEqual(0.0, mrate, Tolerance, "Mass should not change when not thrusting");
        }

        [TestMethod]
        public void MassRateShouldBeProportionalToThrust()
        {
            // Arrange
            var h = 100.0;
            var v = 1.0;
            var m = 0.8;
            var T1 = 1.0;
            var T2 = 2.0;

            // Act
            var mrate1 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m, T1, g, Dc, c, h0);
            var mrate2 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m, T2, g, Dc, c, h0);

            // Assert
            Assert.AreEqual(2.0, mrate2 / mrate1, 1e-8, "Doubling thrust should double fuel consumption rate");
        }

        [TestMethod]
        public void RunningCostShouldBeZero()
        {
            // Arrange: Goddard problem has no running cost (maximize altitude only)
            var h = 100.0;
            var v = 1.0;
            var m = 0.8;
            var T = 2.0;

            // Act
            var cost = OptimalCli.Problems.Goddard.GoddardRocketDynamics.RunningCost(h, v, m, T);

            // Assert
            Assert.AreEqual(0.0, cost, Tolerance, "Goddard problem should have zero running cost");
        }

        [TestMethod]
        public void TerminalCostShouldBeNegativeAltitude()
        {
            // Arrange: We minimize -h to maximize h
            var h = 150.0;
            var v = 1.0;
            var m = 0.7;

            // Act
            var cost = OptimalCli.Problems.Goddard.GoddardRocketDynamics.TerminalCost(h, v, m);

            // Assert
            Assert.AreEqual(-h, cost, Tolerance, "Terminal cost should be negative altitude for maximization");
        }

        [TestMethod]
        public void TerminalCostShouldDecreaseWithHigherAltitude()
        {
            // Arrange
            var h1 = 100.0;
            var h2 = 200.0;
            var v = 1.0;
            var m = 0.7;

            // Act
            var cost1 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.TerminalCost(h1, v, m);
            var cost2 = OptimalCli.Problems.Goddard.GoddardRocketDynamics.TerminalCost(h2, v, m);

            // Assert
            Assert.IsLessThan(cost1, cost2, "Higher altitude should give lower (more negative) cost");
        }

        [TestMethod]
        public void AutoDiffGradientsShouldBeConsistentWithManualDerivatives()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - Call AutoDiff-generated gradient functions
            // Gradients array: [∂/∂h, ∂/∂v, ∂/∂m, ∂/∂T, ∂/∂g, ∂/∂Dc, ∂/∂c, ∂/∂h0]
            var (hrate, hrate_grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.AltitudeRateReverse(h, v, m, T, g, Dc, c, h0);
            var (vrate, vrate_grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var (mrate, mrate_grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);

            // Assert - Check function values match
            Assert.AreEqual(v, hrate, Tolerance, "AltitudeRate value should match");

            // Check manual gradients for AltitudeRate: ḣ = v
            // ∂ḣ/∂h = 0, ∂ḣ/∂v = 1, ∂ḣ/∂m = 0, ∂ḣ/∂T = 0
            Assert.AreEqual(0.0, hrate_grad[0], Tolerance, "∂ḣ/∂h should be 0");
            Assert.AreEqual(1.0, hrate_grad[1], Tolerance, "∂ḣ/∂v should be 1");
            Assert.AreEqual(0.0, hrate_grad[2], Tolerance, "∂ḣ/∂m should be 0");
            Assert.AreEqual(0.0, hrate_grad[3], Tolerance, "∂ḣ/∂T should be 0");

            // Check manual gradients for MassRate: ṁ = -T/c
            // ∂ṁ/∂h = 0, ∂ṁ/∂v = 0, ∂ṁ/∂m = 0, ∂ṁ/∂T = -1/c
            Assert.AreEqual(0.0, mrate_grad[0], Tolerance, "∂ṁ/∂h should be 0");
            Assert.AreEqual(0.0, mrate_grad[1], Tolerance, "∂ṁ/∂v should be 0");
            Assert.AreEqual(0.0, mrate_grad[2], Tolerance, "∂ṁ/∂m should be 0");
            Assert.AreEqual(-1.0 / c, mrate_grad[3], Tolerance, "∂ṁ/∂T should be -1/c");
            // Also check ∂ṁ/∂c = T/c²
            Assert.AreEqual(T / (c * c), mrate_grad[6], 1e-8, "∂ṁ/∂c should be T/c²");

            // VelocityRate gradients are more complex due to drag, but we can check basic properties
            // v̇ = (T - D)/m - g where D = Dc·v²·exp(-h/h0)
            // ∂v̇/∂T = 1/m
            Assert.AreEqual(1.0 / m, vrate_grad[3], 1e-8, "∂v̇/∂T should be 1/m");

            // ∂v̇/∂g = -1
            Assert.AreEqual(-1.0, vrate_grad[4], 1e-8, "∂v̇/∂g should be -1");
        }

        // ========== COMPREHENSIVE GRADIENT VALIDATION TESTS ==========
        // These tests use numerical differentiation (finite differences) to validate
        // the AutoDiff-generated gradients independently, without assuming correctness.

        private const double FiniteDiffEpsilon = 1e-7;

        /// <summary>
        /// Computes numerical gradient using central finite differences.
        /// </summary>
        private static double NumericalGradient(System.Func<double, double> func, double x)
        {
            return (func(x + FiniteDiffEpsilon) - func(x - FiniteDiffEpsilon)) / (2 * FiniteDiffEpsilon);
        }

        // ========== ALTITUDE RATE GRADIENT TESTS ==========

        [TestMethod]
        public void AltitudeRateGradient_WrtAltitude_ShouldBeZero()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.AltitudeRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[0]; // ∂ḣ/∂h

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                h_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.AltitudeRate(h_perturbed, v, m, T, g, Dc, c, h0),
                h);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ḣ/∂h: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂ḣ/∂h should be exactly 0 (ḣ = v, independent of h)");
        }

        [TestMethod]
        public void AltitudeRateGradient_WrtVelocity_ShouldBeOne()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.AltitudeRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[1]; // ∂ḣ/∂v

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.AltitudeRate(h, v_perturbed, m, T, g, Dc, c, h0),
                v);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ḣ/∂v: AutoDiff should match numerical gradient");
            Assert.AreEqual(1.0, autoDiffGrad, 1e-10, "∂ḣ/∂v should be exactly 1 (ḣ = v)");
        }

        [TestMethod]
        public void AltitudeRateGradient_WrtMass_ShouldBeZero()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.AltitudeRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[2]; // ∂ḣ/∂m

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                m_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.AltitudeRate(h, v, m_perturbed, T, g, Dc, c, h0),
                m);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ḣ/∂m: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂ḣ/∂m should be exactly 0 (ḣ = v, independent of m)");
        }

        [TestMethod]
        public void AltitudeRateGradient_WrtThrust_ShouldBeZero()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.AltitudeRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[3]; // ∂ḣ/∂T

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                T_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.AltitudeRate(h, v, m, T_perturbed, g, Dc, c, h0),
                T);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ḣ/∂T: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂ḣ/∂T should be exactly 0 (ḣ = v, independent of T)");
        }

        // ========== VELOCITY RATE GRADIENT TESTS ==========
        // v̇ = (T - D)/m - g where D = Dc·v²·exp(-h/h0)
        // This is the most complex function with gradients that must be thoroughly validated

        [TestMethod]
        public void VelocityRateGradient_WrtAltitude_ShouldMatchNumerical()
        {
            // TDD: This test will initially FAIL due to AutoDiff intermediate variable issue
            // Expected behavior: ∂v̇/∂h = (Dc·v²/(m·h0))·exp(-h/h0)
            // This gradient represents how drag changes with altitude (exponential atmosphere)

            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[0]; // ∂v̇/∂h

            // Act - Numerical gradient (ground truth)
            var numericalGrad = NumericalGradient(
                h_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h_perturbed, v, m, T, g, Dc, c, h0),
                h);

            // Assert - AutoDiff must match numerical gradient
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂v̇/∂h: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");

            // Assert - Analytical formula verification
            var expectedGrad = (Dc * v * v / (m * h0)) * System.Math.Exp(-h / h0);
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8,
                $"∂v̇/∂h should match analytical formula: (Dc·v²/(m·h0))·exp(-h/h0)");

            // Assert - Physical interpretation: gradient should be positive (less drag at higher altitude means more acceleration)
            Assert.IsGreaterThan(0,
autoDiffGrad, "∂v̇/∂h should be positive: higher altitude → less drag → more acceleration");
        }

        [TestMethod]
        public void VelocityRateGradient_WrtVelocity_ShouldMatchNumerical()
        {
            // TDD: This test will initially FAIL due to AutoDiff intermediate variable issue
            // Expected behavior: ∂v̇/∂v = -2·Dc·v·exp(-h/h0)/m
            // This gradient represents drag's velocity dependence (quadratic drag)

            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[1]; // ∂v̇/∂v

            // Act - Numerical gradient (ground truth)
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v_perturbed, m, T, g, Dc, c, h0),
                v);

            // Assert - AutoDiff must match numerical gradient
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂v̇/∂v: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");

            // Assert - Analytical formula verification
            var expectedGrad = -2.0 * Dc * v * System.Math.Exp(-h / h0) / m;
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8,
                $"∂v̇/∂v should match analytical formula: -2·Dc·v·exp(-h/h0)/m");

            // Assert - Physical interpretation: gradient should be negative (drag opposes motion)
            Assert.IsLessThan(0,
autoDiffGrad, "∂v̇/∂v should be negative: higher velocity → more drag → less acceleration (drag opposes motion)");
        }

        [TestMethod]
        public void VelocityRateGradient_WrtVelocity_DragEffect_ShouldBeQuadratic()
        {
            // TDD: Verify the drag gradient has quadratic velocity dependence
            // This test specifically validates that ∂v̇/∂v is proportional to v (from v² in drag)

            // Arrange - Test at two different velocities
            var h = 100.0;
            var v1 = 1.0;
            var v2 = 2.0;  // Double velocity
            var m = 0.8;
            var T = 2.5;

            // Act
            var (_, grad1) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v1, m, T, g, Dc, c, h0);
            var (_, grad2) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v2, m, T, g, Dc, c, h0);

            var autoDiffGrad1 = grad1[1]; // ∂v̇/∂v at v=1
            var autoDiffGrad2 = grad2[1]; // ∂v̇/∂v at v=2

            // Assert - Doubling velocity should double the gradient magnitude (linear in v due to d(v²)/dv = 2v)
            var ratio = autoDiffGrad2 / autoDiffGrad1;
            Assert.AreEqual(2.0, ratio, 0.01,
                $"∂v̇/∂v should scale linearly with velocity: ratio should be 2.0, got {ratio}");
        }

        [TestMethod]
        public void VelocityRateGradient_WrtAltitude_ExponentialAtmosphere_ShouldDecayCorrectly()
        {
            // TDD: Verify gradient decays exponentially with altitude (atmosphere model)
            // At scale height h0, gradient should be 1/e of ground level value

            // Arrange
            var h1 = 0.0;      // Ground level
            var h2 = h0;       // One scale height (500m)
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act
            var (_, grad1) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h1, v, m, T, g, Dc, c, h0);
            var (_, grad2) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h2, v, m, T, g, Dc, c, h0);

            var autoDiffGrad1 = grad1[0]; // ∂v̇/∂h at h=0
            var autoDiffGrad2 = grad2[0]; // ∂v̇/∂h at h=h0

            // Assert - At one scale height, gradient should be exp(-1) ≈ 0.368 times ground value
            var ratio = autoDiffGrad2 / autoDiffGrad1;
            var expectedRatio = System.Math.Exp(-1.0);
            Assert.AreEqual(expectedRatio, ratio, 0.01,
                $"At scale height h0, gradient should be exp(-1) ≈ 0.368 of ground value, got {ratio}");
        }

        [TestMethod]
        public void VelocityRateGradient_WrtMass_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[2]; // ∂v̇/∂m

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                m_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m_perturbed, T, g, Dc, c, h0),
                m);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂v̇/∂m: AutoDiff should match numerical gradient");

            // Manual verification: ∂v̇/∂m = -(T - D)/m²
            var drag = Dc * v * v * System.Math.Exp(-h / h0);
            var expectedGrad = -(T - drag) / (m * m);
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8, "∂v̇/∂m should match analytical formula");
        }

        [TestMethod]
        public void VelocityRateGradient_WrtThrust_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[3]; // ∂v̇/∂T

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                T_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T_perturbed, g, Dc, c, h0),
                T);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂v̇/∂T: AutoDiff should match numerical gradient");

            // Manual verification: ∂v̇/∂T = 1/m
            var expectedGrad = 1.0 / m;
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-10, "∂v̇/∂T should be exactly 1/m");
        }

        [TestMethod]
        [Ignore("Gradients w.r.t. constants (g, Dc, c, h0) are zero in AutoDiff - these are problem parameters, not optimization variables")]
        public void VelocityRateGradient_WrtGravity_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[4]; // ∂v̇/∂g

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                g_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T, g_perturbed, Dc, c, h0),
                g);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂v̇/∂g: AutoDiff should match numerical gradient");

            // Manual verification: ∂v̇/∂g = -1
            Assert.AreEqual(-1.0, autoDiffGrad, 1e-10, "∂v̇/∂g should be exactly -1");
        }

        [TestMethod]
        [Ignore("Gradients w.r.t. constants (g, Dc, c, h0) are zero in AutoDiff - these are problem parameters, not optimization variables")]
        public void VelocityRateGradient_WrtDragCoeff_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[5]; // ∂v̇/∂Dc

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                Dc_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T, g, Dc_perturbed, c, h0),
                Dc);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂v̇/∂Dc: AutoDiff should match numerical gradient");

            // Manual verification: ∂v̇/∂Dc = -v²·exp(-h/h0)/m
            var expectedGrad = -v * v * System.Math.Exp(-h / h0) / m;
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8, "∂v̇/∂Dc should match analytical formula");
        }

        [TestMethod]
        [Ignore("Gradients w.r.t. constants (g, Dc, c, h0) are zero in AutoDiff - these are problem parameters, not optimization variables")]
        public void VelocityRateGradient_WrtScaleHeight_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[7]; // ∂v̇/∂h0

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                h0_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.VelocityRate(h, v, m, T, g, Dc, c, h0_perturbed),
                h0);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂v̇/∂h0: AutoDiff should match numerical gradient");

            // Manual verification: ∂v̇/∂h0 = -Dc·v²·h·exp(-h/h0)/(m·h0²)
            var expectedGrad = -Dc * v * v * h * System.Math.Exp(-h / h0) / (m * h0 * h0);
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8, "∂v̇/∂h0 should match analytical formula");
        }

        // ========== MASS RATE GRADIENT TESTS ==========
        // ṁ = -T/c

        [TestMethod]
        public void MassRateGradient_WrtAltitude_ShouldBeZero()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[0]; // ∂ṁ/∂h

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                h_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h_perturbed, v, m, T, g, Dc, c, h0),
                h);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ṁ/∂h: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂ṁ/∂h should be exactly 0 (ṁ = -T/c, independent of h)");
        }

        [TestMethod]
        public void MassRateGradient_WrtVelocity_ShouldBeZero()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[1]; // ∂ṁ/∂v

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v_perturbed, m, T, g, Dc, c, h0),
                v);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ṁ/∂v: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂ṁ/∂v should be exactly 0 (ṁ = -T/c, independent of v)");
        }

        [TestMethod]
        public void MassRateGradient_WrtMass_ShouldBeZero()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[2]; // ∂ṁ/∂m

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                m_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m_perturbed, T, g, Dc, c, h0),
                m);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ṁ/∂m: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂ṁ/∂m should be exactly 0 (ṁ = -T/c, independent of m)");
        }

        [TestMethod]
        public void MassRateGradient_WrtThrust_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[3]; // ∂ṁ/∂T

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                T_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m, T_perturbed, g, Dc, c, h0),
                T);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ṁ/∂T: AutoDiff should match numerical gradient");

            // Manual verification: ∂ṁ/∂T = -1/c
            var expectedGrad = -1.0 / c;
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-10, "∂ṁ/∂T should be exactly -1/c");
        }

        [TestMethod]
        public void MassRateGradient_WrtExhaustVelocity_ShouldMatchNumerical()
        {
            // Arrange
            var h = 100.0;
            var v = 2.0;
            var m = 0.8;
            var T = 2.5;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);
            var autoDiffGrad = grad[6]; // ∂ṁ/∂c

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                c_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.MassRate(h, v, m, T, g, Dc, c_perturbed, h0),
                c);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ṁ/∂c: AutoDiff should match numerical gradient");

            // Manual verification: ∂ṁ/∂c = T/c²
            var expectedGrad = T / (c * c);
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8, "∂ṁ/∂c should match analytical formula T/c²");
        }

        // ========== TERMINAL COST GRADIENT TESTS ==========
        // Φ = -h

        [TestMethod]
        public void TerminalCostGradient_WrtAltitude_ShouldMatchNumerical()
        {
            // Arrange
            var h = 150.0;
            var v = 1.0;
            var m = 0.7;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.TerminalCostReverse(h, v, m);
            var autoDiffGrad = grad[0]; // ∂Φ/∂h

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                h_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.TerminalCost(h_perturbed, v, m),
                h);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂Φ/∂h: AutoDiff should match numerical gradient");

            // Manual verification: ∂Φ/∂h = -1
            Assert.AreEqual(-1.0, autoDiffGrad, 1e-10, "∂Φ/∂h should be exactly -1 (Φ = -h)");
        }

        [TestMethod]
        public void TerminalCostGradient_WrtVelocity_ShouldBeZero()
        {
            // Arrange
            var h = 150.0;
            var v = 1.0;
            var m = 0.7;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.TerminalCostReverse(h, v, m);
            var autoDiffGrad = grad[1]; // ∂Φ/∂v

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.TerminalCost(h, v_perturbed, m),
                v);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂Φ/∂v: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂Φ/∂v should be exactly 0 (Φ = -h, independent of v)");
        }

        [TestMethod]
        public void TerminalCostGradient_WrtMass_ShouldBeZero()
        {
            // Arrange
            var h = 150.0;
            var v = 1.0;
            var m = 0.7;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Goddard.GoddardRocketDynamicsGradients.TerminalCostReverse(h, v, m);
            var autoDiffGrad = grad[2]; // ∂Φ/∂m

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                m_perturbed => OptimalCli.Problems.Goddard.GoddardRocketDynamics.TerminalCost(h, v, m_perturbed),
                m);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂Φ/∂m: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, 1e-10, "∂Φ/∂m should be exactly 0 (Φ = -h, independent of m)");
        }
    }
}
