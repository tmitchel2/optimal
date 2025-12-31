/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

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
            Assert.IsTrue(vrate2 > vrate1, "Higher thrust should produce higher acceleration");
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
            Assert.IsTrue(vrate2 < vrate1, "Higher velocity should produce more drag, reducing acceleration");
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
            Assert.IsTrue(vrate2 > vrate1, "Drag should decrease with altitude, so acceleration should increase");

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
            Assert.IsTrue(mrate < 0, "Mass should decrease when thrusting (fuel consumption)");
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
            Assert.IsTrue(cost2 < cost1, "Higher altitude should give lower (more negative) cost");
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
    }
}
