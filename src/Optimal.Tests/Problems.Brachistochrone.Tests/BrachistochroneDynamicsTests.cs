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

namespace Optimal.Problems.Brachistochrone.Tests
{
    /// <summary>
    /// Unit tests for Brachistochrone problem dynamics functions.
    /// Tests the dynamics without AutoDiff gradients.
    /// State: [s (position along diagonal), d (perpendicular distance), v (velocity)]
    /// Control: θ (angle relative to diagonal, in RADIANS)
    /// </summary>
    [TestClass]
    public sealed class BrachistochroneDynamicsTests
    {
        private const double Tolerance = 1e-10;

        // Standard test parameters
        private const double g = 9.81;
        private const double alpha = -Math.PI / 4; // 45° downward diagonal

        [TestMethod]
        public void SRateShouldEqualVelocityTimesCosineThetaWhenThetaIsZero()
        {
            // Arrange: Moving along diagonal (theta = 0)
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;
            var theta = 0.0;

            // Act
            var srate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta, g, alpha);

            // Assert: ṡ = v·cos(0) = v
            Assert.AreEqual(v, srate, Tolerance, "When theta=0, ṡ should equal v");
        }

        [TestMethod]
        public void SRateShouldBeZeroWhenThetaIsNinetyDegrees()
        {
            // Arrange: Moving perpendicular to diagonal
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;
            var theta = Math.PI / 2;

            // Act
            var srate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta, g, alpha);

            // Assert: ṡ = v·cos(π/2) = 0
            Assert.AreEqual(0.0, srate, Tolerance, "When theta=90°, ṡ should be zero");
        }

        [TestMethod]
        public void SRateShouldBeNegativeWhenThetaIsGreaterThanNinetyDegrees()
        {
            // Arrange: Moving backward along diagonal
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;
            var theta = 2.0 * Math.PI / 3; // 120°

            // Act
            var srate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta, g, alpha);

            // Assert: ṡ = v·cos(120°) < 0
            Assert.IsTrue(srate < 0, "When theta > 90°, ṡ should be negative");
        }

        [TestMethod]
        public void SRateShouldBeZeroWhenVelocityIsZero()
        {
            // Arrange
            var s = 0.5;
            var d = 0.1;
            var v = 0.0;
            var theta = Math.PI / 6;

            // Act
            var srate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta, g, alpha);

            // Assert
            Assert.AreEqual(0.0, srate, Tolerance, "When v=0, ṡ should be zero");
        }

        [TestMethod]
        public void DRateShouldEqualVelocityTimesSineThetaWhenThetaIsNinetyDegrees()
        {
            // Arrange: Moving perpendicular to diagonal
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;
            var theta = Math.PI / 2;

            // Act
            var drate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta, g, alpha);

            // Assert: ḋ = v·sin(π/2) = v
            Assert.AreEqual(v, drate, Tolerance, "When theta=90°, ḋ should equal v");
        }

        [TestMethod]
        public void DRateShouldBeZeroWhenThetaIsZero()
        {
            // Arrange: Moving along diagonal
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;
            var theta = 0.0;

            // Act
            var drate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta, g, alpha);

            // Assert: ḋ = v·sin(0) = 0
            Assert.AreEqual(0.0, drate, Tolerance, "When theta=0, ḋ should be zero");
        }

        [TestMethod]
        public void DRateShouldBeNegativeWhenThetaIsNegative()
        {
            // Arrange: Curving toward diagonal (negative perpendicular motion)
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = -Math.PI / 4;

            // Act
            var drate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta, g, alpha);

            // Assert: ḋ = v·sin(-π/4) < 0
            Assert.IsTrue(drate < 0, "When theta is negative, ḋ should be negative");
        }

        [TestMethod]
        public void DRateShouldBeZeroWhenVelocityIsZero()
        {
            // Arrange
            var s = 0.5;
            var d = 0.1;
            var v = 0.0;
            var theta = Math.PI / 6;

            // Act
            var drate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta, g, alpha);

            // Assert
            Assert.AreEqual(0.0, drate, Tolerance, "When v=0, ḋ should be zero");
        }

        [TestMethod]
        public void VRateShouldBePositiveForSteepDownwardPath()
        {
            // Arrange: alpha = -45°, theta = 0° → path angle = -45° (downward)
            // For downward motion, particle should accelerate (v̇ > 0)
            var s = 1.0;
            var d = 0.0;
            var v = 1.0;
            var theta = 0.0;

            // Act
            var vrate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta, g, alpha);

            // Assert: v̇ = -g·sin(α + θ) = -g·sin(-π/4) = -g·(-√2/2) = g·√2/2 > 0
            var expected = -g * Math.Sin(alpha + theta);
            Assert.AreEqual(expected, vrate, Tolerance, "v̇ should equal -g·sin(α + θ)");
            Assert.IsTrue(vrate > 0, "For downward motion along diagonal, velocity should increase (positive acceleration)");
        }

        [TestMethod]
        public void VRateShouldMatchGravityComponentAlongPath()
        {
            // Arrange: Various angles
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;
            var theta1 = 0.0;
            var theta2 = Math.PI / 6;
            var theta3 = -Math.PI / 6;

            // Act
            var vrate1 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta1, g, alpha);
            var vrate2 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta2, g, alpha);
            var vrate3 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta3, g, alpha);

            // Assert: v̇ = -g·sin(α + θ)
            Assert.AreEqual(-g * Math.Sin(alpha + theta1), vrate1, Tolerance);
            Assert.AreEqual(-g * Math.Sin(alpha + theta2), vrate2, Tolerance);
            Assert.AreEqual(-g * Math.Sin(alpha + theta3), vrate3, Tolerance);
        }

        [TestMethod]
        public void VRateShouldBeIndependentOfPositionAlongDiagonal()
        {
            // Arrange: Different s positions
            var s1 = 0.0;
            var s2 = 1.5;
            var d = 0.0;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act
            var vrate1 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s1, d, v, theta, g, alpha);
            var vrate2 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s2, d, v, theta, g, alpha);

            // Assert
            Assert.AreEqual(vrate1, vrate2, Tolerance, "Acceleration should be independent of s position");
        }

        [TestMethod]
        public void VRateShouldBeIndependentOfPerpendicularDistance()
        {
            // Arrange: Different d positions
            var s = 1.0;
            var d1 = 0.0;
            var d2 = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act
            var vrate1 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d1, v, theta, g, alpha);
            var vrate2 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d2, v, theta, g, alpha);

            // Assert
            Assert.AreEqual(vrate1, vrate2, Tolerance, "Acceleration should be independent of d position");
        }

        [TestMethod]
        public void VRateShouldBeIndependentOfCurrentVelocity()
        {
            // Arrange: Different velocities
            var s = 1.0;
            var d = 0.0;
            var v1 = 0.5;
            var v2 = 3.0;
            var theta = Math.PI / 6;

            // Act
            var vrate1 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v1, theta, g, alpha);
            var vrate2 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v2, theta, g, alpha);

            // Assert
            Assert.AreEqual(vrate1, vrate2, Tolerance, "Acceleration should be independent of current velocity");
        }

        [TestMethod]
        public void RunningCostShouldAlwaysBeOne()
        {
            // Arrange: Various states and controls
            var cost1 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.RunningCost(0.0, 0.0, 0.0, 0.0);
            var cost2 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.RunningCost(1.5, 0.5, 2.0, Math.PI / 4);
            var cost3 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.RunningCost(2.0, -0.3, 3.5, -Math.PI / 6);

            // Assert: Running cost is always 1.0 to minimize time (∫1 dt = T)
            Assert.AreEqual(1.0, cost1, Tolerance);
            Assert.AreEqual(1.0, cost2, Tolerance);
            Assert.AreEqual(1.0, cost3, Tolerance);
        }

        [TestMethod]
        public void TerminalCostShouldAlwaysBeOne()
        {
            // Arrange: Various final states
            var cost1 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.TerminalCost(0.0, 0.0, 0.0);
            var cost2 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.TerminalCost(2.83, 0.0, 3.5);
            var cost3 = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.TerminalCost(1.5, 0.1, 2.0);

            // Assert: Terminal cost is always 1.0 (time minimization is via ∂Φ/∂t = 1)
            Assert.AreEqual(1.0, cost1, Tolerance);
            Assert.AreEqual(1.0, cost2, Tolerance);
            Assert.AreEqual(1.0, cost3, Tolerance);
        }

        [TestMethod]
        public void DynamicsShouldProduceReasonableTrajectoryForStraightDescent()
        {
            // Arrange: Simulate straight downward motion along diagonal
            var s = 0.0;
            var d = 0.0;
            var v = 0.0;
            var theta = 0.0; // Along diagonal
            var dt = 0.01;

            // Act: Integrate dynamics for a short time
            for (var i = 0; i < 10; i++)
            {
                var srate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta, g, alpha);
                var drate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta, g, alpha);
                var vrate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta, g, alpha);

                s += srate * dt;
                d += drate * dt;
                v += vrate * dt;
            }

            // Assert: After falling for 0.1s
            Assert.IsTrue(s > 0, "Should have moved forward along diagonal");
            Assert.AreEqual(0.0, d, 1e-6, "Should stay on diagonal when theta=0");
            // Velocity might be positive or negative depending on sign convention
            Assert.AreNotEqual(0.0, v, "Velocity should have changed under gravity");
        }

        [TestMethod]
        public void DynamicsShouldConserveEnergyApproximatelyForFreeFall()
        {
            // This is more of an integration test, but useful for validating physics
            // Energy = 0.5·v² + g·h (kinetic + potential)
            // In (s,d) coordinates with yFinal < 0, we have: h = s·sin(alpha)
            
            // Arrange
            var s = 0.0;
            var d = 0.0;
            var v = 0.0;
            var theta = 0.0;
            var dt = 0.001;
            var totalTime = 0.1;

            var initialEnergy = 0.5 * v * v + g * s * Math.Sin(alpha);

            // Act: Integrate
            var steps = (int)(totalTime / dt);
            for (var i = 0; i < steps; i++)
            {
                var srate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta, g, alpha);
                var drate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta, g, alpha);
                var vrate = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta, g, alpha);

                s += srate * dt;
                d += drate * dt;
                v += vrate * dt;
            }

            var finalEnergy = 0.5 * v * v + g * s * Math.Sin(alpha);

            // Assert: Energy should be approximately conserved (within integration error)
            // Note: This test might fail if there's a sign error in the dynamics
            Assert.AreEqual(initialEnergy, finalEnergy, 0.1, "Energy should be approximately conserved");
        }
    }
}
