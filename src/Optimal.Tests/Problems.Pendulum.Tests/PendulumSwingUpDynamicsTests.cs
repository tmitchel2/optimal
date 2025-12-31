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
    /// Unit tests for Pendulum swing-up dynamics functions.
    /// Tests the dynamics without AutoDiff gradients.
    /// </summary>
    [TestClass]
    public sealed class PendulumSwingUpDynamicsTests
    {
        private const double Tolerance = 1e-10;

        // Standard test parameters (pendulum swing-up problem)
        private const double g = 9.81;
        private const double L = 1.0;
        private const double m = 1.0;

        [TestMethod]
        public void AngleRateShouldEqualAngularVelocity()
        {
            // Arrange
            var theta = Math.PI / 4; // 45 degrees
            var thetadot = 2.0;
            var u = 0.0;

            // Act
            var angleRate = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngleRate(theta, thetadot, u, g, L, m);

            // Assert
            Assert.AreEqual(thetadot, angleRate, Tolerance, "Angle rate should equal angular velocity");
        }

        [TestMethod]
        public void AngleRateShouldBeZeroWhenAngularVelocityIsZero()
        {
            // Arrange
            var theta = Math.PI / 2;
            var thetadot = 0.0;
            var u = 1.5;

            // Act
            var angleRate = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngleRate(theta, thetadot, u, g, L, m);

            // Assert
            Assert.AreEqual(0.0, angleRate, Tolerance, "Angle should not change when angular velocity is zero");
        }

        [TestMethod]
        public void AngularVelocityRateShouldBeZeroAtBottomWithNoTorque()
        {
            // Arrange: Hanging straight down (equilibrium), no torque
            var theta = 0.0;
            var thetadot = 0.0;
            var u = 0.0;

            // Act
            var angularAccel = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u, g, L, m);

            // Assert
            Assert.AreEqual(0.0, angularAccel, Tolerance, "At bottom with no torque and no velocity, angular acceleration should be zero (equilibrium)");
        }

        [TestMethod]
        public void AngularVelocityRateShouldBeMaximumAtHorizontal()
        {
            // Arrange: Horizontal position (θ = π/2), no torque
            var theta = Math.PI / 2;
            var thetadot = 0.0;
            var u = 0.0;

            // Act
            var angularAccel = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u, g, L, m);

            // Assert
            // At horizontal, gravity creates maximum restoring torque: θ̈ = -g/L·sin(π/2) = -g/L
            var expectedAccel = -g / L;
            Assert.AreEqual(expectedAccel, angularAccel, 1e-8, "At horizontal position, gravity creates maximum restoring acceleration");
        }

        [TestMethod]
        public void AngularVelocityRateShouldBeZeroAtTopWithNoTorque()
        {
            // Arrange: Inverted upright (unstable equilibrium), no torque
            var theta = Math.PI;
            var thetadot = 0.0;
            var u = 0.0;

            // Act
            var angularAccel = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u, g, L, m);

            // Assert
            Assert.AreEqual(0.0, angularAccel, 1e-8, "At inverted position with no torque, angular acceleration should be zero (unstable equilibrium)");
        }

        [TestMethod]
        public void AngularVelocityRateShouldIncreaseWithPositiveTorque()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 0.5;
            var u1 = 0.0;
            var u2 = 5.0;

            // Act
            var angularAccel1 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u1, g, L, m);
            var angularAccel2 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u2, g, L, m);

            // Assert
            Assert.IsTrue(angularAccel2 > angularAccel1, "Positive torque should increase angular acceleration");
            var expectedIncrease = u2 / (m * L * L);
            Assert.AreEqual(expectedIncrease, angularAccel2 - angularAccel1, 1e-8, "Torque contribution should be u/(m·L²)");
        }

        [TestMethod]
        public void AngularVelocityRateShouldDecreaseWithNegativeTorque()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 0.5;
            var u1 = 0.0;
            var u2 = -5.0;

            // Act
            var angularAccel1 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u1, g, L, m);
            var angularAccel2 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta, thetadot, u2, g, L, m);

            // Assert
            Assert.IsTrue(angularAccel2 < angularAccel1, "Negative torque should decrease angular acceleration");
        }

        [TestMethod]
        public void GravitationalTorqueDirectionShouldBeCorrect()
        {
            // Arrange: Test positive and negative angles
            var theta1 = Math.PI / 6;  // Positive angle (counterclockwise from down)
            var theta2 = -Math.PI / 6; // Negative angle (clockwise from down)
            var thetadot = 0.0;
            var u = 0.0;

            // Act
            var angularAccel1 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta1, thetadot, u, g, L, m);
            var angularAccel2 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.AngularVelocityRate(theta2, thetadot, u, g, L, m);

            // Assert
            Assert.IsTrue(angularAccel1 < 0, "For positive angle, gravity should pull back toward equilibrium (negative acceleration)");
            Assert.IsTrue(angularAccel2 > 0, "For negative angle, gravity should pull back toward equilibrium (positive acceleration)");
            Assert.AreEqual(-angularAccel1, angularAccel2, 1e-8, "Gravitational effect should be symmetric about equilibrium");
        }

        [TestMethod]
        public void RunningCostShouldBeQuadraticInControl()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 1.0;
            var u1 = 2.0;
            var u2 = 4.0;

            // Act
            var cost1 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot, u1);
            var cost2 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot, u2);

            // Assert
            Assert.AreEqual(0.5 * u1 * u1, cost1, Tolerance, "Running cost should be 0.5·u²");
            Assert.AreEqual(0.5 * u2 * u2, cost2, Tolerance, "Running cost should be 0.5·u²");
            Assert.AreEqual(4.0, cost2 / cost1, 1e-8, "Doubling control should quadruple cost");
        }

        [TestMethod]
        public void RunningCostShouldBeZeroWithNoControl()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 1.0;
            var u = 0.0;

            // Act
            var cost = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot, u);

            // Assert
            Assert.AreEqual(0.0, cost, Tolerance, "Running cost should be zero when no control is applied");
        }

        [TestMethod]
        public void RunningCostShouldBeSameForPositiveAndNegativeTorque()
        {
            // Arrange
            var theta = Math.PI / 4;
            var thetadot = 1.0;
            var u1 = 3.0;
            var u2 = -3.0;

            // Act
            var cost1 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot, u1);
            var cost2 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.RunningCost(theta, thetadot, u2);

            // Assert
            Assert.AreEqual(cost1, cost2, Tolerance, "Cost should be same for positive and negative torque of same magnitude");
        }

        [TestMethod]
        public void TerminalCostShouldBeZero()
        {
            // Arrange: Pendulum swing-up has no terminal cost (only running cost)
            var theta = Math.PI; // At target
            var thetadot = 0.0;

            // Act
            var cost = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.TerminalCost(theta, thetadot);

            // Assert
            Assert.AreEqual(0.0, cost, Tolerance, "Pendulum swing-up should have zero terminal cost");
        }

        [TestMethod]
        public void TerminalCostShouldBeIndependentOfState()
        {
            // Arrange: Test at different states
            var cost1 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.TerminalCost(0.0, 0.0);
            var cost2 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.TerminalCost(Math.PI, 0.0);
            var cost3 = OptimalCli.Problems.Pendulum.PendulumSwingUpDynamics.TerminalCost(Math.PI / 2, 2.0);

            // Assert
            Assert.AreEqual(0.0, cost1, Tolerance);
            Assert.AreEqual(0.0, cost2, Tolerance);
            Assert.AreEqual(0.0, cost3, Tolerance);
            Assert.AreEqual(cost1, cost2, Tolerance);
            Assert.AreEqual(cost1, cost3, Tolerance);
        }
    }
}
