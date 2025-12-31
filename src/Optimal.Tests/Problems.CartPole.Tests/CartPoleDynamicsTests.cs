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

namespace Optimal.Problems.CartPole.Tests
{
    /// <summary>
    /// Unit tests for Cart-Pole dynamics functions.
    /// Tests the dynamics without AutoDiff gradients.
    /// </summary>
    [TestClass]
    public sealed class CartPoleDynamicsTests
    {
        private const double Tolerance = 1e-10;

        // Standard test parameters (cart-pole problem)
        private const double M = 1.0;   // Cart mass (kg)
        private const double m = 0.1;   // Pole mass (kg)
        private const double L = 2.0;   // Pole length (m)
        private const double g = 9.81;  // Gravity (m/s²)

        // ========== POSITION RATE TESTS ==========

        [TestMethod]
        public void XRateShouldEqualCartVelocity()
        {
            // Arrange
            var x = 0.5;
            var xdot = 2.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act
            var xrate = OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(xdot, xrate, Tolerance, "Cart position rate should equal cart velocity");
        }

        [TestMethod]
        public void XRateShouldBeZeroWhenCartVelocityIsZero()
        {
            // Arrange
            var x = 0.5;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act
            var xrate = OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(0.0, xrate, Tolerance, "Cart position should not change when velocity is zero");
        }

        [TestMethod]
        public void XRateShouldBeIndependentOfOtherStates()
        {
            // Arrange
            var xdot = 1.5;
            var F = 1.0;

            // Test with different x, theta, thetadot
            var rate1 = OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(0.0, xdot, 0.0, 0.0, F, M, m, L, g);
            var rate2 = OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(1.0, xdot, 0.2, 0.5, F, M, m, L, g);
            var rate3 = OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(-0.5, xdot, -0.1, -0.3, F, M, m, L, g);

            // Assert
            Assert.AreEqual(xdot, rate1, Tolerance);
            Assert.AreEqual(xdot, rate2, Tolerance);
            Assert.AreEqual(xdot, rate3, Tolerance);
            Assert.AreEqual(rate1, rate2, Tolerance, "XRate should depend only on xdot");
            Assert.AreEqual(rate1, rate3, Tolerance, "XRate should depend only on xdot");
        }

        // ========== CART ACCELERATION TESTS ==========

        [TestMethod]
        public void XddotRateShouldIncreaseWithPositiveForce()
        {
            // Arrange: Same state, different forces
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.0;
            var thetadot = 0.0;
            var F1 = 0.0;
            var F2 = 1.0;

            // Act
            var xddot1 = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot, F1, M, m, L, g);
            var xddot2 = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot, F2, M, m, L, g);

            // Assert
            Assert.IsTrue(xddot2 > xddot1, "Positive force should increase cart acceleration");
        }

        [TestMethod]
        public void XddotRateShouldBeZeroWhenBalancedWithNoForce()
        {
            // Arrange: Cart at rest, pole perfectly upright, no force
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.0;  // Upright
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var xddot = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(0.0, xddot, 1e-8, "Cart should have no acceleration when balanced with no force");
        }

        [TestMethod]
        public void XddotRateShouldBeAffectedByPoleAngularVelocity()
        {
            // Arrange: Pole rotating affects cart through centrifugal force
            var x = 0.0;
            var xdot = 0.0;
            var theta = Math.PI / 4;  // 45 degrees
            var thetadot1 = 0.0;
            var thetadot2 = 1.0;
            var F = 0.0;

            // Act
            var xddot1 = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot1, F, M, m, L, g);
            var xddot2 = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot2, F, M, m, L, g);

            // Assert
            Assert.AreNotEqual(xddot1, xddot2, "Pole angular velocity should affect cart acceleration");
            // With positive thetadot and positive theta (leaning right), centrifugal force pushes cart right
            Assert.IsTrue(xddot2 > xddot1, "Positive angular velocity should increase cart acceleration in direction of lean");
        }

        [TestMethod]
        public void XddotRateShouldDependOnPoleAngle()
        {
            // Arrange: Different pole angles
            var x = 0.0;
            var xdot = 0.0;
            var theta1 = 0.0;      // Upright
            var theta2 = 0.1;      // Slight lean
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var xddot1 = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta1, thetadot, F, M, m, L, g);
            var xddot2 = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta2, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreNotEqual(xddot1, xddot2, "Pole angle should affect cart acceleration");
        }

        // ========== ANGLE RATE TESTS ==========

        [TestMethod]
        public void ThetaRateShouldEqualPoleAngularVelocity()
        {
            // Arrange
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 2.0;
            var F = 1.0;

            // Act
            var thetarate = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(thetadot, thetarate, Tolerance, "Pole angle rate should equal pole angular velocity");
        }

        [TestMethod]
        public void ThetaRateShouldBeZeroWhenAngularVelocityIsZero()
        {
            // Arrange
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 0.0;
            var F = 1.0;

            // Act
            var thetarate = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(0.0, thetarate, Tolerance, "Pole angle should not change when angular velocity is zero");
        }

        [TestMethod]
        public void ThetaRateShouldBeIndependentOfOtherStates()
        {
            // Arrange
            var thetadot = 1.5;
            var F = 1.0;

            // Test with different x, xdot, theta
            var rate1 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(0.0, 0.0, 0.0, thetadot, F, M, m, L, g);
            var rate2 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(1.0, 0.5, 0.2, thetadot, F, M, m, L, g);
            var rate3 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(-0.5, -0.3, -0.1, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(thetadot, rate1, Tolerance);
            Assert.AreEqual(thetadot, rate2, Tolerance);
            Assert.AreEqual(thetadot, rate3, Tolerance);
            Assert.AreEqual(rate1, rate2, Tolerance, "ThetaRate should depend only on thetadot");
            Assert.AreEqual(rate1, rate3, Tolerance, "ThetaRate should depend only on thetadot");
        }

        // ========== ANGULAR ACCELERATION TESTS ==========

        [TestMethod]
        public void ThetaddotRateShouldBeZeroWhenBalancedUpright()
        {
            // Arrange: Pole perfectly upright, no force, no motion
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.0;
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var thetaddot = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(0.0, thetaddot, 1e-8, "Pole should have no angular acceleration when perfectly balanced");
        }

        [TestMethod]
        public void ThetaddotRateShouldBePositiveWhenPoleLeansRight()
        {
            // Arrange: Pole leaning slightly right (positive theta), no control
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;  // Leaning right
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var thetaddot = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            // When pole leans right (positive theta), gravity creates positive torque (falls right)
            Assert.IsTrue(thetaddot > 0, "Pole should accelerate in direction of lean (positive angular acceleration for positive angle)");
        }

        [TestMethod]
        public void ThetaddotRateShouldBeNegativeWhenPoleLeansLeft()
        {
            // Arrange: Pole leaning slightly left (negative theta), no control
            var x = 0.0;
            var xdot = 0.0;
            var theta = -0.1;  // Leaning left
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var thetaddot = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot, F, M, m, L, g);

            // Assert
            // When pole leans left (negative theta), gravity creates negative torque (falls left)
            Assert.IsTrue(thetaddot < 0, "Pole should accelerate in direction of lean (negative angular acceleration for negative angle)");
        }

        [TestMethod]
        public void ThetaddotRateShouldBeSymmetricAboutUpright()
        {
            // Arrange: Test symmetry
            var x = 0.0;
            var xdot = 0.0;
            var theta1 = 0.1;
            var theta2 = -0.1;
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var thetaddot1 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta1, thetadot, F, M, m, L, g);
            var thetaddot2 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta2, thetadot, F, M, m, L, g);

            // Assert
            Assert.AreEqual(-thetaddot1, thetaddot2, 1e-8, "Angular acceleration should be symmetric about upright position");
        }

        [TestMethod]
        public void ThetaddotRateShouldBeAffectedByCartAcceleration()
        {
            // Arrange: Force on cart affects pole angular acceleration
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.0;
            var F1 = 0.0;
            var F2 = 1.0;  // Positive force pushes cart right

            // Act
            var thetaddot1 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot, F1, M, m, L, g);
            var thetaddot2 = OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot, F2, M, m, L, g);

            // Assert
            Assert.AreNotEqual(thetaddot1, thetaddot2, "Cart force should affect pole angular acceleration");
            // Positive force (cart accelerates right) creates negative angular acceleration (resists rightward lean)
            Assert.IsTrue(thetaddot2 < thetaddot1, "Positive cart force should reduce angular acceleration in same direction");
        }

        // ========== RUNNING COST TESTS ==========

        [TestMethod]
        public void RunningCostShouldBeZeroAtEquilibrium()
        {
            // Arrange: All states at zero, no control
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.0;
            var thetadot = 0.0;
            var F = 0.0;

            // Act
            var cost = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot, F);

            // Assert
            Assert.AreEqual(0.0, cost, Tolerance, "Cost should be zero at equilibrium with no control");
        }

        [TestMethod]
        public void RunningCostShouldIncreaseWithStateDeviation()
        {
            // Arrange
            var F = 0.0;

            var cost0 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, 0.0, 0.0, F);
            var costX = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.1, 0.0, 0.0, 0.0, F);
            var costXdot = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.1, 0.0, 0.0, F);
            var costTheta = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, 0.1, 0.0, F);
            var costThetadot = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, 0.0, 0.1, F);

            // Assert
            Assert.AreEqual(0.0, cost0, Tolerance);
            Assert.IsTrue(costX > cost0, "Cost should increase with cart position deviation");
            Assert.IsTrue(costXdot > cost0, "Cost should increase with cart velocity deviation");
            Assert.IsTrue(costTheta > cost0, "Cost should increase with pole angle deviation");
            Assert.IsTrue(costThetadot > cost0, "Cost should increase with pole angular velocity deviation");
        }

        [TestMethod]
        public void RunningCostShouldBeQuadraticInStates()
        {
            // Arrange: Test quadratic scaling
            var F = 0.0;

            var x1 = 0.1;
            var x2 = 0.2;

            var cost1 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x1, 0.0, 0.0, 0.0, F);
            var cost2 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x2, 0.0, 0.0, 0.0, F);

            // Assert: Doubling state should quadruple cost component
            // Cost for x is 20·x², so doubling x quadruples the x component
            Assert.AreEqual(20.0 * x1 * x1, cost1, Tolerance, "Cost should be 20·x²");
            Assert.AreEqual(20.0 * x2 * x2, cost2, Tolerance, "Cost should be 20·x²");
            Assert.AreEqual(4.0, cost2 / cost1, 1e-8, "Doubling x should quadruple cost");
        }

        [TestMethod]
        public void RunningCostShouldIncreaseWithControl()
        {
            // Arrange
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.0;
            var thetadot = 0.0;
            var F1 = 0.0;
            var F2 = 1.0;

            // Act
            var cost1 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot, F1);
            var cost2 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot, F2);

            // Assert
            Assert.AreEqual(0.0, cost1, Tolerance, "Cost should be zero with no control");
            Assert.IsTrue(cost2 > cost1, "Cost should increase with control effort");
            Assert.AreEqual(0.5 * F2 * F2, cost2, Tolerance, "Control cost should be 0.5·F²");
        }

        [TestMethod]
        public void RunningCostShouldBeQuadraticInControl()
        {
            // Arrange
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.0;
            var thetadot = 0.0;
            var F1 = 1.0;
            var F2 = 2.0;

            // Act
            var cost1 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot, F1);
            var cost2 = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot, F2);

            // Assert
            Assert.AreEqual(4.0, cost2 / cost1, 1e-8, "Doubling control should quadruple cost");
        }

        [TestMethod]
        public void RunningCostWeightsShouldMatchSpecification()
        {
            // Arrange: Test individual cost components
            // L = 20·x² + 2·ẋ² + 50·θ² + 2·θ̇² + 0.5·F²
            var value = 1.0;

            var costX = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(value, 0.0, 0.0, 0.0, 0.0);
            var costXdot = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, value, 0.0, 0.0, 0.0);
            var costTheta = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, value, 0.0, 0.0);
            var costThetadot = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, 0.0, value, 0.0);
            var costF = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, 0.0, 0.0, value);

            // Assert
            Assert.AreEqual(20.0, costX, Tolerance, "Weight for x should be 20");
            Assert.AreEqual(2.0, costXdot, Tolerance, "Weight for xdot should be 2");
            Assert.AreEqual(50.0, costTheta, Tolerance, "Weight for theta should be 50 (highest penalty)");
            Assert.AreEqual(2.0, costThetadot, Tolerance, "Weight for thetadot should be 2");
            Assert.AreEqual(0.5, costF, Tolerance, "Weight for F should be 0.5");
        }

        [TestMethod]
        public void RunningCostShouldPenalizePoleAngleMostHeavily()
        {
            // Arrange: Same deviation in all states
            var deviation = 0.1;
            var F = 0.0;

            var costX = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(deviation, 0.0, 0.0, 0.0, F);
            var costXdot = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, deviation, 0.0, 0.0, F);
            var costTheta = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, deviation, 0.0, F);
            var costThetadot = OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(0.0, 0.0, 0.0, deviation, F);

            // Assert: Pole angle should have highest penalty
            Assert.IsTrue(costTheta > costX, "Pole angle deviation should be penalized more than cart position");
            Assert.IsTrue(costTheta > costXdot, "Pole angle deviation should be penalized more than cart velocity");
            Assert.IsTrue(costTheta > costThetadot, "Pole angle deviation should be penalized more than angular velocity");
        }
    }
}
