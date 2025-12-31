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
    /// Tests comparing AutoDiff-generated gradients against numerical gradients
    /// for cart-pole dynamics.
    /// </summary>
    [TestClass]
    public sealed class CartPoleDynamicsGradientTests
    {
        private const double Tolerance = 1e-10;
        private const double FiniteDiffEpsilon = 1e-7;

        // Standard test parameters
        private const double M = 1.0;   // Cart mass (kg)
        private const double m = 0.1;   // Pole mass (kg)
        private const double L = 2.0;   // Pole length (m)
        private const double g = 9.81;  // Gravity (m/s²)

        /// <summary>
        /// Computes numerical gradient using central finite differences.
        /// </summary>
        private static double NumericalGradient(System.Func<double, double> func, double x)
        {
            return (func(x + FiniteDiffEpsilon) - func(x - FiniteDiffEpsilon)) / (2 * FiniteDiffEpsilon);
        }

        // ========== X RATE GRADIENT TESTS ==========

        [TestMethod]
        public void XRateGradient_WrtX_ShouldBeZero()
        {
            // Arrange
            var x = 0.5;
            var xdot = 2.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[0]; // ∂ẋ/∂x

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                x_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x_perturbed, xdot, theta, thetadot, F, M, m, L, g),
                x);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẋ/∂x: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ẋ/∂x should be exactly 0 (ẋ = xdot, independent of x)");
        }

        [TestMethod]
        public void XRateGradient_WrtXdot_ShouldBeOne()
        {
            // Arrange
            var x = 0.5;
            var xdot = 2.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[1]; // ∂ẋ/∂ẋ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                xdot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x, xdot_perturbed, theta, thetadot, F, M, m, L, g),
                xdot);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẋ/∂ẋ: AutoDiff should match numerical gradient");
            Assert.AreEqual(1.0, autoDiffGrad, Tolerance, "∂ẋ/∂ẋ should be exactly 1 (ẋ = xdot)");
        }

        [TestMethod]
        public void XRateGradient_WrtTheta_ShouldBeZero()
        {
            // Arrange
            var x = 0.5;
            var xdot = 2.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[2]; // ∂ẋ/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x, xdot, theta_perturbed, thetadot, F, M, m, L, g),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẋ/∂θ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ẋ/∂θ should be exactly 0");
        }

        [TestMethod]
        public void XRateGradient_WrtThetadot_ShouldBeZero()
        {
            // Arrange
            var x = 0.5;
            var xdot = 2.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[3]; // ∂ẋ/∂θ̇

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x, xdot, theta, thetadot_perturbed, F, M, m, L, g),
                thetadot);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẋ/∂θ̇: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ẋ/∂θ̇ should be exactly 0");
        }

        [TestMethod]
        public void XRateGradient_WrtForce_ShouldBeZero()
        {
            // Arrange
            var x = 0.5;
            var xdot = 2.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[4]; // ∂ẋ/∂F

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                F_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XRate(x, xdot, theta, thetadot, F_perturbed, M, m, L, g),
                F);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẋ/∂F: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ẋ/∂F should be exactly 0");
        }

        // ========== XDDOT RATE GRADIENT TESTS ==========
        // ẍ = (F + m·L·θ̇²·sin(θ) - m·g·sin(θ)·cos(θ)) / (M + m·sin²(θ))

        [TestMethod]
        public void XddotRateGradient_WrtX_ShouldBeZero()
        {
            // ẍ does not depend on x directly
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[0];

            var numericalGrad = NumericalGradient(
                x_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x_perturbed, xdot, theta, thetadot, F, M, m, L, g),
                x);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẍ/∂x: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ẍ/∂x should be exactly 0");
        }

        [TestMethod]
        public void XddotRateGradient_WrtXdot_ShouldBeZero()
        {
            // ẍ does not depend on ẋ directly
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[1];

            var numericalGrad = NumericalGradient(
                xdot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot_perturbed, theta, thetadot, F, M, m, L, g),
                xdot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẍ/∂ẋ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ẍ/∂ẋ should be exactly 0");
        }

        [TestMethod]
        public void XddotRateGradient_WrtTheta_ShouldMatchNumerical()
        {
            // Complex gradient due to sin/cos terms
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[2];

            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta_perturbed, thetadot, F, M, m, L, g),
                theta);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂ẍ/∂θ: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");
        }

        [TestMethod]
        public void XddotRateGradient_WrtThetadot_ShouldMatchNumerical()
        {
            // Gradient from centrifugal term m·L·θ̇²·sin(θ)
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[3];

            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot_perturbed, F, M, m, L, g),
                thetadot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂ẍ/∂θ̇: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");
        }

        [TestMethod]
        public void XddotRateGradient_WrtForce_ShouldMatchNumerical()
        {
            // ∂ẍ/∂F = 1 / (M + m·sin²(θ))
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[4];

            var numericalGrad = NumericalGradient(
                F_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot, F_perturbed, M, m, L, g),
                F);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂ẍ/∂F: AutoDiff should match numerical gradient");

            // Verify analytical formula
            var sinTheta = Math.Sin(theta);
            var expectedGrad = 1.0 / (M + m * sinTheta * sinTheta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, 1e-8, "∂ẍ/∂F should match analytical formula");
        }

        // ========== THETA RATE GRADIENT TESTS ==========

        [TestMethod]
        public void ThetaRateGradient_WrtX_ShouldBeZero()
        {
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 2.0;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[0];

            var numericalGrad = NumericalGradient(
                x_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x_perturbed, xdot, theta, thetadot, F, M, m, L, g),
                x);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂x: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̇/∂x should be exactly 0");
        }

        [TestMethod]
        public void ThetaRateGradient_WrtXdot_ShouldBeZero()
        {
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 2.0;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[1];

            var numericalGrad = NumericalGradient(
                xdot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x, xdot_perturbed, theta, thetadot, F, M, m, L, g),
                xdot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂ẋ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̇/∂ẋ should be exactly 0");
        }

        [TestMethod]
        public void ThetaRateGradient_WrtTheta_ShouldBeZero()
        {
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 2.0;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[2];

            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x, xdot, theta_perturbed, thetadot, F, M, m, L, g),
                theta);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂θ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̇/∂θ should be exactly 0");
        }

        [TestMethod]
        public void ThetaRateGradient_WrtThetadot_ShouldBeOne()
        {
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 2.0;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[3];

            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x, xdot, theta, thetadot_perturbed, F, M, m, L, g),
                thetadot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂θ̇: AutoDiff should match numerical gradient");
            Assert.AreEqual(1.0, autoDiffGrad, Tolerance, "∂θ̇/∂θ̇ should be exactly 1 (θ̇ = thetadot)");
        }

        [TestMethod]
        public void ThetaRateGradient_WrtForce_ShouldBeZero()
        {
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 2.0;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[4];

            var numericalGrad = NumericalGradient(
                F_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaRate(x, xdot, theta, thetadot, F_perturbed, M, m, L, g),
                F);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̇/∂F: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̇/∂F should be exactly 0");
        }

        // ========== THETADDOT RATE GRADIENT TESTS ==========
        // θ̈ = (g·sin(θ) - cos(θ)·(F + m·L·θ̇²·sin(θ))/(M+m)) / (L·(4/3 - m·cos²(θ)/(M+m)))

        [TestMethod]
        public void ThetaddotRateGradient_WrtX_ShouldBeZero()
        {
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[0];

            var numericalGrad = NumericalGradient(
                x_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x_perturbed, xdot, theta, thetadot, F, M, m, L, g),
                x);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̈/∂x: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̈/∂x should be exactly 0");
        }

        [TestMethod]
        public void ThetaddotRateGradient_WrtXdot_ShouldBeZero()
        {
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[1];

            var numericalGrad = NumericalGradient(
                xdot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot_perturbed, theta, thetadot, F, M, m, L, g),
                xdot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̈/∂ẋ: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂θ̈/∂ẋ should be exactly 0");
        }

        [TestMethod]
        public void ThetaddotRateGradient_WrtTheta_ShouldMatchNumerical()
        {
            // Complex gradient - most important for stability analysis
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[2];

            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta_perturbed, thetadot, F, M, m, L, g),
                theta);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂θ̈/∂θ: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");
        }

        [TestMethod]
        public void ThetaddotRateGradient_WrtThetadot_ShouldMatchNumerical()
        {
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[3];

            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot_perturbed, F, M, m, L, g),
                thetadot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂θ̈/∂θ̇: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");
        }

        [TestMethod]
        public void ThetaddotRateGradient_WrtForce_ShouldMatchNumerical()
        {
            var x = 0.0;
            var xdot = 0.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.ThetaddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);
            var autoDiffGrad = grad[4];

            var numericalGrad = NumericalGradient(
                F_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.ThetaddotRate(x, xdot, theta, thetadot, F_perturbed, M, m, L, g),
                F);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂θ̈/∂F: AutoDiff should match numerical gradient");
        }

        // ========== RUNNING COST GRADIENT TESTS ==========
        // L = 20·x² + 2·ẋ² + 50·θ² + 2·θ̇² + 0.5·F²

        [TestMethod]
        public void RunningCostGradient_WrtX_ShouldMatchNumerical()
        {
            // ∂L/∂x = 40·x
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.RunningCostReverse(x, xdot, theta, thetadot, F);
            var autoDiffGrad = grad[0];

            var numericalGrad = NumericalGradient(
                x_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x_perturbed, xdot, theta, thetadot, F),
                x);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂x: AutoDiff should match numerical gradient");
            Assert.AreEqual(40.0 * x, autoDiffGrad, 1e-8, "∂L/∂x should be 40·x");
        }

        [TestMethod]
        public void RunningCostGradient_WrtXdot_ShouldMatchNumerical()
        {
            // ∂L/∂ẋ = 4·ẋ
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.RunningCostReverse(x, xdot, theta, thetadot, F);
            var autoDiffGrad = grad[1];

            var numericalGrad = NumericalGradient(
                xdot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot_perturbed, theta, thetadot, F),
                xdot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂ẋ: AutoDiff should match numerical gradient");
            Assert.AreEqual(4.0 * xdot, autoDiffGrad, 1e-8, "∂L/∂ẋ should be 4·ẋ");
        }

        [TestMethod]
        public void RunningCostGradient_WrtTheta_ShouldMatchNumerical()
        {
            // ∂L/∂θ = 100·θ
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.RunningCostReverse(x, xdot, theta, thetadot, F);
            var autoDiffGrad = grad[2];

            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta_perturbed, thetadot, F),
                theta);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂θ: AutoDiff should match numerical gradient");
            Assert.AreEqual(100.0 * theta, autoDiffGrad, 1e-8, "∂L/∂θ should be 100·θ");
        }

        [TestMethod]
        public void RunningCostGradient_WrtThetadot_ShouldMatchNumerical()
        {
            // ∂L/∂θ̇ = 4·θ̇
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.RunningCostReverse(x, xdot, theta, thetadot, F);
            var autoDiffGrad = grad[3];

            var numericalGrad = NumericalGradient(
                thetadot_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot_perturbed, F),
                thetadot);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂θ̇: AutoDiff should match numerical gradient");
            Assert.AreEqual(4.0 * thetadot, autoDiffGrad, 1e-8, "∂L/∂θ̇ should be 4·θ̇");
        }

        [TestMethod]
        public void RunningCostGradient_WrtForce_ShouldMatchNumerical()
        {
            // ∂L/∂F = F
            var x = 0.5;
            var xdot = 1.0;
            var theta = 0.1;
            var thetadot = 0.5;
            var F = 1.0;

            var (_, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.RunningCostReverse(x, xdot, theta, thetadot, F);
            var autoDiffGrad = grad[4];

            var numericalGrad = NumericalGradient(
                F_perturbed => OptimalCli.Problems.CartPole.CartPoleDynamics.RunningCost(x, xdot, theta, thetadot, F_perturbed),
                F);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5, "∂L/∂F: AutoDiff should match numerical gradient");
            Assert.AreEqual(F, autoDiffGrad, 1e-8, "∂L/∂F should be F");
        }

        // ========== COMPREHENSIVE VALIDATION ==========

        [TestMethod]
        public void AllGradientsShouldMatchNumericalGradientsAtVariousStates()
        {
            // Test at multiple state-control combinations
            var testCases = new[]
            {
                (x: 0.0, xdot: 0.0, theta: 0.0, thetadot: 0.0, F: 0.0),    // Equilibrium
                (x: 0.1, xdot: 0.0, theta: 0.0, thetadot: 0.0, F: 0.0),    // Position offset
                (x: 0.0, xdot: 0.0, theta: 0.1, thetadot: 0.0, F: 0.0),    // Angle offset
                (x: 0.0, xdot: 0.0, theta: 0.0, thetadot: 0.0, F: 1.0),    // Force only
                (x: 0.2, xdot: 0.5, theta: 0.1, thetadot: 0.3, F: 1.5),    // Generic state
                (x: -0.1, xdot: -0.3, theta: -0.15, thetadot: -0.2, F: -1.0), // Negative values
            };

            foreach (var (x, xdot, theta, thetadot, F) in testCases)
            {
                // Test XddotRate gradients (most complex dynamics)
                var (value, grad) = OptimalCli.Problems.CartPole.CartPoleDynamicsGradients.XddotRateReverse(x, xdot, theta, thetadot, F, M, m, L, g);

                var numGradTheta = NumericalGradient(
                    t => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, t, thetadot, F, M, m, L, g), theta);
                var numGradThetadot = NumericalGradient(
                    td => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, td, F, M, m, L, g), thetadot);
                var numGradF = NumericalGradient(
                    f => OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot, f, M, m, L, g), F);

                Assert.AreEqual(numGradTheta, grad[2], 1e-5,
                    $"ẍ gradient w.r.t. θ failed at state ({x:F2}, {xdot:F2}, {theta:F2}, {thetadot:F2}, {F:F2})");
                Assert.AreEqual(numGradThetadot, grad[3], 1e-5,
                    $"ẍ gradient w.r.t. θ̇ failed at state ({x:F2}, {xdot:F2}, {theta:F2}, {thetadot:F2}, {F:F2})");
                Assert.AreEqual(numGradF, grad[4], 1e-5,
                    $"ẍ gradient w.r.t. F failed at state ({x:F2}, {xdot:F2}, {theta:F2}, {thetadot:F2}, {F:F2})");

                // Verify function value
                var expectedValue = OptimalCli.Problems.CartPole.CartPoleDynamics.XddotRate(x, xdot, theta, thetadot, F, M, m, L, g);
                Assert.AreEqual(expectedValue, value, Tolerance,
                    $"ẍ function value incorrect at state ({x:F2}, {xdot:F2}, {theta:F2}, {thetadot:F2}, {F:F2})");
            }
        }
    }
}
