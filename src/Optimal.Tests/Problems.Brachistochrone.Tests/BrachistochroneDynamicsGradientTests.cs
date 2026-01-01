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
    /// Tests comparing AutoDiff-generated gradients against numerical gradients
    /// for Brachistochrone problem dynamics.
    /// </summary>
    [TestClass]
    public sealed class BrachistochroneDynamicsGradientTests
    {
        private const double Tolerance = 1e-8;
        private const double FiniteDiffEpsilon = 1e-7;
        private const double NumericalTolerance = 1e-5;

        // Standard test parameters
        private const double g = 9.81;
        private const double alpha = -Math.PI / 4; // 45° downward diagonal

        /// <summary>
        /// Computes numerical gradient using central finite differences.
        /// </summary>
        private static double NumericalGradient(Func<double, double> func, double x)
        {
            return (func(x + FiniteDiffEpsilon) - func(x - FiniteDiffEpsilon)) / (2 * FiniteDiffEpsilon);
        }

        // ========== S RATE GRADIENT TESTS ==========
        // ṡ = v·cos(θ)

        [TestMethod]
        public void SRateGradient_WrtS_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.SRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[0]; // ∂ṡ/∂s

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                s_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s_perturbed, d, v, theta, g, alpha),
                s);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ṡ/∂s: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ṡ/∂s should be 0 (ṡ independent of s)");
        }

        [TestMethod]
        public void SRateGradient_WrtD_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.SRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[1]; // ∂ṡ/∂d

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                d_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d_perturbed, v, theta, g, alpha),
                d);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ṡ/∂d: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ṡ/∂d should be 0 (ṡ independent of d)");
        }

        [TestMethod]
        public void SRateGradient_WrtV_ShouldBeCosTheta()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.SRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[2]; // ∂ṡ/∂v

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v_perturbed, theta, g, alpha),
                v);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ṡ/∂v: AutoDiff should match numerical gradient");
            
            var expectedGrad = Math.Cos(theta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, Tolerance, "∂ṡ/∂v should equal cos(θ)");
        }

        [TestMethod]
        public void SRateGradient_WrtTheta_ShouldBeNegativeVSinTheta()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.SRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[3]; // ∂ṡ/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.SRate(s, d, v, theta_perturbed, g, alpha),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ṡ/∂θ: AutoDiff should match numerical gradient");
            
            var expectedGrad = -v * Math.Sin(theta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, Tolerance, "∂ṡ/∂θ should equal -v·sin(θ)");
        }

        // ========== D RATE GRADIENT TESTS ==========
        // ḋ = v·sin(θ)

        [TestMethod]
        public void DRateGradient_WrtS_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.DRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[0]; // ∂ḋ/∂s

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                s_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s_perturbed, d, v, theta, g, alpha),
                s);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ḋ/∂s: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ḋ/∂s should be 0 (ḋ independent of s)");
        }

        [TestMethod]
        public void DRateGradient_WrtD_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.DRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[1]; // ∂ḋ/∂d

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                d_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d_perturbed, v, theta, g, alpha),
                d);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ḋ/∂d: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂ḋ/∂d should be 0 (ḋ independent of d)");
        }

        [TestMethod]
        public void DRateGradient_WrtV_ShouldBeSinTheta()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.DRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[2]; // ∂ḋ/∂v

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v_perturbed, theta, g, alpha),
                v);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ḋ/∂v: AutoDiff should match numerical gradient");
            
            var expectedGrad = Math.Sin(theta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, Tolerance, "∂ḋ/∂v should equal sin(θ)");
        }

        [TestMethod]
        public void DRateGradient_WrtTheta_ShouldBeVCosTheta()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.DRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[3]; // ∂ḋ/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.DRate(s, d, v, theta_perturbed, g, alpha),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂ḋ/∂θ: AutoDiff should match numerical gradient");
            
            var expectedGrad = v * Math.Cos(theta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, Tolerance, "∂ḋ/∂θ should equal v·cos(θ)");
        }

        // ========== V RATE GRADIENT TESTS ==========
        // v̇ = g·sin(α + θ)

        [TestMethod]
        public void VRateGradient_WrtS_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[0]; // ∂v̇/∂s

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                s_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s_perturbed, d, v, theta, g, alpha),
                s);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂v̇/∂s: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂v̇/∂s should be 0 (v̇ independent of s)");
        }

        [TestMethod]
        public void VRateGradient_WrtD_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[1]; // ∂v̇/∂d

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                d_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d_perturbed, v, theta, g, alpha),
                d);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂v̇/∂d: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂v̇/∂d should be 0 (v̇ independent of d)");
        }

        [TestMethod]
        public void VRateGradient_WrtV_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[2]; // ∂v̇/∂v

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                v_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v_perturbed, theta, g, alpha),
                v);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂v̇/∂v: AutoDiff should match numerical gradient");
            Assert.AreEqual(0.0, autoDiffGrad, Tolerance, "∂v̇/∂v should be 0 (v̇ independent of v)");
        }

        [TestMethod]
        public void VRateGradient_WrtTheta_ShouldBeNegativeGCosAlphaPlusTheta()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (_, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, theta, g, alpha);
            var autoDiffGrad = grad[3]; // ∂v̇/∂θ

            // Act - Numerical gradient
            var numericalGrad = NumericalGradient(
                theta_perturbed => OptimalCli.Problems.Brachistochrone.BrachistochroneDynamics.VRate(s, d, v, theta_perturbed, g, alpha),
                theta);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, NumericalTolerance, "∂v̇/∂θ: AutoDiff should match numerical gradient");
            
            var expectedGrad = -g * Math.Cos(alpha + theta);
            Assert.AreEqual(expectedGrad, autoDiffGrad, Tolerance, "∂v̇/∂θ should equal -g·cos(α + θ)");
        }

        [TestMethod]
        public void VRateGradient_WrtTheta_AtDifferentAngles()
        {
            // Test gradient behavior at key angles
            var s = 1.0;
            var d = 0.0;
            var v = 2.0;

            // Test at theta = 0
            var (_, grad0) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, 0.0, g, alpha);
            var expected0 = -g * Math.Cos(alpha);
            Assert.AreEqual(expected0, grad0[3], Tolerance, "∂v̇/∂θ at θ=0 should equal -g·cos(α)");

            // Test at theta = π/4
            var (_, grad1) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, Math.PI / 4, g, alpha);
            var expected1 = -g * Math.Cos(alpha + Math.PI / 4);
            Assert.AreEqual(expected1, grad1[3], Tolerance, "∂v̇/∂θ at θ=π/4 should equal -g·cos(α + π/4)");

            // Test at theta = -π/4
            var (_, grad2) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, -Math.PI / 4, g, alpha);
            var expected2 = -g * Math.Cos(alpha - Math.PI / 4);
            Assert.AreEqual(expected2, grad2[3], Tolerance, "∂v̇/∂θ at θ=-π/4 should equal -g·cos(α - π/4)");
        }

        // ========== RUNNING COST GRADIENT TESTS ==========

        [TestMethod]
        public void RunningCostGradient_ShouldBeZero()
        {
            // Arrange
            var s = 1.0;
            var d = 0.5;
            var v = 2.0;
            var theta = Math.PI / 6;

            // Act - AutoDiff gradient
            var (cost, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.RunningCostReverse(s, d, v, theta);

            // Assert: Running cost is constant 1.0, so all gradients are 0
            Assert.AreEqual(1.0, cost, Tolerance, "Running cost should be 1.0");
            Assert.AreEqual(0.0, grad[0], Tolerance, "∂L/∂s should be 0");
            Assert.AreEqual(0.0, grad[1], Tolerance, "∂L/∂d should be 0");
            Assert.AreEqual(0.0, grad[2], Tolerance, "∂L/∂v should be 0");
            Assert.AreEqual(0.0, grad[3], Tolerance, "∂L/∂θ should be 0");
        }

        // ========== TERMINAL COST GRADIENT TESTS ==========

        [TestMethod]
        public void TerminalCostGradient_ShouldBeZero()
        {
            // Arrange
            var s = 2.83;
            var d = 0.0;
            var v = 3.5;

            // Act - AutoDiff gradient
            var (cost, grad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.TerminalCostReverse(s, d, v);

            // Assert
            Assert.AreEqual(1.0, cost, Tolerance, "Terminal cost should be 1.0");
            Assert.AreEqual(0.0, grad[0], Tolerance, "∂Φ/∂s should be 0");
            Assert.AreEqual(0.0, grad[1], Tolerance, "∂Φ/∂d should be 0");
            Assert.AreEqual(0.0, grad[2], Tolerance, "∂Φ/∂v should be 0");
        }

        // ========== COMPREHENSIVE GRADIENT TESTS ==========

        [TestMethod]
        public void AllGradients_ShouldMatchNumerical_AtVariousStates()
        {
            // Test at multiple state/control combinations to ensure gradients are correct everywhere

            var testCases = new[]
            {
                (s: 0.0, d: 0.0, v: 0.0, theta: 0.0),
                (s: 1.0, d: 0.0, v: 2.0, theta: Math.PI / 6),
                (s: 2.0, d: 0.5, v: 3.0, theta: -Math.PI / 4),
                (s: 1.5, d: -0.3, v: 2.5, theta: Math.PI / 3),
            };

            foreach (var (s, d, v, theta) in testCases)
            {
                // Test SRate gradients
                var (_, sGrad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.SRateReverse(s, d, v, theta, g, alpha);
                Assert.AreEqual(0.0, sGrad[0], Tolerance, $"SRate: ∂ṡ/∂s at ({s},{d},{v},{theta})");
                Assert.AreEqual(0.0, sGrad[1], Tolerance, $"SRate: ∂ṡ/∂d at ({s},{d},{v},{theta})");
                Assert.AreEqual(Math.Cos(theta), sGrad[2], Tolerance, $"SRate: ∂ṡ/∂v at ({s},{d},{v},{theta})");
                Assert.AreEqual(-v * Math.Sin(theta), sGrad[3], Tolerance, $"SRate: ∂ṡ/∂θ at ({s},{d},{v},{theta})");

                // Test DRate gradients
                var (_, dGrad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.DRateReverse(s, d, v, theta, g, alpha);
                Assert.AreEqual(0.0, dGrad[0], Tolerance, $"DRate: ∂ḋ/∂s at ({s},{d},{v},{theta})");
                Assert.AreEqual(0.0, dGrad[1], Tolerance, $"DRate: ∂ḋ/∂d at ({s},{d},{v},{theta})");
                Assert.AreEqual(Math.Sin(theta), dGrad[2], Tolerance, $"DRate: ∂ḋ/∂v at ({s},{d},{v},{theta})");
                Assert.AreEqual(v * Math.Cos(theta), dGrad[3], Tolerance, $"DRate: ∂ḋ/∂θ at ({s},{d},{v},{theta})");

                // Test VRate gradients
                var (_, vGrad) = OptimalCli.Problems.Brachistochrone.BrachistochroneDynamicsGradients.VRateReverse(s, d, v, theta, g, alpha);
                Assert.AreEqual(0.0, vGrad[0], Tolerance, $"VRate: ∂v̇/∂s at ({s},{d},{v},{theta})");
                Assert.AreEqual(0.0, vGrad[1], Tolerance, $"VRate: ∂v̇/∂d at ({s},{d},{v},{theta})");
                Assert.AreEqual(0.0, vGrad[2], Tolerance, $"VRate: ∂v̇/∂v at ({s},{d},{v},{theta})");
                Assert.AreEqual(-g * Math.Cos(alpha + theta), vGrad[3], Tolerance, $"VRate: ∂v̇/∂θ at ({s},{d},{v},{theta})");
            }
        }
    }
}
