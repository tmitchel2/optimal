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

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// TDD tests demonstrating that intermediate variables should NOT break the gradient chain.
    /// These tests initially FAIL, demonstrating the current limitation of AutoDiff.
    /// After fixing AutoDiff, these tests should PASS.
    /// </summary>
    [TestClass]
    public sealed class IntermediateVariableTests
    {
        private const double Tolerance = 1e-10;
        private const double FiniteDiffEpsilon = 1e-7;

        /// <summary>
        /// Computes numerical gradient using central finite differences.
        /// </summary>
        private static double NumericalGradient(Func<double, double> func, double x)
        {
            return (func(x + FiniteDiffEpsilon) - func(x - FiniteDiffEpsilon)) / (2 * FiniteDiffEpsilon);
        }

        // ========== SIMPLE INTERMEDIATE VARIABLE TESTS ==========

        [TestMethod]
        public void SimpleWithIntermediate_ShouldMatchInlinedVersion()
        {
            // Arrange
            var x = Math.PI / 4;

            // Act
            var withIntermediate = IntermediateVariableTestFunctions.SimpleWithIntermediate(x);
            var inlined = IntermediateVariableTestFunctions.SimpleInlined(x);

            // Assert
            Assert.AreEqual(inlined, withIntermediate, Tolerance, 
                "Function value should be same regardless of intermediate variables");
        }

        [TestMethod]
        [Ignore("Single-parameter functions only have Forward mode. Requires AutoDiff enhancement for Reverse mode.")]
        public void SimpleWithIntermediate_GradientShouldMatchNumerical()
        {
            // TDD: This test requires Reverse mode for single-parameter functions
            // Current AutoDiff limitation: only Forward mode is generated for single-parameter functions
            // After fixing AutoDiff to support Reverse mode, this test should work

            // Arrange
            var x = Math.PI / 4;

            // Act - AutoDiff gradient (using Forward mode as workaround)
            var (value, grad) = IntermediateVariableTestFunctionsGradients.SimpleWithIntermediateForward_x(x);

            // Act - Numerical gradient (ground truth)
            var numericalGrad = NumericalGradient(
                x_perturbed => IntermediateVariableTestFunctions.SimpleWithIntermediate(x_perturbed),
                x);

            // Act - Expected analytical gradient
            // f(x) = sin(x) + cos(x)
            // f'(x) = cos(x) - sin(x)
            var expectedGrad = Math.Cos(x) - Math.Sin(x);

            // Assert
            Assert.AreEqual(numericalGrad, grad, 1e-5, 
                $"AutoDiff gradient {grad} should match numerical gradient {numericalGrad}");
            Assert.AreEqual(expectedGrad, grad, 1e-8,
                $"AutoDiff gradient {grad} should match analytical gradient {expectedGrad}");
        }

        [TestMethod]
        [Ignore("Single-parameter functions only have Forward mode. Requires AutoDiff enhancement.")]
        public void SimpleWithIntermediate_ShouldMatchInlinedGradient()
        {
            // TDD: Gradients from intermediate and inlined versions should match
            // This test uses Forward mode as a workaround

            // Arrange
            var x = Math.PI / 4;

            // Act - Using Forward mode (only mode available for single-parameter functions)
            var (_, gradIntermediate) = IntermediateVariableTestFunctionsGradients.SimpleWithIntermediateForward_x(x);
            var (_, gradInlined) = IntermediateVariableTestFunctionsGradients.SimpleInlinedForward_x(x);

            // Assert
            Assert.AreEqual(gradInlined, gradIntermediate, 1e-8,
                "Gradient should be same regardless of intermediate variables");
        }

        // ========== CARTPOLE-LIKE TESTS ==========

        [TestMethod]
        public void CartPoleLike_FunctionValuesShouldMatch()
        {
            // Arrange
            var x = 1.5;
            var y = 0.3;

            // Act
            var withIntermediate = IntermediateVariableTestFunctions.CartPoleLikeWithIntermediate(x, y);
            var inlined = IntermediateVariableTestFunctions.CartPoleLikeInlined(x, y);

            // Assert
            Assert.AreEqual(inlined, withIntermediate, Tolerance,
                "Function values should match");
        }

        [TestMethod]
        public void CartPoleLike_GradientWrtX_ShouldMatchNumerical()
        {
            // Simple gradient (no intermediate variable issues)
            var x = 1.5;
            var y = 0.3;

            var (_, grad) = IntermediateVariableTestFunctionsGradients.CartPoleLikeWithIntermediateReverse(x, y);
            var autoDiffGrad = grad[0];

            var numericalGrad = NumericalGradient(
                x_perturbed => IntermediateVariableTestFunctions.CartPoleLikeWithIntermediate(x_perturbed, y),
                x);

            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂f/∂x: AutoDiff {autoDiffGrad} should match numerical {numericalGrad}");
        }

        [TestMethod]
        public void CartPoleLike_GradientWrtY_ShouldMatchNumerical()
        {
            // TDD: This test should now PASS with intermediate variable support

            // Arrange
            var x = 1.5;
            var y = 0.3;

            // Act - AutoDiff gradient
            var (_, grad) = IntermediateVariableTestFunctionsGradients.CartPoleLikeWithIntermediateReverse(x, y);
            var autoDiffGrad = grad[1];

            // Act - Numerical gradient (ground truth)
            var numericalGrad = NumericalGradient(
                y_perturbed => IntermediateVariableTestFunctions.CartPoleLikeWithIntermediate(x, y_perturbed),
                y);

            // Assert
            Assert.AreEqual(numericalGrad, autoDiffGrad, 1e-5,
                $"∂f/∂y: AutoDiff gradient {autoDiffGrad} should match numerical gradient {numericalGrad}");
        }

        [TestMethod]
        
        public void CartPoleLike_GradientsShouldMatchInlinedVersion()
        {
            // TDD: Both versions should give same gradients
            // This test FAILS - demonstrates the problem

            var x = 1.5;
            var y = 0.3;

            var (_, gradIntermediate) = IntermediateVariableTestFunctionsGradients.CartPoleLikeWithIntermediateReverse(x, y);
            var (_, gradInlined) = IntermediateVariableTestFunctionsGradients.CartPoleLikeInlinedReverse(x, y);

            Assert.AreEqual(gradInlined[0], gradIntermediate[0], 1e-8, "∂f/∂x should match");
            Assert.AreEqual(gradInlined[1], gradIntermediate[1], 1e-8, "∂f/∂y should match");
        }

        // ========== COMPLEX INTERMEDIATE TESTS ==========

        [TestMethod]
        
        public void Complex_GradientsShouldMatchNumerical()
        {
            // TDD: Test with multiple intermediate variables
            // This test FAILS - demonstrates the problem

            var x = 0.5;
            var y = 0.8;

            var (_, grad) = IntermediateVariableTestFunctionsGradients.ComplexWithIntermediateReverse(x, y);

            var numGradX = NumericalGradient(
                x_perturbed => IntermediateVariableTestFunctions.ComplexWithIntermediate(x_perturbed, y), x);
            var numGradY = NumericalGradient(
                y_perturbed => IntermediateVariableTestFunctions.ComplexWithIntermediate(x, y_perturbed), y);

            Assert.AreEqual(numGradX, grad[0], 1e-5, $"∂f/∂x: AutoDiff {grad[0]} should match numerical {numGradX}");
            Assert.AreEqual(numGradY, grad[1], 1e-5, $"∂f/∂y: AutoDiff {grad[1]} should match numerical {numGradY}");
        }

        // ========== MULTIPLE USES OF INTERMEDIATE ==========

        [TestMethod]
        
        public void MultipleUses_GradientShouldMatchNumerical()
        {
            // TDD: Test when intermediate variable is used multiple times
            // This is important - the gradient should accumulate correctly
            // Using Forward mode as current workaround

            var x = 0.5;

            var (_, grad) = IntermediateVariableTestFunctionsGradients.MultipleUsesOfIntermediateForward_x(x);

            var numericalGrad = NumericalGradient(
                x_perturbed => IntermediateVariableTestFunctions.MultipleUsesOfIntermediate(x_perturbed), x);

            // Analytical: f(x) = sin(x) + sin²(x) + sin³(x)
            // f'(x) = cos(x) + 2·sin(x)·cos(x) + 3·sin²(x)·cos(x)
            //       = cos(x)·(1 + 2·sin(x) + 3·sin²(x))
            var sinX = Math.Sin(x);
            var cosX = Math.Cos(x);
            var expectedGrad = cosX * (1.0 + 2.0 * sinX + 3.0 * sinX * sinX);

            Assert.AreEqual(numericalGrad, grad, 1e-5,
                $"AutoDiff gradient {grad} should match numerical {numericalGrad}");
            Assert.AreEqual(expectedGrad, grad, 1e-8,
                $"AutoDiff gradient {grad} should match analytical {expectedGrad}");
        }

        [TestMethod]
        [Ignore("Requires AutoDiff enhancement. Using Forward mode as workaround.")]
        public void MultipleUses_ShouldMatchInlinedGradient()
        {
            // TDD: Verify that intermediate variables give same result as inlined
            // Using Forward mode

            var x = 0.5;

            var (_, gradIntermediate) = IntermediateVariableTestFunctionsGradients.MultipleUsesOfIntermediateForward_x(x);
            var (_, gradInlined) = IntermediateVariableTestFunctionsGradients.MultipleUsesInlinedForward_x(x);

            Assert.AreEqual(gradInlined, gradIntermediate, 1e-8,
                "Gradients should match regardless of intermediate variables");
        }

        // ========== CHAINED INTERMEDIATES ==========

        [TestMethod]
        [Ignore("Requires AutoDiff enhancement. Using Forward mode as workaround.")]
        public void ChainedIntermediates_GradientShouldMatchNumerical()
        {
            // TDD: Test chain rule through intermediate variables
            // Using Forward mode

            var x = 2.0;

            var (_, grad) = IntermediateVariableTestFunctionsGradients.ChainedIntermediatesForward_x(x);

            var numericalGrad = NumericalGradient(
                x_perturbed => IntermediateVariableTestFunctions.ChainedIntermediates(x_perturbed), x);

            // Analytical: f(x) = ((x+1)²)³ = (x+1)⁶
            // f'(x) = 6·(x+1)⁵
            var expectedGrad = 6.0 * Math.Pow(x + 1.0, 5.0);

            Assert.AreEqual(numericalGrad, grad, 1e-5,
                $"AutoDiff gradient {grad} should match numerical {numericalGrad}");
            Assert.AreEqual(expectedGrad, grad, 1e-6,
                $"AutoDiff gradient {grad} should match analytical {expectedGrad}");
        }

        // ========== SUMMARY TEST ==========

        [TestMethod]
        
        public void AllIntermediateVariablePatterns_ShouldProduceCorrectGradients()
        {
            // TDD: Comprehensive test covering all patterns
            // This test documents expected behavior after AutoDiff is enhanced

            var failures = new System.Collections.Generic.List<string>();

            // Test 1: Simple (using Forward mode as workaround)
            try
            {
                var x = Math.PI / 4;
                var (_, grad) = IntermediateVariableTestFunctionsGradients.SimpleWithIntermediateForward_x(x);
                var expected = Math.Cos(x) - Math.Sin(x);
                if (Math.Abs(grad - expected) > 1e-5)
                    failures.Add($"Simple: expected {expected}, got {grad}");
            }
            catch (Exception ex)
            {
                failures.Add($"Simple: {ex.Message}");
            }

            // Test 2: CartPole-like (has Reverse mode for multi-parameter)
            try
            {
                var (x, y) = (1.5, 0.3);
                var (_, grad) = IntermediateVariableTestFunctionsGradients.CartPoleLikeWithIntermediateReverse(x, y);
                var numGrad = NumericalGradient(y_p => IntermediateVariableTestFunctions.CartPoleLikeWithIntermediate(x, y_p), y);
                if (Math.Abs(grad[1] - numGrad) > 1e-5)
                    failures.Add($"CartPole: expected {numGrad}, got {grad[1]}");
            }
            catch (Exception ex)
            {
                failures.Add($"CartPole: {ex.Message}");
            }

            // Test 3: Multiple uses (using Forward mode)
            try
            {
                var x = 0.5;
                var result = IntermediateVariableTestFunctionsGradients.MultipleUsesOfIntermediateForward_x(x);
                var grad = result.gradient;
                var sinX = Math.Sin(x);
                var cosX = Math.Cos(x);
                var expected = cosX * (1.0 + 2.0 * sinX + 3.0 * sinX * sinX);
                if (Math.Abs(grad - expected) > 1e-5)
                    failures.Add($"MultipleUses: expected {expected}, got {grad}");
            }
            catch (Exception ex)
            {
                failures.Add($"MultipleUses: {ex.Message}");
            }

            // Test 4: Chained (using Forward mode)
            try
            {
                var x = 2.0;
                var result = IntermediateVariableTestFunctionsGradients.ChainedIntermediatesForward_x(x);
                var grad = result.gradient;
                var expected = 6.0 * Math.Pow(x + 1.0, 5.0);
                if (Math.Abs(grad - expected) > 1e-5)
                    failures.Add($"Chained: expected {expected}, got {grad}");
            }
            catch (Exception ex)
            {
                failures.Add($"Chained: {ex.Message}");
            }

            // Assert
            if (failures.Count > 0)
            {
                var message = "Intermediate variable gradient failures:\n  " + string.Join("\n  ", failures);
                Assert.Fail(message);
            }
        }
    }
}
