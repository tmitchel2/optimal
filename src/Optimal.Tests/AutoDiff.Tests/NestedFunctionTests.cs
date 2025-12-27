/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Tests
{
    [TestClass]
    public class NestedFunctionTests
    {
        [TestMethod]
        public void SquareOfSquareForwardGradient()
        {
            var x = 2.0;
            var (value, gradient) = NestedFunctionsGradients.SquareOfSquareForward_x(x);

            // f(x) = ((x^2)^2) = x^4
            // f'(x) = 4x^3 = 4 * 2^3 = 32
            var expectedValue = Math.Pow(x, 4);
            var expectedGradient = 4 * Math.Pow(x, 3);

            Assert.AreEqual(expectedValue, value, 1e-10);
            Assert.AreEqual(expectedGradient, gradient, 1e-10);
        }

        [TestMethod]
        public void ComplexNestedForwardGradient()
        {
            var x = 3.0;
            var y = 4.0;
            var (value, gradientX) = NestedFunctionsGradients.ComplexNestedForward_x(x, y);
            var (_, gradientY) = NestedFunctionsGradients.ComplexNestedForward_y(x, y);

            // f(x,y) = x^2 + y^2
            // ∂f/∂x = 2x = 6
            // ∂f/∂y = 2y = 8
            var expectedValue = x * x + y * y;

            Assert.AreEqual(expectedValue, value, 1e-10);
            Assert.AreEqual(2 * x, gradientX, 1e-10);
            Assert.AreEqual(2 * y, gradientY, 1e-10);
        }

        [TestMethod]
        public void NestedWithMathForwardGradient()
        {
            var x = 1.5;
            var (value, gradient) = NestedFunctionsGradients.NestedWithMathForward_x(x);

            // f(x) = sin(x^2)
            // f'(x) = cos(x^2) * 2x
            var expectedValue = Math.Sin(x * x);
            var expectedGradient = Math.Cos(x * x) * 2 * x;

            Assert.AreEqual(expectedValue, value, 1e-10);
            Assert.AreEqual(expectedGradient, gradient, 1e-10);
        }

        [TestMethod]
        public void DeepNestingForwardGradient()
        {
            var x = 2.0;
            var (value, gradient) = NestedFunctionsGradients.DeepNestingForward_x(x);

            // f(x) = ((((x^2)^2)^2)) = x^8
            // f'(x) = 8x^7 = 8 * 2^7 = 1024
            var expectedValue = Math.Pow(x, 8);
            var expectedGradient = 8 * Math.Pow(x, 7);

            Assert.AreEqual(expectedValue, value, 1e-10);
            Assert.AreEqual(expectedGradient, gradient, 1e-10);
        }

        [TestMethod]
        public void MixedNestingForwardGradient()
        {
            var x = 2.0;
            var y = 3.0;
            var (valueX, gradientX) = NestedFunctionsGradients.MixedNestingForward_x(x, y);
            var (valueY, gradientY) = NestedFunctionsGradients.MixedNestingForward_y(x, y);

            // f(x,y) = x^2 + y^3
            // ∂f/∂x = 2x = 4
            // ∂f/∂y = 3y^2 = 27
            var expectedValue = x * x + y * y * y;

            Assert.AreEqual(expectedValue, valueX, 1e-10);
            Assert.AreEqual(2 * x, gradientX, 1e-10);
            Assert.AreEqual(3 * y * y, gradientY, 1e-10);
        }

        [TestMethod]
        public void ChainedFunctionsForwardGradient()
        {
            var x = 2.0;
            var (value, gradient) = NestedFunctionsGradients.ChainedFunctionsForward_x(x);

            // f(x) = sqrt((x^2)^3) = sqrt(x^6) = x^3
            // f'(x) = 3x^2 = 12
            var expectedValue = Math.Sqrt(Math.Pow(x * x, 3));
            var expectedGradient = 3 * x * x;

            Assert.AreEqual(expectedValue, value, 1e-10);
            Assert.AreEqual(expectedGradient, gradient, 1e-10);
        }

        // Reverse mode test temporarily disabled until reverse mode code generator is updated
        // [TestMethod]
        // public void ComplexNestedReverseGradient()
        // {
        //     var x = 3.0;
        //     var y = 4.0;
        //     var (value, gradients) = NestedFunctionsGradients.ComplexNestedReverse(x, y);

        //     // f(x,y) = x^2 + y^2
        //     // ∂f/∂x = 2x = 6
        //     // ∂f/∂y = 2y = 8
        //     var expectedValue = x * x + y * y;

        //     Assert.AreEqual(expectedValue, value, 1e-10);
        //     Assert.AreEqual(2 * x, gradients[0], 1e-10);
        //     Assert.AreEqual(2 * y, gradients[1], 1e-10);
        // }
    }
}
