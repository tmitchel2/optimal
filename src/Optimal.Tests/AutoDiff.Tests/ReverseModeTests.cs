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
    public class ReverseModeTests
    {
        private const double Epsilon = 1e-5;
        private const double Tolerance = 1e-4;

        [TestMethod]
        public void MultiplyReverse()
        {
            var x = 3.0;
            var y = 4.0;
            var (value, gradients) = SimpleTestFunctionsGradients.MultiplyReverse(x, y);

            Assert.AreEqual(12.0, value, 1e-10, "Multiply(3, 4) = 12");
            Assert.HasCount(2, gradients, "Should have 2 gradients");
            Assert.AreEqual(4.0, gradients[0], 1e-10, "∂(x*y)/∂x = y = 4");
            Assert.AreEqual(3.0, gradients[1], 1e-10, "∂(x*y)/∂y = x = 3");

            ValidateAgainstFiniteDifference(x, y, gradients, (a, b) => SimpleTestFunctions.Multiply(a, b));
        }

        [TestMethod]
        public void AddReverse()
        {
            var x = 5.0;
            var y = 7.0;
            var (value, gradients) = SimpleTestFunctionsGradients.AddReverse(x, y);

            Assert.AreEqual(12.0, value, 1e-10, "Add(5, 7) = 12");
            Assert.HasCount(2, gradients, "Should have 2 gradients");
            Assert.AreEqual(1.0, gradients[0], 1e-10, "∂(x+y)/∂x = 1");
            Assert.AreEqual(1.0, gradients[1], 1e-10, "∂(x+y)/∂y = 1");

            ValidateAgainstFiniteDifference(x, y, gradients, (a, b) => SimpleTestFunctions.Add(a, b));
        }

        [TestMethod]
        public void KineticEnergyReverse()
        {
            var mass = 2.0;
            var velocity = 5.0;
            var (value, gradients) = SimpleTestFunctionsGradients.KineticEnergyReverse(mass, velocity);

            Assert.AreEqual(25.0, value, 1e-10, "KE = 0.5 * 2 * 5^2 = 25");
            Assert.HasCount(2, gradients, "Should have 2 gradients");
            Assert.AreEqual(12.5, gradients[0], 1e-10, "∂KE/∂m = 0.5 * v^2 = 12.5");
            Assert.AreEqual(10.0, gradients[1], 1e-10, "∂KE/∂v = m * v = 10");

            ValidateAgainstFiniteDifference(mass, velocity, gradients, (m, v) => SimpleTestFunctions.KineticEnergy(m, v));
        }

        [TestMethod]
        public void ComplexFunctionReverse()
        {
            var x = 2.0;
            var y = 3.0;
            var (value, gradients) = MathFunctionsGradients.ComplexFunctionReverse(x, y);

            var expectedValue = Math.Sin(x) * Math.Exp(y);
            Assert.AreEqual(expectedValue, value, 1e-10, "ComplexFunction(2, 3) = sin(2) * exp(3)");
            Assert.HasCount(2, gradients, "Should have 2 gradients");

            var expectedDx = Math.Cos(x) * Math.Exp(y);
            var expectedDy = Math.Sin(x) * Math.Exp(y);
            Assert.AreEqual(expectedDx, gradients[0], 1e-10, "∂f/∂x = cos(x) * exp(y)");
            Assert.AreEqual(expectedDy, gradients[1], 1e-10, "∂f/∂y = sin(x) * exp(y)");

            ValidateAgainstFiniteDifference(x, y, gradients, (a, b) => MathFunctions.ComplexFunction(a, b));
        }

        [TestMethod]
        public void ReverseModeMatchesForwardMode()
        {
            var x = 3.0;
            var y = 4.0;

            var (valueReverse, gradientsReverse) = SimpleTestFunctionsGradients.MultiplyReverse(x, y);
            var (valueForwardX, gradientX) = SimpleTestFunctionsGradients.MultiplyForward_x(x, y);
            var (valueForwardY, gradientY) = SimpleTestFunctionsGradients.MultiplyForward_y(x, y);

            Assert.AreEqual(valueForwardX, valueReverse, 1e-10, "Value should match");
            Assert.AreEqual(valueForwardY, valueReverse, 1e-10, "Value should match");
            Assert.AreEqual(gradientX, gradientsReverse[0], 1e-10, "Gradient w.r.t. x should match");
            Assert.AreEqual(gradientY, gradientsReverse[1], 1e-10, "Gradient w.r.t. y should match");
        }

        [TestMethod]
        public void MaxReverse()
        {
            var a = 5.0;
            var b = 3.0;
            var (value1, gradients1) = ConditionalFunctionsGradients.MaxReverse(a, b);

            Assert.AreEqual(5.0, value1, 1e-10, "Max(5, 3) = 5");
            Assert.HasCount(2, gradients1, "Should have 2 gradients");
            Assert.AreEqual(1.0, gradients1[0], 1e-10, "∂Max/∂a = 1 when a > b");
            Assert.AreEqual(0.0, gradients1[1], 1e-10, "∂Max/∂b = 0 when a > b");

            var (value2, gradients2) = ConditionalFunctionsGradients.MaxReverse(b, a);

            Assert.AreEqual(5.0, value2, 1e-10, "Max(3, 5) = 5");
            Assert.HasCount(2, gradients2, "Should have 2 gradients");
            Assert.AreEqual(0.0, gradients2[0], 1e-10, "∂Max/∂a = 0 when a < b");
            Assert.AreEqual(1.0, gradients2[1], 1e-10, "∂Max/∂b = 1 when a < b");
        }

        [TestMethod]
        public void MinReverse()
        {
            var a = 5.0;
            var b = 3.0;
            var (value1, gradients1) = ConditionalFunctionsGradients.MinReverse(a, b);

            Assert.AreEqual(3.0, value1, 1e-10, "Min(5, 3) = 3");
            Assert.HasCount(2, gradients1, "Should have 2 gradients");
            Assert.AreEqual(0.0, gradients1[0], 1e-10, "∂Min/∂a = 0 when a > b");
            Assert.AreEqual(1.0, gradients1[1], 1e-10, "∂Min/∂b = 1 when a > b");

            var (value2, gradients2) = ConditionalFunctionsGradients.MinReverse(b, a);

            Assert.AreEqual(3.0, value2, 1e-10, "Min(3, 5) = 3");
            Assert.HasCount(2, gradients2, "Should have 2 gradients");
            Assert.AreEqual(1.0, gradients2[0], 1e-10, "∂Min/∂a = 1 when a < b");
            Assert.AreEqual(0.0, gradients2[1], 1e-10, "∂Min/∂b = 0 when a < b");
        }

        [TestMethod]
        public void ClampReverse()
        {
            var min = 0.0;
            var max = 10.0;

            // Test x < min
            var (value1, gradients1) = ConditionalFunctionsGradients.ClampReverse(-5.0, min, max);
            Assert.AreEqual(0.0, value1, 1e-10, "Clamp(-5, 0, 10) = 0");
            Assert.AreEqual(0.0, gradients1[0], 1e-10, "∂Clamp/∂x = 0 when x < min");
            Assert.AreEqual(1.0, gradients1[1], 1e-10, "∂Clamp/∂min = 1 when x < min");
            Assert.AreEqual(0.0, gradients1[2], 1e-10, "∂Clamp/∂max = 0 when x < min");

            // Test x > max
            var (value2, gradients2) = ConditionalFunctionsGradients.ClampReverse(15.0, min, max);
            Assert.AreEqual(10.0, value2, 1e-10, "Clamp(15, 0, 10) = 10");
            Assert.AreEqual(0.0, gradients2[0], 1e-10, "∂Clamp/∂x = 0 when x > max");
            Assert.AreEqual(0.0, gradients2[1], 1e-10, "∂Clamp/∂min = 0 when x > max");
            Assert.AreEqual(1.0, gradients2[2], 1e-10, "∂Clamp/∂max = 1 when x > max");

            // Test min <= x <= max
            var (value3, gradients3) = ConditionalFunctionsGradients.ClampReverse(5.0, min, max);
            Assert.AreEqual(5.0, value3, 1e-10, "Clamp(5, 0, 10) = 5");
            Assert.AreEqual(1.0, gradients3[0], 1e-10, "∂Clamp/∂x = 1 when min <= x <= max");
            Assert.AreEqual(0.0, gradients3[1], 1e-10, "∂Clamp/∂min = 0 when min <= x <= max");
            Assert.AreEqual(0.0, gradients3[2], 1e-10, "∂Clamp/∂max = 0 when min <= x <= max");
        }

        [TestMethod]
        public void SquareReverseSingleParameter()
        {
            // Test that reverse mode works for single-parameter functions
            var x = 3.0;
            var (value, gradients) = SimpleTestFunctionsGradients.SquareReverse(x);

            Assert.AreEqual(9.0, value, 1e-10, "Square(3) = 9");
            Assert.HasCount(1, gradients, "Should have 1 gradient");
            Assert.AreEqual(6.0, gradients[0], 1e-10, "∂(x²)/∂x = 2x = 6");

            // Validate against finite difference
            ValidateAgainstFiniteDifference(x, gradients[0], v => SimpleTestFunctions.Square(v));
        }

        [TestMethod]
        public void SquareReverseMatchesForwardMode()
        {
            var x = 5.0;

            var (valueReverse, gradientsReverse) = SimpleTestFunctionsGradients.SquareReverse(x);
            var (valueForward, gradientForward) = SimpleTestFunctionsGradients.SquareForward_x(x);

            Assert.AreEqual(valueForward, valueReverse, 1e-10, "Value should match");
            Assert.AreEqual(gradientForward, gradientsReverse[0], 1e-10, "Gradient should match");
        }

        private static void ValidateAgainstFiniteDifference(double x, double analyticalGradient, Func<double, double> function)
        {
            var fX = function(x);
            var fXPlusEps = function(x + Epsilon);

            var numericalGradient = (fXPlusEps - fX) / Epsilon;

            Assert.AreEqual(numericalGradient, analyticalGradient, Tolerance,
                $"Analytical gradient should match numerical gradient (x={x})");
        }

        private static void ValidateAgainstFiniteDifference(double x, double y, double[] analyticalGradients, Func<double, double, double> function)
        {
            var fXY = function(x, y);
            var fXPlusEps = function(x + Epsilon, y);
            var fYPlusEps = function(x, y + Epsilon);

            var numericalGradientX = (fXPlusEps - fXY) / Epsilon;
            var numericalGradientY = (fYPlusEps - fXY) / Epsilon;

            Assert.AreEqual(numericalGradientX, analyticalGradients[0], Tolerance,
                $"Analytical gradient w.r.t. x should match numerical gradient (x={x}, y={y})");
            Assert.AreEqual(numericalGradientY, analyticalGradients[1], Tolerance,
                $"Analytical gradient w.r.t. y should match numerical gradient (x={x}, y={y})");
        }
    }
}
