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
    public class ConditionalFunctionTests
    {
        private const double Epsilon = 1e-5;
        private const double Tolerance = 1e-4;

        [TestMethod]
        public void ReLUGradient()
        {
            var x1 = 2.0;
            var (value1, gradient1) = ConditionalFunctionsGradients.ReLUForward_x(x1);
            Assert.AreEqual(2.0, value1, 1e-10, "ReLU(2) = 2");
            Assert.AreEqual(1.0, gradient1, 1e-10, "d(ReLU)/dx = 1 for x > 0");
            ValidateAgainstFiniteDifference(x1, gradient1, x => ConditionalFunctions.ReLU(x));

            var x2 = -2.0;
            var (value2, gradient2) = ConditionalFunctionsGradients.ReLUForward_x(x2);
            Assert.AreEqual(0.0, value2, 1e-10, "ReLU(-2) = 0");
            Assert.AreEqual(0.0, gradient2, 1e-10, "d(ReLU)/dx = 0 for x < 0");
            ValidateAgainstFiniteDifference(x2, gradient2, x => ConditionalFunctions.ReLU(x));
        }

        [TestMethod]
        public void LeakyReLUGradient()
        {
            var x1 = 2.0;
            var (value1, gradient1) = ConditionalFunctionsGradients.LeakyReLUForward_x(x1);
            Assert.AreEqual(2.0, value1, 1e-10, "LeakyReLU(2) = 2");
            Assert.AreEqual(1.0, gradient1, 1e-10, "d(LeakyReLU)/dx = 1 for x > 0");

            var x2 = -2.0;
            var (value2, gradient2) = ConditionalFunctionsGradients.LeakyReLUForward_x(x2);
            Assert.AreEqual(-0.02, value2, 1e-10, "LeakyReLU(-2) = -0.02");
            Assert.AreEqual(0.01, gradient2, 1e-10, "d(LeakyReLU)/dx = 0.01 for x < 0");
            ValidateAgainstFiniteDifference(x2, gradient2, x => ConditionalFunctions.LeakyReLU(x));
        }

        [TestMethod]
        public void MaxGradient()
        {
            var a1 = 3.0;
            var b1 = 2.0;
            var (value1, gradA1) = ConditionalFunctionsGradients.MaxForward_a(a1, b1);
            Assert.AreEqual(3.0, value1, 1e-10, "Max(3, 2) = 3");
            Assert.AreEqual(1.0, gradA1, 1e-10, "d(Max)/da = 1 when a > b");

            var (value2, gradB1) = ConditionalFunctionsGradients.MaxForward_b(a1, b1);
            Assert.AreEqual(3.0, value2, 1e-10);
            Assert.AreEqual(0.0, gradB1, 1e-10, "d(Max)/db = 0 when a > b");

            var a2 = 2.0;
            var b2 = 3.0;
            var (value3, gradA2) = ConditionalFunctionsGradients.MaxForward_a(a2, b2);
            Assert.AreEqual(3.0, value3, 1e-10, "Max(2, 3) = 3");
            Assert.AreEqual(0.0, gradA2, 1e-10, "d(Max)/da = 0 when a < b");

            var (value4, gradB2) = ConditionalFunctionsGradients.MaxForward_b(a2, b2);
            Assert.AreEqual(3.0, value4, 1e-10);
            Assert.AreEqual(1.0, gradB2, 1e-10, "d(Max)/db = 1 when a < b");
        }

        [TestMethod]
        public void MinGradient()
        {
            var a1 = 2.0;
            var b1 = 3.0;
            var (value1, gradA1) = ConditionalFunctionsGradients.MinForward_a(a1, b1);
            Assert.AreEqual(2.0, value1, 1e-10, "Min(2, 3) = 2");
            Assert.AreEqual(1.0, gradA1, 1e-10, "d(Min)/da = 1 when a < b");

            var (value2, gradB1) = ConditionalFunctionsGradients.MinForward_b(a1, b1);
            Assert.AreEqual(2.0, value2, 1e-10);
            Assert.AreEqual(0.0, gradB1, 1e-10, "d(Min)/db = 0 when a < b");
        }

        [TestMethod]
        public void ClampGradient()
        {
            var min = -1.0;
            var max = 1.0;

            var xLow = -2.0;
            var (valueLow, gradLow) = ConditionalFunctionsGradients.ClampForward_x(xLow, min, max);
            Assert.AreEqual(-1.0, valueLow, 1e-10, "Clamp(-2, -1, 1) = -1");
            Assert.AreEqual(0.0, gradLow, 1e-10, "d(Clamp)/dx = 0 when x < min");

            var xMid = 0.5;
            var (valueMid, gradMid) = ConditionalFunctionsGradients.ClampForward_x(xMid, min, max);
            Assert.AreEqual(0.5, valueMid, 1e-10, "Clamp(0.5, -1, 1) = 0.5");
            Assert.AreEqual(1.0, gradMid, 1e-10, "d(Clamp)/dx = 1 when min < x < max");

            var xHigh = 2.0;
            var (valueHigh, gradHigh) = ConditionalFunctionsGradients.ClampForward_x(xHigh, min, max);
            Assert.AreEqual(1.0, valueHigh, 1e-10, "Clamp(2, -1, 1) = 1");
            Assert.AreEqual(0.0, gradHigh, 1e-10, "d(Clamp)/dx = 0 when x > max");
        }

        [TestMethod]
        public void PiecewiseLinearGradient()
        {
            var x1 = -2.0;
            var (value1, gradient1) = ConditionalFunctionsGradients.PiecewiseLinearForward_x(x1);
            Assert.AreEqual(1.0, value1, 1e-10, "PiecewiseLinear(-2) = -(-2) - 1 = 1");
            Assert.AreEqual(-1.0, gradient1, 1e-10, "d/dx = -1 for x < -1");

            var x2 = 0.0;
            var (value2, gradient2) = ConditionalFunctionsGradients.PiecewiseLinearForward_x(x2);
            Assert.AreEqual(0.0, value2, 1e-10, "PiecewiseLinear(0) = 0");
            Assert.AreEqual(0.0, gradient2, 1e-10, "d/dx = 0 for -1 <= x <= 1");

            var x3 = 2.0;
            var (value3, gradient3) = ConditionalFunctionsGradients.PiecewiseLinearForward_x(x3);
            Assert.AreEqual(1.0, value3, 1e-10, "PiecewiseLinear(2) = 2 - 1 = 1");
            Assert.AreEqual(1.0, gradient3, 1e-10, "d/dx = 1 for x > 1");
        }

        [TestMethod]
        public void QuadraticGradient()
        {
            var x1 = -2.0;
            var (value1, gradient1) = ConditionalFunctionsGradients.QuadraticForward_x(x1);
            Assert.AreEqual(4.0, value1, 1e-10, "Quadratic(-2) = (-2)^2 = 4");
            Assert.AreEqual(-4.0, gradient1, 1e-10, "d/dx = 2x = -4 for x < 0");
            ValidateAgainstFiniteDifference(x1, gradient1, x => ConditionalFunctions.Quadratic(x));

            var x2 = 2.0;
            var (value2, gradient2) = ConditionalFunctionsGradients.QuadraticForward_x(x2);
            Assert.AreEqual(4.0, value2, 1e-10, "Quadratic(2) = 2*2 = 4");
            Assert.AreEqual(2.0, gradient2, 1e-10, "d/dx = 2 for x >= 0");
            ValidateAgainstFiniteDifference(x2, gradient2, x => ConditionalFunctions.Quadratic(x));
        }

        [TestMethod]
        public void SmoothStepGradient()
        {
            var x1 = -0.5;
            var (value1, gradient1) = ConditionalFunctionsGradients.SmoothStepForward_x(x1);
            Assert.AreEqual(0.0, value1, 1e-10, "SmoothStep(x) = 0 for x <= 0");
            Assert.AreEqual(0.0, gradient1, 1e-10, "d/dx = 0 for x <= 0");

            var x2 = 0.5;
            var (value2, gradient2) = ConditionalFunctionsGradients.SmoothStepForward_x(x2);
            var expectedValue = 3.0 * 0.5 * 0.5 - 2.0 * 0.5 * 0.5 * 0.5;
            Assert.AreEqual(expectedValue, value2, 1e-10);
            var expectedGrad = 6.0 * 0.5 - 6.0 * 0.5 * 0.5;
            Assert.AreEqual(expectedGrad, gradient2, 1e-10);
            ValidateAgainstFiniteDifference(x2, gradient2, x => ConditionalFunctions.SmoothStep(x));

            var x3 = 1.5;
            var (value3, gradient3) = ConditionalFunctionsGradients.SmoothStepForward_x(x3);
            Assert.AreEqual(1.0, value3, 1e-10, "SmoothStep(x) = 1 for x >= 1");
            Assert.AreEqual(0.0, gradient3, 1e-10, "d/dx = 0 for x >= 1");
        }

        private static void ValidateAgainstFiniteDifference(double x, double analyticalGradient, Func<double, double> function)
        {
            var fX = function(x);
            var fXPlusEps = function(x + Epsilon);
            var numericalGradient = (fXPlusEps - fX) / Epsilon;

            Assert.AreEqual(numericalGradient, analyticalGradient, Tolerance,
                $"Analytical gradient should match numerical gradient (x={x})");
        }
    }
}
