/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Tests
{
    [TestClass]
    public class MathFunctionTestCases
    {
        private const double Epsilon = 1e-5;
        private const double Tolerance = 1e-4;

        [TestMethod]
        public void SquareRootGradient()
        {
            var x = 4.0;
            var (value, gradient) = MathFunctionsGradients.SquareRootForward_x(x);

            Assert.AreEqual(2.0, value, 1e-10, "sqrt(4) = 2");
            Assert.AreEqual(0.25, gradient, 1e-10, "d(sqrt(x))/dx = 1/(2*sqrt(x)) = 1/4");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.SquareRoot(x));
        }

        [TestMethod]
        public void SineGradient()
        {
            var x = Math.PI / 4.0;
            var (value, gradient) = MathFunctionsGradients.SineForward_x(x);

            Assert.AreEqual(Math.Sin(x), value, 1e-10);
            Assert.AreEqual(Math.Cos(x), gradient, 1e-10, "d(sin(x))/dx = cos(x)");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.Sine(x));
        }

        [TestMethod]
        public void CosineGradient()
        {
            var x = Math.PI / 3.0;
            var (value, gradient) = MathFunctionsGradients.CosineForward_x(x);

            Assert.AreEqual(Math.Cos(x), value, 1e-10);
            Assert.AreEqual(-Math.Sin(x), gradient, 1e-10, "d(cos(x))/dx = -sin(x)");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.Cosine(x));
        }

        [TestMethod]
        public void TangentGradient()
        {
            var x = Math.PI / 6.0;
            var (value, gradient) = MathFunctionsGradients.TangentForward_x(x);

            Assert.AreEqual(Math.Tan(x), value, 1e-10);
            var expectedGradient = 1.0 / (Math.Cos(x) * Math.Cos(x));
            Assert.AreEqual(expectedGradient, gradient, 1e-10, "d(tan(x))/dx = 1/cos²(x)");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.Tangent(x));
        }

        [TestMethod]
        public void ExponentialGradient()
        {
            var x = 2.0;
            var (value, gradient) = MathFunctionsGradients.ExponentialForward_x(x);

            Assert.AreEqual(Math.Exp(x), value, 1e-10);
            Assert.AreEqual(Math.Exp(x), gradient, 1e-10, "d(exp(x))/dx = exp(x)");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.Exponential(x));
        }

        [TestMethod]
        public void LogarithmGradient()
        {
            var x = 2.0;
            var (value, gradient) = MathFunctionsGradients.LogarithmForward_x(x);

            Assert.AreEqual(Math.Log(x), value, 1e-10);
            Assert.AreEqual(1.0 / x, gradient, 1e-10, "d(log(x))/dx = 1/x");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.Logarithm(x));
        }

        [TestMethod]
        public void PowerGradient()
        {
            var x = 2.0;
            var n = 3.0;

            var (value1, gradX) = MathFunctionsGradients.PowerForward_x(x, n);
            Assert.AreEqual(8.0, value1, 1e-10, "2^3 = 8");
            Assert.AreEqual(12.0, gradX, 1e-10, "d(x^n)/dx = n*x^(n-1) = 3*2^2 = 12");

            ValidateAgainstFiniteDifference(x, gradX, x => MathFunctions.Power(x, n));

            var (value2, gradN) = MathFunctionsGradients.PowerForward_n(x, n);
            Assert.AreEqual(8.0, value2, 1e-10);
            var expectedGradN = Math.Pow(x, n) * Math.Log(x);
            Assert.AreEqual(expectedGradN, gradN, 1e-10, "d(x^n)/dn = x^n * ln(x)");

            ValidateAgainstFiniteDifference(n, gradN, n => MathFunctions.Power(x, n));
        }

        [TestMethod]
        public void AbsoluteValueGradient()
        {
            var x = 3.0;
            var (value, gradient) = MathFunctionsGradients.AbsoluteValueForward_x(x);

            Assert.AreEqual(3.0, value, 1e-10);
            Assert.AreEqual(1.0, gradient, 1e-10, "d(abs(x))/dx = x/abs(x) = 1 for x > 0");

            var xNeg = -3.0;
            var (valueNeg, gradientNeg) = MathFunctionsGradients.AbsoluteValueForward_x(xNeg);
            Assert.AreEqual(3.0, valueNeg, 1e-10);
            Assert.AreEqual(-1.0, gradientNeg, 1e-10, "d(abs(x))/dx = x/abs(x) = -1 for x < 0");
        }

        [TestMethod]
        public void VelocityGradient()
        {
            var height = 10.0;
            var (velocity, gradient) = MathFunctionsGradients.VelocityForward_height(height);

            var expectedVelocity = Math.Sqrt(2.0 * 9.81 * height);
            Assert.AreEqual(expectedVelocity, velocity, 1e-10);

            var expectedGradient = 9.81 / Math.Sqrt(2.0 * 9.81 * height);
            Assert.AreEqual(expectedGradient, gradient, 1e-10);

            ValidateAgainstFiniteDifference(height, gradient, h => MathFunctions.Velocity(h));
        }

        [TestMethod]
        public void SinPlusExpGradient()
        {
            var x = 1.0;
            var (value, gradient) = MathFunctionsGradients.SinPlusExpForward_x(x);

            Assert.AreEqual(Math.Sin(x) + Math.Exp(x), value, 1e-10);
            Assert.AreEqual(Math.Cos(x) + Math.Exp(x), gradient, 1e-10, "d(sin(x) + exp(x))/dx = cos(x) + exp(x)");

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.SinPlusExp(x));
        }

        [TestMethod]
        public void ComplexFunctionGradient()
        {
            var x = 2.0;
            var (value, gradient) = MathFunctionsGradients.ComplexFunctionForward_x(x);

            var expectedValue = Math.Sqrt(x) * Math.Sin(x) + Math.Log(x);
            Assert.AreEqual(expectedValue, value, 1e-10);

            var dSqrt = 1.0 / (2.0 * Math.Sqrt(x));
            var dSin = Math.Cos(x);
            var dLog = 1.0 / x;
            var expectedGradient = dSqrt * Math.Sin(x) + Math.Sqrt(x) * dSin + dLog;
            Assert.AreEqual(expectedGradient, gradient, 1e-9);

            ValidateAgainstFiniteDifference(x, gradient, x => MathFunctions.ComplexFunction(x));
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
