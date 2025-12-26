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
    public class LoopFunctionTests
    {
        private const double Epsilon = 1e-5;
        private const double Tolerance = 1e-4;

        [TestMethod]
        public void SumOfSquaresGradient()
        {
            var x = 3.0;
            var n = 5;
            var (value, gradient) = LoopFunctionsGradients.SumOfSquaresForward_x(x, n);

            Assert.AreEqual(45.0, value, 1e-10, "SumOfSquares(3, 5) = 5 * 3^2 = 45");
            Assert.AreEqual(30.0, gradient, 1e-10, "d(SumOfSquares)/dx = 2 * n * x = 2 * 5 * 3 = 30");
            ValidateAgainstFiniteDifference(x, n, gradient, (xVal, nVal) => LoopFunctions.SumOfSquares(xVal, nVal));
        }

        [TestMethod]
        public void SumOfCubesGradient()
        {
            var x = 2.0;
            var n = 4;
            var (value, gradient) = LoopFunctionsGradients.SumOfCubesForward_x(x, n);

            Assert.AreEqual(32.0, value, 1e-10, "SumOfCubes(2, 4) = 4 * 2^3 = 32");
            Assert.AreEqual(48.0, gradient, 1e-10, "d(SumOfCubes)/dx = 3 * n * x^2 = 3 * 4 * 4 = 48");
            ValidateAgainstFiniteDifferenceLoose(x, n, gradient, (xVal, nVal) => LoopFunctions.SumOfCubes(xVal, nVal));
        }

        private static void ValidateAgainstFiniteDifferenceLoose(double x, int n, double analyticalGradient, Func<double, int, double> function)
        {
            var fX = function(x, n);
            var fXPlusEps = function(x + Epsilon, n);
            var numericalGradient = (fXPlusEps - fX) / Epsilon;

            Assert.AreEqual(numericalGradient, analyticalGradient, 3e-4,
                $"Analytical gradient should match numerical gradient (x={x}, n={n})");
        }

        [TestMethod]
        public void LinearSumGradient()
        {
            var x = 5.0;
            var n = 10;
            var (value, gradient) = LoopFunctionsGradients.LinearSumForward_x(x, n);

            Assert.AreEqual(50.0, value, 1e-10, "LinearSum(5, 10) = 10 * 5 = 50");
            Assert.AreEqual(10.0, gradient, 1e-10, "d(LinearSum)/dx = n = 10");
            ValidateAgainstFiniteDifference(x, n, gradient, (xVal, nVal) => LoopFunctions.LinearSum(xVal, nVal));
        }

        [TestMethod]
        public void ProductAccumulationGradient()
        {
            var x = 2.0;
            var n = 3;
            var (value, gradient) = LoopFunctionsGradients.ProductAccumulationForward_x(x, n);

            Assert.AreEqual(8.0, value, 1e-10, "ProductAccumulation(2, 3) = 2^3 = 8");
            Assert.AreEqual(12.0, gradient, 1e-10, "d(x^n)/dx = n * x^(n-1) = 3 * 2^2 = 12");
            ValidateAgainstFiniteDifference(x, n, gradient, (xVal, nVal) => LoopFunctions.ProductAccumulation(xVal, nVal));
        }

        [TestMethod]
        public void IndexedSumGradient()
        {
            var x = 2.0;
            var n = 5;
            var (value, gradient) = LoopFunctionsGradients.IndexedSumForward_x(x, n);

            var expectedValue = 0.0 + 1.0 * 2.0 + 2.0 * 2.0 + 3.0 * 2.0 + 4.0 * 2.0;
            Assert.AreEqual(expectedValue, value, 1e-10, "IndexedSum(2, 5) = (0+1+2+3+4) * 2 = 20");

            var expectedGradient = 0.0 + 1.0 + 2.0 + 3.0 + 4.0;
            Assert.AreEqual(expectedGradient, gradient, 1e-10, "d(IndexedSum)/dx = 0+1+2+3+4 = 10");
            ValidateAgainstFiniteDifference(x, n, gradient, (xVal, nVal) => LoopFunctions.IndexedSum(xVal, nVal));
        }

        private static void ValidateAgainstFiniteDifference(double x, int n, double analyticalGradient, Func<double, int, double> function)
        {
            var fX = function(x, n);
            var fXPlusEps = function(x + Epsilon, n);
            var numericalGradient = (fXPlusEps - fX) / Epsilon;

            Assert.AreEqual(numericalGradient, analyticalGradient, Tolerance,
                $"Analytical gradient should match numerical gradient (x={x}, n={n})");
        }
    }
}
