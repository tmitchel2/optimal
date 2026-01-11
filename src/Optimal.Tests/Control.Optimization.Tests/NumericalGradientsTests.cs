/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Optimization;

namespace Optimal.Control.Optimization.Tests
{
    [TestClass]
    public sealed class NumericalGradientsTests
    {
        [TestMethod]
        public void ComputeGradientReturnsCorrectSizeForLinearFunction()
        {
            Func<double[], double> f = x => x[0] + 2 * x[1] + 3 * x[2];
            var x = new[] { 1.0, 2.0, 3.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(3, gradient.Length);
        }

        [TestMethod]
        public void ComputeGradientReturnsCorrectValuesForLinearFunction()
        {
            // f(x) = x₀ + 2*x₁ + 3*x₂
            // ∇f = [1, 2, 3]
            Func<double[], double> f = x => x[0] + 2 * x[1] + 3 * x[2];
            var x = new[] { 1.0, 2.0, 3.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(1.0, gradient[0], 1e-6);
            Assert.AreEqual(2.0, gradient[1], 1e-6);
            Assert.AreEqual(3.0, gradient[2], 1e-6);
        }

        [TestMethod]
        public void ComputeGradientReturnsCorrectValuesForQuadraticFunction()
        {
            // f(x) = x₀² + x₁²
            // ∇f = [2*x₀, 2*x₁]
            Func<double[], double> f = x => x[0] * x[0] + x[1] * x[1];
            var x = new[] { 3.0, 4.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(6.0, gradient[0], 1e-6);
            Assert.AreEqual(8.0, gradient[1], 1e-6);
        }

        [TestMethod]
        public void ComputeGradientHandlesScalarFunction()
        {
            Func<double[], double> f = x => x[0] * x[0];
            var x = new[] { 5.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(1, gradient.Length);
            Assert.AreEqual(10.0, gradient[0], 1e-6);
        }

        [TestMethod]
        public void ComputeGradientWorksWithCustomEpsilon()
        {
            Func<double[], double> f = x => x[0] * x[0];
            var x = new[] { 3.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x, epsilon: 1e-5);

            Assert.AreEqual(6.0, gradient[0], 1e-4);
        }

        [TestMethod]
        public void ComputeGradientHandlesZeroPoint()
        {
            Func<double[], double> f = x => x[0] * x[0] + x[1] * x[1];
            var x = new[] { 0.0, 0.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(0.0, gradient[0], 1e-6);
            Assert.AreEqual(0.0, gradient[1], 1e-6);
        }

        [TestMethod]
        public void ComputeConstraintGradientDelegatesToComputeGradient()
        {
            Func<double[], double> f = x => x[0] * x[1];
            var x = new[] { 2.0, 3.0 };

            var constraintGradient = NumericalGradients.ComputeConstraintGradient(f, x);
            var regularGradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(regularGradient.Length, constraintGradient.Length);
            for (var i = 0; i < regularGradient.Length; i++)
            {
                Assert.AreEqual(regularGradient[i], constraintGradient[i], 1e-10);
            }
        }

        [TestMethod]
        public void ChooseEpsilonReturnsBaseEpsilonForSmallNorm()
        {
            var x = new[] { 0.1, 0.1 };

            var epsilon = NumericalGradients.ChooseEpsilon(x);

            Assert.AreEqual(1e-8, epsilon, 1e-10);
        }

        [TestMethod]
        public void ChooseEpsilonScalesForLargeNorm()
        {
            var x = new[] { 100.0, 100.0 };
            var norm = Math.Sqrt(100.0 * 100.0 + 100.0 * 100.0);

            var epsilon = NumericalGradients.ChooseEpsilon(x);

            var expected = 1e-8 * Math.Sqrt(norm);
            Assert.AreEqual(expected, epsilon, 1e-15);
        }

        [TestMethod]
        public void ChooseEpsilonHandlesZeroVector()
        {
            var x = new[] { 0.0, 0.0 };

            var epsilon = NumericalGradients.ChooseEpsilon(x);

            Assert.AreEqual(1e-8, epsilon, 1e-10);
        }

        [TestMethod]
        public void ComputeGradientHandlesTrigonometricFunction()
        {
            // f(x) = sin(x₀)
            // f'(x) = cos(x₀)
            Func<double[], double> f = x => Math.Sin(x[0]);
            var x = new[] { Math.PI / 4 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            var expected = Math.Cos(Math.PI / 4);
            Assert.AreEqual(expected, gradient[0], 1e-6);
        }

        [TestMethod]
        public void ComputeGradientHandlesExponentialFunction()
        {
            // f(x) = exp(x₀)
            // f'(x) = exp(x₀)
            Func<double[], double> f = x => Math.Exp(x[0]);
            var x = new[] { 1.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(Math.E, gradient[0], 1e-5);
        }

        [TestMethod]
        public void ComputeGradientHandlesMixedTermsFunction()
        {
            // f(x) = x₀*x₁ + x₁*x₂
            // ∇f = [x₁, x₀ + x₂, x₁]
            Func<double[], double> f = x => x[0] * x[1] + x[1] * x[2];
            var x = new[] { 1.0, 2.0, 3.0 };

            var gradient = NumericalGradients.ComputeGradient(f, x);

            Assert.AreEqual(2.0, gradient[0], 1e-6); // x₁
            Assert.AreEqual(4.0, gradient[1], 1e-6); // x₀ + x₂
            Assert.AreEqual(2.0, gradient[2], 1e-6); // x₁
        }
    }
}
