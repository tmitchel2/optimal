/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Tests
{
    [TestClass]
    public sealed class GradientDescentOptimizerTests
    {
        private const double Tolerance = 1e-4;
        private static readonly double[] s_quadraticStart = [5.0, 5.0];
        private static readonly double[] s_rosenbrockStart = [-1.2, 1.0];
        private static readonly double[] s_boothStart = [0.0, 0.0];
        private static readonly double[] s_numericalTestPoint = [2.0, 3.0];

        [TestMethod]
        public void CanMinimizeQuadraticFunction()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_quadraticStart);
            optimizer.WithStepSize(0.1);
            optimizer.WithTolerance(1e-6);
            optimizer.WithMaxIterations(1000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(0.0, result.OptimalPoint[0], Tolerance, "x should be near 0");
            Assert.AreEqual(0.0, result.OptimalPoint[1], Tolerance, "y should be near 0");
            Assert.AreEqual(0.0, result.OptimalValue, 1e-6, "f(x*) should be near 0");
            // Can converge via gradient, function, or parameter tolerance
            Assert.IsTrue(
                result.StoppingReason == StoppingReason.GradientTolerance ||
                result.StoppingReason == StoppingReason.FunctionTolerance ||
                result.StoppingReason == StoppingReason.ParameterTolerance,
                $"Should converge via tolerance, got {result.StoppingReason}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrockFunction()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_rosenbrockStart);
            optimizer.WithStepSize(0.001);
            optimizer.WithTolerance(1e-4);
            optimizer.WithMaxIterations(50000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
            Assert.IsTrue(result.OptimalValue < 1e-3, $"f(x*) should be very small, was {result.OptimalValue}");
            // Can converge via gradient, function, or parameter tolerance
            Assert.IsTrue(
                result.StoppingReason == StoppingReason.GradientTolerance ||
                result.StoppingReason == StoppingReason.FunctionTolerance ||
                result.StoppingReason == StoppingReason.ParameterTolerance,
                $"Should converge via tolerance, got {result.StoppingReason}");
        }

        [TestMethod]
        public void GradientMatchesNumericalGradient()
        {
            var x = s_numericalTestPoint;
            var epsilon = 1e-7;

            var (value, analyticalGrad) = TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]);

            // Compute numerical gradient using finite differences
            var fx = TestObjectiveFunctions.Rosenbrock(x[0], x[1]);
            var fxPlusEps0 = TestObjectiveFunctions.Rosenbrock(x[0] + epsilon, x[1]);
            var fxPlusEps1 = TestObjectiveFunctions.Rosenbrock(x[0], x[1] + epsilon);

            var numericalGrad0 = (fxPlusEps0 - fx) / epsilon;
            var numericalGrad1 = (fxPlusEps1 - fx) / epsilon;

            Assert.AreEqual(numericalGrad0, analyticalGrad[0], 1e-2, "Gradient w.r.t. x should match");
            Assert.AreEqual(numericalGrad1, analyticalGrad[1], 1e-2, "Gradient w.r.t. y should match");
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_rosenbrockStart);
            optimizer.WithStepSize(0.001);
            optimizer.WithTolerance(1e-10);
            optimizer.WithMaxIterations(10);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsFalse(result.Success, "Optimization should not succeed");
            Assert.AreEqual(StoppingReason.MaxIterations, result.StoppingReason);
            Assert.AreEqual(10, result.Iterations);
        }

        [TestMethod]
        public void FunctionEvaluationCountIsCorrect()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_quadraticStart);
            optimizer.WithStepSize(0.1);
            optimizer.WithTolerance(1e-6);
            optimizer.WithMaxIterations(1000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success);
            Assert.AreEqual(result.Iterations, result.FunctionEvaluations,
                "Function evaluations should equal iterations for converged case");
        }

        [TestMethod]
        public void CanMinimizeBoothFunction()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_boothStart);
            optimizer.WithStepSize(0.01);
            optimizer.WithTolerance(1e-6);
            optimizer.WithMaxIterations(10000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.BoothReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(3.0, result.OptimalPoint[1], 1e-2, "y should be near 3");
            Assert.AreEqual(0.0, result.OptimalValue, 1e-4, "f(x*) should be near 0");
        }

        [TestMethod]
        public void CanMinimizeWithLineSearch()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_quadraticStart);
            optimizer.WithLineSearch(new LineSearch.BacktrackingLineSearch());
            optimizer.WithTolerance(1e-6);
            optimizer.WithMaxIterations(1000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed with line search");
            Assert.AreEqual(0.0, result.OptimalPoint[0], Tolerance, "x should be near 0");
            Assert.AreEqual(0.0, result.OptimalPoint[1], Tolerance, "y should be near 0");
        }

        [TestMethod]
        public void CanMinimizeWithFunctionTolerance()
        {
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_quadraticStart);
            optimizer.WithStepSize(0.1);
            optimizer.WithFunctionTolerance(1e-8);
            optimizer.WithMaxIterations(1000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed with function tolerance");
            Assert.IsTrue(result.OptimalValue < 1e-4, "Function value should be small");
        }
    }
}
