/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Tests;

namespace Optimal.NonLinear.LineSearch.Tests
{
    [TestClass]
    public sealed class BacktrackingLineSearchTests
    {
        private static readonly double[] s_quadraticStart = new[] { 5.0, 5.0 };
        private static readonly double[] s_rosenbrockStart = new[] { -1.2, 1.0 };

        [TestMethod]
        public void CanFindStepSizeForQuadraticFunction()
        {
            var lineSearch = new BacktrackingLineSearch();
            var x = s_quadraticStart;

            // Evaluate at current point
            var (fx, gradient) = TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]);

            // Search direction is negative gradient
            var direction = new double[] { -gradient[0], -gradient[1] };

            // Find step size
            var alpha = lineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            // Should find a positive step size
            Assert.IsTrue(alpha > 0, $"Step size should be positive, was {alpha}");
            Assert.IsTrue(alpha <= 1.0, $"Step size should not exceed initial, was {alpha}");

            // Verify that the step reduces the objective
            var xNew = new double[] { x[0] + alpha * direction[0], x[1] + alpha * direction[1] };
            var (fNew, _) = TestObjectiveFunctionsGradients.QuadraticReverse(xNew[0], xNew[1]);
            Assert.IsTrue(fNew < fx, $"Function should decrease: {fx} -> {fNew}");
        }

        [TestMethod]
        public void CanFindStepSizeForRosenbrockFunction()
        {
            var lineSearch = new BacktrackingLineSearch();
            var x = s_rosenbrockStart;

            // Evaluate at current point
            var (fx, gradient) = TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]);

            // Search direction is negative gradient
            var direction = new double[] { -gradient[0], -gradient[1] };

            // Find step size
            var alpha = lineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            // Should find a positive step size
            Assert.IsTrue(alpha > 0, $"Step size should be positive, was {alpha}");

            // Verify that the step reduces the objective
            var xNew = new double[] { x[0] + alpha * direction[0], x[1] + alpha * direction[1] };
            var (fNew, _) = TestObjectiveFunctionsGradients.RosenbrockReverse(xNew[0], xNew[1]);
            Assert.IsTrue(fNew < fx, $"Function should decrease: {fx} -> {fNew}");
        }

        [TestMethod]
        public void ReturnsZeroForUphillDirection()
        {
            var lineSearch = new BacktrackingLineSearch();
            var x = new[] { 1.0, 1.0 };

            var (fx, gradient) = TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]);

            // Use the gradient direction (uphill) instead of negative gradient
            var direction = new double[] { gradient[0], gradient[1] };

            var alpha = lineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            Assert.AreEqual(0.0, alpha, "Should return 0 for uphill direction");
        }

        [TestMethod]
        public void OptimizerWithLineSearchConvergesOnRosenbrock()
        {
            var lineSearch = new BacktrackingLineSearch();
            var optimizer = new GradientDescentOptimizer();
            optimizer.WithInitialPoint(s_rosenbrockStart);
            optimizer.WithLineSearch(lineSearch);
            optimizer.WithTolerance(1e-5);
            optimizer.WithMaxIterations(10000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 6e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 6e-2, "y should be near 1");
            Assert.IsTrue(result.OptimalValue < 1e-3, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void OptimizerWithLineSearchIsFasterThanFixed()
        {
            // Test that line search requires fewer iterations than fixed step size
            var lineSearch = new BacktrackingLineSearch();

            // Optimizer with line search
            var optimizerLS = new GradientDescentOptimizer();
            optimizerLS.WithInitialPoint(s_rosenbrockStart);
            optimizerLS.WithLineSearch(lineSearch);
            optimizerLS.WithTolerance(1e-4);
            optimizerLS.WithMaxIterations(10000);

            var resultLS = optimizerLS.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            // Optimizer with fixed step size (same as Phase 1 test)
            var optimizerFixed = new GradientDescentOptimizer();
            optimizerFixed.WithInitialPoint(s_rosenbrockStart);
            optimizerFixed.WithStepSize(0.001);
            optimizerFixed.WithTolerance(1e-4);
            optimizerFixed.WithMaxIterations(50000);

            var resultFixed = optimizerFixed.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            // Line search should require significantly fewer iterations
            Assert.IsTrue(resultLS.Success, "Line search optimization should succeed");
            Assert.IsTrue(resultFixed.Success, "Fixed step optimization should succeed");
            Assert.IsTrue(resultLS.Iterations < resultFixed.Iterations / 2,
                $"Line search ({resultLS.Iterations} iters) should be faster than fixed ({resultFixed.Iterations} iters)");
        }
    }
}
