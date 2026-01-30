/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Tests;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.LineSearch.Tests
{
    [TestClass]
    public sealed class ParallelBacktrackingLineSearchTests
    {
        private static readonly double[] s_quadraticStart = [5.0, 5.0];
        private static readonly double[] s_rosenbrockStart = [-1.2, 1.0];

        [TestMethod]
        public void ParallelLineSearchFindsStepSizeForQuadraticFunction()
        {
            var lineSearch = new ParallelBacktrackingLineSearch(parallelBatchSize: 4);
            var x = s_quadraticStart;

            var (fx, gradient) = TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]);
            var direction = new double[] { -gradient[0], -gradient[1] };

            var alpha = lineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            Assert.IsGreaterThan(0, alpha, $"Step size should be positive, was {alpha}");
            Assert.IsLessThanOrEqualTo(1.0, alpha, $"Step size should not exceed initial, was {alpha}");

            var xNew = new double[] { x[0] + alpha * direction[0], x[1] + alpha * direction[1] };
            var (fNew, _) = TestObjectiveFunctionsGradients.QuadraticReverse(xNew[0], xNew[1]);
            Assert.IsLessThan(fx, fNew, $"Function should decrease: {fx} -> {fNew}");
        }

        [TestMethod]
        public void ParallelLineSearchFindsStepSizeForRosenbrockFunction()
        {
            var lineSearch = new ParallelBacktrackingLineSearch(parallelBatchSize: 4);
            var x = s_rosenbrockStart;

            var (fx, gradient) = TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]);
            var direction = new double[] { -gradient[0], -gradient[1] };

            var alpha = lineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            Assert.IsGreaterThan(0, alpha, $"Step size should be positive, was {alpha}");

            var xNew = new double[] { x[0] + alpha * direction[0], x[1] + alpha * direction[1] };
            var (fNew, _) = TestObjectiveFunctionsGradients.RosenbrockReverse(xNew[0], xNew[1]);
            Assert.IsLessThan(fx, fNew, $"Function should decrease: {fx} -> {fNew}");
        }

        [TestMethod]
        public void ParallelLineSearchReturnsZeroForUphillDirection()
        {
            var lineSearch = new ParallelBacktrackingLineSearch(parallelBatchSize: 4);
            var x = new[] { 1.0, 1.0 };

            var (fx, gradient) = TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]);
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
        public void ParallelLineSearchMatchesSequentialResult()
        {
            var parallelLineSearch = new ParallelBacktrackingLineSearch(parallelBatchSize: 4);
            var sequentialLineSearch = new BacktrackingLineSearch();
            var x = s_rosenbrockStart;

            var (fx, gradient) = TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]);
            var direction = new double[] { -gradient[0], -gradient[1] };

            var alphaParallel = parallelLineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            var alphaSequential = sequentialLineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            Assert.AreEqual(alphaSequential, alphaParallel, 1e-10,
                "Parallel and sequential line search should produce same step size");
        }

        [TestMethod]
        public void ParallelLineSearchCanBeDisabled()
        {
            var lineSearch = new ParallelBacktrackingLineSearch(enableParallelization: false);
            var x = s_quadraticStart;

            var (fx, gradient) = TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]);
            var direction = new double[] { -gradient[0], -gradient[1] };

            var alpha = lineSearch.FindStepSize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                x,
                fx,
                gradient,
                direction,
                1.0);

            Assert.IsGreaterThan(0, alpha, $"Step size should be positive even when parallelization disabled, was {alpha}");
        }

        [TestMethod]
        public void OptimizerWithParallelLineSearchConvergesOnRosenbrock()
        {
            var lineSearch = new ParallelBacktrackingLineSearch(parallelBatchSize: 4);
            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-5, MaxIterations = 1000 },
                lineSearch);

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrockStart);

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-3, "y should be near 1");
            Assert.IsLessThan(1e-6, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void ParallelLineSearchWithDifferentBatchSizes()
        {
            var x = s_rosenbrockStart;
            var (fx, gradient) = TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]);
            var direction = new double[] { -gradient[0], -gradient[1] };

            // Test with different batch sizes
            var batchSizes = new[] { 1, 2, 4, 8 };
            double? expectedAlpha = null;

            foreach (var batchSize in batchSizes)
            {
                var lineSearch = new ParallelBacktrackingLineSearch(parallelBatchSize: batchSize);
                var alpha = lineSearch.FindStepSize(
                    x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                    x,
                    fx,
                    gradient,
                    direction,
                    1.0);

                Assert.IsGreaterThan(0, alpha, $"Step size should be positive for batch size {batchSize}");

                if (expectedAlpha.HasValue)
                {
                    Assert.AreEqual(expectedAlpha.Value, alpha, 1e-10,
                        $"All batch sizes should produce same step size");
                }
                else
                {
                    expectedAlpha = alpha;
                }
            }
        }
    }
}
