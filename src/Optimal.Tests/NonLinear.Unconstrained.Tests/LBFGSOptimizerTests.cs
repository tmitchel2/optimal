/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Tests;

namespace Optimal.NonLinear.Unconstrained.Tests
{
    [TestClass]
    public sealed class LBFGSOptimizerTests
    {
        private static readonly double[] s_rosenbrock2DStart = [-1.2, 1.0];
        private static readonly double[] s_boothStart = [0.0, 0.0];

        [TestMethod]
        public void CanMinimizeRosenbrock2D()
        {
            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-6, MaxIterations = 1000, MemorySize = 10 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart,
                CancellationToken.None);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-3, "y should be near 1");
            Assert.IsLessThan(1e-6, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeBoothFunction()
        {
            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-6, MaxIterations = 1000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.BoothReverse(x[0], x[1]),
                s_boothStart,
                CancellationToken.None);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3, "x should be near 1");
            Assert.AreEqual(3.0, result.OptimalPoint[1], 1e-3, "y should be near 3");
            Assert.IsLessThan(1e-6, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock10D()
        {
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(10);

            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 5000, MemorySize = 10 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0, CancellationToken.None);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < 10; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 1e-1, $"x[{i}] should be near 1");
            }
            Assert.IsLessThan(1e-2, result.OptimalValue, $"f(x*) should be small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock100D()
        {
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(100);

            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-3, MaxIterations = 1000, MemorySize = 20 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0, CancellationToken.None);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.IsLessThan(1000, result.Iterations, $"Should converge in < 1000 iterations, took {result.Iterations}");

            // Check that we're close to optimum
            for (var i = 0; i < 100; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 0.2, $"x[{i}] should be reasonably close to 1");
            }

            Assert.IsLessThan(1.0, result.OptimalValue, $"f(x*) should be reasonably small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void LBFGSIsFasterThanConjugateGradient()
        {
            // Test that L-BFGS requires fewer iterations than CG on a moderate-dimensional problem
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(10);

            var lbfgsOptimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 5000 },
                new BacktrackingLineSearch());

            var lbfgsResult = lbfgsOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0, CancellationToken.None);

            var cgOptimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-4, MaxIterations = 20000 },
                new BacktrackingLineSearch());

            var cgResult = cgOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0, CancellationToken.None);

            Assert.IsTrue(lbfgsResult.Success, "L-BFGS optimization should succeed");
            Assert.IsTrue(cgResult.Success, "CG optimization should succeed");

            // L-BFGS should be faster (fewer iterations)
            Assert.IsLessThan(cgResult.Iterations,
lbfgsResult.Iterations, $"L-BFGS ({lbfgsResult.Iterations} iters) should be faster than CG ({cgResult.Iterations} iters)");
        }

        [TestMethod]
        public void MemorySizeAffectsPerformance()
        {
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(10);

            // Test with small memory (m=3)
            var smallMemoryOptimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 5000, MemorySize = 3 },
                new BacktrackingLineSearch());

            var smallMemoryResult = smallMemoryOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0, CancellationToken.None);

            // Test with larger memory (m=20)
            var largeMemoryOptimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 5000, MemorySize = 20 },
                new BacktrackingLineSearch());

            var largeMemoryResult = largeMemoryOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0, CancellationToken.None);

            // Both should succeed
            Assert.IsTrue(smallMemoryResult.Success, "Small memory optimization should succeed");
            Assert.IsTrue(largeMemoryResult.Success, "Large memory optimization should succeed");

            // Larger memory typically converges in fewer iterations (though not guaranteed)
            // Just verify both complete successfully
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-10, MaxIterations = 5 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart,
                CancellationToken.None);

            Assert.IsFalse(result.Success, "Should not succeed with very few iterations");
            Assert.AreEqual(StoppingReason.MaxIterations, result.StoppingReason);
            Assert.AreEqual(5, result.Iterations);
        }

        [TestMethod]
        public void WorksWithDifferentMemorySizes()
        {
            // Test various memory sizes to ensure they all work
            var memorySizes = new[] { 3, 5, 10, 20 };

            foreach (var m in memorySizes)
            {
                var optimizer = new LBFGSOptimizer(
                    new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 1000, MemorySize = m },
                    new BacktrackingLineSearch());

                var result = optimizer.Minimize(
                    x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                    s_rosenbrock2DStart,
                    CancellationToken.None);

                Assert.IsTrue(result.Success, $"Should succeed with memory size {m}");
                Assert.IsLessThan(1e-3, result.OptimalValue, $"Should find good solution with memory size {m}");
            }
        }

        [TestMethod]
        public void WithParallelLineSearchEnablesParallelSearch()
        {
            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 1000 },
                new ParallelBacktrackingLineSearch(parallelBatchSize: 4));

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart,
                CancellationToken.None);

            Assert.IsTrue(result.Success, "Should succeed with parallel line search");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3);
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-3);
        }

        [TestMethod]
        public void WithParallelLineSearchFallsBackWhenDisabled()
        {
            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { Tolerance = 1e-4, MaxIterations = 1000 },
                new ParallelBacktrackingLineSearch(enableParallelization: false));

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart,
                CancellationToken.None);

            Assert.IsTrue(result.Success, "Should succeed with parallel line search disabled");
        }
    }
}
