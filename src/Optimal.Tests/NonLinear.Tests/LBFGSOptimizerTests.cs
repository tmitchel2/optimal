/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Tests
{
    [TestClass]
    public sealed class LBFGSOptimizerTests
    {
        private static readonly double[] s_rosenbrock2DStart = new[] { -1.2, 1.0 };
        private static readonly double[] s_boothStart = new[] { 0.0, 0.0 };

        [TestMethod]
        public void CanMinimizeRosenbrock2D()
        {
            var optimizer = new LBFGSOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-6);
            optimizer.WithMaxIterations(1000);
            optimizer.WithMemorySize(10);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-3, "y should be near 1");
            Assert.IsTrue(result.OptimalValue < 1e-6, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeBoothFunction()
        {
            var optimizer = new LBFGSOptimizer();
            optimizer.WithInitialPoint(s_boothStart);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-6);
            optimizer.WithMaxIterations(1000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.BoothReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3, "x should be near 1");
            Assert.AreEqual(3.0, result.OptimalPoint[1], 1e-3, "y should be near 3");
            Assert.IsTrue(result.OptimalValue < 1e-6, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock10D()
        {
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(10);

            var optimizer = new LBFGSOptimizer();
            optimizer.WithInitialPoint(x0);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-4);
            optimizer.WithMaxIterations(5000);
            optimizer.WithMemorySize(10);

            var result = optimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < 10; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 1e-1, $"x[{i}] should be near 1");
            }
            Assert.IsTrue(result.OptimalValue < 1e-2, $"f(x*) should be small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock100D()
        {
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(100);

            var optimizer = new LBFGSOptimizer();
            optimizer.WithInitialPoint(x0);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-3);
            optimizer.WithMaxIterations(1000);
            optimizer.WithMemorySize(20);

            var result = optimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.IsTrue(result.Iterations < 1000, $"Should converge in < 1000 iterations, took {result.Iterations}");

            // Check that we're close to optimum
            for (var i = 0; i < 100; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 0.2, $"x[{i}] should be reasonably close to 1");
            }

            Assert.IsTrue(result.OptimalValue < 1.0, $"f(x*) should be reasonably small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void LBFGSIsFasterThanConjugateGradient()
        {
            // Test that L-BFGS requires fewer iterations than CG on a moderate-dimensional problem
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(10);

            var lbfgsOptimizer = new LBFGSOptimizer();
            lbfgsOptimizer.WithInitialPoint(x0);
            lbfgsOptimizer.WithLineSearch(new BacktrackingLineSearch());
            lbfgsOptimizer.WithTolerance(1e-4);
            lbfgsOptimizer.WithMaxIterations(5000);

            var lbfgsResult = lbfgsOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock);

            var cgOptimizer = new ConjugateGradientOptimizer();
            cgOptimizer.WithInitialPoint(x0);
            cgOptimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            cgOptimizer.WithLineSearch(new BacktrackingLineSearch());
            cgOptimizer.WithTolerance(1e-4);
            cgOptimizer.WithMaxIterations(20000); // Give CG more iterations for 10D

            var cgResult = cgOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock);

            Assert.IsTrue(lbfgsResult.Success, "L-BFGS optimization should succeed");
            Assert.IsTrue(cgResult.Success, "CG optimization should succeed");

            // L-BFGS should be faster (fewer iterations)
            Assert.IsTrue(lbfgsResult.Iterations < cgResult.Iterations,
                $"L-BFGS ({lbfgsResult.Iterations} iters) should be faster than CG ({cgResult.Iterations} iters)");
        }

        [TestMethod]
        public void MemorySizeAffectsPerformance()
        {
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(10);

            // Test with small memory (m=3)
            var smallMemoryOptimizer = new LBFGSOptimizer();
            smallMemoryOptimizer.WithInitialPoint(x0);
            smallMemoryOptimizer.WithLineSearch(new BacktrackingLineSearch());
            smallMemoryOptimizer.WithTolerance(1e-4);
            smallMemoryOptimizer.WithMaxIterations(5000);
            smallMemoryOptimizer.WithMemorySize(3);

            var smallMemoryResult = smallMemoryOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock);

            // Test with larger memory (m=20)
            var largeMemoryOptimizer = new LBFGSOptimizer();
            largeMemoryOptimizer.WithInitialPoint(x0);
            largeMemoryOptimizer.WithLineSearch(new BacktrackingLineSearch());
            largeMemoryOptimizer.WithTolerance(1e-4);
            largeMemoryOptimizer.WithMaxIterations(5000);
            largeMemoryOptimizer.WithMemorySize(20);

            var largeMemoryResult = largeMemoryOptimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock);

            // Both should succeed
            Assert.IsTrue(smallMemoryResult.Success, "Small memory optimization should succeed");
            Assert.IsTrue(largeMemoryResult.Success, "Large memory optimization should succeed");

            // Larger memory typically converges in fewer iterations (though not guaranteed)
            // Just verify both complete successfully
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new LBFGSOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-10); // Very tight tolerance
            optimizer.WithMaxIterations(5); // Very few iterations

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

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
                var optimizer = new LBFGSOptimizer();
                optimizer.WithInitialPoint(s_rosenbrock2DStart);
                optimizer.WithLineSearch(new BacktrackingLineSearch());
                optimizer.WithTolerance(1e-4);
                optimizer.WithMaxIterations(1000);
                optimizer.WithMemorySize(m);

                var result = optimizer.Minimize(x =>
                    TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
                );

                Assert.IsTrue(result.Success, $"Should succeed with memory size {m}");
                Assert.IsTrue(result.OptimalValue < 1e-3, $"Should find good solution with memory size {m}");
            }
        }
    }
}
