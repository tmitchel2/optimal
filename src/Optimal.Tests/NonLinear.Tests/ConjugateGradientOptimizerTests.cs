/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.LineSearch;

namespace Optimal.NonLinear.Tests
{
    [TestClass]
    public sealed class ConjugateGradientOptimizerTests
    {
        private static readonly double[] s_rosenbrock2DStart = [-1.2, 1.0];
        private static readonly double[] s_rosenbrock4DStart = [-1.2, 1.0, -1.2, 1.0];
        private static readonly double[] s_rosenbrock10DStart = [-1.2, 1.0, -1.2, 1.0, -1.2, 1.0, -1.2, 1.0, -1.2, 1.0];
        private static readonly double[] s_bealeStart = [0.0, 0.0];

        [TestMethod]
        public void CanMinimizeRosenbrock2DWithFletcherReeves()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-4);
            optimizer.WithMaxIterations(5000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
            Assert.IsTrue(result.OptimalValue < 1e-3, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock2DWithPolakRibiere()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithFormula(ConjugateGradientFormula.PolakRibiere);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-5);
            optimizer.WithMaxIterations(5000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 3e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 3e-2, "y should be near 1");
            Assert.IsTrue(result.OptimalValue < 1e-3, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock2DWithHestenesStiefel()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithFormula(ConjugateGradientFormula.HestenesStiefel);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-4);
            optimizer.WithMaxIterations(5000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
            Assert.IsTrue(result.OptimalValue < 1e-3, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeBealeFunction()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_bealeStart);
            optimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-5);
            optimizer.WithMaxIterations(5000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.BealeReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(3.0, result.OptimalPoint[0], 1e-2, "x should be near 3");
            Assert.AreEqual(0.5, result.OptimalPoint[1], 1e-2, "y should be near 0.5");
            Assert.IsTrue(result.OptimalValue < 1e-3, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock4D()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock4DStart);
            optimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-5);
            optimizer.WithMaxIterations(10000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.Rosenbrock4DReverse(x[0], x[1], x[2], x[3])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < 4; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 1.5, $"x[{i}] should be reasonably close to 1");
            }
            Assert.IsTrue(result.OptimalValue < 0.5, $"f(x*) should be reasonably small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock10D()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock10DStart);
            optimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-4);
            optimizer.WithMaxIterations(20000);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.Rosenbrock10DReverse(
                    x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < 10; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 0.28, $"x[{i}] should be near 1");
            }
            Assert.IsTrue(result.OptimalValue < 1e-1, $"f(x*) should be small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void ConjugateGradientIsFasterThanGradientDescent()
        {
            // Test that CG requires significantly fewer iterations than gradient descent on Rosenbrock
            var cgOptimizer = new ConjugateGradientOptimizer();
            cgOptimizer.WithInitialPoint(s_rosenbrock2DStart);
            cgOptimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            cgOptimizer.WithLineSearch(new BacktrackingLineSearch());
            cgOptimizer.WithTolerance(1e-4);
            cgOptimizer.WithMaxIterations(10000);

            var cgResult = cgOptimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            var gdOptimizer = new GradientDescentOptimizer();
            gdOptimizer.WithInitialPoint(s_rosenbrock2DStart);
            gdOptimizer.WithLineSearch(new BacktrackingLineSearch());
            gdOptimizer.WithTolerance(1e-4);
            gdOptimizer.WithMaxIterations(10000);

            var gdResult = gdOptimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(cgResult.Success, "CG optimization should succeed");
            Assert.IsTrue(gdResult.Success, "GD optimization should succeed");

            // CG should be significantly faster - looking for at least 5x speedup
            // (10x is ideal but 5x is a safe conservative target)
            Assert.IsTrue(cgResult.Iterations < gdResult.Iterations / 5,
                $"CG ({cgResult.Iterations} iters) should be much faster than GD ({gdResult.Iterations} iters)");
        }

        [TestMethod]
        public void RestartIntervalWorks()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-4);
            optimizer.WithMaxIterations(10000);
            optimizer.WithRestartInterval(10); // Restart every 10 iterations

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, "Optimization with restart should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new ConjugateGradientOptimizer();
            optimizer.WithInitialPoint(s_rosenbrock2DStart);
            optimizer.WithFormula(ConjugateGradientFormula.FletcherReeves);
            optimizer.WithLineSearch(new BacktrackingLineSearch());
            optimizer.WithTolerance(1e-10); // Very tight tolerance
            optimizer.WithMaxIterations(10); // Very few iterations

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsFalse(result.Success, "Should not succeed with very few iterations");
            Assert.AreEqual(StoppingReason.MaxIterations, result.StoppingReason);
            Assert.AreEqual(10, result.Iterations);
        }
    }
}
