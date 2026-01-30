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
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-4, MaxIterations = 5000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
            Assert.IsLessThan(1e-3, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock2DWithPolakRibiere()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.PolakRibiere, Tolerance = 1e-5, MaxIterations = 5000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 3e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 3e-2, "y should be near 1");
            Assert.IsLessThan(1e-3, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock2DWithHestenesStiefel()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.HestenesStiefel, Tolerance = 1e-4, MaxIterations = 5000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
            Assert.IsLessThan(1e-3, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeBealeFunction()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-5, MaxIterations = 5000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.BealeReverse(x[0], x[1]),
                s_bealeStart);

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(3.0, result.OptimalPoint[0], 1e-2, "x should be near 3");
            Assert.AreEqual(0.5, result.OptimalPoint[1], 1e-2, "y should be near 0.5");
            Assert.IsLessThan(1e-3, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock4D()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-5, MaxIterations = 10000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.Rosenbrock4DReverse(x[0], x[1], x[2], x[3]),
                s_rosenbrock4DStart);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < 4; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 1.5, $"x[{i}] should be reasonably close to 1");
            }
            Assert.IsLessThan(0.5, result.OptimalValue, $"f(x*) should be reasonably small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock10D()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-4, MaxIterations = 20000 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.Rosenbrock10DReverse(
                    x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9]),
                s_rosenbrock10DStart);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < 10; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 0.28, $"x[{i}] should be near 1");
            }
            Assert.IsLessThan(1e-1, result.OptimalValue, $"f(x*) should be small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void ConjugateGradientIsFasterThanGradientDescent()
        {
            // Test that CG requires significantly fewer iterations than gradient descent on Rosenbrock
            var cgOptimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-4, MaxIterations = 10000 },
                new BacktrackingLineSearch());

            var cgResult = cgOptimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            var gdOptimizer = new GradientDescentOptimizer(
                new GradientDescentOptions { Tolerance = 1e-4, MaxIterations = 10000 },
                new BacktrackingLineSearch());

            var gdResult = gdOptimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            Assert.IsTrue(cgResult.Success, "CG optimization should succeed");
            Assert.IsTrue(gdResult.Success, "GD optimization should succeed");

            // CG should be significantly faster - looking for at least 5x speedup
            // (10x is ideal but 5x is a safe conservative target)
            Assert.IsLessThan(gdResult.Iterations / 5,
cgResult.Iterations, $"CG ({cgResult.Iterations} iters) should be much faster than GD ({gdResult.Iterations} iters)");
        }

        [TestMethod]
        public void RestartIntervalWorks()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-4, MaxIterations = 10000, RestartInterval = 10 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            Assert.IsTrue(result.Success, "Optimization with restart should succeed");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new ConjugateGradientOptimizer(
                new ConjugateGradientOptions { Formula = ConjugateGradientFormula.FletcherReeves, Tolerance = 1e-10, MaxIterations = 10 },
                new BacktrackingLineSearch());

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                s_rosenbrock2DStart);

            Assert.IsFalse(result.Success, "Should not succeed with very few iterations");
            Assert.AreEqual(StoppingReason.MaxIterations, result.StoppingReason);
            Assert.AreEqual(10, result.Iterations);
        }
    }
}
