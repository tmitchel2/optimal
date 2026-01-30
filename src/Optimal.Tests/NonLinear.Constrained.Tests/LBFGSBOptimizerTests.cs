/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Tests;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Constrained.Tests
{
    [TestClass]
    public sealed class LBFGSBOptimizerTests
    {
        #region Basic Functionality Tests

        [TestMethod]
        public void CanMinimizeUnconstrainedQuadratic()
        {
            // f(x,y) = x^2 + y^2, minimum at (0,0)
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [-10.0, -10.0],
                UpperBounds = [10.0, 10.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                [1.0, 1.0]);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(0.0, result.OptimalPoint[0], 1e-4, "x should be near 0");
            Assert.AreEqual(0.0, result.OptimalPoint[1], 1e-4, "y should be near 0");
            Assert.IsLessThan(1e-6, result.OptimalValue, $"f(x*) should be very small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeRosenbrock2DWithWideBounds()
        {
            // Rosenbrock with bounds that don't affect the solution
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-5,
                MaxIterations = 1000,
                MemorySize = 10,
                LowerBounds = [-10.0, -10.0],
                UpperBounds = [10.0, 10.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                [-1.2, 1.0]);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2, "x should be near 1");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2, "y should be near 1");
            Assert.IsLessThan(1e-3, result.OptimalValue, $"f(x*) should be small, was {result.OptimalValue}");
        }

        [TestMethod]
        public void CanMinimizeBoothFunctionWithWideBounds()
        {
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 500,
                LowerBounds = [-10.0, -10.0],
                UpperBounds = [10.0, 10.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.BoothReverse(x[0], x[1]),
                [0.0, 0.0]);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3, "x should be near 1");
            Assert.AreEqual(3.0, result.OptimalPoint[1], 1e-3, "y should be near 3");
        }

        #endregion

        #region Active Constraint Tests

        [TestMethod]
        public void OptimalPointSatisfiesBounds()
        {
            // f(x,y) = (x-2)^2 + (y-2)^2, unconstrained min at (2,2)
            // With bounds [0,1] x [0,1], optimal should be at (1,1)
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.0, 0.0],
                UpperBounds = [1.0, 1.0]
            });

            var result = optimizer.Minimize(x =>
            {
                var fx = (x[0] - 2.0) * (x[0] - 2.0) + (x[1] - 2.0) * (x[1] - 2.0);
                var grad = new[] { 2.0 * (x[0] - 2.0), 2.0 * (x[1] - 2.0) };
                return (fx, grad);
            }, [0.5, 0.5]);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");

            // Solution should be at (1, 1) - the corner closest to (2, 2)
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-4, "x should be at upper bound");
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-4, "y should be at upper bound");

            // Verify bounds are satisfied
            Assert.IsGreaterThanOrEqualTo(0.0 - 1e-10, result.OptimalPoint[0], "x should satisfy lower bound");
            Assert.IsLessThanOrEqualTo(1.0 + 1e-10, result.OptimalPoint[0], "x should satisfy upper bound");
            Assert.IsGreaterThanOrEqualTo(0.0 - 1e-10, result.OptimalPoint[1], "y should satisfy lower bound");
            Assert.IsLessThanOrEqualTo(1.0 + 1e-10, result.OptimalPoint[1], "y should satisfy upper bound");
        }

        [TestMethod]
        public void OptimalPointCorrectlyOnBoundaryWhenConstraintActive()
        {
            // f(x,y) = x^2 + y^2, unconstrained min at (0,0)
            // With bounds x >= 0.5, y >= 0.5, optimal should be at (0.5, 0.5)
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.5, 0.5],
                UpperBounds = [10.0, 10.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                [1.0, 1.0]);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(0.5, result.OptimalPoint[0], 1e-4, "x should be at lower bound");
            Assert.AreEqual(0.5, result.OptimalPoint[1], 1e-4, "y should be at lower bound");
        }

        [TestMethod]
        public void HandlesPartiallyActiveConstraints()
        {
            // f(x,y) = (x-0.3)^2 + y^2
            // Unconstrained min at (0.3, 0)
            // With bounds 0 <= x <= 1, -0.5 <= y <= 0.5
            // Only x constraint is NOT active, y=0 is interior
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.0, -0.5],
                UpperBounds = [1.0, 0.5]
            });

            var result = optimizer.Minimize(x =>
            {
                var fx = (x[0] - 0.3) * (x[0] - 0.3) + x[1] * x[1];
                var grad = new[] { 2.0 * (x[0] - 0.3), 2.0 * x[1] };
                return (fx, grad);
            }, [0.5, 0.25]);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            Assert.AreEqual(0.3, result.OptimalPoint[0], 1e-3, "x should be at interior optimum 0.3");
            Assert.AreEqual(0.0, result.OptimalPoint[1], 1e-3, "y should be at interior optimum 0");
        }

        #endregion

        #region Edge Case Tests

        [TestMethod]
        public void HandlesStartingPointOnBoundary()
        {
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.0, 0.0],
                UpperBounds = [2.0, 2.0]
            });

            var result = optimizer.Minimize(x =>
            {
                // f(x,y) = (x-1)^2 + (y-1)^2, min at (1,1) which is interior
                var fx = (x[0] - 1.0) * (x[0] - 1.0) + (x[1] - 1.0) * (x[1] - 1.0);
                var grad = new[] { 2.0 * (x[0] - 1.0), 2.0 * (x[1] - 1.0) };
                return (fx, grad);
            }, [0.0, 0.0]); // Starting at lower bounds

            Assert.IsTrue(result.Success, $"Optimization should succeed starting from boundary");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-3);
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-3);
        }

        [TestMethod]
        public void HandlesStartingPointOutsideFeasibleRegion()
        {
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.0, 0.0],
                UpperBounds = [1.0, 1.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                [-5.0, 5.0]); // Outside bounds

            Assert.IsTrue(result.Success, $"Optimization should succeed after projection");
            // Result should be at (0, 0) since that's the closest feasible point to the unconstrained minimum
            Assert.AreEqual(0.0, result.OptimalPoint[0], 1e-4);
            Assert.AreEqual(0.0, result.OptimalPoint[1], 1e-4);
        }

        [TestMethod]
        public void HandlesAllVariablesAtBounds()
        {
            // All variables active at solution
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.0, 0.0, 0.0],
                UpperBounds = [1.0, 1.0, 1.0]
            });

            // f(x) = sum((x_i - 2)^2), unconstrained min at (2,2,2)
            // With [0,1] bounds, optimal is at (1,1,1)
            var result = optimizer.Minimize(x =>
            {
                var fx = 0.0;
                var grad = new double[3];
                for (var i = 0; i < 3; i++)
                {
                    fx += (x[i] - 2.0) * (x[i] - 2.0);
                    grad[i] = 2.0 * (x[i] - 2.0);
                }
                return (fx, grad);
            }, [0.5, 0.5, 0.5]);

            Assert.IsTrue(result.Success, $"Optimization should succeed");
            for (var i = 0; i < 3; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 1e-4, $"x[{i}] should be at upper bound");
            }
        }

        [TestMethod]
        public void HandlesInfinityBounds()
        {
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-6,
                MaxIterations = 100,
                LowerBounds = [0.0, double.NegativeInfinity],
                UpperBounds = [double.PositiveInfinity, 0.0]
            });

            // f(x,y) = x^2 + y^2, min at (0,0)
            // With x >= 0 and y <= 0, min is still at (0,0)
            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                [5.0, -5.0]);

            Assert.IsTrue(result.Success, $"Optimization should succeed with infinity bounds");
            Assert.AreEqual(0.0, result.OptimalPoint[0], 1e-4, "x should be at lower bound 0");
            Assert.AreEqual(0.0, result.OptimalPoint[1], 1e-4, "y should be at upper bound 0");
        }

        [TestMethod]
        public void HandlesNoBoundsSpecified()
        {
            // No bounds specified - should work as unconstrained
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-5,
                MaxIterations = 1000
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                [-1.2, 1.0]);

            Assert.IsTrue(result.Success, $"Optimization should succeed without bounds");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-2);
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-2);
        }

        #endregion

        #region High-Dimensional Tests

        [TestMethod]
        public void CanMinimizeRosenbrock10DWithBounds()
        {
            var n = 10;
            var x0 = HighDimensionalFunctions.RosenbrockStartingPoint(n);
            var lower = new double[n];
            var upper = new double[n];
            for (var i = 0; i < n; i++)
            {
                lower[i] = -5.0;
                upper[i] = 5.0;
            }

            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-4,
                MaxIterations = 5000,
                MemorySize = 10,
                LowerBounds = lower,
                UpperBounds = upper
            });

            var result = optimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0);

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");
            for (var i = 0; i < n; i++)
            {
                Assert.AreEqual(1.0, result.OptimalPoint[i], 0.2, $"x[{i}] should be near 1");
                Assert.IsGreaterThanOrEqualTo(lower[i] - 1e-10, result.OptimalPoint[i], $"x[{i}] should satisfy lower bound");
                Assert.IsLessThanOrEqualTo(upper[i] + 1e-10, result.OptimalPoint[i], $"x[{i}] should satisfy upper bound");
            }
        }

        [TestMethod]
        public void CanMinimizeHighDimensionalWithActiveBounds()
        {
            var n = 20;
            var x0 = new double[n];
            var lower = new double[n];
            var upper = new double[n];
            for (var i = 0; i < n; i++)
            {
                x0[i] = 0.5;
                lower[i] = 0.0;
                upper[i] = 0.8; // Upper bound will be active since Rosenbrock min is at 1.0
            }

            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-4,
                MaxIterations = 2000,
                MemorySize = 15,
                LowerBounds = lower,
                UpperBounds = upper
            });

            var result = optimizer.Minimize(HighDimensionalFunctions.ExtendedRosenbrock, x0);

            Assert.IsTrue(result.Success, $"Optimization should succeed with active bounds");

            // All points should be at or below upper bound
            for (var i = 0; i < n; i++)
            {
                Assert.IsLessThanOrEqualTo(upper[i] + 1e-6, result.OptimalPoint[i], $"x[{i}] should satisfy upper bound");
                Assert.IsGreaterThanOrEqualTo(lower[i] - 1e-6, result.OptimalPoint[i], $"x[{i}] should satisfy lower bound");
            }
        }

        #endregion

        #region Comparison Tests

        [TestMethod]
        public void LBFGSBMatchesLBFGSWhenBoundsInactive()
        {
            // With wide bounds, L-BFGS-B should find same solution as L-BFGS
            var x0 = new[] { -1.2, 1.0 };

            var lbfgsbOptimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-5,
                MaxIterations = 1000,
                LowerBounds = [-100.0, -100.0],
                UpperBounds = [100.0, 100.0]
            });

            var lbfgsbResult = lbfgsbOptimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                x0);

            var lbfgsOptimizer = new LBFGSOptimizer(new LBFGSOptions
            {
                Tolerance = 1e-5,
                MaxIterations = 1000
            }, new BacktrackingLineSearch());

            var lbfgsResult = lbfgsOptimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                x0);

            Assert.IsTrue(lbfgsbResult.Success, "L-BFGS-B should succeed");
            Assert.IsTrue(lbfgsResult.Success, "L-BFGS should succeed");

            // Both should find the same solution
            Assert.AreEqual(lbfgsResult.OptimalPoint[0], lbfgsbResult.OptimalPoint[0], 1e-2,
                "Both optimizers should find same x");
            Assert.AreEqual(lbfgsResult.OptimalPoint[1], lbfgsbResult.OptimalPoint[1], 1e-2,
                "Both optimizers should find same y");
        }

        #endregion

        #region Convergence Tests

        [TestMethod]
        public void ReturnsGradientToleranceWhenConverged()
        {
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-4,
                MaxIterations = 100,
                LowerBounds = [-1.0, -1.0],
                UpperBounds = [1.0, 1.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.QuadraticReverse(x[0], x[1]),
                [0.1, 0.1]);

            Assert.IsTrue(result.Success, "Optimization should succeed");
            Assert.AreEqual(StoppingReason.GradientTolerance, result.StoppingReason,
                "Should stop due to gradient tolerance");
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
            {
                Tolerance = 1e-15,
                MaxIterations = 5,
                LowerBounds = [-10.0, -10.0],
                UpperBounds = [10.0, 10.0]
            });

            var result = optimizer.Minimize(
                x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                [-1.2, 1.0]);

            Assert.IsFalse(result.Success, "Should not succeed with very few iterations");
            Assert.AreEqual(StoppingReason.MaxIterations, result.StoppingReason);
        }

        #endregion

        #region Memory Size Tests

        [TestMethod]
        public void WorksWithDifferentMemorySizes()
        {
            var memorySizes = new[] { 3, 5, 10, 20 };

            foreach (var m in memorySizes)
            {
                var optimizer = new LBFGSBOptimizer(new LBFGSBOptions
                {
                    Tolerance = 1e-4,
                    MaxIterations = 1000,
                    MemorySize = m,
                    LowerBounds = [-5.0, -5.0],
                    UpperBounds = [5.0, 5.0]
                });

                var result = optimizer.Minimize(
                    x => TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1]),
                    [-1.2, 1.0]);

                Assert.IsTrue(result.Success, $"Should succeed with memory size {m}");
                Assert.IsLessThan(1e-3, result.OptimalValue, $"Should find good solution with memory size {m}");
            }
        }

        #endregion
    }
}
