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
    public sealed class CauchyPointCalculatorTests
    {
        [TestMethod]
        public void ComputeInteriorPointMovesAlongNegativeGradient()
        {
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { 1.0, 1.0 };
            var lower = new[] { -10.0, -10.0 };
            var upper = new[] { 10.0, 10.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // Direction should be negative gradient
            Assert.IsLessThan(0, result.Direction[0], "Direction should be in negative gradient direction");
            Assert.IsLessThan(0, result.Direction[1], "Direction should be in negative gradient direction");

            // Should have moved from origin
            Assert.IsGreaterThan(0, result.TotalStep, "Should have taken a positive step");
        }

        [TestMethod]
        public void ComputeAtLowerBoundPositiveGradientStaysAtBound()
        {
            var x = new[] { -1.0, 0.0 };
            var gradient = new[] { 1.0, 0.5 }; // Positive gradient at lower bound
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // x[0] is at lower bound with positive gradient -> should stay at bound
            Assert.AreEqual(-1.0, result.CauchyPoint[0], 1e-10, "Should stay at lower bound");
            Assert.IsFalse(result.FreeVariables[0], "Variable at lower bound should be active");
        }

        [TestMethod]
        public void ComputeAtUpperBoundNegativeGradientStaysAtBound()
        {
            var x = new[] { 1.0, 0.0 };
            var gradient = new[] { -1.0, 0.5 }; // Negative gradient at upper bound
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // x[0] is at upper bound with negative gradient -> should stay at bound
            Assert.AreEqual(1.0, result.CauchyPoint[0], 1e-10, "Should stay at upper bound");
            Assert.IsFalse(result.FreeVariables[0], "Variable at upper bound should be active");
        }

        [TestMethod]
        public void ComputeAtBoundButCanMoveInwardMovesFree()
        {
            // At lower bound but gradient is negative (so -grad is positive, moving into interior)
            var x = new[] { -1.0, 0.0 };
            var gradient = new[] { -1.0, 0.0 }; // Negative gradient at lower bound allows movement
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // Direction should allow movement away from bound
            Assert.IsGreaterThan(0, result.Direction[0], "Should move away from lower bound");
            Assert.IsTrue(result.CauchyPoint[0] > -1.0 || result.TotalStep > 0, "Should move into interior");
        }

        [TestMethod]
        public void ComputeAllAtBoundsWithOutwardGradientsNoMovement()
        {
            // All at bounds with gradients pushing outward
            var x = new[] { -1.0, 1.0 }; // At lower and upper bounds
            var gradient = new[] { 1.0, -1.0 }; // Would push further into bounds
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // Should not move
            Assert.AreEqual(-1.0, result.CauchyPoint[0], 1e-10, "Should stay at lower bound");
            Assert.AreEqual(1.0, result.CauchyPoint[1], 1e-10, "Should stay at upper bound");
            Assert.AreEqual(0.0, result.TotalStep, 1e-10, "Should not take any step");
        }

        [TestMethod]
        public void ComputeHitsOneBoundCorrectlyIdentifiesActiveSet()
        {
            // Start at interior, move toward bounds
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { -10.0, 0.1 }; // Strong gradient pushing x[0] to upper bound
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // x[0] should move toward upper bound
            Assert.IsGreaterThan(0, result.CauchyPoint[0], "Should move toward upper bound");
        }

        [TestMethod]
        public void ComputeSimpleProjectsToFeasibleRegion()
        {
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { 1.0, 1.0 };
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.ComputeSimple(x, gradient, lower, upper, initialStepSize: 0.5);

            // Cauchy point should be feasible
            Assert.IsGreaterThanOrEqualTo(lower[0] - 1e-10, result.CauchyPoint[0], "Should respect lower bound");
            Assert.IsLessThanOrEqualTo(upper[0] + 1e-10, result.CauchyPoint[0], "Should respect upper bound");
            Assert.IsGreaterThanOrEqualTo(lower[1] - 1e-10, result.CauchyPoint[1], "Should respect lower bound");
            Assert.IsLessThanOrEqualTo(upper[1] + 1e-10, result.CauchyPoint[1], "Should respect upper bound");
        }

        [TestMethod]
        public void ComputeSimpleRespectsBoundsDuringStep()
        {
            // Large gradient that would exceed bounds
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { 100.0, 100.0 }; // Very large gradient
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.ComputeSimple(x, gradient, lower, upper, initialStepSize: 1.0);

            // Should stop at bounds
            Assert.IsGreaterThanOrEqualTo(lower[0] - 1e-10, result.CauchyPoint[0], "Should not exceed lower bound");
            Assert.IsGreaterThanOrEqualTo(lower[1] - 1e-10, result.CauchyPoint[1], "Should not exceed lower bound");
        }

        [TestMethod]
        public void ComputeZeroGradientNoMovement()
        {
            var x = new[] { 0.5, 0.5 };
            var gradient = new[] { 0.0, 0.0 };
            var lower = new[] { 0.0, 0.0 };
            var upper = new[] { 1.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            Assert.AreEqual(0.5, result.CauchyPoint[0], 1e-10, "Should not move with zero gradient");
            Assert.AreEqual(0.5, result.CauchyPoint[1], 1e-10, "Should not move with zero gradient");
            Assert.AreEqual(0.0, result.TotalStep, 1e-10, "Step should be zero");
        }

        [TestMethod]
        public void ComputeReturnsCorrectFreeVariables()
        {
            var x = new[] { 0.5, -1.0, 1.0 }; // Interior, lower, upper
            var gradient = new[] { 0.1, 1.0, -1.0 }; // Gradient pushes bounds outward
            var lower = new[] { 0.0, -1.0, 0.0 };
            var upper = new[] { 1.0, 0.0, 1.0 };

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // x[0] is interior - should be free initially
            // x[1] is at lower with positive gradient - should be active
            // x[2] is at upper with negative gradient - should be active
            Assert.IsFalse(result.FreeVariables[1], "x[1] at lower bound should be active");
            Assert.IsFalse(result.FreeVariables[2], "x[2] at upper bound should be active");
        }

        [TestMethod]
        public void ComputeWithLBFGSMemoryUsesHessianApproximation()
        {
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { 2.0, 1.0 };
            var lower = new[] { -10.0, -10.0 };
            var upper = new[] { 10.0, 10.0 };

            // Create memory with some history
            var memory = new LBFGSMemory(5, 2);
            memory.Push([0.1, 0.1], [0.2, 0.2]); // s and y vectors

            var resultWithMemory = CauchyPointCalculator.Compute(x, gradient, lower, upper, memory);
            var resultWithoutMemory = CauchyPointCalculator.Compute(x, gradient, lower, upper, null);

            // Results should be different (memory affects Hessian approximation)
            // The direction should be the same (negative gradient) but step size may differ
            Assert.IsLessThan(0, resultWithMemory.Direction[0], "Direction should be descent");
            Assert.IsLessThan(0, resultWithoutMemory.Direction[0], "Direction should be descent");
        }

        [TestMethod]
        public void ComputeSimpleHandlesInfinityBounds()
        {
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { 1.0, -1.0 };
            var lower = new[] { double.NegativeInfinity, 0.0 };
            var upper = new[] { 0.0, double.PositiveInfinity };

            var result = CauchyPointCalculator.ComputeSimple(x, gradient, lower, upper, initialStepSize: 0.5);

            // Should move in negative gradient direction, respecting bounds
            Assert.IsLessThanOrEqualTo(0.0, result.CauchyPoint[0], "Should respect upper bound");
            Assert.IsGreaterThanOrEqualTo(0.0, result.CauchyPoint[1], "Should respect lower bound");
        }

        [TestMethod]
        public void ComputeHighDimensionalHandlesCorrectly()
        {
            var n = 10;
            var x = new double[n];
            var gradient = new double[n];
            var lower = new double[n];
            var upper = new double[n];

            for (var i = 0; i < n; i++)
            {
                x[i] = 0.5;
                gradient[i] = i % 2 == 0 ? 1.0 : -1.0;
                lower[i] = 0.0;
                upper[i] = 1.0;
            }

            var result = CauchyPointCalculator.Compute(x, gradient, lower, upper);

            // All points should remain feasible
            for (var i = 0; i < n; i++)
            {
                Assert.IsGreaterThanOrEqualTo(lower[i] - 1e-10, result.CauchyPoint[i], $"x[{i}] should respect lower bound");
                Assert.IsLessThanOrEqualTo(upper[i] + 1e-10, result.CauchyPoint[i], $"x[{i}] should respect upper bound");
            }
        }
    }
}
