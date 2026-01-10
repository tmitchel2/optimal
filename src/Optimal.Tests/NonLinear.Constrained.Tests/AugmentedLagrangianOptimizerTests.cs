/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear;
using Optimal.NonLinear.Constrained;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Tests;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Constrained.Tests
{
    [TestClass]
    public sealed class AugmentedLagrangianOptimizerTests
    {
        private static readonly double[] s_boxInitial = new[] { 0.0, 0.0 };
        private static readonly double[] s_boxLower = new[] { -0.5, -0.5 };
        private static readonly double[] s_boxUpper = new[] { 0.5, 0.5 };
        private static readonly double[] s_equalityInitial = new[] { 0.0, 1.0 };
        private static readonly double[] s_inequalityInitial = new[] { 0.5, 0.5 };

        [TestMethod]
        public void CanMinimizeWithBoxConstraints()
        {
            // Minimize Rosenbrock with box constraints: -0.5 ≤ x, y ≤ 0.5
            // True minimum (1, 1) is outside box, should find boundary optimum
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(s_boxInitial);
            optimizer.WithBoxConstraints(s_boxLower, s_boxUpper);
            optimizer.WithTolerance(1e-4);
            optimizer.WithConstraintTolerance(1e-6);
            optimizer.WithMaxIterations(30);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");

            // Should be at boundary (0.5, something)
            Assert.IsTrue(result.OptimalPoint[0] >= -0.5 - 1e-6 && result.OptimalPoint[0] <= 0.5 + 1e-6,
                $"x should be within bounds, was {result.OptimalPoint[0]}");
            Assert.IsTrue(result.OptimalPoint[1] >= -0.5 - 1e-6 && result.OptimalPoint[1] <= 0.5 + 1e-6,
                $"y should be within bounds, was {result.OptimalPoint[1]}");
        }

        [TestMethod]
        public void CanMinimizeWithEqualityConstraint()
        {
            // Minimize (x - 1)² + (y - 1)² subject to x + y = 1
            // Solution: x = y = 0.5 (on the line, closest to (1,1))
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(s_equalityInitial);
            optimizer.WithEqualityConstraint(x =>
                TestObjectiveFunctionsGradients.LinearEqualityConstraintReverse(x[0], x[1]));
            optimizer.WithTolerance(1e-4);
            optimizer.WithConstraintTolerance(1e-6);
            optimizer.WithMaxIterations(30);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.DistanceFromTargetReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");

            // Check constraint is satisfied: x + y = 1
            var constraintValue = result.OptimalPoint[0] + result.OptimalPoint[1] - 1.0;
            Assert.IsTrue(Math.Abs(constraintValue) < 1e-4,
                $"Equality constraint should be satisfied: x + y = {result.OptimalPoint[0] + result.OptimalPoint[1]}, expected 1.0");

            // Check we're close to expected solution (0.5, 0.5)
            Assert.AreEqual(0.5, result.OptimalPoint[0], 1e-2, "x should be near 0.5");
            Assert.AreEqual(0.5, result.OptimalPoint[1], 1e-2, "y should be near 0.5");
        }

        [TestMethod]
        public void CanMinimizeWithInequalityConstraint()
        {
            // Minimize (x - 1)² + (y - 1)² subject to x² + y² ≤ 1
            // Solution: point on unit circle closest to (1,1), which is (√2/2, √2/2)
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(s_inequalityInitial);
            optimizer.WithInequalityConstraint(x =>
                TestObjectiveFunctionsGradients.UnitCircleConstraintReverse(x[0], x[1]));
            optimizer.WithTolerance(1e-4);
            optimizer.WithConstraintTolerance(1e-5);
            optimizer.WithMaxIterations(30);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.DistanceFromTargetReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");

            // Check constraint is satisfied: x² + y² ≤ 1
            var circleValue = result.OptimalPoint[0] * result.OptimalPoint[0] +
                             result.OptimalPoint[1] * result.OptimalPoint[1];
            Assert.IsTrue(circleValue <= 1.0 + 1e-4,
                $"Inequality constraint should be satisfied: x² + y² = {circleValue}, should be ≤ 1.0");

            // Should be on the boundary (active constraint)
            Assert.AreEqual(1.0, circleValue, 1e-2, "Should be on unit circle boundary");

            // Check direction: should be toward (1,1) from origin
            var expectedX = Math.Sqrt(2) / 2.0; // ≈ 0.707
            var expectedY = Math.Sqrt(2) / 2.0;
            Assert.AreEqual(expectedX, result.OptimalPoint[0], 1e-1, "x should be near √2/2");
            Assert.AreEqual(expectedY, result.OptimalPoint[1], 1e-1, "y should be near √2/2");
        }

        [TestMethod]
        public void CanMinimizeRosenbrockWithEqualityConstraint()
        {
            // Minimize Rosenbrock subject to x + y = 1
            // This tests a more challenging equality-constrained problem
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(s_equalityInitial);
            optimizer.WithEqualityConstraint(x =>
                TestObjectiveFunctionsGradients.LinearEqualityConstraintReverse(x[0], x[1]));
            optimizer.WithUnconstrainedOptimizer(new LBFGSOptimizer());
            optimizer.WithTolerance(1e-4);
            optimizer.WithConstraintTolerance(1e-6);
            optimizer.WithMaxIterations(30);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");

            // Check constraint is satisfied
            var constraintValue = result.OptimalPoint[0] + result.OptimalPoint[1] - 1.0;
            Assert.IsTrue(Math.Abs(constraintValue) < 1e-4,
                $"Equality constraint should be satisfied: x + y = {result.OptimalPoint[0] + result.OptimalPoint[1]}");
        }

        [TestMethod]
        public void CanMinimizeWithMixedConstraints()
        {
            // Minimize (x - 1)² + (y - 1)² subject to:
            //   x + y = 1 (equality)
            //   x² + y² ≤ 1 (inequality)
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(s_inequalityInitial);
            optimizer.WithEqualityConstraint(x =>
                TestObjectiveFunctionsGradients.LinearEqualityConstraintReverse(x[0], x[1]));
            optimizer.WithInequalityConstraint(x =>
                TestObjectiveFunctionsGradients.UnitCircleConstraintReverse(x[0], x[1]));
            optimizer.WithTolerance(1e-4);
            optimizer.WithConstraintTolerance(1e-5);
            optimizer.WithMaxIterations(30);

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.DistanceFromTargetReverse(x[0], x[1])
            );

            Assert.IsTrue(result.Success, $"Optimization should succeed, got: {result.Message}");

            // Check both constraints
            var equalityViolation = Math.Abs(result.OptimalPoint[0] + result.OptimalPoint[1] - 1.0);
            var circleValue = result.OptimalPoint[0] * result.OptimalPoint[0] +
                             result.OptimalPoint[1] * result.OptimalPoint[1];

            Assert.IsTrue(equalityViolation < 1e-4, $"Equality constraint violated: {equalityViolation}");
            Assert.IsTrue(circleValue <= 1.0 + 1e-4, $"Inequality constraint violated: {circleValue}");

            // With x + y = 1, the solution should be (0.5, 0.5) since
            // 0.5² + 0.5² = 0.5 < 1, so inequality is inactive
            Assert.AreEqual(0.5, result.OptimalPoint[0], 1e-2, "x should be near 0.5");
            Assert.AreEqual(0.5, result.OptimalPoint[1], 1e-2, "y should be near 0.5");
        }

        [TestMethod]
        public void ReturnsMaxIterationsWhenNotConverged()
        {
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(s_equalityInitial);
            optimizer.WithEqualityConstraint(x =>
                TestObjectiveFunctionsGradients.LinearEqualityConstraintReverse(x[0], x[1]));
            optimizer.WithTolerance(1e-10); // Very tight
            optimizer.WithConstraintTolerance(1e-10);
            optimizer.WithMaxIterations(2); // Very few iterations

            var result = optimizer.Minimize(x =>
                TestObjectiveFunctionsGradients.RosenbrockReverse(x[0], x[1])
            );

            Assert.IsFalse(result.Success, "Should not succeed with very few iterations");
            Assert.AreEqual(StoppingReason.MaxIterations, result.StoppingReason);
        }

        [TestMethod]
        public void WorksWithDifferentUnconstrainedOptimizers()
        {
            // Test that we can use different unconstrained optimizers
            var gdOptimizer = new GradientDescentOptimizer();
            gdOptimizer.WithLineSearch(new BacktrackingLineSearch());

            var optimizers = new IOptimizer[]
            {
                new LBFGSOptimizer(),
                new ConjugateGradientOptimizer(),
                gdOptimizer
            };

            foreach (var unconstrainedOpt in optimizers)
            {
                var optimizer = new AugmentedLagrangianOptimizer();
                optimizer.WithInitialPoint(s_equalityInitial);
                optimizer.WithEqualityConstraint(x =>
                    TestObjectiveFunctionsGradients.LinearEqualityConstraintReverse(x[0], x[1]));
                optimizer.WithUnconstrainedOptimizer(unconstrainedOpt);
                optimizer.WithTolerance(1e-3);
                optimizer.WithConstraintTolerance(1e-4);
                optimizer.WithMaxIterations(30);

                var result = optimizer.Minimize(x =>
                    TestObjectiveFunctionsGradients.DistanceFromTargetReverse(x[0], x[1])
                );

                Assert.IsTrue(result.Success, $"Should succeed with {unconstrainedOpt.GetType().Name}");

                var constraintValue = result.OptimalPoint[0] + result.OptimalPoint[1] - 1.0;
                Assert.IsTrue(Math.Abs(constraintValue) < 1e-3,
                    $"Constraint should be satisfied with {unconstrainedOpt.GetType().Name}");
            }
        }
    }
}
