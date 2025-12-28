/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear.Tests
{
    /// <summary>
    /// Test objective functions for nonlinear optimization.
    /// </summary>
    [OptimalCode]
    public static class TestObjectiveFunctions
    {
        /// <summary>
        /// Simple quadratic function: f(x,y) = x^2 + y^2
        /// Minimum: (0, 0), f(0, 0) = 0
        /// </summary>
        public static double Quadratic(double x, double y) => x * x + y * y;

        /// <summary>
        /// Rosenbrock function: f(x,y) = (1-x)^2 + 100(y-x^2)^2
        /// Minimum: (1, 1), f(1, 1) = 0
        /// </summary>
        public static double Rosenbrock(double x, double y)
        {
            return (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
        }

        /// <summary>
        /// Beale function: f(x,y) = (1.5 - x + xy)^2 + (2.25 - x + xy^2)^2 + (2.625 - x + xy^3)^2
        /// Minimum: (3, 0.5), f(3, 0.5) = 0
        /// </summary>
        public static double Beale(double x, double y)
        {
            return (1.5 - x + x * y) * (1.5 - x + x * y) +
                   (2.25 - x + x * y * y) * (2.25 - x + x * y * y) +
                   (2.625 - x + x * y * y * y) * (2.625 - x + x * y * y * y);
        }

        /// <summary>
        /// Booth function: f(x,y) = (x + 2y - 7)^2 + (2x + y - 5)^2
        /// Minimum: (1, 3), f(1, 3) = 0
        /// </summary>
        public static double Booth(double x, double y)
        {
            return (x + 2.0 * y - 7.0) * (x + 2.0 * y - 7.0) +
                   (2.0 * x + y - 5.0) * (2.0 * x + y - 5.0);
        }

        /// <summary>
        /// Extended Rosenbrock function (4D): sum of Rosenbrock terms
        /// Minimum: (1, 1, 1, 1), f = 0
        /// </summary>
        public static double Rosenbrock4D(double x1, double x2, double x3, double x4)
        {
            return 100.0 * (x2 - x1 * x1) * (x2 - x1 * x1) + (1.0 - x1) * (1.0 - x1) +
                   100.0 * (x3 - x2 * x2) * (x3 - x2 * x2) + (1.0 - x2) * (1.0 - x2) +
                   100.0 * (x4 - x3 * x3) * (x4 - x3 * x3) + (1.0 - x3) * (1.0 - x3);
        }

        /// <summary>
        /// Extended Rosenbrock function (10D): sum of Rosenbrock terms
        /// Minimum: (1, 1, ..., 1), f = 0
        /// </summary>
        public static double Rosenbrock10D(double x1, double x2, double x3, double x4, double x5,
                                          double x6, double x7, double x8, double x9, double x10)
        {
            return 100.0 * (x2 - x1 * x1) * (x2 - x1 * x1) + (1.0 - x1) * (1.0 - x1) +
                   100.0 * (x3 - x2 * x2) * (x3 - x2 * x2) + (1.0 - x2) * (1.0 - x2) +
                   100.0 * (x4 - x3 * x3) * (x4 - x3 * x3) + (1.0 - x3) * (1.0 - x3) +
                   100.0 * (x5 - x4 * x4) * (x5 - x4 * x4) + (1.0 - x4) * (1.0 - x4) +
                   100.0 * (x6 - x5 * x5) * (x6 - x5 * x5) + (1.0 - x5) * (1.0 - x5) +
                   100.0 * (x7 - x6 * x6) * (x7 - x6 * x6) + (1.0 - x6) * (1.0 - x6) +
                   100.0 * (x8 - x7 * x7) * (x8 - x7 * x7) + (1.0 - x7) * (1.0 - x7) +
                   100.0 * (x9 - x8 * x8) * (x9 - x8 * x8) + (1.0 - x8) * (1.0 - x8) +
                   100.0 * (x10 - x9 * x9) * (x10 - x9 * x9) + (1.0 - x9) * (1.0 - x9);
        }

        // ===== Constraint Functions =====

        /// <summary>
        /// Linear equality constraint: x + y - 1 = 0
        /// Used for testing equality-constrained optimization.
        /// </summary>
        public static double LinearEqualityConstraint(double x, double y) => x + y - 1.0;

        /// <summary>
        /// Unit circle inequality constraint: x² + y² - 1 ≤ 0 (inside or on unit circle)
        /// </summary>
        public static double UnitCircleConstraint(double x, double y) => x * x + y * y - 1.0;

        /// <summary>
        /// Distance from origin objective: (x - 1)² + (y - 1)²
        /// Used with constraints to test constrained optimization.
        /// </summary>
        public static double DistanceFromTarget(double x, double y) => (x - 1.0) * (x - 1.0) + (y - 1.0) * (y - 1.0);
    }
}

