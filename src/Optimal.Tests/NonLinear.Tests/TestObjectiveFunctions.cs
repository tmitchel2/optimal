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
    }
}
