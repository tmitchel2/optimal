/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.Control.Core.Tests
{
    /// <summary>
    /// Test cost functions for optimal control validation.
    /// </summary>
    [OptimalCode]
    public static class TestCostFunctions
    {
        /// <summary>
        /// Quadratic control cost: L(x, u) = 0.5 * u²
        /// Used for minimum energy problems.
        /// </summary>
        /// <param name="x">State (unused).</param>
        /// <param name="u">Control input.</param>
        public static double QuadraticControlCost(double x, double u)
        {
            return 0.5 * u * u;
        }

        /// <summary>
        /// Quadratic control cost for double integrator: L = 0.5 * u²
        /// </summary>
        /// <param name="x0">Position.</param>
        /// <param name="x1">Velocity.</param>
        /// <param name="u">Control input.</param>
        public static double DoubleIntegratorEnergyCost(double x0, double x1, double u)
        {
            return 0.5 * u * u;
        }

        /// <summary>
        /// Quadratic state and control cost: L = 0.5 * (x² + u²)
        /// </summary>
        /// <param name="x">State.</param>
        /// <param name="u">Control.</param>
        public static double QuadraticStateAndControl(double x, double u)
        {
            return 0.5 * (x * x + u * u);
        }

        /// <summary>
        /// Time cost: L = 1 (for minimum time problems)
        /// </summary>
        public static double UnitCost(double x, double u)
        {
            return 1.0 + 0.0 * x + 0.0 * u;
        }

        /// <summary>
        /// Terminal cost: distance from target state.
        /// Φ(x) = 0.5 * (x - target)²
        /// </summary>
        /// <param name="x">Final state.</param>
        /// <param name="target">Target state.</param>
        public static double DistanceFromTarget(double x, double target)
        {
            var diff = x - target;
            return 0.5 * diff * diff;
        }

        /// <summary>
        /// Terminal cost for double integrator: distance from target position.
        /// Φ = 0.5 * (position - target)²
        /// </summary>
        /// <param name="x0">Final position.</param>
        /// <param name="x1">Final velocity.</param>
        /// <param name="target">Target position.</param>
        public static double DoubleIntegratorTerminalCost(double x0, double x1, double target)
        {
            var diff = x0 - target;
            return 0.5 * diff * diff;
        }
    }
}
