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
    /// Test dynamics functions for validating optimal control implementation.
    /// </summary>
    [OptimalCode]
    public static class TestDynamics
    {
        /// <summary>
        /// Simple 1D integrator: ẋ = u
        /// </summary>
        /// <param name="x">State (position).</param>
        /// <param name="u">Control (velocity).</param>
        public static double SimpleIntegrator(double x, double u)
        {
            return u;
        }

        /// <summary>
        /// Double integrator state derivative 0: ẋ₀ = x₁
        /// </summary>
        /// <param name="x0">Position.</param>
        /// <param name="x1">Velocity.</param>
        /// <param name="u">Acceleration (control).</param>
        public static double DoubleIntegratorDx0(double x0, double x1, double u)
        {
            return x1;
        }

        /// <summary>
        /// Double integrator state derivative 1: ẋ₁ = u
        /// </summary>
        /// <param name="x0">Position.</param>
        /// <param name="x1">Velocity.</param>
        /// <param name="u">Acceleration (control).</param>
        public static double DoubleIntegratorDx1(double x0, double x1, double u)
        {
            return u;
        }
    }
}
