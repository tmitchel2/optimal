/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Corner
{
    /// <summary>
    /// Corner problem dynamics with AutoDiff support.
    /// State: [x (position), y (position), θ (heading angle), v (velocity)]
    /// Control: [a (acceleration), ω (steering rate)]
    ///
    /// The corner problem finds the optimal racing line through a 90° turn
    /// while staying within road boundaries.
    /// </summary>
    [OptimalCode]
    public static class CornerDynamics
    {
        /// <summary>
        /// Horizontal position rate: ẋ = v·cos(θ)
        /// </summary>
        public static double XRate(double x, double y, double theta, double v)
        {
            return v * Math.Cos(theta);
        }

        /// <summary>
        /// Vertical position rate: ẏ = v·sin(θ)
        /// </summary>
        public static double YRate(double x, double y, double theta, double v)
        {
            return v * Math.Sin(theta);
        }

        /// <summary>
        /// Heading rate: θ̇ = ω (steering rate)
        /// </summary>
        public static double ThetaRate(double omega)
        {
            return omega;
        }

        /// <summary>
        /// Velocity rate: v̇ = a (acceleration)
        /// </summary>
        public static double VelocityRate(double a)
        {
            return a;
        }

        /// <summary>
        /// Running cost: Minimize time (constant = 1)
        /// This minimizes total time when integrated over the trajectory.
        /// </summary>
        public static double RunningCost()
        {
            return 1.0;
        }
    }
}
