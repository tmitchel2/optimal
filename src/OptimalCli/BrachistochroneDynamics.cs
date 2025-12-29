/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli
{
    /// <summary>
    /// Brachistochrone problem dynamics with AutoDiff support.
    /// State: [x (horizontal position), y (vertical position), v (velocity)]
    /// Control: θ (path angle from horizontal)
    /// 
    /// The brachistochrone problem finds the curve of fastest descent under gravity
    /// between two points. The solution is a cycloid curve.
    /// </summary>
    [OptimalCode]
    public static class BrachistochroneDynamics
    {
        /// <summary>
        /// Horizontal position rate: ẋ = v·cos(θ)
        /// </summary>
        public static double XRate(double x, double y, double v, double theta)
        {
            return v * Math.Cos(theta);
        }

        /// <summary>
        /// Vertical position rate: ẏ = v·sin(θ)
        /// </summary>
        public static double YRate(double x, double y, double v, double theta)
        {
            return v * Math.Sin(theta);
        }

        /// <summary>
        /// Velocity rate: v̇ = -g·sin(θ)
        /// Negative because gravity accelerates downward (θ < 0 for downward paths)
        /// </summary>
        public static double VelocityRate(double x, double y, double v, double theta, double g)
        {
            return -g * Math.Sin(theta);
        }

        /// <summary>
        /// Running cost: Minimize time L = 1
        /// </summary>
        public static double RunningCost(double x, double y, double v, double theta)
        {
            return 1.0;
        }
    }
}
