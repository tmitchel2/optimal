/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Dubin
{
    /// <summary>
    /// Dubins car problem dynamics with AutoDiff support.
    /// State: [x (horizontal position), y (vertical position), θ (heading angle)]
    /// Control: ω (turning rate)
    ///
    /// The Dubins car problem finds the shortest path for a car moving at constant
    /// velocity with a bounded turning rate between two configurations.
    /// </summary>
    [OptimalCode]
    public static class DubinsCarDynamics
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
        /// Heading rate: θ̇ = ω (turning rate)
        /// </summary>
        public static double ThetaRate(double x, double y, double theta, double omega)
        {
            return omega;
        }

        /// <summary>
        /// Running cost: Minimize control effort L = 0.1·ω²
        /// </summary>
        public static double RunningCost(double x, double y, double theta, double omega)
        {
            return 0.1 * omega * omega;
        }
    }
}
