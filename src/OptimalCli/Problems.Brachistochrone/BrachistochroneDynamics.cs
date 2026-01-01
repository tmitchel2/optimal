/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Brachistochrone
{
    /// <summary>
    /// Brachistochrone problem dynamics with AutoDiff support.
    /// State: [x (horizontal position), y (vertical position), v (velocity)]
    /// Control: θ (angle from horizontal)
    ///
    /// The brachistochrone problem (Johann Bernoulli, 1696) seeks the curve of fastest descent
    /// for a bead sliding under gravity from point A to point B. The optimal solution is a cycloid.
    ///
    /// </summary>
    [OptimalCode]
    public static class BrachistochroneDynamics
    {
        /// <summary>
        /// Horizontal velocity: ẋ = v·cos(θ)
        /// </summary>
        public static double XRate(double x, double y, double v, double theta, double g)
        {
            return v * Math.Cos(theta);
        }

        /// <summary>
        /// Vertical velocity: ẏ = -v·sin(θ)
        /// (negative because y decreases as bead descends - left-hand rule with y-axis up)
        /// </summary>
        public static double YRate(double x, double y, double v, double theta, double g)
        {
            return -v * Math.Sin(theta);
        }

        /// <summary>
        /// Speed rate: v̇ = g·sin(θ)
        /// (acceleration due to gravity component along trajectory)
        /// </summary>
        public static double VRate(double x, double y, double v, double theta, double g)
        {
            return g * Math.Sin(theta);
        }

        /// <summary>
        /// Running cost: L = 1 (minimize time)
        /// Since dt is integrated, minimizing ∫1·dt minimizes total time.
        /// </summary>
        public static double RunningCost(double x, double y, double v, double theta)
        {
            return 1.0;
        }

        /// <summary>
        /// Terminal cost: Φ = 0 (no terminal cost, only running cost)
        /// </summary>
        public static double TerminalCost(double x, double y, double v)
        {
            return 0.0;
        }
    }
}
