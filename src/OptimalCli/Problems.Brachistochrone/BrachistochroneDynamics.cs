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
    /// State: [s (position along diagonal), d (perpendicular distance from diagonal), v (velocity)]
    /// Control: θ (path angle relative to diagonal, in RADIANS)
    ///
    /// The brachistochrone problem finds the curve of fastest descent under gravity
    /// between two points. The solution is a cycloid curve.
    ///
    /// Coordinate system: Instead of using (x,y), we use (s,d) where:
    /// - s is the position along the diagonal from start (0,0) to end (xFinal, yFinal)
    /// - d is the perpendicular distance from the diagonal line
    /// - Diagonal length L = sqrt(xFinal² + yFinal²)
    ///
    /// IMPORTANT: All angle parameters (theta) are in RADIANS, not degrees.
    /// </summary>
    [OptimalCode]
    public static class BrachistochroneDynamics
    {
        /// <summary>
        /// Position rate along diagonal: ṡ = v·cos(θ)
        /// where θ is the path angle relative to the diagonal direction.
        /// </summary>
        /// <param name="theta">Path angle in RADIANS</param>
        public static double SRate(double s, double d, double v, double theta)
        {
            return v * Math.Cos(theta);
        }

        /// <summary>
        /// Perpendicular distance rate: ḋ = v·sin(θ)
        /// Positive θ moves away from diagonal in positive d direction.
        /// </summary>
        /// <param name="theta">Path angle in RADIANS</param>
        public static double DRate(double s, double d, double v, double theta)
        {
            return v * Math.Sin(theta);
        }

        /// <summary>
        /// Velocity rate from gravity components with damping near the target.
        /// Gravity (0, -g) in Cartesian coordinates has components in (s,d) coordinates:
        /// - g_s = -g·yFinal/L (component along diagonal)
        /// - g_d = -g·xFinal/L (component perpendicular to diagonal)
        ///
        /// The acceleration along the path is: v̇ = g_s·cos(θ) + g_d·sin(θ) - damping
        /// Which simplifies to: v̇ = -g·(yFinal·cos(θ) + xFinal·sin(θ))/L - damping·v
        ///
        /// The damping term activates near the final point to bring the ball to a stop.
        /// </summary>
        /// <param name="theta">Path angle in RADIANS</param>
        public static double VelocityRate(double s, double d, double v, double theta, double g, double xFinal, double yFinal)
        {
            var L = Math.Sqrt(xFinal * xFinal + yFinal * yFinal);

            // Gravity acceleration component along the path
            var gravityAccel = -g * (yFinal * Math.Cos(theta) + xFinal * Math.Sin(theta)) / L;

            // Damping that activates near the final point
            // Distance to target in (s,d) space
            var distanceToTarget = Math.Sqrt((s - L) * (s - L) + d * d);

            // Damping coefficient that ramps up as we approach the target
            // Using a smooth transition: damping = k / (1 + (distance/threshold)^n)
            var dampingThreshold = 0.3; // Start damping when within 30cm of target
            var dampingStrength = 50.0; // Strong damping coefficient
            var dampingPower = 4.0; // Controls sharpness of transition

            var dampingCoeff = dampingStrength / (1.0 + Math.Pow(distanceToTarget / dampingThreshold, dampingPower));

            // Apply damping force proportional to velocity
            var dampingAccel = -dampingCoeff * v;

            return gravityAccel + dampingAccel;
        }

        /// <summary>
        /// Running cost: Minimize time L = 1, but smoothly reduced near destination.
        /// Uses a smooth sigmoidal function to transition cost to 0 at target.
        /// In (s,d) coordinates, the target is at (s=L, d=0).
        /// </summary>
        public static double RunningCost(double s, double d, double v, double theta, double xFinal, double yFinal)
        {
            var L = Math.Sqrt(xFinal * xFinal + yFinal * yFinal);

            // Distance to target in (s,d) coordinates
            var ds = s - L;
            var dd = d - 0.0;
            var distanceSquared = ds * ds + dd * dd;

            // Smooth transition: cost approaches 0 as distance approaches 0
            // Using: cost = 1.0 / (1.0 + exp(-k*(d^2 - threshold^2)))
            // When d=0, cost ≈ 0; when d is large, cost ≈ 1
            var threshold = 0.1; // 10cm threshold
            var steepness = 100.0; // Controls transition sharpness
            var scaledDistance = steepness * (distanceSquared - threshold * threshold);

            // Sigmoid: maps (-inf, inf) to (0, 1)
            var sigmoid = 1.0 / (1.0 + Math.Exp(-scaledDistance));

            return sigmoid;
        }
    }
}
