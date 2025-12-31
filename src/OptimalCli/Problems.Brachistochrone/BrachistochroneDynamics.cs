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
        /// Rate of change of s (position along diagonal).
        /// ṡ = v·cos(θ)
        /// where θ is the angle of motion relative to the diagonal direction.
        /// </summary>
        public static double SRate(double s, double d, double v, double theta, double g, double alpha)
        {
            return v * Math.Cos(theta);
        }

        /// <summary>
        /// Rate of change of d (perpendicular distance from diagonal).
        /// ḋ = v·sin(θ)
        /// where θ is the angle of motion relative to the diagonal direction.
        /// Positive θ means curving away from the diagonal.
        /// </summary>
        public static double DRate(double s, double d, double v, double theta, double g, double alpha)
        {
            return v * Math.Sin(theta);
        }

        /// <summary>
        /// Rate of change of velocity (acceleration).
        /// v̇ = g·sin(α + θ)
        /// where:
        /// - g is gravitational acceleration
        /// - α is the angle of the diagonal line from horizontal
        /// - θ is the angle of motion relative to the diagonal
        /// 
        /// The acceleration comes from the component of gravity along the path direction.
        /// The path angle from horizontal is (α + θ).
        /// </summary>
        public static double VRate(double s, double d, double v, double theta, double g, double alpha)
        {
            return g * Math.Sin(alpha + theta);
        }

        /// <summary>
        /// Running cost: L = 0 (we minimize time via terminal cost, no running cost).
        /// </summary>
        public static double RunningCost(double s, double d, double v, double theta)
        {
            return 0.0;
        }

        /// <summary>
        /// Terminal cost: Φ = t (time, which is the independent variable).
        /// To minimize time, we return a constant (the actual time minimization
        /// is handled by the time horizon being free).
        /// </summary>
        public static double TerminalCost(double s, double d, double v)
        {
            return 1.0;
        }
    }
}
