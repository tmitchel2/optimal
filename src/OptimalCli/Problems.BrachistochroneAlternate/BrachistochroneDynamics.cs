/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.BrachistochroneAlternate
{
    /// <summary>
    /// Brachistochrone problem dynamics with arc-length parameterization.
    ///
    /// This formulation mirrors the CornerProblemSolver structure but simplified for
    /// the Brachistochrone problem (finding the curve of fastest descent under gravity).
    ///
    /// State: [v, n, alpha, t]
    ///   - v: Speed (m/s)
    ///   - n: Vertical position below start (m, positive downward = descended)
    ///   - alpha: Heading angle from horizontal (rad, positive = descending)
    ///   - t: Elapsed time (s)
    ///
    /// Control: [k]
    ///   - k: Path curvature = dalpha/ds (rad/m)
    ///
    /// Independent variable: s (horizontal distance from start)
    ///
    /// Key insight: All dynamics are dx/ds where s = horizontal position x.
    /// The transformation is: df/ds = (df/dt) / (ds/dt) = (df/dt) / (v * cos(alpha))
    /// </summary>
    [OptimalCode]
    public static class BrachistochroneAlternateDynamics
    {
        // === Arc-length transformation ===

        /// <summary>
        /// Time rate: dt/ds = 1 / (v * cos(alpha))
        /// This is the core transformation from time to horizontal-distance parameterization.
        /// </summary>
        public static double TimeRateS(double v, double alpha)
        {
            var safeV = v < 0.01 ? 0.01 : v;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return 1.0 / (safeV * safeCos);
        }

        // === State dynamics (derivatives w.r.t. horizontal distance s) ===

        /// <summary>
        /// dv/ds - Speed rate due to gravity component along trajectory.
        /// dv/dt = g * sin(alpha), so dv/ds = g * sin(alpha) / (v * cos(alpha)) = g * tan(alpha) / v
        /// </summary>
        public static double SpeedRateS(double v, double alpha, double g)
        {
            var safeV = v < 0.01 ? 0.01 : v;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return g * Math.Sin(alpha) / (safeV * safeCos);
        }

        /// <summary>
        /// dn/ds - Vertical position rate (descent rate per unit horizontal distance).
        /// dn/dt = v * sin(alpha), so dn/ds = tan(alpha)
        /// </summary>
        public static double VerticalRateS(double alpha)
        {
            return Math.Tan(alpha);
        }

        /// <summary>
        /// dalpha/ds - Heading angle rate (this is the curvature, directly controlled).
        /// </summary>
        public static double AlphaRateS(double k)
        {
            return k;
        }

        /// <summary>
        /// dt/ds - Time rate (same as TimeRateS, explicit for state dynamics).
        /// </summary>
        public static double TimeRate(double v, double alpha)
        {
            var safeV = v < 0.01 ? 0.01 : v;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return 1.0 / (safeV * safeCos);
        }

        // === Running cost ===

        /// <summary>
        /// Running cost = dt/ds. Integrating this over s gives total elapsed time.
        /// </summary>
        public static double RunningCostS(double v, double alpha)
        {
            var safeV = v < 0.01 ? 0.01 : v;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return 1.0 / (safeV * safeCos);
        }
    }
}
