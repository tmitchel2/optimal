/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */


namespace OptimalCli.Problems.BrachistochroneAlternate
{
    /// <summary>
    /// Brachistochrone problem dynamics with reference-line-aligned coordinate system.
    ///
    /// This formulation uses a rotated coordinate system where:
    /// - s: distance along the straight reference line connecting start to end points
    /// - n: perpendicular distance from that reference line (positive = "below" line)
    /// - alpha: heading angle relative to reference line direction
    ///
    /// Reference line geometry:
    /// - From (0,0) to (Xf=10, Nf=5) in Cartesian coordinates
    /// - Reference angle: theta_ref = atan(Nf/Xf) ≈ 26.57°
    /// - Total line distance: S_total = sqrt(Xf² + Nf²) ≈ 11.18m
    ///
    /// State: [v, n, alpha, t]
    ///   - v: Speed (m/s)
    ///   - n: Perpendicular distance from reference line (m, positive = below line)
    ///   - alpha: Heading angle relative to reference line (rad)
    ///   - t: Elapsed time (s)
    ///
    /// Control: [k]
    ///   - k: Path curvature = dalpha/ds (rad/m)
    ///
    /// Independent variable: s (distance along reference line from start)
    ///
    /// Coordinate transformation from (s, n) to Cartesian (x, y_down):
    ///   x = s * cos(theta_ref) - n * sin(theta_ref)
    ///   y_down = s * sin(theta_ref) + n * cos(theta_ref)
    /// </summary>
    public static class BrachistochroneAlternateDynamics
    {
        // Reference line geometry
        private const double Xf = 10.0;
        private const double Nf = 5.0;

        /// <summary>
        /// Reference angle: angle of the straight line from start to end point.
        /// theta_ref = atan2(Nf, Xf) ≈ 0.4636 rad ≈ 26.57°
        /// </summary>
        public static readonly double ThetaRef = Math.Atan2(Nf, Xf);

        /// <summary>
        /// Total distance along the reference line from start to end.
        /// S_total = sqrt(Xf² + Nf²) ≈ 11.18m
        /// </summary>
        public static readonly double STotal = Math.Sqrt(Xf * Xf + Nf * Nf);

        // === Arc-length transformation ===

        /// <summary>
        /// Time rate: dt/ds = 1 / (v * cos(alpha))
        /// This is the core transformation from time to horizontal-distance parameterization.
        /// </summary>
        public static double TimeRateS(double v, double alpha)
        {
            return 1.0 / (v * Math.Cos(alpha));
        }

        // === State dynamics (derivatives w.r.t. horizontal distance s) ===

        /// <summary>
        /// dv/ds - Speed rate due to gravity component along trajectory.
        ///
        /// In the rotated frame, alpha is measured relative to the reference line direction.
        /// The actual world angle (from horizontal) is alpha + thetaRef.
        ///
        /// Gravity acceleration along velocity = g * sin(alpha_world) = g * sin(alpha + thetaRef)
        /// The arc-length rate ds/dt = v * cos(alpha) (movement along reference line direction)
        ///
        /// Therefore: dv/ds = (dv/dt) / (ds/dt) = g * sin(alpha + thetaRef) / (v * cos(alpha))
        /// </summary>
        public static double SpeedRateS(double v, double alpha, double g, double thetaRef)
        {
            return g * Math.Sin(alpha + thetaRef) / (v * Math.Cos(alpha));
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
            return 1.0 / (v * Math.Cos(alpha));
        }

        // === Running cost ===

        /// <summary>
        /// Running cost = dt/ds. Integrating this over s gives total elapsed time.
        /// </summary>
        public static double RunningCostS(double v, double alpha)
        {
            return 1.0 / (v * Math.Cos(alpha));
        }
    }
}
