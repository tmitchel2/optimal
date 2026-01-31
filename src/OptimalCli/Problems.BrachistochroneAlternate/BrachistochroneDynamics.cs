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
    // NOTE: [OptimalCode] attribute removed because the smooth clamping functions use
    // conditionals which the AutoDiff source generator doesn't support.
    // The solver uses numerical gradients with central differences instead.
    public static class BrachistochroneAlternateDynamics
    {
        // Reference line geometry
        private const double Xf = 10.0;
        private const double Nf = 5.0;

        // Clamping thresholds
        private const double VMin = 1.0;
        private const double CosMin = 0.1;

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

        // === Clamping functions ===

        /// <summary>
        /// Clamps velocity to ensure v >= VMin.
        /// Uses hard clamping for stability but the thresholds are chosen
        /// to be small enough that typical trajectories stay above them.
        /// </summary>
        public static double SafeVelocity(double v)
        {
            return v < VMin ? VMin : v;
        }

        /// <summary>
        /// Clamps cos(alpha) to ensure |cos(alpha)| >= CosMin.
        /// Uses hard clamping for stability.
        /// </summary>
        public static double SafeCosine(double cosAlpha)
        {
            if (Math.Abs(cosAlpha) < CosMin)
            {
                return cosAlpha >= 0 ? CosMin : -CosMin;
            }
            return cosAlpha;
        }

        // === Arc-length transformation ===

        /// <summary>
        /// Time rate: dt/ds = 1 / (v * cos(alpha))
        /// This is the core transformation from time to horizontal-distance parameterization.
        /// Uses clamping to prevent numerical instability.
        /// </summary>
        public static double TimeRateS(double v, double alpha)
        {
            var safeV = SafeVelocity(v);
            var cosAlpha = Math.Cos(alpha);
            var safeCos = SafeCosine(cosAlpha);
            var result = 1.0 / (safeV * safeCos);
            // Clamp output to reasonable range to prevent solver divergence
            return Math.Clamp(result, -1000.0, 1000.0);
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
        /// Uses clamping to prevent numerical instability.
        /// </summary>
        public static double SpeedRateS(double v, double alpha, double g, double thetaRef)
        {
            var safeV = SafeVelocity(v);
            var cosAlpha = Math.Cos(alpha);
            var safeCos = SafeCosine(cosAlpha);
            var result = g * Math.Sin(alpha + thetaRef) / (safeV * safeCos);
            // Clamp output to reasonable range to prevent solver divergence
            return Math.Clamp(result, -1000.0, 1000.0);
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
        /// Uses clamping to prevent numerical instability.
        /// </summary>
        public static double TimeRate(double v, double alpha)
        {
            var safeV = SafeVelocity(v);
            var cosAlpha = Math.Cos(alpha);
            var safeCos = SafeCosine(cosAlpha);
            var result = 1.0 / (safeV * safeCos);
            // Clamp output to reasonable range to prevent solver divergence
            return Math.Clamp(result, -1000.0, 1000.0);
        }

        // === Running cost ===

        /// <summary>
        /// Running cost = dt/ds. Integrating this over s gives total elapsed time.
        /// Uses clamping to prevent numerical instability.
        /// </summary>
        public static double RunningCostS(double v, double alpha)
        {
            var safeV = SafeVelocity(v);
            var cosAlpha = Math.Cos(alpha);
            var safeCos = SafeCosine(cosAlpha);
            var result = 1.0 / (safeV * safeCos);
            // Clamp output to reasonable range to prevent solver divergence
            return Math.Clamp(result, -1000.0, 1000.0);
        }
    }
}
