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
    /// Corner problem dynamics with AutoDiff support using curvilinear coordinates.
    /// State: [s (progress along centerline), n (lateral deviation), θ (heading angle), v (velocity)]
    /// Control: [a (acceleration), ω (steering rate)]
    ///
    /// Curvilinear coordinate system:
    /// - s: Distance along centerline (0 at start, increases monotonically)
    /// - n: Perpendicular distance from centerline (negative = left of centerline when facing forward)
    /// 
    /// Centerline path:
    /// - Entry straight: s ∈ [0, EntryLength)
    /// - Quarter-circle arc: s ∈ [EntryLength, EntryLength + ArcLength)
    /// - Exit straight: s ≥ EntryLength + ArcLength
    ///
    /// The corner problem finds the optimal racing line through a 90° turn
    /// while staying within road boundaries.
    /// </summary>
    [OptimalCode]
    public static class CornerDynamics
    {
        // Road geometry constants
        public const double EntryLength = 15.0;
        public const double CenterlineRadius = 5.0;  // Radius of centerline arc
        public const double RoadHalfWidth = 5.0;     // Half-width of road

        // Computed properties for use outside OptimalCode methods
        public static double ArcLength => Math.PI * CenterlineRadius / 2.0;
        public static double EntryEndS => EntryLength;
        public static double ArcEndS => EntryLength + ArcLength;

        /// <summary>
        /// Road heading angle at position s along centerline.
        /// Entry: θ_road = 0 (heading east)
        /// Arc: θ_road = -arcProgress × π/2
        /// Exit: θ_road = -π/2 (heading south)
        /// </summary>
        public static double RoadHeading(double s)
        {
            var arcLength = Math.PI * CenterlineRadius / 2.0;
            var entryEnd = EntryLength;
            var arcEnd = EntryLength + arcLength;

            if (s < entryEnd)
            {
                return 0.0;
            }
            else if (s < arcEnd)
            {
                var arcProgress = (s - entryEnd) / arcLength;
                return -arcProgress * Math.PI / 2.0;
            }
            else
            {
                return -Math.PI / 2.0;
            }
        }

        /// <summary>
        /// Road curvature κ at position s along centerline.
        /// Straight sections: κ = 0
        /// Arc section: κ = 1/R (positive for right turn)
        /// </summary>
        public static double RoadCurvature(double s)
        {
            var arcLength = Math.PI * CenterlineRadius / 2.0;
            var entryEnd = EntryLength;
            var arcEnd = EntryLength + arcLength;

            if (s < entryEnd)
            {
                return 0.0;
            }
            else if (s < arcEnd)
            {
                return 1.0 / CenterlineRadius;
            }
            else
            {
                return 0.0;
            }
        }

        /// <summary>
        /// Progress rate along centerline: ṡ = v × cos(θ - θ_road) / (1 - n × κ)
        /// </summary>
        public static double ProgressRate(double s, double n, double theta, double v)
        {
            var arcLength = Math.PI * CenterlineRadius / 2.0;
            var entryEnd = EntryLength;
            var arcEnd = EntryLength + arcLength;

            double thetaRoad;
            double curvature;

            if (s < entryEnd)
            {
                thetaRoad = 0.0;
                curvature = 0.0;
            }
            else if (s < arcEnd)
            {
                var arcProgress = (s - entryEnd) / arcLength;
                thetaRoad = -arcProgress * Math.PI / 2.0;
                curvature = 1.0 / CenterlineRadius;
            }
            else
            {
                thetaRoad = -Math.PI / 2.0;
                curvature = 0.0;
            }

            var headingError = theta - thetaRoad;
            var denominator = 1.0 - n * curvature;

            // Prevent division by zero (shouldn't happen within road bounds)
            if (denominator < 0.1)
            {
                denominator = 0.1;
            }

            return v * Math.Cos(headingError) / denominator;
        }

        /// <summary>
        /// Lateral deviation rate: ṅ = v × sin(θ - θ_road)
        /// </summary>
        public static double LateralRate(double s, double n, double theta, double v)
        {
            var arcLength = Math.PI * CenterlineRadius / 2.0;
            var entryEnd = EntryLength;
            var arcEnd = EntryLength + arcLength;

            double thetaRoad;

            if (s < entryEnd)
            {
                thetaRoad = 0.0;
            }
            else if (s < arcEnd)
            {
                var arcProgress = (s - entryEnd) / arcLength;
                thetaRoad = -arcProgress * Math.PI / 2.0;
            }
            else
            {
                thetaRoad = -Math.PI / 2.0;
            }

            var headingError = theta - thetaRoad;
            return v * Math.Sin(headingError);
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

        /// <summary>
        /// Convert curvilinear coordinates (s, n) to Cartesian coordinates (x, y).
        /// Used for visualization.
        /// </summary>
        public static (double x, double y) CurvilinearToCartesian(double s, double n)
        {
            var arcLength = ArcLength;
            var entryEnd = EntryLength;
            var arcEnd = entryEnd + arcLength;

            double x, y;

            if (s < entryEnd)
            {
                // Entry straight: centerline is y=0, heading east
                // x = s - EntryLength (so at s=0, x=-15; at s=EntryLength, x=0)
                // n positive = right of centerline = negative y
                x = s - EntryLength;
                y = -n;  // Flipped: positive n = right of centerline = negative y
            }
            else if (s < arcEnd)
            {
                // Arc: centerline is quarter circle from (0,0) to (5,-5)
                // Arc center is at (5, -5) with radius 5
                // At s=entryEnd (arc start), angle=π/2 (pointing up), position on circle is (5, 0)
                // Wait, we need to re-derive this properly.
                //
                // The centerline arc starts at (0, 0) heading east, curves right 90°.
                // The arc is centered at (0, -5) with radius 5 (so it starts at (0, 0) and ends at (5, -5)).
                // Wait, that doesn't give the right geometry either.
                //
                // Let's define it correctly:
                // - Entry ends at world position (0, 0), heading θ=0 (east)
                // - Arc curves right (clockwise) by 90°
                // - Exit starts heading θ=-π/2 (south)
                //
                // For a right turn: center of curvature is to the right of the path.
                // At (0, 0) heading east, right is negative y, so center is at (0, -R) = (0, -5)
                // Arc: from (0, 0) to (5, -5), centered at (0, -5), radius 5
                //
                // Parametric: angle α goes from π/2 (north) to 0 (east) as we traverse
                // Actually with center at (0, -5):
                // - Start point (0, 0): angle = π/2 (from center, going up)
                // - End point (5, -5): angle = 0 (from center, going right)
                //
                // For the centerline position at arc progress p ∈ [0, 1]:
                // angle = π/2 - p × π/2 = π/2 × (1 - p)
                // x = 0 + 5 × cos(angle) = 5 × cos(π/2 × (1 - p)) = 5 × sin(π/2 × p)
                // y = -5 + 5 × sin(angle) = -5 + 5 × sin(π/2 × (1 - p)) = -5 + 5 × cos(π/2 × p)
                
                var arcProgress = (s - entryEnd) / arcLength;
                var angle = Math.PI / 2.0 * (1.0 - arcProgress);
                
                // Centerline position
                var cx = CenterlineRadius * Math.Cos(angle);
                var cy = -CenterlineRadius + CenterlineRadius * Math.Sin(angle);
                
                // n offset: perpendicular to arc, positive = outward (right of centerline)
                // Outward direction from center at this angle: (cos(angle), sin(angle))
                // But n positive = right of centerline when facing forward
                // Forward tangent is perpendicular to radius, going clockwise
                // If radius points at angle α, tangent points at α - π/2
                // Right is perpendicular to tangent, which is same as radius direction (outward)
                x = cx + n * Math.Cos(angle);
                y = cy + n * Math.Sin(angle);
            }
            else
            {
                // Exit straight: centerline is x=5, heading south
                // Position along exit: s - arcEnd
                var exitProgress = s - arcEnd;
                // Centerline: x=5, y=-5-exitProgress
                // n positive = right of centerline = positive x direction
                x = CenterlineRadius + n;
                y = -CenterlineRadius - exitProgress;
            }

            return (x, y);
        }
    }
}
