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
    }

    /// <summary>
    /// Helper methods for coordinate conversions and visualization.
    /// Not marked with [OptimalCode] since these don't need gradients.
    /// </summary>
    public static class CornerDynamicsHelpers
    {
        /// <summary>
        /// Convert curvilinear coordinates (s, n) to Cartesian coordinates (x, y).
        /// Used for visualization.
        /// </summary>
        public static (double x, double y) CurvilinearToCartesian(double s, double n)
        {
            var arcLength = CornerDynamics.ArcLength;
            var entryEnd = CornerDynamics.EntryLength;
            var arcEnd = entryEnd + arcLength;

            double x, y;

            if (s < entryEnd)
            {
                // Entry straight: centerline is y=0, heading east
                // x = s - EntryLength (so at s=0, x=-15; at s=EntryLength, x=0)
                // n positive = right of centerline = negative y
                x = s - CornerDynamics.EntryLength;
                y = -n;  // Flipped: positive n = right of centerline = negative y
            }
            else if (s < arcEnd)
            {
                // Arc: centerline is quarter circle from (0,0) to (5,-5)
                // The centerline arc starts at (0, 0) heading east, curves right 90°.
                // Arc is centered at (0, -5) with radius 5.
                // Start point (0, 0): angle = π/2 (from center, going up)
                // End point (5, -5): angle = 0 (from center, going right)
                
                var arcProgress = (s - entryEnd) / arcLength;
                var angle = Math.PI / 2.0 * (1.0 - arcProgress);
                
                // Centerline position (radius = CenterlineRadius from arc center)
                var cx = CornerDynamics.CenterlineRadius * Math.Cos(angle);
                var cy = -CornerDynamics.CenterlineRadius + CornerDynamics.CenterlineRadius * Math.Sin(angle);
                
                // n offset: perpendicular to arc
                // For a right turn, positive n = right of centerline = OUTWARD from arc center
                // The outward radial direction from center (0, -5) is (cos(angle), sin(angle))
                // So positive n should SUBTRACT from this (move toward center = inner = left? NO)
                // Wait: positive n = right of centerline when facing forward
                // When facing east at arc start (angle=π/2), right is DOWN (negative y)
                // The radial direction at angle=π/2 is (0, 1) pointing UP
                // So positive n should be in the OPPOSITE direction: -sin, -cos? No...
                // 
                // Actually, the perpendicular to the path (not radial):
                // Road heading at this point is θ_road = -arcProgress * π/2
                // Right-hand perpendicular to heading: (sin(-θ_road), -cos(-θ_road)) = (-sin(θ_road), -cos(θ_road))
                // At arc start: θ_road = 0, perpendicular = (0, -1) → positive n goes DOWN (y negative) ✓
                // At arc end: θ_road = -π/2, perpendicular = (sin(π/2), -cos(π/2)) = (1, 0) → positive n goes RIGHT ✓
                //
                // So the correct offset is: n * (sin(-θ_road), -cos(-θ_road)) = n * (-sin(θ_road), -cos(θ_road))
                // θ_road = -arcProgress * π/2 = -(1 - angle/(π/2)) * π/2 = angle - π/2
                // sin(θ_road) = sin(angle - π/2) = -cos(angle)
                // cos(θ_road) = cos(angle - π/2) = sin(angle)
                // perpendicular = (-cos(angle), -sin(angle))
                x = cx - n * Math.Cos(angle);
                y = cy - n * Math.Sin(angle);
            }
            else
            {
                // Exit straight: centerline is x=5, heading south (θ = -π/2)
                var exitProgress = s - arcEnd;
                // Centerline: x=5, y=-5-exitProgress
                // When heading south, right of centerline = WEST = negative x direction
                // So positive n should DECREASE x
                x = CornerDynamics.CenterlineRadius - n;
                y = -CornerDynamics.CenterlineRadius - exitProgress;
            }

            return (x, y);
        }
    }
}
