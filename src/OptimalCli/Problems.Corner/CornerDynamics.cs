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
    /// Coordinate conventions (LEFT-HAND RULE):
    /// - s: Distance along centerline (0 at start, increases monotonically)
    /// - n: Perpendicular distance from centerline (positive = right of centerline when facing forward)
    /// - θ: Heading angle where positive = clockwise rotation
    ///   - θ = 0: heading east
    ///   - θ = +π/2: heading south
    ///   - θ = -π/2: heading north
    ///
    /// Centerline path (90° right turn):
    /// - Entry straight: s ∈ [0, EntryLength), θ_road = 0
    /// - Quarter-circle arc: s ∈ [EntryLength, EntryLength + ArcLength), θ_road increases from 0 to +π/2
    /// - Exit straight: s ≥ EntryLength + ArcLength, θ_road = +π/2
    ///
    /// The corner problem finds the optimal racing line through a 90° right turn
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
        /// Road heading angle at position s along centerline (left-hand rule: positive = clockwise).
        /// Entry: θ_road = 0 (heading east)
        /// Arc: θ_road = +arcProgress × π/2 (turning right/clockwise)
        /// Exit: θ_road = +π/2 (heading south)
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
                return arcProgress * Math.PI / 2.0;  // Positive for clockwise (right) turn
            }
            else
            {
                return Math.PI / 2.0;
            }
        }

        /// <summary>
        /// Road curvature κ at position s along centerline.
        /// Straight sections: κ = 0
        /// Arc section: κ = +1/R (positive for right turn with left-hand rule)
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
                thetaRoad = arcProgress * Math.PI / 2.0;  // Positive for right turn
                curvature = 1.0 / CenterlineRadius;
            }
            else
            {
                thetaRoad = Math.PI / 2.0;
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
        /// With left-hand rule (positive θ = clockwise, positive n = right of centerline):
        /// - When heading more right than road (θ > θ_road), headingError > 0, ṅ > 0, moving right ✓
        /// - When heading more left than road (θ &lt; θ_road), headingError &lt; 0, ṅ &lt; 0, moving left ✓
        /// </summary>
        public static double LateralRate(double s, double n, double theta, double v)
        {
            _ = n; // Parameter required for autodiff signature but not used in formula
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
                thetaRoad = arcProgress * Math.PI / 2.0;  // Positive for right turn
            }
            else
            {
                thetaRoad = Math.PI / 2.0;
            }

            var headingError = theta - thetaRoad;
            return v * Math.Sin(headingError);  // Positive sign for left-hand rule
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
        ///
        /// Cartesian coordinate system:
        /// - x increases to the right (east)
        /// - y increases upward (north)
        /// - The track goes east, then turns right (south)
        /// </summary>
        public static (double x, double y) CurvilinearToCartesian(double s, double n)
        {
            var arcLength = CornerDynamics.ArcLength;
            var entryEnd = CornerDynamics.EntryLength;
            var arcEnd = entryEnd + arcLength;

            double x, y;

            if (s < entryEnd)
            {
                // Entry straight: centerline is y=0, heading east (θ_road = 0)
                // x = s - EntryLength (so at s=0, x=-15; at s=EntryLength, x=0)
                // Positive n = right of centerline = negative y (south)
                x = s - CornerDynamics.EntryLength;
                y = -n;
            }
            else if (s < arcEnd)
            {
                // Arc: centerline is quarter circle from (0,0) to (5,-5)
                // The centerline arc starts at (0, 0) heading east, curves right (clockwise) 90°.
                // Arc is centered at (0, -5) with radius 5.
                // Start point (0, 0): geometric angle = π/2 (from center, pointing up)
                // End point (5, -5): geometric angle = 0 (from center, pointing right)

                var arcProgress = (s - entryEnd) / arcLength;
                var angle = Math.PI / 2.0 * (1.0 - arcProgress);  // Geometric angle from arc center

                // Centerline position (radius = CenterlineRadius from arc center at (0, -5))
                var cx = CornerDynamics.CenterlineRadius * Math.Cos(angle);
                var cy = -CornerDynamics.CenterlineRadius + CornerDynamics.CenterlineRadius * Math.Sin(angle);

                // n offset: perpendicular to path, positive n = right of centerline
                // The perpendicular direction (pointing right) is (-cos(angle), -sin(angle))
                // At arc start (angle=π/2): perpendicular = (0, -1) → right is south ✓
                // At arc end (angle=0): perpendicular = (-1, 0) → right is west ✓
                x = cx - n * Math.Cos(angle);
                y = cy - n * Math.Sin(angle);
            }
            else
            {
                // Exit straight: centerline is x=5, heading south (θ_road = +π/2)
                var exitProgress = s - arcEnd;
                // Centerline: x=5, y=-5-exitProgress
                // When heading south, right of centerline = west = negative x direction
                // So positive n decreases x
                x = CornerDynamics.CenterlineRadius - n;
                y = -CornerDynamics.CenterlineRadius - exitProgress;
            }

            return (x, y);
        }
    }
}
