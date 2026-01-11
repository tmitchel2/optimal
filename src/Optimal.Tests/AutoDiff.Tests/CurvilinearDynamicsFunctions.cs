/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// Test functions for curvilinear dynamics that use Math.PI and conditional logic
    /// based on computed boundaries. This tests the AutoDiff source generator's ability
    /// to handle:
    /// - Math.PI constant
    /// - Conditionals with computed thresholds
    /// - Complex trigonometric expressions
    /// - Variables declared before conditionals and assigned in branches
    /// </summary>
    [OptimalCode]
    public static class CurvilinearDynamicsFunctions
    {
        // Constants that can be evaluated at compile time
        public const double EntryLength = 15.0;
        public const double CenterlineRadius = 5.0;

        /// <summary>
        /// Arc length of a quarter circle: π × R / 2
        /// Tests use of Math.PI in a simple expression.
        /// </summary>
        public static double QuarterCircleArcLength(double radius)
        {
            return Math.PI * radius / 2.0;
        }

        /// <summary>
        /// Simpler function: uses Math.PI in a trigonometric context.
        /// Returns sin(x + π/4)
        /// </summary>
        public static double SineWithPiOffset(double x)
        {
            return Math.Sin(x + Math.PI / 4.0);
        }

        /// <summary>
        /// Function using Math.PI as a threshold for conditionals.
        /// </summary>
        public static double AngleWrap(double theta)
        {
            if (theta > Math.PI)
            {
                return theta - 2.0 * Math.PI;
            }
            else if (theta < -Math.PI)
            {
                return theta + 2.0 * Math.PI;
            }
            else
            {
                return theta;
            }
        }

        /// <summary>
        /// Road heading angle based on progress s along a path.
        /// Tests conditionals with Math.PI in the result computation.
        /// Uses direct returns to avoid intermediate variable scoping issues.
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
        /// Curvature of the road at progress s.
        /// Uses direct returns to avoid intermediate variable scoping issues.
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
        /// Progress rate along centerline in curvilinear coordinates.
        /// This function tests the fix for variables declared before if-else
        /// and assigned inside branches.
        /// 
        /// ṡ = v × cos(θ - θ_road) / (1 - n × κ)
        /// </summary>
        public static double ProgressRateInline(double s, double n, double theta, double v)
        {
            var arcLength = Math.PI * CenterlineRadius / 2.0;
            var entryEnd = EntryLength;
            var arcEnd = EntryLength + arcLength;

            // Variables assigned inside conditionals and used after
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

            // Prevent division by zero
            if (denominator < 0.1)
            {
                denominator = 0.1;
            }

            return v * Math.Cos(headingError) / denominator;
        }

        /// <summary>
        /// Lateral deviation rate in curvilinear coordinates.
        /// Tests variables assigned in conditionals.
        /// 
        /// ṅ = v × sin(θ - θ_road)
        /// </summary>
        public static double LateralRateInline(double s, double n, double theta, double v)
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
    }
}
