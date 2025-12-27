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
    /// Simplified test without loop
    /// </summary>
    [OptimalCode]
    public static class SimpleGradientTest
    {
        public static double TestFunction(double x, double y)
        {
            // Simple function: compute arc angle and use it
            var chordLen = Math.Sqrt(x * x + y * y);
            var angle = Math.Atan2(y, x);
            var result = chordLen * angle;
            return result;
        }
    }

    /// <summary>
    /// Simplified Brachistochrone problem - calculates descent time for a bead sliding down a curve
    /// This version uses only primitive operations to enable automatic differentiation
    /// </summary>
    [OptimalCode]
    public static class SimpleBrachistochrone
    {
        private const double Gravity = 9.81;
        private const double Epsilon = 1e-10; // Small value to prevent division by zero in gradients

        /// <summary>
        /// Calculates the approximate time for a bead to slide from (0, startHeight) to (endX, 0)
        /// along a circular arc with the given radius.
        /// Reformulated to avoid gradient cancellation by using a more direct parameterization.
        /// </summary>
        /// <param name="radius">Radius of the circular arc path</param>
        /// <param name="startHeight">Starting height (Y coordinate)</param>
        /// <param name="endX">Ending horizontal position (X coordinate)</param>
        /// <param name="segments">Number of segments for numerical integration</param>
        /// <returns>Total descent time in seconds</returns>
        public static double GetDescentTime(double radius, double startHeight, double endX, int segments)
        {
            // NEW APPROACH: Use arc length parameterization directly
            // Compute total arc length first, then parameterize by arc length fraction
            // This avoids Atan2 and angle-based dependencies

            var chordLength = Math.Sqrt(endX * endX + startHeight * startHeight + Epsilon);

            // Perpendicular direction (normalized)
            var perpX = startHeight / chordLength;
            var perpY = endX / chordLength;

            // Distance from chord midpoint to arc center
            var halfChord = chordLength / 2.0;
            var sagitta = radius - Math.Sqrt(radius * radius - halfChord * halfChord + Epsilon);

            // Approximate arc length using circular segment formula
            // For small angles: arcLength ≈ chordLength * (1 + sagitta^2 / (6 * chordLength^2))
            var sagittaRatio = sagitta / chordLength;
            var arcLength = chordLength * (1.0 + sagittaRatio * sagittaRatio / 6.0);

            var totalTime = 0.0;

            for (var i = 0; i < segments; i++)
            {
                var s1 = i / (double)segments;
                var s2 = (i + 1) / (double)segments;

                // Linear interpolation along chord
                var x1_chord = endX * s1;
                var y1_chord = startHeight * (1.0 - s1);
                var x2_chord = endX * s2;
                var y2_chord = startHeight * (1.0 - s2);

                // Perpendicular offset (circular arc bulge)
                // For circular arc: offset = R - sqrt(R^2 - d^2) where d is distance from chord center
                // Distance along chord from center point
                var dist1_from_center = chordLength * Math.Abs(s1 - 0.5);
                var dist2_from_center = chordLength * Math.Abs(s2 - 0.5);

                // Circular arc offset perpendicular to chord
                var offset1 = radius - Math.Sqrt(radius * radius - dist1_from_center * dist1_from_center + Epsilon);
                var offset2 = radius - Math.Sqrt(radius * radius - dist2_from_center * dist2_from_center + Epsilon);

                // Apply offset perpendicular to chord
                var x1 = x1_chord + perpX * offset1;
                var y1 = y1_chord - perpY * offset1;
                var x2 = x2_chord + perpX * offset2;
                var y2 = y2_chord - perpY * offset2;

                // Height drop from start
                var h1 = startHeight - y1 + Epsilon;
                var h2 = startHeight - y2 + Epsilon;

                // Velocity from conservation of energy
                var v1 = Math.Sqrt(2.0 * Gravity * h1);
                var v2 = Math.Sqrt(2.0 * Gravity * h2);
                var avgVel = (v1 + v2) / 2.0;

                // Segment length
                var dx = x2 - x1;
                var dy = y2 - y1;
                var segLength = Math.Sqrt(dx * dx + dy * dy + Epsilon);

                // Time for segment
                var segTime = segLength / (avgVel + Epsilon);
                totalTime += segTime;
            }

            return totalTime;
        }
    }
}
