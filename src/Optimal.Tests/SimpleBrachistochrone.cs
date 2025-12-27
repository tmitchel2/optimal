/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal;

namespace Optimal.Tests
{
    /// <summary>
    /// Simplified Brachistochrone problem - calculates descent time for a bead sliding down a curve
    /// This version uses only primitive operations to enable automatic differentiation
    /// </summary>
    [OptimalCode]
    public static class SimpleBrachistochrone
    {
        private const double Gravity = 9.81;

        /// <summary>
        /// Calculates the approximate time for a bead to slide from (0, startHeight) to (endX, 0)
        /// along a circular arc with the given radius.
        /// </summary>
        /// <param name="radius">Radius of the circular arc path</param>
        /// <param name="startHeight">Starting height (Y coordinate)</param>
        /// <param name="endX">Ending horizontal position (X coordinate)</param>
        /// <param name="segments">Number of segments for numerical integration</param>
        /// <returns>Total descent time in seconds</returns>
        public static double GetDescentTime(double radius, double startHeight, double endX, int segments)
        {
            // Calculate arc center using basic geometry
            var chordLength = Math.Sqrt(endX * endX + startHeight * startHeight);
            var halfChord = chordLength / 2.0;
            var distanceToCenter = Math.Sqrt(radius * radius - halfChord * halfChord);

            // Center offset perpendicular to chord
            var perpX = startHeight / chordLength;
            var perpY = endX / chordLength;

            var centerX = endX / 2.0 + perpX * distanceToCenter;
            var centerY = startHeight / 2.0 - perpY * distanceToCenter;

            // Start and end angles
            var startAngle = Math.Atan2(startHeight - centerY, 0.0 - centerX);
            var endAngle = Math.Atan2(0.0 - centerY, endX - centerX);

            // Sweep angle (counter-clockwise)
            var sweepAngle = endAngle - startAngle;
            if (sweepAngle < 0.0)
            {
                sweepAngle += 2.0 * Math.PI;
            }

            var totalTime = 0.0;

            for (var i = 0; i < segments; i++)
            {
                var t1 = i / (double)segments;
                var t2 = (i + 1) / (double)segments;

                var angle1 = startAngle + sweepAngle * t1;
                var angle2 = startAngle + sweepAngle * t2;

                var x1 = centerX + radius * Math.Cos(angle1);
                var y1 = centerY + radius * Math.Sin(angle1);

                var x2 = centerX + radius * Math.Cos(angle2);
                var y2 = centerY + radius * Math.Sin(angle2);

                var heightDrop1 = startHeight - y1;
                var heightDrop2 = startHeight - y2;

                var startVelocity = heightDrop1 > 0.0 ? Math.Sqrt(2.0 * Gravity * heightDrop1) : 0.0;
                var endVelocity = heightDrop2 > 0.0 ? Math.Sqrt(2.0 * Gravity * heightDrop2) : 0.0;

                var velocity = (startVelocity + endVelocity) / 2.0;

                if (velocity > 0.0)
                {
                    var dx = x2 - x1;
                    var dy = y2 - y1;
                    var distance = Math.Sqrt(dx * dx + dy * dy);
                    var time = distance / velocity;
                    totalTime += time;
                }
            }

            return totalTime;
        }
    }
}
