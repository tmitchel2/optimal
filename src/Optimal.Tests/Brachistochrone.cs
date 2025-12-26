/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Geometry;

namespace Optimal.Tests
{
    [OptimalCode]
    public static class Brachistochrone
    {
        private const double Gravity = 9.81;

        public static TimeSpan GetDuration(double radius, int segments = 100)
        {
            var startPoint = new Point2D(0, 100);
            var endPoint = new Point2D(100, 0);

            var arc = new Arc(startPoint, endPoint, radius, isCounterClockwise: true);

            var totalTime = 0.0;
            var startHeight = startPoint.Y;

            for (var i = 0; i < segments; i++)
            {
                var t1 = i / (double)segments;
                var t2 = (i + 1) / (double)segments;

                var p1 = arc.GetPointAtParameter(t1);
                var p2 = arc.GetPointAtParameter(t2);

                var h1 = p1.Y;
                var h2 = p2.Y;

                if (h1 > startHeight || h2 > startHeight)
                {
                    throw new InvalidOperationException("Arc goes above starting height - invalid path for brachistochrone");
                }

                var heightDrop1 = startHeight - h1;
                var heightDrop2 = startHeight - h2;

                var startVelocity = heightDrop1 > 0 ? Math.Sqrt(2.0 * Gravity * heightDrop1) : 0.0;
                var endVelocity = heightDrop2 > 0 ? Math.Sqrt(2.0 * Gravity * heightDrop2) : 0.0;

                var velocity = (startVelocity + endVelocity) / 2.0;

                if (velocity == 0.0)
                {
                    continue;
                }

                var distance = p1.DistanceTo(p2);
                var time = distance / velocity;
                totalTime += time;
            }

            return TimeSpan.FromSeconds(totalTime);
        }
    }
}
