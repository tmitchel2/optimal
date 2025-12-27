/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Geometry.Tests
{
    public readonly record struct Point2D(double X, double Y)
    {
        public double DistanceTo(Point2D other)
        {
            var dx = other.X - X;
            var dy = other.Y - Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        public Point2D Midpoint(Point2D other)
        {
            return new Point2D((X + other.X) / 2.0, (Y + other.Y) / 2.0);
        }

        public override string ToString()
        {
            return $"({X}, {Y})";
        }
    }
}
