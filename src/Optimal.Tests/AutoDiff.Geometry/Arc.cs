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
    public sealed class Arc
    {
        public Point2D StartPoint { get; }
        public Point2D EndPoint { get; }
        public double Radius { get; }
        public Point2D Center { get; }
        public bool IsCounterClockwise { get; }
        public double SweepAngle { get; }
        public double ArcLength { get; }

        public Arc(Point2D startPoint, Point2D endPoint, double radius, bool isCounterClockwise = true)
        {
            if (radius <= 0)
            {
                throw new ArgumentException("Radius must be positive.", nameof(radius));
            }

            StartPoint = startPoint;
            EndPoint = endPoint;
            Radius = radius;
            IsCounterClockwise = isCounterClockwise;

            var chordLength = startPoint.DistanceTo(endPoint);
            var halfChord = chordLength / 2.0;

            if (chordLength > 2.0 * radius)
            {
                throw new ArgumentException(
                    $"Radius {radius} is too small for the given points (minimum: {halfChord}).",
                    nameof(radius));
            }

            if (chordLength == 0)
            {
                throw new ArgumentException("Start and end points must be different.", nameof(endPoint));
            }

            var midpoint = startPoint.Midpoint(endPoint);
            var distanceToCenter = Math.Sqrt(radius * radius - halfChord * halfChord);

            var dx = endPoint.X - startPoint.X;
            var dy = endPoint.Y - startPoint.Y;

            var perpX = -dy / chordLength;
            var perpY = dx / chordLength;

            if (isCounterClockwise)
            {
                Center = new Point2D(
                    midpoint.X + perpX * distanceToCenter,
                    midpoint.Y + perpY * distanceToCenter);
            }
            else
            {
                Center = new Point2D(
                    midpoint.X - perpX * distanceToCenter,
                    midpoint.Y - perpY * distanceToCenter);
            }

            var startAngle = Math.Atan2(startPoint.Y - Center.Y, startPoint.X - Center.X);
            var endAngle = Math.Atan2(endPoint.Y - Center.Y, endPoint.X - Center.X);

            SweepAngle = CalculateSweepAngle(startAngle, endAngle, isCounterClockwise);
            ArcLength = Math.Abs(SweepAngle) * radius;
        }

        private static double CalculateSweepAngle(double startAngle, double endAngle, bool isCounterClockwise)
        {
            var angle = endAngle - startAngle;

            if (isCounterClockwise)
            {
                while (angle < 0)
                {
                    angle += 2.0 * Math.PI;
                }
            }
            else
            {
                while (angle > 0)
                {
                    angle -= 2.0 * Math.PI;
                }
            }

            return angle;
        }

        public Point2D GetPointAtAngle(double angle)
        {
            return new Point2D(
                Center.X + Radius * Math.Cos(angle),
                Center.Y + Radius * Math.Sin(angle));
        }

        public Point2D GetPointAtParameter(double t)
        {
            if (t < 0.0 || t > 1.0)
            {
                throw new ArgumentOutOfRangeException(nameof(t), "Parameter must be between 0 and 1.");
            }

            var startAngle = Math.Atan2(StartPoint.Y - Center.Y, StartPoint.X - Center.X);
            var angle = startAngle + SweepAngle * t;

            return GetPointAtAngle(angle);
        }

        public override string ToString()
        {
            var direction = IsCounterClockwise ? "CCW" : "CW";
            return $"Arc[{StartPoint} -> {EndPoint}, R={Radius:F2}, Center={Center}, {direction}, Sweep={SweepAngle * 180 / Math.PI:F1}°]";
        }
    }
}
