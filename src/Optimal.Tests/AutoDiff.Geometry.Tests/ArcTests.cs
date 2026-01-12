/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Geometry.Tests
{
    [TestClass]
    public sealed class ArcTests
    {
        private const double Tolerance = 1e-10;

        [TestMethod]
        public void CanCreateBasicArc()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            Assert.IsNotNull(arc);
            Assert.AreEqual(start, arc.StartPoint);
            Assert.AreEqual(end, arc.EndPoint);
            Assert.AreEqual(radius, arc.Radius);
            Assert.IsTrue(arc.IsCounterClockwise);
        }

        [TestMethod]
        public void CanCalculateCounterClockwiseArcCenter()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            Assert.AreEqual(1.0, arc.Center.X, Tolerance);
            Assert.AreEqual(0.0, arc.Center.Y, Tolerance);
        }

        [TestMethod]
        public void CanCalculateClockwiseArcCenter()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: false);

            Assert.AreEqual(1.0, arc.Center.X, Tolerance);
            Assert.AreEqual(0.0, arc.Center.Y, Tolerance);
        }

        [TestMethod]
        public void CanCalculateSemicircleProperties()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            var expectedSweepAngle = Math.PI;
            var expectedArcLength = Math.PI * radius;

            Assert.AreEqual(expectedSweepAngle, arc.SweepAngle, Tolerance);
            Assert.AreEqual(expectedArcLength, arc.ArcLength, Tolerance);
        }

        [TestMethod]
        public void CanCalculateQuarterCircleProperties()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(1, 1);
            var radius = Math.Sqrt(2);

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            var expectedSweepAngle = Math.PI / 3.0;
            var expectedArcLength = expectedSweepAngle * radius;

            Assert.AreEqual(expectedSweepAngle, arc.SweepAngle, Tolerance);
            Assert.AreEqual(expectedArcLength, arc.ArcLength, Tolerance);
        }

        [TestMethod]
        public void CounterClockwiseAndClockwiseArcsHaveDifferentSweepAngles()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 2.0;

            var ccwArc = new Arc(start, end, radius, isCounterClockwise: true);
            var cwArc = new Arc(start, end, radius, isCounterClockwise: false);

            Assert.IsTrue(ccwArc.SweepAngle > 0);
            Assert.IsTrue(cwArc.SweepAngle < 0);
            Assert.AreNotEqual(ccwArc.Center, cwArc.Center);
            Assert.AreEqual(Math.Abs(ccwArc.SweepAngle), Math.Abs(cwArc.SweepAngle), Tolerance);
        }

        [TestMethod]
        public void CanGetPointAtParameter()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: false);

            var midPoint = arc.GetPointAtParameter(0.5);

            Assert.AreEqual(1.0, midPoint.X, Tolerance);
            Assert.AreEqual(1.0, midPoint.Y, Tolerance);
        }

        [TestMethod]
        public void GetPointAtParameterReturnsStartPointAtZero()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            var point = arc.GetPointAtParameter(0.0);

            Assert.AreEqual(start.X, point.X, Tolerance);
            Assert.AreEqual(start.Y, point.Y, Tolerance);
        }

        [TestMethod]
        public void GetPointAtParameterReturnsEndPointAtOne()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            var point = arc.GetPointAtParameter(1.0);

            Assert.AreEqual(end.X, point.X, Tolerance);
            Assert.AreEqual(end.Y, point.Y, Tolerance);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void GetPointAtParameterThrowsForNegativeParameter()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            arc.GetPointAtParameter(-0.1);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentOutOfRangeException))]
        public void GetPointAtParameterThrowsForParameterGreaterThanOne()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            arc.GetPointAtParameter(1.1);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentException))]
        public void ThrowsWhenRadiusIsNegative()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);

            _ = new Arc(start, end, -1.0);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentException))]
        public void ThrowsWhenRadiusIsZero()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);

            _ = new Arc(start, end, 0.0);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentException))]
        public void ThrowsWhenRadiusIsTooSmall()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(10, 0);

            _ = new Arc(start, end, 1.0);
        }

        [TestMethod]
        [ExpectedException(typeof(ArgumentException))]
        public void ThrowsWhenPointsAreIdentical()
        {
            var point = new Point2D(1, 1);

            _ = new Arc(point, point, 1.0);
        }

        [TestMethod]
        public void ToStringReturnsFormattedString()
        {
            var start = new Point2D(0, 0);
            var end = new Point2D(2, 0);
            var radius = 1.0;

            var arc = new Arc(start, end, radius, isCounterClockwise: true);

            var str = arc.ToString();

            Assert.IsNotNull(str);
            Assert.IsTrue(str.Contains("Arc"));
            Assert.IsTrue(str.Contains("CCW"));
        }
    }
}
