/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Tests
{
    [TestClass]
    public sealed class Point2DTests
    {
        private const double Tolerance = 1e-10;

        [TestMethod]
        public void CanCreatePoint()
        {
            var point = new Point2D(3.0, 4.0);

            Assert.AreEqual(3.0, point.X);
            Assert.AreEqual(4.0, point.Y);
        }

        [TestMethod]
        public void CanCalculateDistance()
        {
            var p1 = new Point2D(0, 0);
            var p2 = new Point2D(3, 4);

            var distance = p1.DistanceTo(p2);

            Assert.AreEqual(5.0, distance, Tolerance);
        }

        [TestMethod]
        public void CanCalculateMidpoint()
        {
            var p1 = new Point2D(0, 0);
            var p2 = new Point2D(4, 6);

            var midpoint = p1.Midpoint(p2);

            Assert.AreEqual(2.0, midpoint.X, Tolerance);
            Assert.AreEqual(3.0, midpoint.Y, Tolerance);
        }

        [TestMethod]
        public void DistanceToSelfIsZero()
        {
            var point = new Point2D(5, 7);

            var distance = point.DistanceTo(point);

            Assert.AreEqual(0.0, distance, Tolerance);
        }

        [TestMethod]
        public void MidpointOfSamePointReturnsSamePoint()
        {
            var point = new Point2D(5, 7);

            var midpoint = point.Midpoint(point);

            Assert.AreEqual(point.X, midpoint.X, Tolerance);
            Assert.AreEqual(point.Y, midpoint.Y, Tolerance);
        }

        [TestMethod]
        public void ToStringReturnsFormattedCoordinates()
        {
            var point = new Point2D(3.5, 4.2);

            var str = point.ToString();

            Assert.IsTrue(str.Contains("3.5"));
            Assert.IsTrue(str.Contains("4.2"));
        }

        [TestMethod]
        public void PointsWithSameCoordinatesAreEqual()
        {
            var p1 = new Point2D(1.0, 2.0);
            var p2 = new Point2D(1.0, 2.0);

            Assert.AreEqual(p1, p2);
        }

        [TestMethod]
        public void PointsWithDifferentCoordinatesAreNotEqual()
        {
            var p1 = new Point2D(1.0, 2.0);
            var p2 = new Point2D(2.0, 1.0);

            Assert.AreNotEqual(p1, p2);
        }
    }
}
