/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Control.Collocation.Tests
{
    [TestClass]
    public sealed class CollocationGridTests
    {
        [TestMethod]
        public void CanCreateUniformGrid()
        {
            var grid = new CollocationGrid(t0: 0.0, tf: 10.0, segments: 10);

            Assert.AreEqual(0.0, grid.InitialTime);
            Assert.AreEqual(10.0, grid.FinalTime);
            Assert.AreEqual(10, grid.Segments);
            Assert.AreEqual(1.0, grid.TimeStep);
            Assert.HasCount(11, grid.TimePoints);
        }

        [TestMethod]
        public void TimePointsAreCorrect()
        {
            var grid = new CollocationGrid(t0: 0.0, tf: 5.0, segments: 5);

            Assert.AreEqual(0.0, grid.TimePoints[0]);
            Assert.AreEqual(1.0, grid.TimePoints[1]);
            Assert.AreEqual(2.0, grid.TimePoints[2]);
            Assert.AreEqual(3.0, grid.TimePoints[3]);
            Assert.AreEqual(4.0, grid.TimePoints[4]);
            Assert.AreEqual(5.0, grid.TimePoints[5]);
        }

        [TestMethod]
        public void CanGetMidpoint()
        {
            var grid = new CollocationGrid(t0: 0.0, tf: 10.0, segments: 10);

            Assert.AreEqual(0.5, grid.GetMidpoint(0));
            Assert.AreEqual(1.5, grid.GetMidpoint(1));
            Assert.AreEqual(9.5, grid.GetMidpoint(9));
        }

        [TestMethod]
        public void CanGetTimeStep()
        {
            var grid = new CollocationGrid(t0: 0.0, tf: 10.0, segments: 10);

            for (var i = 0; i < 10; i++)
            {
                Assert.AreEqual(1.0, grid.GetTimeStep(i));
            }
        }

        [TestMethod]
        public void ThrowsOnInvalidSegmentCount()
        {
            Assert.Throws<ArgumentException>(() => new CollocationGrid(0.0, 10.0, 0));
            Assert.Throws<ArgumentException>(() => new CollocationGrid(0.0, 10.0, -1));
        }

        [TestMethod]
        public void ThrowsOnInvalidTimeRange()
        {
            Assert.Throws<ArgumentException>(() => new CollocationGrid(10.0, 10.0, 10));
            Assert.Throws<ArgumentException>(() => new CollocationGrid(10.0, 5.0, 10));
        }

        [TestMethod]
        public void ThrowsOnInvalidMidpointIndex()
        {
            var grid = new CollocationGrid(t0: 0.0, tf: 10.0, segments: 10);

            Assert.Throws<ArgumentOutOfRangeException>(() => grid.GetMidpoint(-1));
            Assert.Throws<ArgumentOutOfRangeException>(() => grid.GetMidpoint(10));
        }
    }
}
