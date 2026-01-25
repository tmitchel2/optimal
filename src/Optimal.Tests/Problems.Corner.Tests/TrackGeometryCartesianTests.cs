/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using OptimalCli.Problems.Corner;

namespace Optimal.Problems.Corner.Tests
{
    /// <summary>
    /// Tests that verify cartesian position calculations for TrackGeometry.
    /// Each test builds on the previous one, adding one segment at a time
    /// to help isolate position errors in arc-to-cartesian conversions.
    ///
    /// Uses left-hand rule: heading 0 = east, heading +π/2 = south.
    /// </summary>
    [TestClass]
    public sealed class TrackGeometryCartesianTests
    {
        private const double Tolerance = 1e-10;

        #region Test 1: Starting Point Only

        [TestMethod]
        public void StartingPointHasCorrectPosition()
        {
            // Just verify the builder starts at the correct position
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 0.001) // Need at least one segment to build
                .Build(1);

            // The first segment should start at (0, 0)
            Assert.AreEqual(0.0, track[0].StartPosition.X, Tolerance, "Start X");
            Assert.AreEqual(0.0, track[0].StartPosition.Y, Tolerance, "Start Y");
            Assert.AreEqual(0.0, track[0].StartHeading, Tolerance, "Start heading");
        }

        #endregion

        #region Test 2: Single Line Segment

        [TestMethod]
        public void SingleLineHasCorrectEndPosition()
        {
            // Start at (0, 0) heading east (0), add 10m line
            // Expected end: (10, 0) - moving east
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .Build(1);

            var segment = track[0];

            // Verify start
            Assert.AreEqual(0.0, segment.StartPosition.X, Tolerance, "Start X");
            Assert.AreEqual(0.0, segment.StartPosition.Y, Tolerance, "Start Y");

            // Verify end - heading 0 means direction (cos(0), -sin(0)) = (1, 0)
            Assert.AreEqual(10.0, segment.EndPosition.X, Tolerance, "End X");
            Assert.AreEqual(0.0, segment.EndPosition.Y, Tolerance, "End Y");

            // Heading unchanged for straight line
            Assert.AreEqual(0.0, segment.EndHeading, Tolerance, "End heading");
        }

        #endregion

        #region Test 3: Line + Arc

        [TestMethod]
        public void LineAndArcHaveCorrectEndPositions()
        {
            // Start at (0, 0), heading east
            // Add 10m line: ends at (10, 0)
            // Add 90° right arc (radius 10):
            //   - Center is to the right (south): (10, -10)
            //   - End heading: 0 + π/2 = π/2
            //   - End position: center + radius * (sin(π/2), cos(π/2)) = (10, -10) + (10, 0) = (20, -10)
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .Build(1);

            // Line segment (segment 0)
            var line = track[0];
            Assert.AreEqual(10.0, line.EndPosition.X, Tolerance, "Line end X");
            Assert.AreEqual(0.0, line.EndPosition.Y, Tolerance, "Line end Y");

            // Arc segment (segment 1)
            var arc = (ArcSegment)track[1];

            // Arc should start where line ends
            Assert.AreEqual(10.0, arc.StartPosition.X, Tolerance, "Arc start X");
            Assert.AreEqual(0.0, arc.StartPosition.Y, Tolerance, "Arc start Y");

            // Arc center: right of path at start = south of (10, 0) = (10, -10)
            Assert.AreEqual(10.0, arc.Center.X, Tolerance, "Arc center X");
            Assert.AreEqual(-10.0, arc.Center.Y, Tolerance, "Arc center Y");

            // Arc end position: (20, -10)
            Assert.AreEqual(20.0, arc.EndPosition.X, Tolerance, "Arc end X");
            Assert.AreEqual(-10.0, arc.EndPosition.Y, Tolerance, "Arc end Y");

            // Arc end heading: π/2 (south)
            Assert.AreEqual(Math.PI / 2, arc.EndHeading, Tolerance, "Arc end heading");
        }

        #endregion

        #region Test 4: Line + Arc + Line

        [TestMethod]
        public void LineArcLineHaveCorrectEndPositions()
        {
            // After first line + arc: position (20, -10), heading π/2 (south)
            // Add 10m line heading south: direction (cos(π/2), -sin(π/2)) = (0, -1)
            // End position: (20, -10) + 10 * (0, -1) = (20, -20)
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .Build(1);

            // Second line (segment 2)
            var line2 = track[2];

            // Should start where arc ends
            Assert.AreEqual(20.0, line2.StartPosition.X, Tolerance, "Line2 start X");
            Assert.AreEqual(-10.0, line2.StartPosition.Y, Tolerance, "Line2 start Y");
            Assert.AreEqual(Math.PI / 2, line2.StartHeading, Tolerance, "Line2 start heading");

            // End position: (20, -20)
            Assert.AreEqual(20.0, line2.EndPosition.X, Tolerance, "Line2 end X");
            Assert.AreEqual(-20.0, line2.EndPosition.Y, Tolerance, "Line2 end Y");

            // Heading unchanged
            Assert.AreEqual(Math.PI / 2, line2.EndHeading, Tolerance, "Line2 end heading");
        }

        #endregion

        #region Test 5: Line + Arc + Line + Arc

        [TestMethod]
        public void LineArcLineArcHaveCorrectEndPositions()
        {
            // After L-A-L: position (20, -20), heading π/2 (south)
            // Add 90° right arc (radius 10):
            //   - Right of south-heading path = west
            //   - Right direction at heading π/2: (-sin(π/2), -cos(π/2)) = (-1, 0)
            //   - Center: (20, -20) + 10 * (-1, 0) = (10, -20)
            //   - End heading: π/2 + π/2 = π (west)
            //   - End position: center + radius * (sin(π), cos(π)) = (10, -20) + (0, -10) = (10, -30)
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .Build(1);

            // Second arc (segment 3)
            var arc2 = (ArcSegment)track[3];

            // Should start where second line ends
            Assert.AreEqual(20.0, arc2.StartPosition.X, Tolerance, "Arc2 start X");
            Assert.AreEqual(-20.0, arc2.StartPosition.Y, Tolerance, "Arc2 start Y");
            Assert.AreEqual(Math.PI / 2, arc2.StartHeading, Tolerance, "Arc2 start heading");

            // Arc center: (10, -20) - WEST of start position
            Assert.AreEqual(10.0, arc2.Center.X, Tolerance, "Arc2 center X");
            Assert.AreEqual(-20.0, arc2.Center.Y, Tolerance, "Arc2 center Y");

            // Arc end position: (10, -30)
            Assert.AreEqual(10.0, arc2.EndPosition.X, Tolerance, "Arc2 end X");
            Assert.AreEqual(-30.0, arc2.EndPosition.Y, Tolerance, "Arc2 end Y");

            // Arc end heading: π (west)
            Assert.AreEqual(Math.PI, arc2.EndHeading, Tolerance, "Arc2 end heading");
        }

        #endregion

        #region Test 6: Full Track (Line + Arc + Line + Arc + Line)

        [TestMethod]
        public void FullTrackHasCorrectEndPositions()
        {
            // After L-A-L-A: position (10, -30), heading π (west)
            // Add 10m line heading west: direction (cos(π), -sin(π)) = (-1, 0)
            // End position: (10, -30) + 10 * (-1, 0) = (0, -30)
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .Build(1);

            Assert.AreEqual(5, track.SegmentCount, "Segment count");

            // Third line (segment 4)
            var line3 = track[4];

            // Should start where second arc ends
            Assert.AreEqual(10.0, line3.StartPosition.X, Tolerance, "Line3 start X");
            Assert.AreEqual(-30.0, line3.StartPosition.Y, Tolerance, "Line3 start Y");
            Assert.AreEqual(Math.PI, line3.StartHeading, Tolerance, "Line3 start heading");

            // End position: (0, -30)
            Assert.AreEqual(0.0, line3.EndPosition.X, Tolerance, "Line3 end X");
            Assert.AreEqual(-30.0, line3.EndPosition.Y, Tolerance, "Line3 end Y");

            // Heading unchanged
            Assert.AreEqual(Math.PI, line3.EndHeading, Tolerance, "Line3 end heading");
        }

        #endregion

        #region ToCartesian Tests - Verifying curvilinear to cartesian conversion

        [TestMethod]
        public void ToCartesianAtLineStartReturnsStartPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .Build(1);

            var pos = track.CurvilinearToCartesian(s: 0, n: 0);
            Assert.AreEqual(0.0, pos.X, Tolerance, "X at s=0");
            Assert.AreEqual(0.0, pos.Y, Tolerance, "Y at s=0");
        }

        [TestMethod]
        public void ToCartesianAtLineMidpointReturnsCorrectPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .Build(1);

            var pos = track.CurvilinearToCartesian(s: 5, n: 0);
            Assert.AreEqual(5.0, pos.X, Tolerance, "X at s=5");
            Assert.AreEqual(0.0, pos.Y, Tolerance, "Y at s=5");
        }

        [TestMethod]
        public void ToCartesianAtLineEndReturnsEndPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .Build(1);

            var pos = track.CurvilinearToCartesian(s: 10, n: 0);
            Assert.AreEqual(10.0, pos.X, Tolerance, "X at s=10");
            Assert.AreEqual(0.0, pos.Y, Tolerance, "Y at s=10");
        }

        [TestMethod]
        public void ToCartesianWithLateralOffsetReturnsOffsetPosition()
        {
            // Line heading east (0), n positive = right of centerline = south
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .Build(1);

            // At s=5, n=2 (2m to the right/south)
            var pos = track.CurvilinearToCartesian(s: 5, n: 2);
            Assert.AreEqual(5.0, pos.X, Tolerance, "X at s=5, n=2");
            Assert.AreEqual(-2.0, pos.Y, Tolerance, "Y at s=5, n=2 (south is negative Y)");
        }

        [TestMethod]
        public void ToCartesianAtArcMidpointReturnsCorrectPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .Build(1);

            // Arc starts at s=10, has length = 10 * π/2 ≈ 15.708
            // Midpoint of arc: s = 10 + (π/2 * 10) / 2 = 10 + 5π/2 ≈ 17.854
            // At midpoint, heading = 0 + π/4 = π/4
            // Position on arc: center + radius * (sin(heading), cos(heading))
            // Center = (10, -10)
            // Position = (10, -10) + 10 * (sin(π/4), cos(π/4))
            //          = (10, -10) + 10 * (√2/2, √2/2)
            //          = (10 + 5√2, -10 + 5√2)
            var arcMidS = 10.0 + (Math.PI / 4) * 10.0;
            var pos = track.CurvilinearToCartesian(s: arcMidS, n: 0);

            var expectedX = 10.0 + 10.0 * Math.Sin(Math.PI / 4);
            var expectedY = -10.0 + 10.0 * Math.Cos(Math.PI / 4);

            Assert.AreEqual(expectedX, pos.X, Tolerance, "X at arc midpoint");
            Assert.AreEqual(expectedY, pos.Y, Tolerance, "Y at arc midpoint");
        }

        [TestMethod]
        public void ToCartesianAtArcEndReturnsEndPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .Build(1);

            // Arc ends at s = 10 + π/2 * 10 = 10 + 5π
            var arcEndS = 10.0 + (Math.PI / 2) * 10.0;
            var pos = track.CurvilinearToCartesian(s: arcEndS, n: 0);

            Assert.AreEqual(20.0, pos.X, Tolerance, "X at arc end");
            Assert.AreEqual(-10.0, pos.Y, Tolerance, "Y at arc end");
        }

        [TestMethod]
        public void ToCartesianInSecondLineReturnsCorrectPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .Build(1);

            // Second line starts at (20, -10), heading south (π/2)
            // Arc ends at s = 10 + 5π
            // At s = 10 + 5π + 5 (5m into second line)
            var secondLineMidS = 10.0 + (Math.PI / 2) * 10.0 + 5.0;
            var pos = track.CurvilinearToCartesian(s: secondLineMidS, n: 0);

            Assert.AreEqual(20.0, pos.X, Tolerance, "X in second line");
            Assert.AreEqual(-15.0, pos.Y, Tolerance, "Y in second line (5m south of -10)");
        }

        [TestMethod]
        public void ToCartesianInSecondArcReturnsCorrectPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)   // 10,0
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true) // 20,-10
                .AddLine(distance: 10.0) // 20,-20
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true) // 10,-30
                .Build(1);

            // Second arc starts at (20, -20), heading south (π/2)
            // Arc center at (10, -20) - WEST of start
            // At arc end: (10, -30)
            var secondArcEndS = 10.0 + (Math.PI / 2) * 10.0 + 10.0 + (Math.PI / 2) * 10.0;
            var pos = track.CurvilinearToCartesian(s: secondArcEndS, n: 0);

            Assert.AreEqual(10.0, pos.X, Tolerance, "X at second arc end");
            Assert.AreEqual(-30.0, pos.Y, Tolerance, "Y at second arc end");
        }

        [TestMethod]
        public void ToCartesianInSecondArcReturnsCorrectPositionWithNegativeOffset()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)   // 10,0
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true) // 20,-10
                .AddLine(distance: 10.0) // 20,-20
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true) // 10,-30
                .Build(1);

            // Second arc starts at (20, -20), heading south (π/2)
            // Arc center at (10, -20) - WEST of start
            // At arc end: (10, -30)
            var secondArcEndS = 10.0 + (Math.PI / 2) * 10.0 + 10.0 + (Math.PI / 2) * 10.0;
            var pos = track.CurvilinearToCartesian(s: secondArcEndS, n: -10);

            Assert.AreEqual(10.0, pos.X, Tolerance, "X at second arc end");
            Assert.AreEqual(-40.0, pos.Y, Tolerance, "Y at second arc end");
        }

        [TestMethod]
        public void ToCartesianInSecondArcReturnsCorrectPositionWithPositiveOffset()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)   // 10,0
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true) // 20,-10
                .AddLine(distance: 10.0) // 20,-20
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true) // 10,-30
                .Build(1);

            // Second arc starts at (20, -20), heading south (π/2)
            // Arc center at (10, -20) - WEST of start
            // At arc end: (10, -30)
            var secondArcEndS = 10.0 + (Math.PI / 2) * 10.0 + 10.0 + (Math.PI / 2) * 10.0;
            var pos = track.CurvilinearToCartesian(s: secondArcEndS, n: 10);

            Assert.AreEqual(10.0, pos.X, Tolerance, "X at second arc end");
            Assert.AreEqual(-20.0, pos.Y, Tolerance, "Y at second arc end");
        }

        [TestMethod]
        public void ToCartesianAtFinalLineEndReturnsCorrectPosition()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .Build(1);

            // End of full track: (0, -30)
            var pos = track.CurvilinearToCartesian(s: track.TotalLength, n: 0);

            Assert.AreEqual(0.0, pos.X, Tolerance, "X at track end");
            Assert.AreEqual(-30.0, pos.Y, Tolerance, "Y at track end");
        }

        #endregion

        #region Total Length Verification

        [TestMethod]
        public void FullTrackHasCorrectTotalLength()
        {
            var track = TrackGeometry
                .StartAt(x: 0, y: 0, heading: 0)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .AddArc(radius: 10.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 10.0)
                .Build(1);

            // Total = 10 + π*10/2 + 10 + π*10/2 + 10 = 30 + 10π
            var expected = 30.0 + 10.0 * Math.PI;
            Assert.AreEqual(expected, track.TotalLength, Tolerance, "Total length");
        }

        #endregion
    }
}
