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
    [TestClass]
    public sealed class CornerDynamicsTests
    {
        private const double Tolerance = 1e-10;
        private const double GradientTolerance = 1e-5;
        private const double Epsilon = 1e-7;
        private readonly TrackGeometry _trackGeometry;

        public CornerDynamicsTests()
        {
            // Initialize track geometry before each test
            // Entry straight (15m) -> 90 deg right turn (radius 5m) -> Exit straight (20m)
            _trackGeometry = TrackGeometry.StartAt(x: -15, y: 0, heading: 0)
                .AddLine(distance: 15.0)
                .AddArc(radius: 5.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 20.0)
                .Build(1);
        }

        #region Road Geometry Tests

        [TestMethod]
        public void RoadHeadingInEntryRegionIsZero()
        {
            // In entry region (s < 15), road heading should be 0 (east)
            var s = 10.0;
            var heading = _trackGeometry.RoadHeading(s);
            Assert.AreEqual(0.0, heading, Tolerance);
        }

        #endregion

        #region Time Rate Tests (Arc-Length Parameterization)

        [TestMethod]
        public void TimeRateIsInverseVelocityOnStraight()
        {
            // On straight with alpha = 0, dt/ds = 1/V
            var kappa = 0.0;
            var n = 0.0;
            var alpha = 0.0;
            var vValue = 15.0;

            var timeRate = CornerDynamics.TimeRate(kappa, n, alpha, vValue);
            Assert.AreEqual(1.0 / vValue, timeRate, Tolerance, "dt/ds should equal 1/V when aligned on straight");
        }

        [TestMethod]
        public void TimeRateIncreasesWithHeadingError()
        {
            // With heading error, dt/ds = 1 / (V * cos(alpha)) > 1/V
            var kappa = 0.0;
            var n = 0.0;
            var alpha = 0.3; // 0.3 rad heading error
            var vValue = 15.0;

            var timeRate = CornerDynamics.TimeRate(kappa, n, alpha, vValue);
            var expected = 1.0 / (vValue * Math.Cos(alpha));
            Assert.AreEqual(expected, timeRate, Tolerance);
            Assert.IsGreaterThan(1.0 / vValue, timeRate, "dt/ds should increase with heading error");
        }

        #endregion

        #region Lateral Rate Tests (Arc-Length)

        [TestMethod]
        public void LateralRateSIsZeroWhenAligned()
        {
            // When aligned with road (alpha = 0), dn/ds = 0
            var kappa = 0.0;
            var n = 0.0;
            var alpha = 0.0;

            var lateralRate = CornerDynamics.LateralRateS(kappa, n, alpha);
            Assert.AreEqual(0.0, lateralRate, Tolerance);
        }

        [TestMethod]
        public void LateralRateSPositiveWhenTurningRight()
        {
            // When heading right of road (alpha > 0), dn/ds > 0
            var kappa = 0.0;
            var n = 0.0;
            var alpha = 0.2;

            var lateralRate = CornerDynamics.LateralRateS(kappa, n, alpha);
            // dn/ds = (1 - n*kappa) * tan(alpha) = tan(0.2) > 0
            var expected = Math.Tan(alpha);
            Assert.AreEqual(expected, lateralRate, Tolerance);
            Assert.IsGreaterThan(0, lateralRate, "dn/ds should be positive when heading right");
        }

        [TestMethod]
        public void LateralRateSNegativeWhenTurningLeft()
        {
            // When heading left of road (alpha < 0), dn/ds < 0
            var kappa = 0.0;
            var n = 0.0;
            var alpha = -0.2;

            var lateralRate = CornerDynamics.LateralRateS(kappa, n, alpha);
            Assert.IsLessThan(0, lateralRate, "dn/ds should be negative when heading left");
        }

        #endregion

        #region Speed Rate Tests

        [TestMethod]
        public void SpeedRateSIncludesDrag()
        {
            // dV/ds = (ax - drag) * dt/ds
            var kappa = 0.0;
            var n = 0.0;
            var alpha = 0.0;
            var vValue = 20.0;
            var ax = 5.0;
            var rho = 1.2;
            var cdA = 2.0;
            var mass = 800.0;

            var speedRate = CornerDynamics.SpeedRateS(kappa, n, alpha, vValue, ax, rho, cdA, mass);

            var drag = 0.5 * rho * vValue * vValue * cdA / mass;
            var dtds = 1.0 / vValue;
            var expected = (ax - drag) * dtds;
            Assert.AreEqual(expected, speedRate, Tolerance);
        }

        #endregion

        #region Running Cost Tests

        [TestMethod]
        public void RunningCostEqualsTimeRate()
        {
            // Running cost = dt/ds for minimum time problems
            var kappa = 0.1;
            var n = 0.5;
            var alpha = 0.1;
            var vValue = 20.0;

            var cost = CornerDynamics.RunningCostS(kappa, n, alpha, vValue);
            var timeRate = CornerDynamics.TimeRate(kappa, n, alpha, vValue);
            Assert.AreEqual(timeRate, cost, Tolerance);
        }

        #endregion

        #region Normal Force Tests

        [TestMethod]
        public void NormalForcesSumToTotalWeight()
        {
            // Total normal force should equal weight plus aero downforce
            var mass = 800.0;
            var g = 9.81;
            var a = 1.404;
            var b = 1.356;
            var tw = 0.807;
            var h = 0.35;
            var chi = 0.5;
            var ax = 0.0;
            var ay = 0.0;
            var rho = 1.2;
            var vValue = 20.0;
            var clA = 4.0;
            var coP = 1.6;

            var fzFR = CornerDynamics.NormalForceFR(mass, g, a, b, tw, h, chi, ax, ay, rho, vValue, clA, coP);
            var fzFL = CornerDynamics.NormalForceFL(mass, g, a, b, tw, h, chi, ax, ay, rho, vValue, clA, coP);
            var fzRR = CornerDynamics.NormalForceRR(mass, g, a, b, tw, h, chi, ax, ay, rho, vValue, clA, coP);
            var fzRL = CornerDynamics.NormalForceRL(mass, g, a, b, tw, h, chi, ax, ay, rho, vValue, clA, coP);

            var totalFz = fzFR + fzFL + fzRR + fzRL;
            var weight = mass * g;
            var aeroDownforce = 0.5 * rho * vValue * vValue * clA;
            var expected = weight + aeroDownforce;

            Assert.AreEqual(expected, totalFz, 1.0, "Total normal force should equal weight + aero downforce");
        }

        [TestMethod]
        public void LongitudinalAccelerationShiftsLoadRearward()
        {
            // Positive ax (accelerating) should increase rear normal forces
            var mass = 800.0;
            var g = 9.81;
            var a = 1.404;
            var b = 1.356;
            var tw = 0.807;
            var h = 0.35;
            var chi = 0.5;
            var rho = 1.2;
            var vValue = 20.0;
            var clA = 4.0;
            var coP = 1.6;

            var fzRR_no_ax = CornerDynamics.NormalForceRR(mass, g, a, b, tw, h, chi, 0.0, 0.0, rho, vValue, clA, coP);
            var fzRR_with_ax = CornerDynamics.NormalForceRR(mass, g, a, b, tw, h, chi, 5.0, 0.0, rho, vValue, clA, coP);

            Assert.IsGreaterThan(fzRR_no_ax, fzRR_with_ax, "Acceleration should increase rear normal force");
        }

        #endregion

        #region TrackGeometry Builder Tests

        [TestMethod]
        public void TrackGeometryBuilderCreatesCorrectTotalLength()
        {
            // Entry (15) + Arc (pi*5/2 ~ 7.854) + Exit (20) ~ 42.854
            var expected = 15.0 + Math.PI * 5.0 / 2.0 + 20.0;
            Assert.AreEqual(expected, _trackGeometry.TotalLength, Tolerance);
        }

        [TestMethod]
        public void TrackGeometryBuilderCreatesThreeSegments()
        {
            Assert.AreEqual(3, _trackGeometry.SegmentCount);
        }

        [TestMethod]
        public void TrackGeometryFirstSegmentIsLine()
        {
            Assert.IsInstanceOfType<LineSegment>(_trackGeometry[0]);
            Assert.AreEqual(15.0, _trackGeometry[0].Length, Tolerance);
        }

        [TestMethod]
        public void TrackGeometrySecondSegmentIsArc()
        {
            Assert.IsInstanceOfType<ArcSegment>(_trackGeometry[1]);
            var arc = (ArcSegment)_trackGeometry[1];
            Assert.AreEqual(5.0, arc.Radius, Tolerance);
            Assert.AreEqual(Math.PI / 2, arc.SweepAngle, Tolerance);
        }

        [TestMethod]
        public void TrackGeometryThirdSegmentIsLine()
        {
            Assert.IsInstanceOfType<LineSegment>(_trackGeometry[2]);
            Assert.AreEqual(20.0, _trackGeometry[2].Length, Tolerance);
        }

        #endregion

        #region Constraint Tests

        [TestMethod]
        public void PowerConstraintIsNegativeWhenWithinLimit()
        {
            var thrust = 0.5;
            var vValue = 50.0;
            var fmax = 12000.0;
            var pmax = 960000.0;

            var constraint = CornerDynamics.PowerConstraint(thrust, vValue, fmax, pmax);
            var power = thrust * fmax * vValue; // 0.5 * 12000 * 50 = 300000 < 960000

            Assert.IsLessThan(0, constraint, "Power constraint should be negative when within limit");
        }

        [TestMethod]
        public void PowerConstraintIsPositiveWhenExceedingLimit()
        {
            var thrust = 1.0;
            var vValue = 100.0;
            var fmax = 12000.0;
            var pmax = 960000.0;

            var constraint = CornerDynamics.PowerConstraint(thrust, vValue, fmax, pmax);
            var power = thrust * fmax * vValue; // 1.0 * 12000 * 100 = 1200000 > 960000

            Assert.IsGreaterThan(0, constraint, "Power constraint should be positive when exceeding limit");
        }

        [TestMethod]
        public void FrictionUtilizationIsOneAtLimit()
        {
            var fz = 2000.0;
            var muX = 1.5;
            var muY = 1.5;
            var fxMax = muX * fz;
            var fyMax = muY * fz;

            // At the limit: Fx = Fx_max, Fy = 0
            var utilization = CornerDynamics.FrictionUtilization(fxMax, 0.0, muX, muY, fz);
            Assert.AreEqual(1.0, utilization, Tolerance);

            // At the limit: Fx = 0, Fy = Fy_max
            utilization = CornerDynamics.FrictionUtilization(0.0, fyMax, muX, muY, fz);
            Assert.AreEqual(1.0, utilization, Tolerance);
        }

        #endregion
    }
}
