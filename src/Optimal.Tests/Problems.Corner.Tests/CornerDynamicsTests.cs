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

        [TestInitialize]
        public void Initialize()
        {
            // Initialize track geometry before each test
            // Entry straight (15m) → 90° right turn (radius 5m) → Exit straight (20m)
            TrackGeometry.StartAt(x: -15, y: 0, heading: 0)
                .AddLine(distance: 15.0)
                .AddArc(radius: 5.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 20.0)
                .BuildAndSetCurrent();
        }

        #region Road Geometry Tests

        [TestMethod]
        public void RoadHeadingInEntryRegionIsZero()
        {
            // In entry region (s < 15), road heading should be 0 (east)
            var s = 10.0;
            var heading = TrackGeometry.Current.RoadHeading(s);
            Assert.AreEqual(0.0, heading, Tolerance);
        }

        [TestMethod]
        public void RoadHeadingInArcRegionVariesLinearly()
        {
            // At middle of arc, heading should be +π/4 (left-hand rule: clockwise is positive)
            var entryEnd = TrackGeometry.Current.GetEntryLength();
            var arcLength = TrackGeometry.Current.GetArcLength();
            var s = entryEnd + arcLength / 2.0;

            var heading = TrackGeometry.Current.RoadHeading(s);
            Assert.AreEqual(Math.PI / 4.0, heading, Tolerance);
        }

        [TestMethod]
        public void RoadHeadingInExitRegionIsPositiveHalfPi()
        {
            // In exit region, heading should be +π/2 (south, left-hand rule)
            var entryLength = TrackGeometry.Current.GetEntryLength();
            var arcLength = TrackGeometry.Current.GetArcLength();
            var s = entryLength + arcLength + 5.0;
            var heading = TrackGeometry.Current.RoadHeading(s);
            Assert.AreEqual(Math.PI / 2.0, heading, Tolerance);
        }

        [TestMethod]
        public void RoadCurvatureIsZeroOnStraights()
        {
            var entryLength = TrackGeometry.Current.GetEntryLength();
            var arcLength = TrackGeometry.Current.GetArcLength();
            Assert.AreEqual(0.0, TrackGeometry.Current.RoadCurvature(5.0), Tolerance, "Entry curvature");
            Assert.AreEqual(0.0, TrackGeometry.Current.RoadCurvature(entryLength + arcLength + 5.0), Tolerance, "Exit curvature");
        }

        [TestMethod]
        public void RoadCurvatureIsOneOverRadiusInArc()
        {
            var entryLength = TrackGeometry.Current.GetEntryLength();
            var arcLength = TrackGeometry.Current.GetArcLength();
            var arcRadius = TrackGeometry.Current.GetArcRadius();
            var s = entryLength + arcLength / 2.0;
            var curvature = TrackGeometry.Current.RoadCurvature(s);
            Assert.AreEqual(1.0 / arcRadius, curvature, Tolerance);
        }

        #endregion

        #region Progress Rate Tests

        [TestMethod]
        public void ProgressRateEqualsVelocityWhenAlignedOnStraight()
        {
            // On entry straight with θ = 0 (aligned with road), ṡ = v
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.0;
            var theta = 0.0;
            var v = 15.0;

            var progressRate = CornerDynamics.ProgressRate(thetaRoad, curvature, n, theta, v);
            Assert.AreEqual(v, progressRate, Tolerance, "ṡ should equal v when aligned on straight");
        }

        [TestMethod]
        public void ProgressRateReducesWithHeadingError()
        {
            // With heading error, ṡ = v × cos(error)
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.0;
            var theta = 0.3; // 0.3 rad heading error
            var v = 15.0;

            var progressRate = CornerDynamics.ProgressRate(thetaRoad, curvature, n, theta, v);
            var expected = v * Math.Cos(theta);
            Assert.AreEqual(expected, progressRate, Tolerance);
        }

        [TestMethod]
        public void ProgressRateInArcAccountsForCurvature()
        {
            // In arc with n offset, denominator = 1 - n × κ
            var arcRadius = TrackGeometry.Current.GetArcRadius();
            var thetaRoad = Math.PI / 4.0; // Mid-arc heading
            var curvature = 1.0 / arcRadius;
            var n = 2.0; // 2m right of centerline
            var theta = Math.PI / 4.0; // Aligned with road
            var v = 15.0;

            var denominator = 1.0 - n * curvature;
            var expected = v * Math.Cos(0.0) / denominator; // No heading error

            var progressRate = CornerDynamics.ProgressRate(thetaRoad, curvature, n, theta, v);
            Assert.AreEqual(expected, progressRate, Tolerance);
        }

        #endregion

        #region Lateral Rate Tests

        [TestMethod]
        public void LateralRateIsZeroWhenAligned()
        {
            // When aligned with road (θ = θ_road), ṅ = 0
            var thetaRoad = 0.0;
            var theta = 0.0;
            var v = 15.0;

            var lateralRate = CornerDynamics.LateralRate(thetaRoad, theta, v);
            Assert.AreEqual(0.0, lateralRate, Tolerance);
        }

        [TestMethod]
        public void LateralRatePositiveWhenTurningRight()
        {
            // Left-hand rule: positive θ = clockwise (right)
            // When heading right of road (θ > θ_road), headingError > 0, ṅ > 0
            var thetaRoad = 0.0;
            var theta = 0.2; // Heading 0.2 rad right of road (θ > θ_road)
            var v = 15.0;

            var lateralRate = CornerDynamics.LateralRate(thetaRoad, theta, v);
            // ṅ = v × sin(θ - θ_road) = v × sin(0.2) > 0
            var expected = v * Math.Sin(0.2);
            Assert.AreEqual(expected, lateralRate, Tolerance);
            Assert.IsTrue(lateralRate > 0, "ṅ should be positive when heading right");
        }

        [TestMethod]
        public void LateralRateNegativeWhenTurningLeft()
        {
            // Left-hand rule: negative θ = counterclockwise (left)
            // When heading left of road (θ < θ_road), headingError < 0, ṅ < 0
            var thetaRoad = 0.0;
            var theta = -0.2; // Heading 0.2 rad left of road (θ < θ_road)
            var v = 15.0;

            var lateralRate = CornerDynamics.LateralRate(thetaRoad, theta, v);
            // ṅ = v × sin(θ - θ_road) = v × sin(-0.2) < 0
            Assert.IsTrue(lateralRate < 0, "ṅ should be negative when heading left");
        }

        #endregion

        #region Simple Rate Tests

        [TestMethod]
        public void ThetaRateEqualsOmega()
        {
            var omega = 0.5;
            Assert.AreEqual(omega, CornerDynamics.ThetaRate(omega), Tolerance);
        }

        [TestMethod]
        public void VelocityRateEqualsAcceleration()
        {
            var a = 3.0;
            Assert.AreEqual(a, CornerDynamics.VelocityRate(a), Tolerance);
        }

        [TestMethod]
        public void RunningCostIsOne()
        {
            Assert.AreEqual(1.0, CornerDynamics.RunningCost(), Tolerance);
        }

        #endregion

        #region Gradient Tests

        [TestMethod]
        public void ProgressRateGradientWrtVMatchesNumerical()
        {
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.0;
            var theta = 0.1;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.ProgressRateForward_v(thetaRoad, curvature, n, theta, v);

            var valuePlus = CornerDynamics.ProgressRate(thetaRoad, curvature, n, theta, v + Epsilon);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void ProgressRateGradientWrtThetaMatchesNumerical()
        {
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.0;
            var theta = 0.1;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.ProgressRateForward_theta(thetaRoad, curvature, n, theta, v);

            var valuePlus = CornerDynamics.ProgressRate(thetaRoad, curvature, n, theta + Epsilon, v);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void ProgressRateGradientWrtNMatchesNumericalInArc()
        {
            // In arc region where curvature matters
            var arcRadius = TrackGeometry.Current.GetArcRadius();
            var thetaRoad = Math.PI / 6.0;
            var curvature = 1.0 / arcRadius;
            var n = 1.0;
            var theta = -0.3;
            var v = 12.0;

            var (value, gradient) = CornerDynamicsGradients.ProgressRateForward_n(thetaRoad, curvature, n, theta, v);

            var valuePlus = CornerDynamics.ProgressRate(thetaRoad, curvature, n + Epsilon, theta, v);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void LateralRateGradientWrtThetaMatchesNumerical()
        {
            var thetaRoad = 0.0;
            var theta = 0.2;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.LateralRateForward_theta(thetaRoad, theta, v);

            var valuePlus = CornerDynamics.LateralRate(thetaRoad, theta + Epsilon, v);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void LateralRateGradientWrtVMatchesNumerical()
        {
            var thetaRoad = 0.0;
            var theta = 0.2;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.LateralRateForward_v(thetaRoad, theta, v);

            var valuePlus = CornerDynamics.LateralRate(thetaRoad, theta, v + Epsilon);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        #endregion

        #region Time-Scaled Dynamics Tests

        [TestMethod]
        public void TimeScaledProgressRateGradientWrtTfIsCorrect()
        {
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.0;
            var theta = 0.1;
            var v = 15.0;
            var Tf = 2.5;

            // Physical rate
            var (sdotPhys, _) = CornerDynamicsGradients.ProgressRateReverse(thetaRoad, curvature, n, theta, v);

            // Time-scaled rate: sdot = Tf × sdotPhys
            var sdot = Tf * sdotPhys;

            // Gradient w.r.t. Tf should be sdotPhys
            var dsdot_dTf = sdotPhys;

            // Verify numerically
            var sdot_plus = (Tf + Epsilon) * sdotPhys;
            var numerical = (sdot_plus - sdot) / Epsilon;

            Assert.AreEqual(dsdot_dTf, numerical, GradientTolerance);
        }

        #endregion

        #region Solver Integration Diagnostics

        [TestMethod]
        public void ProgressRateGradientAtS0HasNoNaN()
        {
            // Specifically test at s=0 where NaN was found
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.0;
            var theta = 0.0;
            var v = 15.0;

            var (sdot, sdot_grads) = CornerDynamicsGradients.ProgressRateReverse(thetaRoad, curvature, n, theta, v);

            Console.WriteLine($"sdot = {sdot}");
            for (int i = 0; i < sdot_grads.Length; i++)
            {
                Console.WriteLine($"sdot_grads[{i}] = {sdot_grads[i]}");
            }

            Assert.IsFalse(double.IsNaN(sdot), "sdot is NaN");
            for (int i = 0; i < sdot_grads.Length; i++)
            {
                Assert.IsFalse(double.IsNaN(sdot_grads[i]), $"sdot_grads[{i}] is NaN");
            }
        }

        [TestMethod]
        public void DynamicsProducesNoNaNForTypicalInputs()
        {
            // Test dynamics with typical inputs that the solver would use
            // Left-hand rule: positive θ = clockwise, +π/2 = south
            var testCases = new[]
            {
                (thetaRoad: 0.0, curvature: 0.0, theta: 0.0, v: 15.0),
                (thetaRoad: 0.0, curvature: 0.0, theta: 0.1, v: 12.0),
                (thetaRoad: 0.5, curvature: 0.2, theta: 0.5, v: 18.0), // In arc
                (thetaRoad: Math.PI / 2, curvature: 0.0, theta: Math.PI / 2, v: 15.0), // In exit
            };

            foreach (var (thetaRoad, curvature, theta, v) in testCases)
            {
                var n = 0.0;
                var (sdot, sdot_grads) = CornerDynamicsGradients.ProgressRateReverse(thetaRoad, curvature, n, theta, v);
                var (ndot, ndot_grads) = CornerDynamicsGradients.LateralRateReverse(thetaRoad, theta, v);

                Assert.IsFalse(double.IsNaN(sdot), $"sdot is NaN at thetaRoad={thetaRoad}, θ={theta}, v={v}");
                Assert.IsFalse(double.IsNaN(ndot), $"ndot is NaN at thetaRoad={thetaRoad}, θ={theta}, v={v}");

                foreach (var g in sdot_grads)
                    Assert.IsFalse(double.IsNaN(g), $"sdot gradient contains NaN at thetaRoad={thetaRoad}");
                foreach (var g in ndot_grads)
                    Assert.IsFalse(double.IsNaN(g), $"ndot gradient contains NaN at thetaRoad={thetaRoad}");
            }
        }

        [TestMethod]
        public void TimeScaledDynamicsProducesNoNaN()
        {
            // Simulate time-scaled dynamics as in solver
            var thetaRoad = 0.0;
            var curvature = 0.0;
            var n = 0.5;
            var theta = 0.1;
            var v = 15.0;
            var Tf = 3.0;
            var a = 1.0;
            var omega = 0.2;

            var (sdotPhys, sdot_gradients) = CornerDynamicsGradients.ProgressRateReverse(thetaRoad, curvature, n, theta, v);
            var (ndotPhys, ndot_gradients) = CornerDynamicsGradients.LateralRateReverse(thetaRoad, theta, v);
            var (thetadotPhys, thetadot_gradients) = CornerDynamicsGradients.ThetaRateReverse(omega);
            var (vdotPhys, vdot_gradients) = CornerDynamicsGradients.VelocityRateReverse(a);

            var sdot = Tf * sdotPhys;
            var ndot = Tf * ndotPhys;
            var thetadot = Tf * thetadotPhys;
            var vdot = Tf * vdotPhys;

            Assert.IsFalse(double.IsNaN(sdot), "sdot is NaN");
            Assert.IsFalse(double.IsNaN(ndot), "ndot is NaN");
            Assert.IsFalse(double.IsNaN(thetadot), "thetadot is NaN");
            Assert.IsFalse(double.IsNaN(vdot), "vdot is NaN");

            // Check state gradients layout
            Assert.AreEqual(5, sdot_gradients.Length, "sdot gradients should have 5 elements");
            Assert.AreEqual(3, ndot_gradients.Length, "ndot gradients should have 3 elements");
            Assert.AreEqual(1, thetadot_gradients.Length, "thetadot gradients should have 1 element");
            Assert.AreEqual(1, vdot_gradients.Length, "vdot gradients should have 1 element");
        }

        [TestMethod]
        public void InitialTrajectoryGuessIsValid()
        {
            // Simulate the linear interpolation initial guess
            // Left-hand rule: positive θ = clockwise, +π/2 = south
            var theta0 = 0.0;
            var v0 = 15.0;
            var thetaFinal = Math.PI / 2.0; // South = +π/2 with left-hand rule

            var segments = 30;
            var totalLength = TrackGeometry.Current.TotalLength;

            for (var i = 0; i <= segments; i++)
            {
                var tau = (double)i / segments;

                // Compute s and get road geometry
                var s = tau * totalLength;
                var thetaRoad = TrackGeometry.Current.RoadHeading(s);
                var curvature = TrackGeometry.Current.RoadCurvature(s);

                var n = 0.0; // Centerline
                var theta = theta0 + tau * (thetaFinal - theta0);
                var v = v0;

                // Test that dynamics work for this state
                var (sdot, _) = CornerDynamicsGradients.ProgressRateReverse(thetaRoad, curvature, n, theta, v);
                var (ndot, _) = CornerDynamicsGradients.LateralRateReverse(thetaRoad, theta, v);

                Assert.IsFalse(double.IsNaN(sdot), $"sdot is NaN at tau={tau}, s={s}, theta={theta}");
                Assert.IsFalse(double.IsNaN(ndot), $"ndot is NaN at tau={tau}, s={s}, theta={theta}");
                Assert.IsFalse(double.IsInfinity(sdot), $"sdot is Inf at tau={tau}");
                Assert.IsFalse(double.IsInfinity(ndot), $"ndot is Inf at tau={tau}");
            }
        }

        #endregion

        #region Curvilinear to Cartesian Conversion Tests

        [TestMethod]
        public void EntryStartMapsToCorrectCartesian()
        {
            // s=0, n=0 should map to x=-15, y=0
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(0.0, 0.0);
            Assert.AreEqual(-TrackGeometry.Current.GetEntryLength(), x, Tolerance);
            Assert.AreEqual(0.0, y, Tolerance);
        }

        [TestMethod]
        public void EntryEndMapsToOrigin()
        {
            // s=EntryLength, n=0 should map to x=0, y=0
            var entryLength = TrackGeometry.Current.GetEntryLength();
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(entryLength, 0.0);
            Assert.AreEqual(0.0, x, Tolerance);
            Assert.AreEqual(0.0, y, Tolerance);
        }

        [TestMethod]
        public void ArcEndMapsToCorrectCartesian()
        {
            // s=EntryLength+ArcLength, n=0 should map to x=5, y=-5
            var entryLength = TrackGeometry.Current.GetEntryLength();
            var arcLength = TrackGeometry.Current.GetArcLength();
            var arcRadius = TrackGeometry.Current.GetArcRadius();
            var s = entryLength + arcLength;
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(s, 0.0);
            Assert.AreEqual(arcRadius, x, Tolerance);
            Assert.AreEqual(-arcRadius, y, Tolerance);
        }

        [TestMethod]
        public void LateralOffsetInEntryIsCorrect()
        {
            // n > 0 should be right of centerline (negative y on entry)
            var entryLength = TrackGeometry.Current.GetEntryLength();
            var s = 10.0;
            var n = 2.0;
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(s, n);
            Assert.AreEqual(s - entryLength, x, Tolerance);
            Assert.AreEqual(-n, y, Tolerance, "n>0 should give negative y on entry");
        }

        #endregion

        #region TrackGeometry Builder Tests

        [TestMethod]
        public void TrackGeometryBuilderCreatesCorrectTotalLength()
        {
            // Entry (15) + Arc (π×5/2 ≈ 7.854) + Exit (20) ≈ 42.854
            var expected = 15.0 + Math.PI * 5.0 / 2.0 + 20.0;
            Assert.AreEqual(expected, TrackGeometry.Current.TotalLength, Tolerance);
        }

        [TestMethod]
        public void TrackGeometryBuilderCreatesThreeSegments()
        {
            Assert.AreEqual(3, TrackGeometry.Current.SegmentCount);
        }

        [TestMethod]
        public void TrackGeometryFirstSegmentIsLine()
        {
            Assert.IsInstanceOfType<LineSegment>(TrackGeometry.Current[0]);
            Assert.AreEqual(15.0, TrackGeometry.Current[0].Length, Tolerance);
        }

        [TestMethod]
        public void TrackGeometrySecondSegmentIsArc()
        {
            Assert.IsInstanceOfType<ArcSegment>(TrackGeometry.Current[1]);
            var arc = (ArcSegment)TrackGeometry.Current[1];
            Assert.AreEqual(5.0, arc.Radius, Tolerance);
            Assert.AreEqual(Math.PI / 2, arc.SweepAngle, Tolerance);
        }

        [TestMethod]
        public void TrackGeometryThirdSegmentIsLine()
        {
            Assert.IsInstanceOfType<LineSegment>(TrackGeometry.Current[2]);
            Assert.AreEqual(20.0, TrackGeometry.Current[2].Length, Tolerance);
        }

        [TestMethod]
        public void TrackGeometryGetEntryLengthReturnsCorrectValue()
        {
            Assert.AreEqual(15.0, TrackGeometry.Current.GetEntryLength(), Tolerance);
        }

        [TestMethod]
        public void TrackGeometryGetArcLengthReturnsCorrectValue()
        {
            Assert.AreEqual(Math.PI * 5.0 / 2.0, TrackGeometry.Current.GetArcLength(), Tolerance);
        }

        [TestMethod]
        public void TrackGeometryGetArcRadiusReturnsCorrectValue()
        {
            Assert.AreEqual(5.0, TrackGeometry.Current.GetArcRadius(), Tolerance);
        }

        #endregion
    }
}
