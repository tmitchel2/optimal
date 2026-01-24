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
            // Entry straight (15m) → 90° right turn (radius 5m) → Exit straight (20m)
            _trackGeometry = TrackGeometry.StartAt(x: -15, y: 0, heading: 0)
                .AddLine(distance: 15.0)
                .AddArc(radius: 5.0, angle: Math.PI / 2, turnRight: true)
                .AddLine(distance: 20.0)
                .Build();
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
            var totalLength = _trackGeometry.TotalLength;

            for (var i = 0; i <= segments; i++)
            {
                var tau = (double)i / segments;

                // Compute s and get road geometry
                var s = tau * totalLength;
                var thetaRoad = _trackGeometry.RoadHeading(s);
                var curvature = _trackGeometry.RoadCurvature(s);

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

        #region TrackGeometry Builder Tests

        [TestMethod]
        public void TrackGeometryBuilderCreatesCorrectTotalLength()
        {
            // Entry (15) + Arc (π×5/2 ≈ 7.854) + Exit (20) ≈ 42.854
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
    }
}
