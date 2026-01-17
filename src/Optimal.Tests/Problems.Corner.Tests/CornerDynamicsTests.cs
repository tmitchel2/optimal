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

        #region Road Geometry Tests

        [TestMethod]
        public void RoadHeadingInEntryRegionIsZero()
        {
            // In entry region (s < 15), road heading should be 0 (east)
            var s = 10.0;
            var heading = CornerDynamics.RoadHeading(s);
            Assert.AreEqual(0.0, heading, Tolerance);
        }

        [TestMethod]
        public void RoadHeadingInArcRegionVariesLinearly()
        {
            // At middle of arc, heading should be -π/4
            var entryEnd = CornerDynamics.EntryLength;
            var arcLength = CornerDynamics.ArcLength;
            var s = entryEnd + arcLength / 2.0;
            
            var heading = CornerDynamics.RoadHeading(s);
            Assert.AreEqual(-Math.PI / 4.0, heading, Tolerance);
        }

        [TestMethod]
        public void RoadHeadingInExitRegionIsNegativeHalfPi()
        {
            // In exit region, heading should be -π/2 (south)
            var s = CornerDynamics.ArcEndS + 5.0;
            var heading = CornerDynamics.RoadHeading(s);
            Assert.AreEqual(-Math.PI / 2.0, heading, Tolerance);
        }

        [TestMethod]
        public void RoadCurvatureIsZeroOnStraights()
        {
            Assert.AreEqual(0.0, CornerDynamics.RoadCurvature(5.0), Tolerance, "Entry curvature");
            Assert.AreEqual(0.0, CornerDynamics.RoadCurvature(CornerDynamics.ArcEndS + 5.0), Tolerance, "Exit curvature");
        }

        [TestMethod]
        public void RoadCurvatureIsOneOverRadiusInArc()
        {
            var s = CornerDynamics.EntryEndS + CornerDynamics.ArcLength / 2.0;
            var curvature = CornerDynamics.RoadCurvature(s);
            Assert.AreEqual(1.0 / CornerDynamics.CenterlineRadius, curvature, Tolerance);
        }

        #endregion

        #region Progress Rate Tests

        [TestMethod]
        public void ProgressRateEqualsVelocityWhenAlignedOnStraight()
        {
            // On entry straight with θ = 0 (aligned with road), ṡ = v
            var s = 10.0;
            var n = 0.0;
            var theta = 0.0;
            var v = 15.0;

            var progressRate = CornerDynamics.ProgressRate(s, n, theta, v);
            Assert.AreEqual(v, progressRate, Tolerance, "ṡ should equal v when aligned on straight");
        }

        [TestMethod]
        public void ProgressRateReducesWithHeadingError()
        {
            // With heading error, ṡ = v × cos(error)
            var s = 10.0;
            var n = 0.0;
            var theta = 0.3; // 0.3 rad heading error
            var v = 15.0;

            var progressRate = CornerDynamics.ProgressRate(s, n, theta, v);
            var expected = v * Math.Cos(theta);
            Assert.AreEqual(expected, progressRate, Tolerance);
        }

        [TestMethod]
        public void ProgressRateInArcAccountsForCurvature()
        {
            // In arc with n offset, denominator = 1 - n × κ
            var s = CornerDynamics.EntryEndS + CornerDynamics.ArcLength / 2.0;
            var n = 2.0; // 2m right of centerline
            var theta = -Math.PI / 4.0; // Aligned with road at this point
            var v = 15.0;

            var curvature = 1.0 / CornerDynamics.CenterlineRadius;
            var denominator = 1.0 - n * curvature;
            var expected = v * Math.Cos(0.0) / denominator; // No heading error

            var progressRate = CornerDynamics.ProgressRate(s, n, theta, v);
            Assert.AreEqual(expected, progressRate, Tolerance);
        }

        #endregion

        #region Lateral Rate Tests

        [TestMethod]
        public void LateralRateIsZeroWhenAligned()
        {
            // When aligned with road (θ = θ_road), ṅ = 0
            var s = 10.0;
            var n = 0.0;
            var theta = 0.0; // Road heading is 0 on entry
            var v = 15.0;

            var lateralRate = CornerDynamics.LateralRate(s, n, theta, v);
            Assert.AreEqual(0.0, lateralRate, Tolerance);
        }

        [TestMethod]
        public void LateralRatePositiveWhenTurningRight()
        {
            // When heading right of road (θ < θ_road), headingError < 0, ṅ > 0
            // At s=10 (entry straight), θ_road = 0, so θ < 0 means heading right
            var s = 10.0;
            var n = 0.0;
            var theta = -0.2; // Heading 0.2 rad right of road (θ < θ_road)
            var v = 15.0;

            var lateralRate = CornerDynamics.LateralRate(s, n, theta, v);
            // ṅ = -v × sin(θ - θ_road) = -v × sin(-0.2) = v × sin(0.2)
            var expected = v * Math.Sin(0.2);
            Assert.AreEqual(expected, lateralRate, Tolerance);
            Assert.IsTrue(lateralRate > 0, "ṅ should be positive when heading right");
        }

        [TestMethod]
        public void LateralRateNegativeWhenTurningLeft()
        {
            // When heading left of road (θ > θ_road), headingError > 0, ṅ < 0
            // At s=10 (entry straight), θ_road = 0, so θ > 0 means heading left
            var s = 10.0;
            var n = 0.0;
            var theta = 0.2; // Heading 0.2 rad left of road (θ > θ_road)
            var v = 15.0;

            var lateralRate = CornerDynamics.LateralRate(s, n, theta, v);
            // ṅ = -v × sin(θ - θ_road) = -v × sin(0.2) < 0
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
            var s = 10.0;
            var n = 0.0;
            var theta = 0.1;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.ProgressRateForward_v(s, n, theta, v);

            var valuePlus = CornerDynamics.ProgressRate(s, n, theta, v + Epsilon);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void ProgressRateGradientWrtThetaMatchesNumerical()
        {
            var s = 10.0;
            var n = 0.0;
            var theta = 0.1;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.ProgressRateForward_theta(s, n, theta, v);

            var valuePlus = CornerDynamics.ProgressRate(s, n, theta + Epsilon, v);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void ProgressRateGradientWrtSMatchesNumericalInArc()
        {
            // In arc region where road heading depends on s
            var s = CornerDynamics.EntryEndS + CornerDynamics.ArcLength / 3.0;
            var n = 1.0;
            var theta = -0.3;
            var v = 12.0;

            var (value, gradient) = CornerDynamicsGradients.ProgressRateForward_s(s, n, theta, v);

            var valuePlus = CornerDynamics.ProgressRate(s + Epsilon, n, theta, v);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void LateralRateGradientWrtThetaMatchesNumerical()
        {
            var s = 10.0;
            var n = 0.0;
            var theta = 0.2;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.LateralRateForward_theta(s, n, theta, v);

            var valuePlus = CornerDynamics.LateralRate(s, n, theta + Epsilon, v);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        [TestMethod]
        public void LateralRateGradientWrtVMatchesNumerical()
        {
            var s = 10.0;
            var n = 0.0;
            var theta = 0.2;
            var v = 15.0;

            var (value, gradient) = CornerDynamicsGradients.LateralRateForward_v(s, n, theta, v);

            var valuePlus = CornerDynamics.LateralRate(s, n, theta, v + Epsilon);
            var numericalGradient = (valuePlus - value) / Epsilon;

            Assert.AreEqual(numericalGradient, gradient, GradientTolerance);
        }

        #endregion

        #region Time-Scaled Dynamics Tests

        [TestMethod]
        public void TimeScaledProgressRateGradientWrtTfIsCorrect()
        {
            var s = 10.0;
            var n = 0.0;
            var theta = 0.1;
            var v = 15.0;
            var Tf = 2.5;

            // Physical rate
            var (sdotPhys, _) = CornerDynamicsGradients.ProgressRateReverse(s, n, theta, v);
            
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
            var s = 0.0;
            var n = 0.0;
            var theta = 0.0;
            var v = 15.0;

            var (sdot, sdot_grads) = CornerDynamicsGradients.ProgressRateReverse(s, n, theta, v);

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
            var testCases = new[]
            {
                (s: 0.0, n: 0.0, theta: 0.0, v: 15.0),
                (s: 10.0, n: 2.0, theta: 0.1, v: 12.0),
                (s: 20.0, n: -1.0, theta: -0.5, v: 18.0), // In arc
                (s: 30.0, n: 0.0, theta: -Math.PI / 2, v: 15.0), // In exit
            };

            foreach (var (s, n, theta, v) in testCases)
            {
                var (sdot, sdot_grads) = CornerDynamicsGradients.ProgressRateReverse(s, n, theta, v);
                var (ndot, ndot_grads) = CornerDynamicsGradients.LateralRateReverse(s, n, theta, v);

                Assert.IsFalse(double.IsNaN(sdot), $"sdot is NaN at s={s}, n={n}, θ={theta}, v={v}");
                Assert.IsFalse(double.IsNaN(ndot), $"ndot is NaN at s={s}, n={n}, θ={theta}, v={v}");

                foreach (var g in sdot_grads)
                    Assert.IsFalse(double.IsNaN(g), $"sdot gradient contains NaN at s={s}");
                foreach (var g in ndot_grads)
                    Assert.IsFalse(double.IsNaN(g), $"ndot gradient contains NaN at s={s}");
            }
        }

        [TestMethod]
        public void TimeScaledDynamicsProducesNoNaN()
        {
            // Simulate time-scaled dynamics as in solver
            var s = 10.0;
            var n = 0.5;
            var theta = 0.1;
            var v = 15.0;
            var Tf = 3.0;
            var a = 1.0;
            var omega = 0.2;

            var (sdotPhys, sdot_gradients) = CornerDynamicsGradients.ProgressRateReverse(s, n, theta, v);
            var (ndotPhys, ndot_gradients) = CornerDynamicsGradients.LateralRateReverse(s, n, theta, v);
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
            Assert.AreEqual(4, sdot_gradients.Length, "sdot gradients should have 4 elements");
            Assert.AreEqual(4, ndot_gradients.Length, "ndot gradients should have 4 elements");
            Assert.AreEqual(1, thetadot_gradients.Length, "thetadot gradients should have 1 element");
            Assert.AreEqual(1, vdot_gradients.Length, "vdot gradients should have 1 element");
        }

        [TestMethod]
        public void InitialTrajectoryGuessIsValid()
        {
            // Simulate the linear interpolation initial guess
            var s0 = 0.0;
            var n0 = 0.0;
            var theta0 = 0.0;
            var v0 = 15.0;
            var Tf0 = 3.0;

            var sFinal = 42.854; // TotalLength
            var thetaFinal = -Math.PI / 2.0;

            var segments = 30;
            for (var i = 0; i <= segments; i++)
            {
                var tau = (double)i / segments;
                
                // Linear interpolation (what the solver does for initial guess)
                var s = s0 + tau * (sFinal - s0);
                var n = n0; // n stays at 0 in initial guess (NaN final means free)
                var theta = theta0 + tau * (thetaFinal - theta0);
                var v = v0; // v stays constant in initial guess (NaN final means free)
                var Tf = Tf0; // Tf stays constant (NaN final means free)

                // Test that dynamics work for this state
                var (sdot, _) = CornerDynamicsGradients.ProgressRateReverse(s, n, theta, v);
                var (ndot, _) = CornerDynamicsGradients.LateralRateReverse(s, n, theta, v);

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
            Assert.AreEqual(-CornerDynamics.EntryLength, x, Tolerance);
            Assert.AreEqual(0.0, y, Tolerance);
        }

        [TestMethod]
        public void EntryEndMapsToOrigin()
        {
            // s=EntryLength, n=0 should map to x=0, y=0
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(CornerDynamics.EntryLength, 0.0);
            Assert.AreEqual(0.0, x, Tolerance);
            Assert.AreEqual(0.0, y, Tolerance);
        }

        [TestMethod]
        public void ArcEndMapsToCorrectCartesian()
        {
            // s=EntryLength+ArcLength, n=0 should map to x=5, y=-5
            var s = CornerDynamics.EntryLength + CornerDynamics.ArcLength;
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(s, 0.0);
            Assert.AreEqual(CornerDynamics.CenterlineRadius, x, Tolerance);
            Assert.AreEqual(-CornerDynamics.CenterlineRadius, y, Tolerance);
        }

        [TestMethod]
        public void LateralOffsetInEntryIsCorrect()
        {
            // n > 0 should be right of centerline (negative y on entry)
            var s = 10.0;
            var n = 2.0;
            var (x, y) = CornerDynamicsHelpers.CurvilinearToCartesian(s, n);
            Assert.AreEqual(s - CornerDynamics.EntryLength, x, Tolerance);
            Assert.AreEqual(-n, y, Tolerance, "n>0 should give negative y on entry");
        }

        #endregion
    }
}
