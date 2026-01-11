/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// Tests for AutoDiff-generated gradients of curvilinear dynamics functions.
    /// These tests verify that the source generator correctly handles:
    /// - Math.PI constant in expressions
    /// - Conditionals with computed thresholds
    /// - Complex trigonometric expressions
    /// </summary>
    [TestClass]
    public class CurvilinearDynamicsTests
    {
        private const double Epsilon = 1e-6;
        private const double Tolerance = 1e-4;

        #region QuarterCircleArcLength Tests

        [TestMethod]
        public void QuarterCircleArcLengthValue()
        {
            var radius = 5.0;
            var expected = Math.PI * radius / 2.0;
            var actual = CurvilinearDynamicsFunctions.QuarterCircleArcLength(radius);
            Assert.AreEqual(expected, actual, 1e-10, "π × 5 / 2 ≈ 7.854");
        }

        [TestMethod]
        public void QuarterCircleArcLengthGradient()
        {
            var radius = 5.0;
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.QuarterCircleArcLengthForward_radius(radius);

            Assert.AreEqual(Math.PI * radius / 2.0, value, 1e-10);
            Assert.AreEqual(Math.PI / 2.0, gradient, 1e-10, "d(π×r/2)/dr = π/2");

            ValidateGradient(radius, gradient, r => CurvilinearDynamicsFunctions.QuarterCircleArcLength(r));
        }

        #endregion

        #region SineWithPiOffset Tests

        [TestMethod]
        public void SineWithPiOffsetValue()
        {
            var x = 0.0;
            var expected = Math.Sin(Math.PI / 4.0);
            var actual = CurvilinearDynamicsFunctions.SineWithPiOffset(x);
            Assert.AreEqual(expected, actual, 1e-10, "sin(0 + π/4) = sin(π/4) ≈ 0.707");
        }

        [TestMethod]
        public void SineWithPiOffsetGradient()
        {
            var x = 0.5;
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.SineWithPiOffsetForward_x(x);

            Assert.AreEqual(Math.Sin(x + Math.PI / 4.0), value, 1e-10);
            Assert.AreEqual(Math.Cos(x + Math.PI / 4.0), gradient, 1e-10, "d(sin(x + π/4))/dx = cos(x + π/4)");

            ValidateGradient(x, gradient, x => CurvilinearDynamicsFunctions.SineWithPiOffset(x));
        }

        #endregion

        #region AngleWrap Tests

        [TestMethod]
        public void AngleWrapInRange()
        {
            var theta = 1.0;
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.AngleWrapForward_theta(theta);

            Assert.AreEqual(1.0, value, 1e-10, "theta in range returns theta");
            Assert.AreEqual(1.0, gradient, 1e-10, "d(theta)/dtheta = 1 when in range");
        }

        [TestMethod]
        public void AngleWrapAbovePi()
        {
            var theta = 4.0; // > π
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.AngleWrapForward_theta(theta);

            Assert.AreEqual(4.0 - 2.0 * Math.PI, value, 1e-10, "theta - 2π when > π");
            Assert.AreEqual(1.0, gradient, 1e-10, "d(theta - 2π)/dtheta = 1");

            ValidateGradient(theta, gradient, t => CurvilinearDynamicsFunctions.AngleWrap(t));
        }

        [TestMethod]
        public void AngleWrapBelowNegPi()
        {
            var theta = -4.0; // < -π
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.AngleWrapForward_theta(theta);

            Assert.AreEqual(-4.0 + 2.0 * Math.PI, value, 1e-10, "theta + 2π when < -π");
            Assert.AreEqual(1.0, gradient, 1e-10, "d(theta + 2π)/dtheta = 1");

            ValidateGradient(theta, gradient, t => CurvilinearDynamicsFunctions.AngleWrap(t));
        }

        #endregion

        #region RoadHeading Tests

        [TestMethod]
        public void RoadHeadingEntryRegion()
        {
            var s = 10.0; // In entry straight (s < 15)
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.RoadHeadingForward_s(s);

            Assert.AreEqual(0.0, value, 1e-10, "Heading is 0 on entry straight");
            Assert.AreEqual(0.0, gradient, 1e-10, "d(0)/ds = 0");
        }

        [TestMethod]
        public void RoadHeadingArcRegion()
        {
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength / 2.0; // Middle of arc

            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.RoadHeadingForward_s(s);

            // At middle of arc, heading should be -π/4
            var expectedValue = -Math.PI / 4.0;
            Assert.AreEqual(expectedValue, value, 1e-10, "Heading is -π/4 at middle of arc");

            // Gradient: d(-arcProgress × π/2)/ds = -(1/arcLength) × π/2
            var expectedGradient = -Math.PI / 2.0 / arcLength;
            Assert.AreEqual(expectedGradient, gradient, 1e-10);

            ValidateGradient(s, gradient, s => CurvilinearDynamicsFunctions.RoadHeading(s));
        }

        [TestMethod]
        public void RoadHeadingExitRegion()
        {
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength + 5.0; // In exit straight

            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.RoadHeadingForward_s(s);

            Assert.AreEqual(-Math.PI / 2.0, value, 1e-10, "Heading is -π/2 on exit straight");
            Assert.AreEqual(0.0, gradient, 1e-10, "d(-π/2)/ds = 0");
        }

        #endregion

        #region RoadCurvature Tests

        [TestMethod]
        public void RoadCurvatureEntryRegion()
        {
            var s = 10.0;
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.RoadCurvatureForward_s(s);

            Assert.AreEqual(0.0, value, 1e-10, "Curvature is 0 on straight");
            Assert.AreEqual(0.0, gradient, 1e-10, "d(0)/ds = 0");
        }

        [TestMethod]
        public void RoadCurvatureArcRegion()
        {
            var s = 18.0; // In arc region
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.RoadCurvatureForward_s(s);

            Assert.AreEqual(1.0 / 5.0, value, 1e-10, "Curvature is 1/R = 0.2 on arc");
            Assert.AreEqual(0.0, gradient, 1e-10, "d(constant)/ds = 0");
        }

        #endregion

        #region ProgressRateInline Tests

        [TestMethod]
        public void ProgressRateInlineEntryRegion()
        {
            // On entry straight: thetaRoad=0, curvature=0, so ṡ = v×cos(θ)
            var s = 10.0;
            var n = 0.5;
            var theta = 0.1;
            var v = 15.0;

            var expected = v * Math.Cos(theta); // denominator = 1 when curvature = 0
            var actual = CurvilinearDynamicsFunctions.ProgressRateInline(s, n, theta, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. v
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.ProgressRateInlineForward_v(s, n, theta, v);
            Assert.AreEqual(expected, value, 1e-10);
            ValidateGradient(v, gradient, vv => CurvilinearDynamicsFunctions.ProgressRateInline(s, n, theta, vv));
        }

        [TestMethod]
        public void ProgressRateInlineArcRegion()
        {
            // In arc: curvature = 0.2, so denominator = 1 - n×0.2
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength / 2.0; // Middle of arc
            var n = 1.0;
            var theta = -0.3;
            var v = 12.0;

            // thetaRoad at middle of arc = -π/4
            var thetaRoad = -Math.PI / 4.0;
            var headingError = theta - thetaRoad;
            var denominator = 1.0 - n * 0.2;
            var expected = v * Math.Cos(headingError) / denominator;

            var actual = CurvilinearDynamicsFunctions.ProgressRateInline(s, n, theta, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. theta
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.ProgressRateInlineForward_theta(s, n, theta, v);
            Assert.AreEqual(expected, value, 1e-10);
            ValidateGradient(theta, gradient, t => CurvilinearDynamicsFunctions.ProgressRateInline(s, n, t, v));
        }

        [TestMethod]
        public void ProgressRateInlineExitRegion()
        {
            // On exit straight: thetaRoad=-π/2, curvature=0
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength + 5.0;
            var n = 0.0;
            var theta = -Math.PI / 2.0 + 0.05;
            var v = 10.0;

            var headingError = theta - (-Math.PI / 2.0);
            var expected = v * Math.Cos(headingError);

            var actual = CurvilinearDynamicsFunctions.ProgressRateInline(s, n, theta, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. n
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.ProgressRateInlineForward_n(s, n, theta, v);
            Assert.AreEqual(expected, value, 1e-10);
            ValidateGradient(n, gradient, nn => CurvilinearDynamicsFunctions.ProgressRateInline(s, nn, theta, v));
        }

        #endregion

        #region LateralRateInline Tests

        [TestMethod]
        public void LateralRateInlineEntryRegion()
        {
            // On entry straight: thetaRoad=0, so ṅ = v×sin(θ)
            var s = 5.0;
            var n = 0.0;
            var theta = 0.2;
            var v = 15.0;

            var expected = v * Math.Sin(theta);
            var actual = CurvilinearDynamicsFunctions.LateralRateInline(s, n, theta, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. theta
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.LateralRateInlineForward_theta(s, n, theta, v);
            Assert.AreEqual(expected, value, 1e-10);
            Assert.AreEqual(v * Math.Cos(theta), gradient, 1e-10, "d(v×sin(θ))/dθ = v×cos(θ)");
            ValidateGradient(theta, gradient, t => CurvilinearDynamicsFunctions.LateralRateInline(s, n, t, v));
        }

        [TestMethod]
        public void LateralRateInlineArcRegion()
        {
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength * 0.75; // 3/4 through arc
            var n = -0.5;
            var theta = -1.0;
            var v = 20.0;

            // thetaRoad at 3/4 of arc = -3π/8
            var thetaRoad = -0.75 * Math.PI / 2.0;
            var headingError = theta - thetaRoad;
            var expected = v * Math.Sin(headingError);

            var actual = CurvilinearDynamicsFunctions.LateralRateInline(s, n, theta, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. v
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.LateralRateInlineForward_v(s, n, theta, v);
            Assert.AreEqual(expected, value, 1e-10);
            Assert.AreEqual(Math.Sin(headingError), gradient, 1e-10);
            ValidateGradient(v, gradient, vv => CurvilinearDynamicsFunctions.LateralRateInline(s, n, theta, vv));
        }

        [TestMethod]
        public void LateralRateInlineGradientWrtS()
        {
            // Test gradient w.r.t. s in arc region where thetaRoad depends on s
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength / 3.0;
            var n = 0.0;
            var theta = -0.5;
            var v = 10.0;

            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.LateralRateInlineForward_s(s, n, theta, v);

            // Validate against numerical gradient
            ValidateGradient(s, gradient, ss => CurvilinearDynamicsFunctions.LateralRateInline(ss, n, theta, v));
        }

        #endregion

        #region ProgressWithProperty Tests (Property Access)

        [TestMethod]
        public void ProgressWithPropertyEntryRegion()
        {
            // In entry region (s < 15), returns v directly
            var s = 10.0;
            var v = 20.0;

            var expected = v;
            var actual = CurvilinearDynamicsFunctions.ProgressWithProperty(s, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. v
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.ProgressWithPropertyForward_v(s, v);
            Assert.AreEqual(expected, value, 1e-10);
            Assert.AreEqual(1.0, gradient, 1e-10, "d(v)/dv = 1");
        }

        [TestMethod]
        public void ProgressWithPropertyArcRegion()
        {
            // In arc region, returns v * (1 - 0.3 * arcProgress)
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength / 2.0; // Middle of arc
            var v = 20.0;

            var arcProgress = (s - entryEnd) / arcLength;
            var expected = v * (1.0 - 0.3 * arcProgress);
            var actual = CurvilinearDynamicsFunctions.ProgressWithProperty(s, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. s
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.ProgressWithPropertyForward_s(s, v);
            Assert.AreEqual(expected, value, 1e-10);
            ValidateGradient(s, gradient, ss => CurvilinearDynamicsFunctions.ProgressWithProperty(ss, v));
        }

        [TestMethod]
        public void ProgressWithPropertyExitRegion()
        {
            // In exit region (s >= arcEnd), returns v * 0.7
            var entryEnd = 15.0;
            var arcLength = Math.PI * 5.0 / 2.0;
            var s = entryEnd + arcLength + 5.0;
            var v = 20.0;

            var expected = v * 0.7;
            var actual = CurvilinearDynamicsFunctions.ProgressWithProperty(s, v);
            Assert.AreEqual(expected, actual, 1e-10);

            // Verify gradient w.r.t. v
            var (value, gradient) = CurvilinearDynamicsFunctionsGradients.ProgressWithPropertyForward_v(s, v);
            Assert.AreEqual(expected, value, 1e-10);
            Assert.AreEqual(0.7, gradient, 1e-10, "d(0.7*v)/dv = 0.7");
        }

        #endregion

        #region ConditionalWithDivision Tests - Minimal test for reverse mode branch bug

        /// <summary>
        /// This test demonstrates the reverse mode conditional adjoint bug.
        /// When x=0, the first branch (factor=2.0) is taken.
        /// But the backward pass unconditionally computes adjoints for ALL branches,
        /// including the else branch (factor = y/x) which causes division by zero.
        /// </summary>
        [TestMethod]
        public void ConditionalWithDivisionReverseAtX0HasNoNaN()
        {
            var x = 0.0;  // First branch will be taken (x < 10)
            var y = 5.0;

            var (value, gradients) = CurvilinearDynamicsFunctionsGradients.ConditionalWithDivisionReverse(x, y);

            // First branch: factor = 2.0, result = y * factor = 10
            Assert.AreEqual(10.0, value, 1e-10, "Value should be y * 2 = 10");

            // Gradients should be well-defined
            Assert.IsFalse(double.IsNaN(gradients[0]), "Gradient w.r.t. x should not be NaN");
            Assert.IsFalse(double.IsNaN(gradients[1]), "Gradient w.r.t. y should not be NaN");

            // In branch 1, result = y * 2, so d/dy = 2, d/dx = 0
            Assert.AreEqual(0.0, gradients[0], 1e-10, "d/dx should be 0 in first branch");
            Assert.AreEqual(2.0, gradients[1], 1e-10, "d/dy should be 2 in first branch");
        }

        [TestMethod]
        public void ConditionalWithDivisionReverseInSecondBranch()
        {
            var x = 15.0;  // Second branch will be taken (x >= 10)
            var y = 30.0;

            var (value, gradients) = CurvilinearDynamicsFunctionsGradients.ConditionalWithDivisionReverse(x, y);

            // Second branch: factor = y/x = 2, result = y * factor = y * y/x = y^2/x = 60
            var expected = y * y / x;
            Assert.AreEqual(expected, value, 1e-10, "Value should be y^2/x = 60");

            // d(y^2/x)/dx = -y^2/x^2 = -4
            // d(y^2/x)/dy = 2y/x = 4
            var expectedDx = -y * y / (x * x);
            var expectedDy = 2 * y / x;

            Assert.IsFalse(double.IsNaN(gradients[0]), "Gradient w.r.t. x should not be NaN");
            Assert.IsFalse(double.IsNaN(gradients[1]), "Gradient w.r.t. y should not be NaN");

            Assert.AreEqual(expectedDx, gradients[0], 1e-10, $"d/dx should be {expectedDx}");
            Assert.AreEqual(expectedDy, gradients[1], 1e-10, $"d/dy should be {expectedDy}");
        }

        #endregion

        #region Helper Methods

        private static void ValidateGradient(double x, double analyticalGradient, Func<double, double> function)
        {
            var fX = function(x);
            var fXPlusEps = function(x + Epsilon);
            var numericalGradient = (fXPlusEps - fX) / Epsilon;

            Assert.AreEqual(numericalGradient, analyticalGradient, Tolerance,
                $"Analytical gradient ({analyticalGradient:G6}) should match numerical gradient ({numericalGradient:G6}) at x={x}");
        }

        #endregion
    }
}
