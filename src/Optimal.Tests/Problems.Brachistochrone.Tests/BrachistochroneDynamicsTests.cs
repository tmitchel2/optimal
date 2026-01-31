/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using OptimalCli.Problems.Brachistochrone;

namespace Optimal.Problems.Brachistochrone.Tests
{
    /// <summary>
    /// Unit tests for the arc-length parameterized Brachistochrone dynamics.
    ///
    /// Arc-length formulation:
    /// - Independent variable: s (horizontal distance)
    /// - State: [v, n, alpha, t]
    /// - Control: [k] curvature
    ///
    /// Key dynamics:
    /// - dv/ds = g*tan(alpha)/v (speed increases due to gravity)
    /// - dn/ds = tan(alpha) (vertical descent rate)
    /// - dalpha/ds = k (curvature is the control)
    /// - dt/ds = 1/(v*cos(alpha)) (time rate)
    /// </summary>
    [TestClass]
    public sealed class BrachistochroneDynamicsTests
    {
        private const double Gravity = 9.80665;

        #region TimeRateS Tests

        [TestMethod]
        public void TimeRateSIsPositiveForPositiveVelocityAndSmallAngle()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0; // 30 degrees

            var dtds = BrachistochroneDynamics.TimeRateS(v, alpha);

            Assert.IsGreaterThan(0.0, dtds, "Time rate should be positive");
        }

        [TestMethod]
        public void TimeRateSDecreasesWithHigherVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dtds_slow = BrachistochroneDynamics.TimeRateS(2.0, alpha);
            var dtds_fast = BrachistochroneDynamics.TimeRateS(10.0, alpha);

            Assert.IsGreaterThan(dtds_fast, dtds_slow, "Time rate should decrease with higher velocity");
        }

        [TestMethod]
        public void TimeRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            // Should not throw or return NaN for very small velocity
            var dtds = BrachistochroneDynamics.TimeRateS(0.001, alpha);

            Assert.IsFalse(double.IsNaN(dtds), "Time rate should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dtds), "Time rate should not be infinite for small velocity");
        }

        [TestMethod]
        public void TimeRateSHandlesLargeAngle()
        {
            var v = 5.0;

            // Near vertical angle - should be handled gracefully
            var dtds = BrachistochroneDynamics.TimeRateS(v, Math.PI / 2.5);

            Assert.IsFalse(double.IsNaN(dtds), "Time rate should not be NaN for large angle");
            Assert.IsFalse(double.IsInfinity(dtds), "Time rate should not be infinite for large angle");
        }

        #endregion

        #region SpeedRateS Tests

        [TestMethod]
        public void SpeedRateSIsPositiveForDescendingAngle()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0; // 30 degrees (descending)

            var dvds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);

            Assert.IsGreaterThan(0.0, dvds, "Speed should increase when descending");
        }

        [TestMethod]
        public void SpeedRateSIsZeroForHorizontalAngle()
        {
            var v = 5.0;
            var alpha = 0.0; // Horizontal

            var dvds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);

            Assert.AreEqual(0.0, dvds, 0.001, "Speed rate should be zero for horizontal motion");
        }

        [TestMethod]
        public void SpeedRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneDynamics.SpeedRateS(0.001, alpha, Gravity);

            Assert.IsFalse(double.IsNaN(dvds), "Speed rate should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dvds), "Speed rate should not be infinite for small velocity");
        }

        #endregion

        #region VerticalRateS Tests

        [TestMethod]
        public void VerticalRateSIsPositiveForDescendingAngle()
        {
            var alpha = Math.PI / 6.0; // 30 degrees

            var dnds = BrachistochroneDynamics.VerticalRateS(alpha);

            Assert.IsGreaterThan(0.0, dnds, "Should descend for positive angle");
        }

        [TestMethod]
        public void VerticalRateSIsZeroForHorizontalAngle()
        {
            var dnds = BrachistochroneDynamics.VerticalRateS(0.0);

            Assert.AreEqual(0.0, dnds, 1e-10, "No descent for horizontal motion");
        }

        [TestMethod]
        public void VerticalRateSMatchesTangent()
        {
            var alpha = Math.PI / 4.0; // 45 degrees

            var dnds = BrachistochroneDynamics.VerticalRateS(alpha);
            var expected = Math.Tan(alpha);

            Assert.AreEqual(expected, dnds, 1e-10, "Vertical rate should equal tan(alpha)");
        }

        #endregion

        #region AlphaRateS Tests

        [TestMethod]
        public void AlphaRateSEqualsCurvature()
        {
            var k = 0.5;

            var dalphads = BrachistochroneDynamics.AlphaRateS(k);

            Assert.AreEqual(k, dalphads, 1e-10, "Alpha rate should equal curvature");
        }

        [TestMethod]
        public void AlphaRateSNegativeForNegativeCurvature()
        {
            var k = -0.3;

            var dalphads = BrachistochroneDynamics.AlphaRateS(k);

            Assert.AreEqual(k, dalphads, 1e-10, "Alpha rate should equal negative curvature");
        }

        #endregion

        #region RunningCostS Tests

        [TestMethod]
        public void RunningCostSEqualsTimeRate()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var cost = BrachistochroneDynamics.RunningCostS(v, alpha);
            var timeRate = BrachistochroneDynamics.TimeRateS(v, alpha);

            Assert.AreEqual(timeRate, cost, 1e-10, "Running cost should equal time rate");
        }

        [TestMethod]
        public void RunningCostSIntegratesCorrectly()
        {
            // For a simple case: constant v and alpha over horizontal distance s
            // Time = integral of dt/ds from 0 to s = s / (v * cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var horizontalDistance = 10.0;

            var costRate = BrachistochroneDynamics.RunningCostS(v, alpha);
            var integratedTime = costRate * horizontalDistance;
            var expectedTime = horizontalDistance / (v * Math.Cos(alpha));

            Assert.AreEqual(expectedTime, integratedTime, 0.01, "Integrated cost should equal expected time");
        }

        #endregion

        #region Energy Conservation Tests

        [TestMethod]
        public void DynamicsSatisfyEnergyConservation()
        {
            // Energy conservation: d(0.5*v² - g*n)/ds = 0
            // This means: v*(dv/ds) - g*(dn/ds) = 0
            // Substituting: v * g*tan(alpha)/v - g*tan(alpha) = 0

            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
            var dnds = BrachistochroneDynamics.VerticalRateS(alpha);

            // d(KE)/ds = v * dv/ds
            var dKE_ds = v * dvds;

            // d(PE)/ds = g * dn/ds (PE = g*n since n is positive downward)
            var dPE_ds = Gravity * dnds;

            // Energy should be conserved: d(KE)/ds = d(PE)/ds
            // (kinetic energy gain = potential energy loss)
            Assert.AreEqual(dPE_ds, dKE_ds, 0.01, "Energy should be conserved: dKE/ds = dPE/ds");
        }

        [TestMethod]
        public void EnergyConservationHoldsForVariousAngles()
        {
            var v = 3.0;

            foreach (var alphaDeg in new[] { 10.0, 20.0, 30.0, 45.0, 60.0 })
            {
                var alpha = alphaDeg * Math.PI / 180.0;

                var dvds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
                var dnds = BrachistochroneDynamics.VerticalRateS(alpha);

                var dKE_ds = v * dvds;
                var dPE_ds = Gravity * dnds;

                Assert.AreEqual(dPE_ds, dKE_ds, 0.01,
                    $"Energy conservation failed for alpha = {alphaDeg} degrees");
            }
        }

        #endregion

        #region Numerical Gradient Verification

        [TestMethod]
        public void SpeedRateSGradientIsCorrect()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var eps = 1e-7;

            var f0 = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);

            // Numerical gradient w.r.t. v
            var fv = BrachistochroneDynamics.SpeedRateS(v + eps, alpha, Gravity);
            var dvds_dv_numerical = (fv - f0) / eps;

            // Analytical: d/dv[g*tan(alpha)/v] = -g*tan(alpha)/v²
            var dvds_dv_analytical = -Gravity * Math.Tan(alpha) / (v * v);

            Assert.AreEqual(dvds_dv_analytical, dvds_dv_numerical, 1e-4, "Gradient w.r.t. v incorrect");

            // Numerical gradient w.r.t. alpha
            var falpha = BrachistochroneDynamics.SpeedRateS(v, alpha + eps, Gravity);
            var dvds_dalpha_numerical = (falpha - f0) / eps;

            // Analytical: d/dalpha[g*tan(alpha)/v] = g*sec²(alpha)/v
            var cosAlpha = Math.Cos(alpha);
            var dvds_dalpha_analytical = Gravity / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(dvds_dalpha_analytical, dvds_dalpha_numerical, 1e-4, "Gradient w.r.t. alpha incorrect");
        }

        [TestMethod]
        public void TimeRateSGradientIsCorrect()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var eps = 1e-7;

            var f0 = BrachistochroneDynamics.TimeRateS(v, alpha);

            // Numerical gradient w.r.t. v
            var fv = BrachistochroneDynamics.TimeRateS(v + eps, alpha);
            var dtds_dv_numerical = (fv - f0) / eps;

            // Analytical: d/dv[1/(v*cos(alpha))] = -1/(v²*cos(alpha))
            var cosAlpha = Math.Cos(alpha);
            var dtds_dv_analytical = -1.0 / (v * v * cosAlpha);

            Assert.AreEqual(dtds_dv_analytical, dtds_dv_numerical, 1e-4, "Gradient w.r.t. v incorrect");

            // Numerical gradient w.r.t. alpha
            var falpha = BrachistochroneDynamics.TimeRateS(v, alpha + eps);
            var dtds_dalpha_numerical = (falpha - f0) / eps;

            // Analytical: d/dalpha[1/(v*cos(alpha))] = sin(alpha)/(v*cos²(alpha))
            var sinAlpha = Math.Sin(alpha);
            var dtds_dalpha_analytical = sinAlpha / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(dtds_dalpha_analytical, dtds_dalpha_numerical, 1e-4, "Gradient w.r.t. alpha incorrect");
        }

        #endregion
    }
}
