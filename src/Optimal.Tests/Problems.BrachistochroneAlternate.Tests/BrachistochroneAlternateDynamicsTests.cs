/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using OptimalCli.Problems.BrachistochroneAlternate;

namespace Optimal.Problems.BrachistochroneAlternate.Tests
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
    public sealed class BrachistochroneAlternateDynamicsTests
    {
        private const double Gravity = 9.80665;
        private static readonly double ThetaRef = BrachistochroneAlternateDynamics.ThetaRef;

        #region TimeRateS Tests

        [TestMethod]
        public void TimeRateSIsPositiveForPositiveVelocityAndSmallAngle()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0; // 30 degrees

            var dtds = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

            Assert.IsGreaterThan(0.0, dtds, "Time rate should be positive");
        }

        [TestMethod]
        public void TimeRateSDecreasesWithHigherVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dtds_slow = BrachistochroneAlternateDynamics.TimeRateS(2.0, alpha);
            var dtds_fast = BrachistochroneAlternateDynamics.TimeRateS(10.0, alpha);

            Assert.IsGreaterThan(dtds_fast, dtds_slow, "Time rate should decrease with higher velocity");
        }

        [TestMethod]
        public void TimeRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            // Should not throw or return NaN for very small velocity
            var dtds = BrachistochroneAlternateDynamics.TimeRateS(0.001, alpha);

            Assert.IsFalse(double.IsNaN(dtds), "Time rate should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dtds), "Time rate should not be infinite for small velocity");
        }

        [TestMethod]
        public void TimeRateSHandlesLargeAngle()
        {
            var v = 5.0;

            // Near vertical angle - should be handled gracefully
            var dtds = BrachistochroneAlternateDynamics.TimeRateS(v, Math.PI / 2.5);

            Assert.IsFalse(double.IsNaN(dtds), "Time rate should not be NaN for large angle");
            Assert.IsFalse(double.IsInfinity(dtds), "Time rate should not be infinite for large angle");
        }

        #endregion

        #region SpeedRateS Tests

        [TestMethod]
        public void SpeedRateSIsPositiveForDescendingAngle()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0; // 30 degrees (descending relative to reference line)

            var dvds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);

            Assert.IsGreaterThan(0.0, dvds, "Speed should increase when descending");
        }

        [TestMethod]
        public void SpeedRateSIsPositiveForZeroAlpha()
        {
            // With the rotated coordinate system, alpha=0 means following the reference line
            // which still has a downward slope of ThetaRef, so speed should still increase
            var v = 5.0;
            var alpha = 0.0; // Following reference line

            var dvds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);

            // Since ThetaRef > 0, the world angle is ThetaRef, so we're still descending
            Assert.IsGreaterThan(0.0, dvds, "Speed should increase when following reference line (downward slope)");
        }

        [TestMethod]
        public void SpeedRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneAlternateDynamics.SpeedRateS(0.001, alpha, Gravity, ThetaRef);

            Assert.IsFalse(double.IsNaN(dvds), "Speed rate should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dvds), "Speed rate should not be infinite for small velocity");
        }

        #endregion

        #region VerticalRateS Tests

        [TestMethod]
        public void VerticalRateSIsPositiveForDescendingAngle()
        {
            var alpha = Math.PI / 6.0; // 30 degrees

            var dnds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

            Assert.IsGreaterThan(0.0, dnds, "Should descend for positive angle");
        }

        [TestMethod]
        public void VerticalRateSIsZeroForHorizontalAngle()
        {
            var dnds = BrachistochroneAlternateDynamics.VerticalRateS(0.0);

            Assert.AreEqual(0.0, dnds, 1e-10, "No descent for horizontal motion");
        }

        [TestMethod]
        public void VerticalRateSMatchesTangent()
        {
            var alpha = Math.PI / 4.0; // 45 degrees

            var dnds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);
            var expected = Math.Tan(alpha);

            Assert.AreEqual(expected, dnds, 1e-10, "Vertical rate should equal tan(alpha)");
        }

        #endregion

        #region AlphaRateS Tests

        [TestMethod]
        public void AlphaRateSEqualsCurvature()
        {
            var k = 0.5;

            var dalphads = BrachistochroneAlternateDynamics.AlphaRateS(k);

            Assert.AreEqual(k, dalphads, 1e-10, "Alpha rate should equal curvature");
        }

        [TestMethod]
        public void AlphaRateSNegativeForNegativeCurvature()
        {
            var k = -0.3;

            var dalphads = BrachistochroneAlternateDynamics.AlphaRateS(k);

            Assert.AreEqual(k, dalphads, 1e-10, "Alpha rate should equal negative curvature");
        }

        #endregion

        #region RunningCostS Tests

        [TestMethod]
        public void RunningCostSEqualsTimeRate()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var cost = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);
            var timeRate = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

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

            var costRate = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);
            var integratedTime = costRate * horizontalDistance;
            var expectedTime = horizontalDistance / (v * Math.Cos(alpha));

            Assert.AreEqual(expectedTime, integratedTime, 0.01, "Integrated cost should equal expected time");
        }

        #endregion

        #region Energy Conservation Tests

        [TestMethod]
        public void DynamicsSatisfyEnergyConservation()
        {
            // In the rotated coordinate system:
            // The actual vertical drop rate (in Cartesian) is dy_down/ds
            // where y_down = s * sin(ThetaRef) + n * cos(ThetaRef)
            // So: dy_down/ds = sin(ThetaRef) + cos(ThetaRef) * dn/ds
            //                = sin(ThetaRef) + cos(ThetaRef) * tan(alpha)
            //
            // Energy conservation: d(0.5*v²)/ds = g * dy_down/ds
            // i.e., v * dv/ds = g * [sin(ThetaRef) + cos(ThetaRef) * tan(alpha)]

            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var dnds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

            // d(KE)/ds = v * dv/ds
            var dKE_ds = v * dvds;

            // dy_down/ds = sin(ThetaRef) + cos(ThetaRef) * dn/ds
            var dy_down_ds = Math.Sin(ThetaRef) + Math.Cos(ThetaRef) * dnds;

            // d(PE)/ds = g * dy_down/ds (potential energy change)
            var dPE_ds = Gravity * dy_down_ds;

            // Energy should be conserved: d(KE)/ds = d(PE)/ds
            Assert.AreEqual(dPE_ds, dKE_ds, 0.01, "Energy should be conserved: dKE/ds = dPE/ds");
        }

        [TestMethod]
        public void EnergyConservationHoldsForVariousAngles()
        {
            var v = 3.0;

            foreach (var alphaDeg in new[] { 10.0, 20.0, 30.0, 45.0, 60.0 })
            {
                var alpha = alphaDeg * Math.PI / 180.0;

                var dvds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
                var dnds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

                var dKE_ds = v * dvds;
                var dy_down_ds = Math.Sin(ThetaRef) + Math.Cos(ThetaRef) * dnds;
                var dPE_ds = Gravity * dy_down_ds;

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

            var f0 = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);

            // Numerical gradient w.r.t. v
            var fv = BrachistochroneAlternateDynamics.SpeedRateS(v + eps, alpha, Gravity, ThetaRef);
            var dvds_dv_numerical = (fv - f0) / eps;

            // Analytical: d/dv[g*sin(alpha+ThetaRef)/(v*cos(alpha))] = -g*sin(alpha+ThetaRef)/(v²*cos(alpha))
            var worldAlpha = alpha + ThetaRef;
            var cosAlpha = Math.Cos(alpha);
            var dvds_dv_analytical = -Gravity * Math.Sin(worldAlpha) / (v * v * cosAlpha);

            Assert.AreEqual(dvds_dv_analytical, dvds_dv_numerical, 1e-4, "Gradient w.r.t. v incorrect");

            // Numerical gradient w.r.t. alpha
            var falpha = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha + eps, Gravity, ThetaRef);
            var dvds_dalpha_numerical = (falpha - f0) / eps;

            // Analytical: d/dalpha[g*sin(alpha+ThetaRef)/(v*cos(alpha))]
            // Using quotient rule: d/dalpha[sin(A+T)/cos(A)] where A=alpha, T=ThetaRef
            // = [cos(A+T)*cos(A) + sin(A+T)*sin(A)] / (v*cos²(A))
            // = cos(T) / (v*cos²(A))  using cos(A+T)*cos(A) + sin(A+T)*sin(A) = cos(T)
            var sinAlpha = Math.Sin(alpha);
            var dvds_dalpha_analytical = Gravity * (Math.Cos(worldAlpha) * cosAlpha + Math.Sin(worldAlpha) * sinAlpha) / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(dvds_dalpha_analytical, dvds_dalpha_numerical, 1e-4, "Gradient w.r.t. alpha incorrect");
        }

        [TestMethod]
        public void TimeRateSGradientIsCorrect()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var eps = 1e-7;

            var f0 = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

            // Numerical gradient w.r.t. v
            var fv = BrachistochroneAlternateDynamics.TimeRateS(v + eps, alpha);
            var dtds_dv_numerical = (fv - f0) / eps;

            // Analytical: d/dv[1/(v*cos(alpha))] = -1/(v²*cos(alpha))
            var cosAlpha = Math.Cos(alpha);
            var dtds_dv_analytical = -1.0 / (v * v * cosAlpha);

            Assert.AreEqual(dtds_dv_analytical, dtds_dv_numerical, 1e-4, "Gradient w.r.t. v incorrect");

            // Numerical gradient w.r.t. alpha
            var falpha = BrachistochroneAlternateDynamics.TimeRateS(v, alpha + eps);
            var dtds_dalpha_numerical = (falpha - f0) / eps;

            // Analytical: d/dalpha[1/(v*cos(alpha))] = sin(alpha)/(v*cos²(alpha))
            var sinAlpha = Math.Sin(alpha);
            var dtds_dalpha_analytical = sinAlpha / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(dtds_dalpha_analytical, dtds_dalpha_numerical, 1e-4, "Gradient w.r.t. alpha incorrect");
        }

        #endregion

        #region Clamping Behavior Tests

        [TestMethod]
        public void GradientIsNonZeroAboveVelocityThreshold()
        {
            // Test that gradients are non-zero when velocity is ABOVE the clamping threshold (v > 0.01)
            var alpha = Math.PI / 6.0;
            var eps = 1e-6;

            // Test at velocities above the threshold
            var velocities = new[] { 0.02, 0.05, 0.1, 0.5, 1.0, 5.0 };

            foreach (var v in velocities)
            {
                // Compute central difference gradient
                var fPlus = BrachistochroneAlternateDynamics.TimeRateS(v + eps, alpha);
                var fMinus = BrachistochroneAlternateDynamics.TimeRateS(v - eps, alpha);
                var gradient = (fPlus - fMinus) / (2.0 * eps);

                // Gradient should be non-zero (negative, since time rate decreases with velocity)
                Assert.AreNotEqual(0.0, gradient,
                    $"Gradient should be non-zero at v={v}. Got gradient={gradient:E4}");

                // Gradient should be negative (increasing v decreases time rate)
                Assert.IsLessThan(0.0, gradient,
                    $"Gradient should be negative at v={v}. Got gradient={gradient:E4}");

                Console.WriteLine($"v={v:F3}, gradient={gradient:E4}");
            }
        }

        [TestMethod]
        public void GradientIsNonZeroAtValidAlphaAngles()
        {
            // Test that gradients are non-zero at valid angles (|cos(alpha)| > 0.1)
            // Note: At alpha=0, the gradient w.r.t. alpha is sin(alpha)/(v*cos²(alpha)) = 0,
            // so we skip alpha=0 and test other angles.
            var v = 5.0;
            var eps = 1e-6;

            // Test angles where cos(alpha) is well above the threshold
            // and sin(alpha) is non-zero
            var angles = new[]
            {
                Math.PI / 6.0,  // cos ≈ 0.866, sin ≈ 0.5
                Math.PI / 4.0,  // cos ≈ 0.707, sin ≈ 0.707
                Math.PI / 3.0,  // cos = 0.5, sin ≈ 0.866
                1.2,            // cos ≈ 0.362, sin ≈ 0.932
            };

            foreach (var alpha in angles)
            {
                // Compute central difference gradient
                var fPlus = BrachistochroneAlternateDynamics.TimeRateS(v, alpha + eps);
                var fMinus = BrachistochroneAlternateDynamics.TimeRateS(v, alpha - eps);
                var gradient = (fPlus - fMinus) / (2.0 * eps);

                // Gradient should be non-zero
                Assert.AreNotEqual(0.0, gradient,
                    $"Gradient should be non-zero at alpha={alpha:F3} rad ({alpha * 180 / Math.PI:F1}°). " +
                    $"Got gradient={gradient:E4}");

                Console.WriteLine($"alpha={alpha:F3} rad ({alpha * 180 / Math.PI:F1}°), cos(alpha)={Math.Cos(alpha):F3}, gradient={gradient:E4}");
            }
        }

        [TestMethod]
        public void SpeedRateSGradientIsNonZeroAboveVelocityThreshold()
        {
            // Test that SpeedRateS gradients are non-zero above the velocity threshold
            var alpha = Math.PI / 6.0;
            var eps = 1e-6;

            var velocities = new[] { 0.02, 0.05, 0.1, 0.5, 1.0, 5.0 };

            foreach (var v in velocities)
            {
                var fPlus = BrachistochroneAlternateDynamics.SpeedRateS(v + eps, alpha, Gravity, ThetaRef);
                var fMinus = BrachistochroneAlternateDynamics.SpeedRateS(v - eps, alpha, Gravity, ThetaRef);
                var gradient = (fPlus - fMinus) / (2.0 * eps);

                // Gradient should be non-zero
                Assert.AreNotEqual(0.0, gradient,
                    $"SpeedRateS gradient should be non-zero at v={v}. Got gradient={gradient:E4}");

                Console.WriteLine($"SpeedRateS: v={v:F3}, gradient={gradient:E4}");
            }
        }

        #endregion
    }
}
