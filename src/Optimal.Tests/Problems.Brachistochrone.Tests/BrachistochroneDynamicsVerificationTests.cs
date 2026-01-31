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
    /// Verification tests for arc-length parameterized Brachistochrone dynamics.
    ///
    /// Arc-length formulation:
    /// - Independent variable: s (horizontal distance)
    /// - State: [v, n, alpha, t]
    /// - Control: [k] curvature
    ///
    /// These tests verify:
    /// 1. Physical dynamics values are correct
    /// 2. Numerical gradients match expected analytical gradients
    /// 3. Energy conservation is satisfied
    /// </summary>
    [TestClass]
    public sealed class BrachistochroneDynamicsVerificationTests
    {
        private const double Gravity = 9.80665;
        private const double Epsilon = 1e-7;
        private const double GradientTolerance = 1e-5;

        #region Physical Dynamics Values

        [TestMethod]
        public void TimeRateSIsCorrectForKnownInputs()
        {
            // dt/ds = 1/(v*cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0; // 30 degrees
            var expected = 1.0 / (v * Math.Cos(alpha));

            var actual = BrachistochroneDynamics.TimeRateS(v, alpha);

            Assert.AreEqual(expected, actual, 1e-10, "TimeRateS should be 1/(v*cos(alpha))");
        }

        [TestMethod]
        public void SpeedRateSIsCorrectForKnownInputs()
        {
            // dv/ds = g*sin(alpha)/(v*cos(alpha)) = g*tan(alpha)/v
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var expected = Gravity * Math.Tan(alpha) / v;

            var actual = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);

            Assert.AreEqual(expected, actual, 1e-10, "SpeedRateS should be g*tan(alpha)/v");
        }

        [TestMethod]
        public void VerticalRateSIsCorrectForKnownInputs()
        {
            // dn/ds = tan(alpha)
            var alpha = Math.PI / 4.0; // 45 degrees
            var expected = Math.Tan(alpha);

            var actual = BrachistochroneDynamics.VerticalRateS(alpha);

            Assert.AreEqual(expected, actual, 1e-10, "VerticalRateS should be tan(alpha)");
        }

        [TestMethod]
        public void AlphaRateSIsCorrectForKnownInputs()
        {
            // dalpha/ds = k (curvature)
            var k = 0.3;
            var expected = k;

            var actual = BrachistochroneDynamics.AlphaRateS(k);

            Assert.AreEqual(expected, actual, 1e-10, "AlphaRateS should equal curvature k");
        }

        [TestMethod]
        public void RunningCostSIsCorrectForKnownInputs()
        {
            // Running cost = dt/ds = 1/(v*cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var expected = 1.0 / (v * Math.Cos(alpha));

            var actual = BrachistochroneDynamics.RunningCostS(v, alpha);

            Assert.AreEqual(expected, actual, 1e-10, "RunningCostS should equal TimeRateS");
        }

        [TestMethod]
        public void SpeedRateSIsZeroForHorizontalMotion()
        {
            // When alpha = 0 (horizontal), no gravity component, dv/ds = 0
            var v = 5.0;
            var alpha = 0.0;

            var actual = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);

            Assert.AreEqual(0.0, actual, 1e-10, "SpeedRateS should be zero for horizontal motion");
        }

        [TestMethod]
        public void VerticalRateSIsZeroForHorizontalMotion()
        {
            // When alpha = 0, dn/ds = tan(0) = 0
            var alpha = 0.0;

            var actual = BrachistochroneDynamics.VerticalRateS(alpha);

            Assert.AreEqual(0.0, actual, 1e-10, "VerticalRateS should be zero for horizontal motion");
        }

        #endregion

        #region Numerical Safeguards

        [TestMethod]
        public void TimeRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dtds = BrachistochroneDynamics.TimeRateS(0.001, alpha);

            Assert.IsFalse(double.IsNaN(dtds), "TimeRateS should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dtds), "TimeRateS should not be infinite for small velocity");
            Assert.IsGreaterThan(0.0, dtds, "TimeRateS should be positive");
        }

        [TestMethod]
        public void SpeedRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneDynamics.SpeedRateS(0.001, alpha, Gravity);

            Assert.IsFalse(double.IsNaN(dvds), "SpeedRateS should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dvds), "SpeedRateS should not be infinite for small velocity");
        }

        [TestMethod]
        public void TimeRateSHandlesLargeAngle()
        {
            var v = 5.0;
            var alpha = Math.PI / 2.5; // Large angle near singularity

            var dtds = BrachistochroneDynamics.TimeRateS(v, alpha);

            Assert.IsFalse(double.IsNaN(dtds), "TimeRateS should not be NaN for large angle");
            Assert.IsFalse(double.IsInfinity(dtds), "TimeRateS should not be infinite for large angle");
        }

        [TestMethod]
        public void RunningCostSHandlesEdgeCases()
        {
            // Small velocity
            var cost1 = BrachistochroneDynamics.RunningCostS(0.001, Math.PI / 6.0);
            Assert.IsFalse(double.IsNaN(cost1), "RunningCostS should handle small velocity");
            Assert.IsFalse(double.IsInfinity(cost1), "RunningCostS should be finite for small velocity");

            // Large angle
            var cost2 = BrachistochroneDynamics.RunningCostS(5.0, Math.PI / 2.5);
            Assert.IsFalse(double.IsNaN(cost2), "RunningCostS should handle large angle");
            Assert.IsFalse(double.IsInfinity(cost2), "RunningCostS should be finite for large angle");
        }

        #endregion

        #region Energy Conservation

        [TestMethod]
        public void DynamicsSatisfyEnergyConservation()
        {
            // Energy: E = 0.5*v² - g*n (kinetic + potential, n positive = descended)
            // dE/ds = v*(dv/ds) - g*(dn/ds)
            //       = v * [g*tan(alpha)/v] - g * tan(alpha)
            //       = g*tan(alpha) - g*tan(alpha) = 0
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
            var dnds = BrachistochroneDynamics.VerticalRateS(alpha);

            var dKE_ds = v * dvds;
            var dPE_ds = Gravity * dnds;

            Assert.AreEqual(dPE_ds, dKE_ds, 1e-6, "Energy should be conserved: dKE/ds = dPE/ds");
        }

        [TestMethod]
        public void EnergyConservationHoldsForVariousStates()
        {
            foreach (var v in new[] { 1.0, 3.0, 5.0, 10.0 })
            {
                foreach (var alphaDeg in new[] { 10.0, 20.0, 30.0, 45.0, 60.0 })
                {
                    var alpha = alphaDeg * Math.PI / 180.0;

                    var dvds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
                    var dnds = BrachistochroneDynamics.VerticalRateS(alpha);

                    var dKE_ds = v * dvds;
                    var dPE_ds = Gravity * dnds;

                    Assert.AreEqual(dPE_ds, dKE_ds, 1e-6,
                        $"Energy conservation failed for v={v}, alpha={alphaDeg} degrees");
                }
            }
        }

        #endregion

        #region Gradient Verification

        [TestMethod]
        public void SpeedRateSGradientWrtVelocity()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
            var fPlus = BrachistochroneDynamics.SpeedRateS(v + Epsilon, alpha, Gravity);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dv[g*tan(alpha)/v] = -g*tan(alpha)/v²
            var analyticalGradient = -Gravity * Math.Tan(alpha) / (v * v);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "SpeedRateS gradient w.r.t. v is incorrect");
        }

        [TestMethod]
        public void SpeedRateSGradientWrtAlpha()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
            var fPlus = BrachistochroneDynamics.SpeedRateS(v, alpha + Epsilon, Gravity);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dalpha[g*tan(alpha)/v] = g*sec²(alpha)/v
            var cosAlpha = Math.Cos(alpha);
            var analyticalGradient = Gravity / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "SpeedRateS gradient w.r.t. alpha is incorrect");
        }

        [TestMethod]
        public void TimeRateSGradientWrtVelocity()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneDynamics.TimeRateS(v, alpha);
            var fPlus = BrachistochroneDynamics.TimeRateS(v + Epsilon, alpha);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dv[1/(v*cos(alpha))] = -1/(v²*cos(alpha))
            var cosAlpha = Math.Cos(alpha);
            var analyticalGradient = -1.0 / (v * v * cosAlpha);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "TimeRateS gradient w.r.t. v is incorrect");
        }

        [TestMethod]
        public void TimeRateSGradientWrtAlpha()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneDynamics.TimeRateS(v, alpha);
            var fPlus = BrachistochroneDynamics.TimeRateS(v, alpha + Epsilon);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dalpha[1/(v*cos(alpha))] = sin(alpha)/(v*cos²(alpha))
            var cosAlpha = Math.Cos(alpha);
            var sinAlpha = Math.Sin(alpha);
            var analyticalGradient = sinAlpha / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "TimeRateS gradient w.r.t. alpha is incorrect");
        }

        [TestMethod]
        public void VerticalRateSGradientWrtAlpha()
        {
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneDynamics.VerticalRateS(alpha);
            var fPlus = BrachistochroneDynamics.VerticalRateS(alpha + Epsilon);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dalpha[tan(alpha)] = sec²(alpha) = 1/cos²(alpha)
            var cosAlpha = Math.Cos(alpha);
            var analyticalGradient = 1.0 / (cosAlpha * cosAlpha);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "VerticalRateS gradient w.r.t. alpha is incorrect");
        }

        #endregion

        #region Running Cost Integration

        [TestMethod]
        public void RunningCostIntegratesCorrectlyForConstantState()
        {
            // For constant v and alpha over horizontal distance s:
            // Time = integral of dt/ds from 0 to s = s / (v * cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var horizontalDistance = 10.0;

            var costRate = BrachistochroneDynamics.RunningCostS(v, alpha);
            var integratedTime = costRate * horizontalDistance;
            var expectedTime = horizontalDistance / (v * Math.Cos(alpha));

            Assert.AreEqual(expectedTime, integratedTime, 1e-10,
                "Integrated running cost should equal expected time");
        }

        [TestMethod]
        public void RunningCostEqualsTimeRate()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var runningCost = BrachistochroneDynamics.RunningCostS(v, alpha);
            var timeRate = BrachistochroneDynamics.TimeRateS(v, alpha);

            Assert.AreEqual(timeRate, runningCost, 1e-10,
                "Running cost should equal time rate");
        }

        #endregion

        #region Complete Dynamics Function Test

        [TestMethod]
        public void CompleteDynamicsFunctionReturnsCorrectValues()
        {
            // State: [v, n, alpha, t]
            var state = new double[] { 5.0, 2.0, Math.PI / 6.0, 0.5 };
            var control = new double[] { 0.1 }; // k = curvature

            var v = state[0];
            var alpha = state[2];
            var k = control[0];

            // Compute all dynamics
            var dVds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
            var dNds = BrachistochroneDynamics.VerticalRateS(alpha);
            var dAlphads = BrachistochroneDynamics.AlphaRateS(k);
            var dTds = BrachistochroneDynamics.TimeRateS(v, alpha);

            // Verify individual values
            Assert.AreEqual(Gravity * Math.Tan(alpha) / v, dVds, 1e-10, "dv/ds value");
            Assert.AreEqual(Math.Tan(alpha), dNds, 1e-10, "dn/ds value");
            Assert.AreEqual(k, dAlphads, 1e-10, "dalpha/ds value");
            Assert.AreEqual(1.0 / (v * Math.Cos(alpha)), dTds, 1e-10, "dt/ds value");
        }

        [TestMethod]
        public void NumericalGradientsMatchForCompleteDynamics()
        {
            var state = new double[] { 5.0, 2.0, Math.PI / 6.0, 0.5 };
            var control = new double[] { 0.1 };

            // Compute base derivatives
            var f0 = ComputeDerivatives(state, control);

            // Numerical gradients for dv/ds w.r.t. v (state[0])
            var statePlusV = (double[])state.Clone();
            statePlusV[0] += Epsilon;
            var fPlusV = ComputeDerivatives(statePlusV, control);
            var dfdv_numerical = (fPlusV[0] - f0[0]) / Epsilon;

            // Analytical: d(g*tan(alpha)/v)/dv = -g*tan(alpha)/v²
            var v = state[0];
            var alpha = state[2];
            var dfdv_analytical = -Gravity * Math.Tan(alpha) / (v * v);

            Assert.AreEqual(dfdv_analytical, dfdv_numerical, GradientTolerance,
                "d(dv/ds)/dv numerical gradient mismatch");

            // Numerical gradient for dalpha/ds w.r.t. k (control[0])
            var controlPlusK = (double[])control.Clone();
            controlPlusK[0] += Epsilon;
            var fPlusK = ComputeDerivatives(state, controlPlusK);
            var dfdk_numerical = (fPlusK[2] - f0[2]) / Epsilon;

            // Analytical: d(k)/dk = 1
            Assert.AreEqual(1.0, dfdk_numerical, GradientTolerance,
                "d(dalpha/ds)/dk should be 1");
        }

        private static double[] ComputeDerivatives(double[] state, double[] control)
        {
            var v = state[0];
            var alpha = state[2];
            var k = control[0];

            return new[]
            {
                BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity),
                BrachistochroneDynamics.VerticalRateS(alpha),
                BrachistochroneDynamics.AlphaRateS(k),
                BrachistochroneDynamics.TimeRateS(v, alpha)
            };
        }

        #endregion
    }
}
