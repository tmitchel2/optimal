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
    public sealed class BrachistochroneAlternateDynamicsVerificationTests
    {
        private const double Gravity = 9.80665;
        private const double Epsilon = 1e-7;
        private const double GradientTolerance = 1e-5;
        private static readonly double ThetaRef = BrachistochroneAlternateDynamics.ThetaRef;

        #region Physical Dynamics Values

        [TestMethod]
        public void TimeRateSIsCorrectForKnownInputs()
        {
            // dt/ds = 1/(v*cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0; // 30 degrees
            var expected = 1.0 / (v * Math.Cos(alpha));

            var actual = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

            Assert.AreEqual(expected, actual, 1e-10, "TimeRateS should be 1/(v*cos(alpha))");
        }

        [TestMethod]
        public void SpeedRateSIsCorrectForKnownInputs()
        {
            // dv/ds = g*sin(alpha+ThetaRef)/(v*cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var expected = Gravity * Math.Sin(alpha + ThetaRef) / (v * Math.Cos(alpha));

            var actual = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);

            Assert.AreEqual(expected, actual, 1e-10, "SpeedRateS should be g*sin(alpha+ThetaRef)/(v*cos(alpha))");
        }

        [TestMethod]
        public void VerticalRateSIsCorrectForKnownInputs()
        {
            // dn/ds = tan(alpha)
            var alpha = Math.PI / 4.0; // 45 degrees
            var expected = Math.Tan(alpha);

            var actual = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

            Assert.AreEqual(expected, actual, 1e-10, "VerticalRateS should be tan(alpha)");
        }

        [TestMethod]
        public void AlphaRateSIsCorrectForKnownInputs()
        {
            // dalpha/ds = k (curvature)
            var k = 0.3;
            var expected = k;

            var actual = BrachistochroneAlternateDynamics.AlphaRateS(k);

            Assert.AreEqual(expected, actual, 1e-10, "AlphaRateS should equal curvature k");
        }

        [TestMethod]
        public void RunningCostSIsCorrectForKnownInputs()
        {
            // Running cost = dt/ds = 1/(v*cos(alpha))
            var v = 5.0;
            var alpha = Math.PI / 6.0;
            var expected = 1.0 / (v * Math.Cos(alpha));

            var actual = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);

            Assert.AreEqual(expected, actual, 1e-10, "RunningCostS should equal TimeRateS");
        }

        [TestMethod]
        public void SpeedRateSIsPositiveForZeroAlpha()
        {
            // When alpha = 0, we're following the reference line which has slope ThetaRef
            // So the world angle is ThetaRef, which means we're still descending
            var v = 5.0;
            var alpha = 0.0;

            var actual = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var expected = Gravity * Math.Sin(ThetaRef) / v;

            Assert.AreEqual(expected, actual, 1e-10, "SpeedRateS should be g*sin(ThetaRef)/v for alpha=0");
            Assert.IsGreaterThan(0.0, actual, "SpeedRateS should be positive when following reference line");
        }

        [TestMethod]
        public void VerticalRateSIsZeroForHorizontalMotion()
        {
            // When alpha = 0, dn/ds = tan(0) = 0
            var alpha = 0.0;

            var actual = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

            Assert.AreEqual(0.0, actual, 1e-10, "VerticalRateS should be zero for horizontal motion");
        }

        #endregion

        #region Numerical Safeguards

        [TestMethod]
        public void TimeRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dtds = BrachistochroneAlternateDynamics.TimeRateS(0.001, alpha);

            Assert.IsFalse(double.IsNaN(dtds), "TimeRateS should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dtds), "TimeRateS should not be infinite for small velocity");
            Assert.IsGreaterThan(0.0, dtds, "TimeRateS should be positive");
        }

        [TestMethod]
        public void SpeedRateSHandlesSmallVelocity()
        {
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneAlternateDynamics.SpeedRateS(0.001, alpha, Gravity, ThetaRef);

            Assert.IsFalse(double.IsNaN(dvds), "SpeedRateS should not be NaN for small velocity");
            Assert.IsFalse(double.IsInfinity(dvds), "SpeedRateS should not be infinite for small velocity");
        }

        [TestMethod]
        public void TimeRateSHandlesLargeAngle()
        {
            var v = 5.0;
            var alpha = Math.PI / 2.5; // Large angle near singularity

            var dtds = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

            Assert.IsFalse(double.IsNaN(dtds), "TimeRateS should not be NaN for large angle");
            Assert.IsFalse(double.IsInfinity(dtds), "TimeRateS should not be infinite for large angle");
        }

        [TestMethod]
        public void RunningCostSHandlesEdgeCases()
        {
            // Small velocity
            var cost1 = BrachistochroneAlternateDynamics.RunningCostS(0.001, Math.PI / 6.0);
            Assert.IsFalse(double.IsNaN(cost1), "RunningCostS should handle small velocity");
            Assert.IsFalse(double.IsInfinity(cost1), "RunningCostS should be finite for small velocity");

            // Large angle
            var cost2 = BrachistochroneAlternateDynamics.RunningCostS(5.0, Math.PI / 2.5);
            Assert.IsFalse(double.IsNaN(cost2), "RunningCostS should handle large angle");
            Assert.IsFalse(double.IsInfinity(cost2), "RunningCostS should be finite for large angle");
        }

        #endregion

        #region Energy Conservation

        [TestMethod]
        public void DynamicsSatisfyEnergyConservation()
        {
            // In the rotated coordinate system:
            // Actual vertical drop rate: dy_down/ds = sin(ThetaRef) + cos(ThetaRef) * dn/ds
            // Energy conservation: v * dv/ds = g * dy_down/ds
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var dvds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var dnds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

            var dKE_ds = v * dvds;
            var dy_down_ds = Math.Sin(ThetaRef) + Math.Cos(ThetaRef) * dnds;
            var dPE_ds = Gravity * dy_down_ds;

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

                    var dvds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
                    var dnds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);

                    var dKE_ds = v * dvds;
                    var dy_down_ds = Math.Sin(ThetaRef) + Math.Cos(ThetaRef) * dnds;
                    var dPE_ds = Gravity * dy_down_ds;

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

            var f0 = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var fPlus = BrachistochroneAlternateDynamics.SpeedRateS(v + Epsilon, alpha, Gravity, ThetaRef);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dv[g*sin(alpha+ThetaRef)/(v*cos(alpha))] = -g*sin(alpha+ThetaRef)/(v²*cos(alpha))
            var worldAlpha = alpha + ThetaRef;
            var cosAlpha = Math.Cos(alpha);
            var analyticalGradient = -Gravity * Math.Sin(worldAlpha) / (v * v * cosAlpha);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "SpeedRateS gradient w.r.t. v is incorrect");
        }

        [TestMethod]
        public void SpeedRateSGradientWrtAlpha()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var fPlus = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha + Epsilon, Gravity, ThetaRef);
            var numericalGradient = (fPlus - f0) / Epsilon;

            // Analytical: d/dalpha[g*sin(alpha+ThetaRef)/(v*cos(alpha))]
            // Using quotient rule: d[sin(A+T)/cos(A)]/dA = [cos(A+T)*cos(A) + sin(A+T)*sin(A)] / cos²(A)
            var worldAlpha = alpha + ThetaRef;
            var cosAlpha = Math.Cos(alpha);
            var sinAlpha = Math.Sin(alpha);
            var analyticalGradient = Gravity * (Math.Cos(worldAlpha) * cosAlpha + Math.Sin(worldAlpha) * sinAlpha) / (v * cosAlpha * cosAlpha);

            Assert.AreEqual(analyticalGradient, numericalGradient, GradientTolerance,
                "SpeedRateS gradient w.r.t. alpha is incorrect");
        }

        [TestMethod]
        public void TimeRateSGradientWrtVelocity()
        {
            var v = 5.0;
            var alpha = Math.PI / 6.0;

            var f0 = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);
            var fPlus = BrachistochroneAlternateDynamics.TimeRateS(v + Epsilon, alpha);
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

            var f0 = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);
            var fPlus = BrachistochroneAlternateDynamics.TimeRateS(v, alpha + Epsilon);
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

            var f0 = BrachistochroneAlternateDynamics.VerticalRateS(alpha);
            var fPlus = BrachistochroneAlternateDynamics.VerticalRateS(alpha + Epsilon);
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

            var costRate = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);
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

            var runningCost = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);
            var timeRate = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

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
            var dVds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var dNds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);
            var dAlphads = BrachistochroneAlternateDynamics.AlphaRateS(k);
            var dTds = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

            // Verify individual values
            Assert.AreEqual(Gravity * Math.Sin(alpha + ThetaRef) / (v * Math.Cos(alpha)), dVds, 1e-10, "dv/ds value");
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

            // Analytical: d(g*sin(alpha+ThetaRef)/(v*cos(alpha)))/dv = -g*sin(alpha+ThetaRef)/(v²*cos(alpha))
            var v = state[0];
            var alpha = state[2];
            var worldAlpha = alpha + ThetaRef;
            var dfdv_analytical = -Gravity * Math.Sin(worldAlpha) / (v * v * Math.Cos(alpha));

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
                BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef),
                BrachistochroneAlternateDynamics.VerticalRateS(alpha),
                BrachistochroneAlternateDynamics.AlphaRateS(k),
                BrachistochroneAlternateDynamics.TimeRateS(v, alpha)
            };
        }

        #endregion
    }
}
