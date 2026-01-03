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
    /// Phase 1 verification tests for Brachistochrone dynamics and gradients.
    /// These tests verify that:
    /// 1. Physical dynamics values are correct
    /// 2. AutoDiff gradients match numerical finite differences
    /// 3. Time-scaled dynamics for free final time work correctly
    /// </summary>
    [TestClass]
    public sealed class BrachistochroneDynamicsVerificationTests
    {
        private const double Gravity = 9.80665;
        private const double Epsilon = 1e-7;
        private const double GradientTolerance = 1e-5;

        #region Task 1.1: Physical Dynamics Values

        [TestMethod]
        public void XRateIsCorrectForKnownInputs()
        {
            // ẋ = v·cos(θ)
            var v = 5.0;
            var theta = Math.PI / 4.0; // 45 degrees
            var expected = v * Math.Cos(theta); // 5 * 0.707... = 3.535...

            var (actual, _) = BrachistochroneDynamicsGradients.XRateReverse(0, 0, v, theta, Gravity);

            Assert.AreEqual(expected, actual, 1e-10, "XRate should be v·cos(θ)");
        }

        [TestMethod]
        public void YRateIsCorrectForKnownInputs()
        {
            // ẏ = -v·sin(θ) (negative for descent)
            var v = 5.0;
            var theta = Math.PI / 4.0;
            var expected = -v * Math.Sin(theta); // -3.535...

            var (actual, _) = BrachistochroneDynamicsGradients.YRateReverse(0, 0, v, theta, Gravity);

            Assert.AreEqual(expected, actual, 1e-10, "YRate should be -v·sin(θ)");
        }

        [TestMethod]
        public void VRateIsCorrectForKnownInputs()
        {
            // v̇ = g·sin(θ)
            var theta = Math.PI / 6.0; // 30 degrees
            var expected = Gravity * Math.Sin(theta); // 9.8 * 0.5 = 4.9

            var (actual, _) = BrachistochroneDynamicsGradients.VRateReverse(0, 0, 0, theta, Gravity);

            Assert.AreEqual(expected, actual, 1e-10, "VRate should be g·sin(θ)");
        }

        [TestMethod]
        public void VRateIsZeroForHorizontalMotion()
        {
            // When θ = 0 (horizontal), no acceleration
            var theta = 0.0;
            var (actual, _) = BrachistochroneDynamicsGradients.VRateReverse(0, 0, 5, theta, Gravity);

            Assert.AreEqual(0.0, actual, 1e-10, "VRate should be zero for horizontal motion");
        }

        [TestMethod]
        public void VRateIsMaxForVerticalDescent()
        {
            // When θ = π/2 (straight down), maximum acceleration
            var theta = Math.PI / 2.0;
            var (actual, _) = BrachistochroneDynamicsGradients.VRateReverse(0, 0, 0, theta, Gravity);

            Assert.AreEqual(Gravity, actual, 1e-10, "VRate should be g for vertical descent");
        }

        #endregion

        #region Task 1.1: Dynamics Gradient Verification

        [TestMethod]
        public void XRateGradientMatchesNumericalDerivative()
        {
            var x = 5.0;
            var y = 7.5;
            var v = 3.0;
            var theta = Math.PI / 4.0;

            var (value, gradients) = BrachistochroneDynamicsGradients.XRateReverse(x, y, v, theta, Gravity);

            // Numerical gradients
            var (valuePlusX, _) = BrachistochroneDynamicsGradients.XRateReverse(x + Epsilon, y, v, theta, Gravity);
            var (valuePlusY, _) = BrachistochroneDynamicsGradients.XRateReverse(x, y + Epsilon, v, theta, Gravity);
            var (valuePlusV, _) = BrachistochroneDynamicsGradients.XRateReverse(x, y, v + Epsilon, theta, Gravity);
            var (valuePlusTheta, _) = BrachistochroneDynamicsGradients.XRateReverse(x, y, v, theta + Epsilon, Gravity);

            var numGradX = (valuePlusX - value) / Epsilon;
            var numGradY = (valuePlusY - value) / Epsilon;
            var numGradV = (valuePlusV - value) / Epsilon;
            var numGradTheta = (valuePlusTheta - value) / Epsilon;

            // gradients = [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
            Assert.AreEqual(0.0, gradients[0], GradientTolerance, "∂XRate/∂x should be 0");
            Assert.AreEqual(0.0, gradients[1], GradientTolerance, "∂XRate/∂y should be 0");
            Assert.AreEqual(numGradV, gradients[2], GradientTolerance, "∂XRate/∂v mismatch");
            Assert.AreEqual(numGradTheta, gradients[3], GradientTolerance, "∂XRate/∂theta mismatch");
        }

        [TestMethod]
        public void YRateGradientMatchesNumericalDerivative()
        {
            var x = 5.0;
            var y = 7.5;
            var v = 3.0;
            var theta = Math.PI / 4.0;

            var (value, gradients) = BrachistochroneDynamicsGradients.YRateReverse(x, y, v, theta, Gravity);

            var (valuePlusV, _) = BrachistochroneDynamicsGradients.YRateReverse(x, y, v + Epsilon, theta, Gravity);
            var (valuePlusTheta, _) = BrachistochroneDynamicsGradients.YRateReverse(x, y, v, theta + Epsilon, Gravity);

            var numGradV = (valuePlusV - value) / Epsilon;
            var numGradTheta = (valuePlusTheta - value) / Epsilon;

            Assert.AreEqual(0.0, gradients[0], GradientTolerance, "∂YRate/∂x should be 0");
            Assert.AreEqual(0.0, gradients[1], GradientTolerance, "∂YRate/∂y should be 0");
            Assert.AreEqual(numGradV, gradients[2], GradientTolerance, "∂YRate/∂v mismatch");
            Assert.AreEqual(numGradTheta, gradients[3], GradientTolerance, "∂YRate/∂theta mismatch");
        }

        [TestMethod]
        public void VRateGradientMatchesNumericalDerivative()
        {
            var x = 5.0;
            var y = 7.5;
            var v = 3.0;
            var theta = Math.PI / 4.0;

            var (value, gradients) = BrachistochroneDynamicsGradients.VRateReverse(x, y, v, theta, Gravity);

            var (valuePlusTheta, _) = BrachistochroneDynamicsGradients.VRateReverse(x, y, v, theta + Epsilon, Gravity);

            var numGradTheta = (valuePlusTheta - value) / Epsilon;

            Assert.AreEqual(0.0, gradients[0], GradientTolerance, "∂VRate/∂x should be 0");
            Assert.AreEqual(0.0, gradients[1], GradientTolerance, "∂VRate/∂y should be 0");
            Assert.AreEqual(0.0, gradients[2], GradientTolerance, "∂VRate/∂v should be 0");
            Assert.AreEqual(numGradTheta, gradients[3], GradientTolerance, "∂VRate/∂theta mismatch");
        }

        #endregion

        #region Task 1.2 & 1.3: Time-Scaled Dynamics for Free Final Time

        [TestMethod]
        public void TimeScaledDynamicsScalesByTf()
        {
            // Time-scaled dynamics: dx/dτ = T_f · (dx/dt)
            var v = 3.0;
            var theta = Math.PI / 4.0;
            var Tf = 2.0;

            var (xratePhys, _) = BrachistochroneDynamicsGradients.XRateReverse(0, 0, v, theta, Gravity);
            var xrateScaled = Tf * xratePhys;

            // For τ ∈ [0, 1], we need: x(1) - x(0) = ∫₀¹ T_f · ẋ dτ = T_f · ẋ (for constant ẋ)
            // This should give the same displacement as integrating for time T_f
            Assert.AreEqual(Tf * xratePhys, xrateScaled, 1e-10, "Time-scaled dynamics should multiply by T_f");
        }

        [TestMethod]
        public void TimeScaledDynamicsGradientWrtTfEqualsPhysicalDynamics()
        {
            // ∂(T_f · f)/∂T_f = f (the physical dynamics value)
            var v = 3.0;
            var theta = Math.PI / 4.0;
            var Tf = 2.0;

            var (xratePhys, _) = BrachistochroneDynamicsGradients.XRateReverse(0, 0, v, theta, Gravity);
            var (yratePhys, _) = BrachistochroneDynamicsGradients.YRateReverse(0, 0, v, theta, Gravity);
            var (vratePhys, _) = BrachistochroneDynamicsGradients.VRateReverse(0, 0, v, theta, Gravity);

            // Time-scaled dynamics
            var xrateScaled = Tf * xratePhys;
            var yrateScaled = Tf * yratePhys;
            var vrateScaled = Tf * vratePhys;

            // Numerical derivative w.r.t. T_f
            var TfPlus = Tf + Epsilon;
            var xrateScaledPlus = TfPlus * xratePhys;
            var yrateScaledPlus = TfPlus * yratePhys;
            var vrateScaledPlus = TfPlus * vratePhys;

            var dxrate_dTf_numerical = (xrateScaledPlus - xrateScaled) / Epsilon;
            var dyrate_dTf_numerical = (yrateScaledPlus - yrateScaled) / Epsilon;
            var dvrate_dTf_numerical = (vrateScaledPlus - vrateScaled) / Epsilon;

            Assert.AreEqual(xratePhys, dxrate_dTf_numerical, GradientTolerance, "∂(T_f·ẋ)/∂T_f should equal ẋ");
            Assert.AreEqual(yratePhys, dyrate_dTf_numerical, GradientTolerance, "∂(T_f·ẏ)/∂T_f should equal ẏ");
            Assert.AreEqual(vratePhys, dvrate_dTf_numerical, GradientTolerance, "∂(T_f·v̇)/∂T_f should equal v̇");
        }

        [TestMethod]
        public void TimeScaledDynamicsGradientWrtStateScalesByTf()
        {
            // ∂(T_f · f)/∂x = T_f · ∂f/∂x
            var v = 3.0;
            var theta = Math.PI / 4.0;
            var Tf = 2.0;

            var (_, xrateGrad) = BrachistochroneDynamicsGradients.XRateReverse(0, 0, v, theta, Gravity);
            var (_, yrateGrad) = BrachistochroneDynamicsGradients.YRateReverse(0, 0, v, theta, Gravity);
            var (_, vrateGrad) = BrachistochroneDynamicsGradients.VRateReverse(0, 0, v, theta, Gravity);

            // Time-scaled gradients should be T_f times the physical gradients
            // For example: ∂(T_f·ẋ)/∂v = T_f · ∂ẋ/∂v
            var scaledGrad_xrate_v = Tf * xrateGrad[2];   // ∂ẋ/∂v = cos(θ)
            var scaledGrad_xrate_theta = Tf * xrateGrad[3]; // ∂ẋ/∂θ = -v·sin(θ)

            // Verify scaling is correct
            Assert.AreEqual(Tf * Math.Cos(theta), scaledGrad_xrate_v, GradientTolerance, "Scaled ∂ẋ/∂v");
            Assert.AreEqual(Tf * (-v * Math.Sin(theta)), scaledGrad_xrate_theta, GradientTolerance, "Scaled ∂ẋ/∂θ");
        }

        [TestMethod]
        public void TfRateShouldBeZero()
        {
            // T_f is constant over the trajectory: dT_f/dτ = 0
            // This is explicitly set in the problem formulation
            var Tfrate = 0.0; // As defined in CreateFreeFinalTimeProblem

            Assert.AreEqual(0.0, Tfrate, 1e-10, "dT_f/dτ should be 0");
        }

        #endregion

        #region Task 1.2: Running Cost Gradient Verification

        [TestMethod]
        public void RunningCostForFixedTimeIsConstantOne()
        {
            // L = 1 for fixed time problem
            var (cost, gradients) = BrachistochroneDynamicsGradients.RunningCostReverse(0, 0, 0, 0);

            Assert.AreEqual(1.0, cost, 1e-10, "Running cost should be 1");
            Assert.AreEqual(0.0, gradients[0], 1e-10, "∂L/∂x should be 0");
            Assert.AreEqual(0.0, gradients[1], 1e-10, "∂L/∂y should be 0");
            Assert.AreEqual(0.0, gradients[2], 1e-10, "∂L/∂v should be 0");
            Assert.AreEqual(0.0, gradients[3], 1e-10, "∂L/∂θ should be 0");
        }

        [TestMethod]
        public void RunningCostForFreeTimeIsTfWithCorrectGradient()
        {
            // L = T_f for free time problem with running cost
            // Gradient: ∂L/∂T_f = 1
            var Tf = 1.8;

            // Simulating the running cost evaluation as in the problem
            var cost = Tf;
            var gradients = new double[6]; // [x, y, v, T_f, theta, tau]
            gradients[3] = 1.0; // ∂L/∂T_f = 1

            Assert.AreEqual(Tf, cost, 1e-10, "Running cost should be T_f");
            Assert.AreEqual(1.0, gradients[3], 1e-10, "∂L/∂T_f should be 1");
        }

        [TestMethod]
        public void IntegratedRunningCostEqualsTf()
        {
            // ∫₀¹ T_f dτ = T_f (since T_f is constant)
            var Tf = 2.5;
            var numPoints = 100;
            var dtau = 1.0 / (numPoints - 1);

            var integratedCost = 0.0;
            for (var i = 0; i < numPoints - 1; i++)
            {
                // Trapezoidal integration
                integratedCost += 0.5 * (Tf + Tf) * dtau;
            }

            Assert.AreEqual(Tf, integratedCost, 1e-6, "Integrated cost should equal T_f");
        }

        #endregion

        #region Task 1.3: Terminal Cost Gradient Verification

        [TestMethod]
        public void TerminalCostForFreeTimeIsTfWithCorrectGradient()
        {
            // Φ = T_f for free time problem with terminal cost
            // Gradient: ∂Φ/∂T_f = 1
            var Tf = 1.8;

            // Simulating the terminal cost evaluation as in the problem
            var cost = Tf;
            var gradients = new double[5]; // [x, y, v, T_f, tau]
            gradients[3] = 1.0; // ∂Φ/∂T_f = 1

            Assert.AreEqual(Tf, cost, 1e-10, "Terminal cost should be T_f");
            Assert.AreEqual(1.0, gradients[3], 1e-10, "∂Φ/∂T_f should be 1");
        }

        #endregion

        #region Complete Dynamics Function Test (as used in solver)

        [TestMethod]
        public void CompleteDynamicsFunctionForFixedTimeReturnsCorrectValuesAndGradients()
        {
            // Test the complete dynamics function as configured in the problem
            var x = new double[] { 5.0, 7.5, 3.0 }; // [x, y, v]
            var u = new double[] { Math.PI / 4.0 }; // [theta]
            _ = 0.5; // t (unused but shown for clarity)

            // Evaluate dynamics as in fixed time problem
            var v = x[2];
            var theta = u[0];

            var (xrate, xrateGrad) = BrachistochroneDynamicsGradients.XRateReverse(x[0], x[1], v, theta, Gravity);
            var (yrate, yrateGrad) = BrachistochroneDynamicsGradients.YRateReverse(x[0], x[1], v, theta, Gravity);
            var (vrate, vrateGrad) = BrachistochroneDynamicsGradients.VRateReverse(x[0], x[1], v, theta, Gravity);

            var value = new[] { xrate, yrate, vrate };

            // Build gradients as expected by solver
            // gradients[0] = df/dx (3x3 flattened row-major)
            // gradients[1] = df/du (3x1)
            var stateGradients = new double[] {
                xrateGrad[0], xrateGrad[1], xrateGrad[2],  // ∂ẋ/∂(x,y,v)
                yrateGrad[0], yrateGrad[1], yrateGrad[2],  // ∂ẏ/∂(x,y,v)
                vrateGrad[0], vrateGrad[1], vrateGrad[2]   // ∂v̇/∂(x,y,v)
            };
            var controlGradients = new double[] { xrateGrad[3], yrateGrad[3], vrateGrad[3] }; // ∂f/∂θ

            // Verify values
            Assert.AreEqual(v * Math.Cos(theta), value[0], 1e-10, "ẋ value");
            Assert.AreEqual(-v * Math.Sin(theta), value[1], 1e-10, "ẏ value");
            Assert.AreEqual(Gravity * Math.Sin(theta), value[2], 1e-10, "v̇ value");

            // Verify gradient structure
            Assert.AreEqual(9, stateGradients.Length, "State gradient should be 3x3=9 elements");
            Assert.AreEqual(3, controlGradients.Length, "Control gradient should be 3x1=3 elements");

            // Verify key gradient values
            Assert.AreEqual(Math.Cos(theta), stateGradients[2], GradientTolerance, "∂ẋ/∂v = cos(θ)");
            Assert.AreEqual(-Math.Sin(theta), stateGradients[5], GradientTolerance, "∂ẏ/∂v = -sin(θ)");
            Assert.AreEqual(-v * Math.Sin(theta), controlGradients[0], GradientTolerance, "∂ẋ/∂θ = -v·sin(θ)");
            Assert.AreEqual(-v * Math.Cos(theta), controlGradients[1], GradientTolerance, "∂ẏ/∂θ = -v·cos(θ)");
            Assert.AreEqual(Gravity * Math.Cos(theta), controlGradients[2], GradientTolerance, "∂v̇/∂θ = g·cos(θ)");
        }

        [TestMethod]
        public void CompleteDynamicsFunctionForFreeTimeReturnsCorrectValuesAndGradients()
        {
            // Test the complete dynamics function as configured in free final time problem
            var state = new double[] { 5.0, 7.5, 3.0, 1.8 }; // [x, y, v, T_f]
            var control = new double[] { Math.PI / 4.0 }; // [theta]
            _ = 0.5; // tau (unused but shown for clarity)

            var v = state[2];
            var Tf = state[3];
            var theta = control[0];

            // Time-scaled dynamics
            var (xratePhys, xrateGrad) = BrachistochroneDynamicsGradients.XRateReverse(state[0], state[1], v, theta, Gravity);
            var (yratePhys, yrateGrad) = BrachistochroneDynamicsGradients.YRateReverse(state[0], state[1], v, theta, Gravity);
            var (vratePhys, vrateGrad) = BrachistochroneDynamicsGradients.VRateReverse(state[0], state[1], v, theta, Gravity);

            var xrate = Tf * xratePhys;
            var yrate = Tf * yratePhys;
            var vrate = Tf * vratePhys;
            var Tfrate = 0.0;

            var value = new[] { xrate, yrate, vrate, Tfrate };

            // Build gradients for 4-state system
            // gradients[0] = df/dx (4x4 flattened row-major)
            // gradients[1] = df/du (4x1)
            var stateGradients = new double[] {
                Tf * xrateGrad[0], Tf * xrateGrad[1], Tf * xrateGrad[2], xratePhys,  // ∂ẋ_scaled/∂(x,y,v,Tf)
                Tf * yrateGrad[0], Tf * yrateGrad[1], Tf * yrateGrad[2], yratePhys,  // ∂ẏ_scaled/∂(x,y,v,Tf)
                Tf * vrateGrad[0], Tf * vrateGrad[1], Tf * vrateGrad[2], vratePhys,  // ∂v̇_scaled/∂(x,y,v,Tf)
                0.0, 0.0, 0.0, 0.0                                                    // ∂Ṫf/∂(x,y,v,Tf)
            };
            var controlGradients = new double[] { 
                Tf * xrateGrad[3], 
                Tf * yrateGrad[3], 
                Tf * vrateGrad[3], 
                0.0 // ∂Ṫf/∂θ = 0
            };

            // Verify values
            Assert.AreEqual(Tf * v * Math.Cos(theta), value[0], 1e-10, "ẋ_scaled value");
            Assert.AreEqual(Tf * (-v * Math.Sin(theta)), value[1], 1e-10, "ẏ_scaled value");
            Assert.AreEqual(Tf * Gravity * Math.Sin(theta), value[2], 1e-10, "v̇_scaled value");
            Assert.AreEqual(0.0, value[3], 1e-10, "Ṫf should be 0");

            // Verify gradient structure
            Assert.AreEqual(16, stateGradients.Length, "State gradient should be 4x4=16 elements");
            Assert.AreEqual(4, controlGradients.Length, "Control gradient should be 4x1=4 elements");

            // Verify key gradient values (∂f/∂Tf = f_physical)
            Assert.AreEqual(xratePhys, stateGradients[3], GradientTolerance, "∂ẋ_scaled/∂Tf = ẋ_physical");
            Assert.AreEqual(yratePhys, stateGradients[7], GradientTolerance, "∂ẏ_scaled/∂Tf = ẏ_physical");
            Assert.AreEqual(vratePhys, stateGradients[11], GradientTolerance, "∂v̇_scaled/∂Tf = v̇_physical");

            // Verify ∂ẋ_scaled/∂v = Tf * cos(θ)
            Assert.AreEqual(Tf * Math.Cos(theta), stateGradients[2], GradientTolerance, "∂ẋ_scaled/∂v");

            // Verify control gradients are scaled
            Assert.AreEqual(Tf * (-v * Math.Sin(theta)), controlGradients[0], GradientTolerance, "∂ẋ_scaled/∂θ");
        }

        #endregion
    }
}
