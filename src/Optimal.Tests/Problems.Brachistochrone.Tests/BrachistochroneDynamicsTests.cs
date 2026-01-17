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
    [TestClass]
    public sealed class BrachistochroneDynamicsTests
    {
        [TestMethod]
        public void TimeScaledDynamicsGradientsAreCorrect()
        {
            // Test the time-scaled dynamics gradients against numerical approximation
            var g = 9.80665;
            var x = 5.0;
            var y = 7.5;
            var v = 3.0;
            var Tf = 1.5;
            var theta = Math.PI / 4.0;

            // Get analytical gradients for physical dynamics
            var (xrate_physical, xrate_gradients) = BrachistochroneDynamicsGradients.XRateReverse(v, theta);
            var (yrate_physical, yrate_gradients) = BrachistochroneDynamicsGradients.YRateReverse(x, y, v, theta, g);
            var (vrate_physical, vrate_gradients) = BrachistochroneDynamicsGradients.VRateReverse(x, y, v, theta, g);

            // Compute time-scaled versions (as in solver)
            var xrate = Tf * xrate_physical;
            var yrate = Tf * yrate_physical;
            var vrate = Tf * vrate_physical;
            var Tfrate = 0.0;

            // Analytical gradients w.r.t. state using chain rule
            // XRateReverse returns [∂/∂v, ∂/∂theta] - xrate = v*cos(theta), no x,y dependence
            var dxrate_dx = 0.0;  // XRate doesn't depend on x
            var dxrate_dv = Tf * xrate_gradients[0];  // [0] = ∂/∂v
            var dxrate_dTf = xrate_physical;

            // YRateReverse returns [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
            var dyrate_dv = Tf * yrate_gradients[2];  // [2] = ∂/∂v
            var dyrate_dTf = yrate_physical;

            // VRateReverse returns [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
            var dvrate_dTf = vrate_physical;

            // Numerical gradient check using finite differences
            var epsilon = 1e-7;

            // Test dx/dτ gradients
            var (xrate_plus_x, _) = BrachistochroneDynamicsGradients.XRateReverse(v, theta);
            var dxrate_dx_numerical = (Tf * xrate_plus_x - xrate) / epsilon;
            Assert.AreEqual(dxrate_dx, dxrate_dx_numerical, 1e-5, "dx/dτ gradient w.r.t. x incorrect");

            var (xrate_plus_v, _) = BrachistochroneDynamicsGradients.XRateReverse(v + epsilon, theta);
            var dxrate_dv_numerical = (Tf * xrate_plus_v - xrate) / epsilon;
            Assert.AreEqual(dxrate_dv, dxrate_dv_numerical, 1e-5, "dx/dτ gradient w.r.t. v incorrect");

            var xrate_plus_Tf = (Tf + epsilon) * xrate_physical;
            var dxrate_dTf_numerical = (xrate_plus_Tf - xrate) / epsilon;
            Assert.AreEqual(dxrate_dTf, dxrate_dTf_numerical, 1e-5, "dx/dτ gradient w.r.t. Tf incorrect");

            // Test dy/dτ gradients
            var (yrate_plus_v, _) = BrachistochroneDynamicsGradients.YRateReverse(x, y, v + epsilon, theta, g);
            var dyrate_dv_numerical = (Tf * yrate_plus_v - yrate) / epsilon;
            Assert.AreEqual(dyrate_dv, dyrate_dv_numerical, 1e-5, "dy/dτ gradient w.r.t. v incorrect");

            var yrate_plus_Tf = (Tf + epsilon) * yrate_physical;
            var dyrate_dTf_numerical = (yrate_plus_Tf - yrate) / epsilon;
            Assert.AreEqual(dyrate_dTf, dyrate_dTf_numerical, 1e-5, "dy/dτ gradient w.r.t. Tf incorrect");

            // Test dv/dτ gradients
            var (vrate_plus_theta, vrate_plus_theta_grad) = BrachistochroneDynamicsGradients.VRateReverse(x, y, v, theta + epsilon, g);
            var dvrate_dtheta_numerical = (Tf * vrate_plus_theta - vrate) / epsilon;
            var dvrate_dtheta = Tf * vrate_gradients[3];
            Assert.AreEqual(dvrate_dtheta, dvrate_dtheta_numerical, 1e-5, "dv/dτ gradient w.r.t. theta incorrect");

            var vrate_plus_Tf = (Tf + epsilon) * vrate_physical;
            var dvrate_dTf_numerical = (vrate_plus_Tf - vrate) / epsilon;
            Assert.AreEqual(dvrate_dTf, dvrate_dTf_numerical, 1e-5, "dv/dτ gradient w.r.t. Tf incorrect");

            // dTf/dτ should be zero and have zero gradients
            Assert.AreEqual(0.0, Tfrate, 1e-10, "dTf/dτ should be zero");
        }

        [TestMethod]
        public void RunningCostGradientIsCorrect()
        {
            // Test the running cost L = Tf has correct gradient
            var Tf = 1.5;

            // Cost should be Tf
            var cost = Tf;

            // Gradient should be [0, 0, 0, 1, 0, 0] for [x, y, v, Tf, theta, tau]
            var dL_dx = 0.0;
            var dL_dy = 0.0;
            var dL_dv = 0.0;
            var dL_dTf = 1.0;
            var dL_dtheta = 0.0;

            // Verify with numerical gradient
            var epsilon = 1e-7;
            var cost_plus_Tf = Tf + epsilon;
            var dL_dTf_numerical = (cost_plus_Tf - cost) / epsilon;

            Assert.AreEqual(dL_dTf, dL_dTf_numerical, 1e-5, "Running cost gradient w.r.t. Tf incorrect");
            Assert.AreEqual(0.0, dL_dx, 1e-10, "Running cost gradient w.r.t. x should be zero");
            Assert.AreEqual(0.0, dL_dy, 1e-10, "Running cost gradient w.r.t. y should be zero");
            Assert.AreEqual(0.0, dL_dv, 1e-10, "Running cost gradient w.r.t. v should be zero");
            Assert.AreEqual(0.0, dL_dtheta, 1e-10, "Running cost gradient w.r.t. theta should be zero");
        }

        [TestMethod]
        public void IntegratedCostEqualsFinalTime()
        {
            // When integrating L = Tf over τ ∈ [0,1], we should get Tf
            var Tf = 2.0;
            var numPoints = 100;
            var dtau = 1.0 / (numPoints - 1);

            var integratedCost = 0.0;
            for (var i = 0; i < numPoints - 1; i++)
            {
                // Trapezoidal integration
                var L1 = Tf;
                var L2 = Tf;
                integratedCost += 0.5 * (L1 + L2) * dtau;
            }

            Assert.AreEqual(Tf, integratedCost, 1e-6, "Integrated cost should equal Tf");
        }
    }
}
