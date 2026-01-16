/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;

namespace Optimal.Control.Collocation.Tests
{
    [TestClass]
    public abstract class TranscriptionTests
    {
        private const double Tolerance = 1e-10;
        private static readonly double[] s_testState = [1.5, 2.5];
        private static readonly double[] s_testControl = [3.5];
        private static readonly double[] s_initialState = [0.0, 1.0];
        private static readonly double[] s_finalState = [10.0, 5.0];
        private static readonly double[] s_zeroControl = [0.0];
        private static readonly double[] s_unitControl = [1.0];
        private static readonly double[] s_maxDefectTest = [-0.5, 0.2, -0.8, 0.3];

        [TestMethod]
        public void CanCreateTranscription()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = CreateTranscription(problem, grid);

            Assert.AreEqual(10, transcription.Segments);
            Assert.AreEqual(11 * 3, transcription.DecisionVectorSize); // 11 nodes * (2 states + 1 control)
        }

        [TestMethod]
        public void CanPackAndUnpackDecisionVector()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 5);
            var transcription = CreateTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];

            transcription.SetState(z, 2, s_testState);
            transcription.SetControl(z, 2, s_testControl);

            var retrievedState = transcription.GetState(z, 2);
            var retrievedControl = transcription.GetControl(z, 2);

            Assert.AreEqual(1.5, retrievedState[0], Tolerance);
            Assert.AreEqual(2.5, retrievedState[1], Tolerance);
            Assert.AreEqual(3.5, retrievedControl[0], Tolerance);
        }

        [TestMethod]
        public void HermiteInterpolationIsCorrect()
        {
            var xk = new[] { 0.0 };
            var xk1 = new[] { 2.0 };
            var fk = new[] { 1.0 };
            var fk1 = new[] { 1.0 };
            var h = 1.0;

            var xMid = HermiteSimpsonTranscription.HermiteInterpolation(xk, xk1, fk, fk1, h);

            // With constant velocity, midpoint should be exactly at 1.0
            Assert.AreEqual(1.0, xMid[0], Tolerance);
        }

        [TestMethod]
        public void HermiteInterpolationWithAcceleration()
        {
            // Test case: quadratic motion x(t) = t^2
            // At t=0: x=0, dx/dt=0
            // At t=1: x=1, dx/dt=2
            var xk = new[] { 0.0 };
            var xk1 = new[] { 1.0 };
            var fk = new[] { 0.0 };   // velocity at t=0
            var fk1 = new[] { 2.0 };  // velocity at t=1
            var h = 1.0;

            var xMid = HermiteSimpsonTranscription.HermiteInterpolation(xk, xk1, fk, fk1, h);

            // Hermite gives: x_mid = (0 + 1)/2 + 1/8 * (0 - 2) = 0.5 - 0.25 = 0.25
            // Analytical: x(0.5) = 0.5^2 = 0.25 ✓
            Assert.AreEqual(0.25, xMid[0], Tolerance);
        }

        [TestMethod]
        public void ControlMidpointIsAverage()
        {
            var uk = new[] { 1.0, 2.0 };
            var uk1 = new[] { 3.0, 4.0 };

            var uMid = HermiteSimpsonTranscription.ControlMidpoint(uk, uk1);

            Assert.AreEqual(2.0, uMid[0], Tolerance);
            Assert.AreEqual(3.0, uMid[1], Tolerance);
        }

        [TestMethod]
        public void DefectIsZeroForExactSolution()
        {
            // Test: constant velocity motion x' = u with u = 1
            // Analytical solution: x(t) = t
            var xk = new[] { 0.0 };
            var xk1 = new[] { 1.0 };
            var fk = new[] { 1.0 };
            var fMid = new[] { 1.0 };
            var fk1 = new[] { 1.0 };
            var h = 1.0;

            var defect = HermiteSimpsonTranscription.ComputeDefect(xk, xk1, fk, fMid, fk1, h);

            // x_{k+1} - x_k - h/6 * (f_k + 4*f_mid + f_{k+1})
            // = 1.0 - 0.0 - 1.0/6 * (1 + 4 + 1)
            // = 1.0 - 1.0 = 0 ✓
            Assert.AreEqual(0.0, defect[0], Tolerance);
        }

        [TestMethod]
        public void DefectDetectsBadTrajectory()
        {
            // Test: constant velocity but wrong endpoint
            var xk = new[] { 0.0 };
            var xk1 = new[] { 0.5 };  // Wrong! Should be 1.0
            var fk = new[] { 1.0 };
            var fMid = new[] { 1.0 };
            var fk1 = new[] { 1.0 };
            var h = 1.0;

            var defect = HermiteSimpsonTranscription.ComputeDefect(xk, xk1, fk, fMid, fk1, h);

            // Defect = 0.5 - 0.0 - 1.0/6 * 6 = 0.5 - 1.0 = -0.5
            Assert.AreEqual(-0.5, defect[0], Tolerance);
        }

        [TestMethod]
        public void SimpleIntegratorSatisfiesDynamics()
        {
            // Test: ẋ = u with u = 1, from t=0 to t=5
            // Analytical solution: x(t) = t
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 5.0, 10);
            var transcription = CreateTranscription(problem, grid);

            // Create exact analytical solution
            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                var t = grid.TimePoints[k];
                transcription.SetState(z, k, [t]);
                transcription.SetControl(z, k, s_unitControl);
            }

            // Dynamics: ẋ = u
#pragma warning disable RCS1163 // Unused parameter
            double[] DynamicsEvaluator(double[] x, double[] u, double t)
            {
                return [u[0]];
            }
#pragma warning restore RCS1163 // Unused parameter

            var defects = transcription.ComputeAllDefects(z, DynamicsEvaluator);
            var maxDefect = HermiteSimpsonTranscription.MaxDefect(defects);

            // Should be zero (or very close) for exact solution
            Assert.IsTrue(maxDefect < 1e-10, $"Max defect should be near zero, was {maxDefect}");
        }

        [TestMethod]
        public void DoubleIntegratorSatisfiesDynamics()
        {
            // Test: ẍ = u with u = 1 (constant acceleration)
            // State: [position, velocity]
            // Analytical solution: x(t) = 0.5*t^2, v(t) = t
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 4.0, 20);
            var transcription = CreateTranscription(problem, grid);

            // Create exact analytical solution
            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                var t = grid.TimePoints[k];
                transcription.SetState(z, k, [0.5 * t * t, t]);
                transcription.SetControl(z, k, s_unitControl);
            }

            // Dynamics: [dx/dt, dv/dt] = [v, u]
            double[] DynamicsEvaluator(double[] x, double[] u, double _)
            {
                return [x[1], u[0]];
            }

            var defects = transcription.ComputeAllDefects(z, DynamicsEvaluator);
            var maxDefect = HermiteSimpsonTranscription.MaxDefect(defects);

            // Should be very small for exact solution with fine grid
            Assert.IsTrue(maxDefect < 1e-8, $"Max defect should be near zero, was {maxDefect}");
        }

        [TestMethod]
        public void InitialGuessCreatesLinearInterpolation()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = CreateTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(s_initialState, s_finalState, s_zeroControl);

            // Check initial state
            var state0 = transcription.GetState(z, 0);
            Assert.AreEqual(0.0, state0[0], Tolerance);
            Assert.AreEqual(1.0, state0[1], Tolerance);

            // Check final state
            var stateN = transcription.GetState(z, 10);
            Assert.AreEqual(10.0, stateN[0], Tolerance);
            Assert.AreEqual(5.0, stateN[1], Tolerance);

            // Check midpoint (should be linear interpolation)
            var state5 = transcription.GetState(z, 5);
            Assert.AreEqual(5.0, state5[0], Tolerance);
            Assert.AreEqual(3.0, state5[1], Tolerance);

            // Check control is constant
            var control5 = transcription.GetControl(z, 5);
            Assert.AreEqual(0.0, control5[0], Tolerance);
        }

        [TestMethod]
        public void MaxDefectReturnsMaximumAbsoluteValue()
        {
            var max = HermiteSimpsonTranscription.MaxDefect(s_maxDefectTest);
            Assert.AreEqual(0.8, max, Tolerance);
        }

        protected abstract HermiteSimpsonTranscription CreateTranscription(ControlProblem problem, CollocationGrid grid);
    }
}
