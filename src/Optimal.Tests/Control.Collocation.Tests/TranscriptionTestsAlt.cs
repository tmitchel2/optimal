/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;

namespace Optimal.Control.Collocation.Tests
{
    [TestClass]
    public abstract class TranscriptionTestsAlt
    {
        private const double Tolerance = 1e-10;
        private static readonly double[] s_testState = new[] { 1.5, 2.5 };
        private static readonly double[] s_testControl = new[] { 3.5 };
        private static readonly double[] s_initialState = new[] { 0.0, 1.0 };
        private static readonly double[] s_finalState = new[] { 10.0, 5.0 };
        private static readonly double[] s_zeroControl = new[] { 0.0 };
        private static readonly double[] s_unitControl = new[] { 1.0 };
        private static readonly double[] s_doubleControl = new[] { 2.0 };
        private static readonly double[] s_initialState1D = new[] { 0.0 };
        private static readonly double[] s_finalState1D = new[] { 1.0 };
        private static readonly double[] s_constantState5 = new[] { 5.0 };

        [TestMethod]
        public void CanCreateTranscription()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 4);

            Assert.AreEqual(10, transcription.Segments);
            Assert.AreEqual(4, transcription.Order);
            // Total points: N*(p-1) + 1 = 10*3 + 1 = 31
            Assert.AreEqual(31, transcription.TotalPoints);
            // Decision vector: 31 * (2 states + 1 control) = 93
            Assert.AreEqual(93, transcription.DecisionVectorSize);
        }

        [TestMethod]
        public void ConstructorThrowsForInvalidOrder()
        {
            var problem = new ControlProblem().WithStateSize(2).WithControlSize(1);
            var grid = new CollocationGrid(0.0, 10.0, 5);

            Assert.ThrowsException<ArgumentException>(() =>
                new LegendreGaussLobattoTranscription(problem, grid, order: 1));
            Assert.ThrowsException<ArgumentException>(() =>
                new LegendreGaussLobattoTranscription(problem, grid, order: 0));
        }

        [TestMethod]
        public void TotalPointsComputationIsCorrect()
        {
            var problem = new ControlProblem().WithStateSize(1).WithControlSize(1);
            var grid = new CollocationGrid(0.0, 1.0, 5);

            // Order 2: 5 segments * 1 interior + 1 = 6 points
            var t2 = new LegendreGaussLobattoTranscription(problem, grid, order: 2);
            Assert.AreEqual(6, t2.TotalPoints);

            // Order 3: 5 segments * 2 interior + 1 = 11 points
            var t3 = new LegendreGaussLobattoTranscription(problem, grid, order: 3);
            Assert.AreEqual(11, t3.TotalPoints);

            // Order 4: 5 segments * 3 interior + 1 = 16 points
            var t4 = new LegendreGaussLobattoTranscription(problem, grid, order: 4);
            Assert.AreEqual(16, t4.TotalPoints);

            // Order 5: 5 segments * 4 interior + 1 = 21 points
            var t5 = new LegendreGaussLobattoTranscription(problem, grid, order: 5);
            Assert.AreEqual(21, t5.TotalPoints);
        }

        [TestMethod]
        public void CanPackAndUnpackDecisionVector()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 5);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var z = new double[transcription.DecisionVectorSize];

            transcription.SetState(z, 5, s_testState);
            transcription.SetControl(z, 5, s_testControl);

            var retrievedState = transcription.GetState(z, 5);
            var retrievedControl = transcription.GetControl(z, 5);

            Assert.AreEqual(1.5, retrievedState[0], Tolerance);
            Assert.AreEqual(2.5, retrievedState[1], Tolerance);
            Assert.AreEqual(3.5, retrievedControl[0], Tolerance);
        }

        [TestMethod]
        public void GetStateThrowsForInvalidIndex()
        {
            var problem = new ControlProblem().WithStateSize(2).WithControlSize(1);
            var grid = new CollocationGrid(0.0, 10.0, 5);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var z = new double[transcription.DecisionVectorSize];

            Assert.ThrowsException<ArgumentOutOfRangeException>(() => transcription.GetState(z, -1));
            Assert.ThrowsException<ArgumentOutOfRangeException>(() => transcription.GetState(z, transcription.TotalPoints));
        }

        [TestMethod]
        public void SetStateThrowsForWrongDimension()
        {
            var problem = new ControlProblem().WithStateSize(2).WithControlSize(1);
            var grid = new CollocationGrid(0.0, 10.0, 5);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var z = new double[transcription.DecisionVectorSize];
            var wrongState = new[] { 1.0, 2.0, 3.0 }; // 3 elements instead of 2

            Assert.ThrowsException<ArgumentException>(() => transcription.SetState(z, 0, wrongState));
        }

        [TestMethod]
        public void SimpleIntegratorSatisfiesDynamics()
        {
            // Test: ẋ = u with u = constant
            // For constant control, LGL should have very small defects
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 5.0, 10);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 4);

            // Create solution x(t) = t (exact for ẋ = 1)
            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                // Compute time at this global point
                // Need to determine which segment and local index
                var segmentIndex = Math.Min(i / (transcription.Order - 1), grid.Segments - 1);
                var localIndex = i - segmentIndex * (transcription.Order - 1);

                // Get LGL points for reference coordinate
                var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(transcription.Order);
                var tau = lglPoints[localIndex];

                var tk = grid.TimePoints[segmentIndex];
                var h = grid.GetTimeStep(segmentIndex);
                var t = tk + (tau + 1.0) * h / 2.0;

                transcription.SetState(z, i, new[] { t });
                transcription.SetControl(z, i, s_unitControl);
            }

            // Dynamics: ẋ = u
            double[] DynamicsEvaluator(double[] x, double[] u, double t)
            {
                return new[] { u[0] };
            }

            var defects = transcription.ComputeAllDefects(z, DynamicsEvaluator);
            var maxDefect = LegendreGaussLobattoTranscription.MaxDefect(defects);

            // Should be very small for exact linear solution
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

            var grid = new CollocationGrid(0.0, 4.0, 10);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 5);

            // Create exact analytical solution at all LGL points
            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                var segmentIndex = Math.Min(i / (transcription.Order - 1), grid.Segments - 1);
                var localIndex = i - segmentIndex * (transcription.Order - 1);

                var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(transcription.Order);
                var tau = lglPoints[localIndex];

                var tk = grid.TimePoints[segmentIndex];
                var h = grid.GetTimeStep(segmentIndex);
                var t = tk + (tau + 1.0) * h / 2.0;

                transcription.SetState(z, i, new[] { 0.5 * t * t, t });
                transcription.SetControl(z, i, s_unitControl);
            }

            // Dynamics: [dx/dt, dv/dt] = [v, u]
            double[] DynamicsEvaluator(double[] x, double[] u, double t)
            {
                return new[] { x[1], u[0] };
            }

            var defects = transcription.ComputeAllDefects(z, DynamicsEvaluator);
            var maxDefect = LegendreGaussLobattoTranscription.MaxDefect(defects);

            // LGL with order 5 should exactly represent polynomials up to degree 4
            // Our solution is degree 2, so defects should be essentially zero
            Assert.IsTrue(maxDefect < 1e-10, $"Max defect should be near zero for polynomial solution, was {maxDefect}");
        }

        [TestMethod]
        public void HigherOrderGivesBetterAccuracy()
        {
            // Test: exponential dynamics ẋ = -x with x(0) = 1
            // Analytical: x(t) = e^(-t)
            // Higher LGL order should give better approximation
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 2.0, 5);

            // Dynamics: ẋ = -x
            double[] DynamicsEvaluator(double[] x, double[] u, double t)
            {
                return new[] { -x[0] };
            }

            var maxDefectOrder3 = TestExponentialDecay(problem, grid, order: 3, DynamicsEvaluator);
            var maxDefectOrder5 = TestExponentialDecay(problem, grid, order: 5, DynamicsEvaluator);
            var maxDefectOrder7 = TestExponentialDecay(problem, grid, order: 7, DynamicsEvaluator);

            // Higher orders should give progressively better approximations
            Assert.IsTrue(maxDefectOrder5 < maxDefectOrder3,
                $"Order 5 defect ({maxDefectOrder5}) should be less than order 3 ({maxDefectOrder3})");
            Assert.IsTrue(maxDefectOrder7 < maxDefectOrder5,
                $"Order 7 defect ({maxDefectOrder7}) should be less than order 5 ({maxDefectOrder5})");
        }

        private static double TestExponentialDecay(
            ControlProblem problem,
            CollocationGrid grid,
            int order,
            Func<double[], double[], double, double[]> dynamicsEvaluator)
        {
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            // Create analytical solution x(t) = e^(-t)
            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                var segmentIndex = Math.Min(i / (order - 1), grid.Segments - 1);
                var localIndex = i - segmentIndex * (order - 1);

                var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(order);
                var tau = lglPoints[localIndex];

                var tk = grid.TimePoints[segmentIndex];
                var h = grid.GetTimeStep(segmentIndex);
                var t = tk + (tau + 1.0) * h / 2.0;

                transcription.SetState(z, i, new[] { Math.Exp(-t) });
                transcription.SetControl(z, i, s_zeroControl);
            }

            var defects = transcription.ComputeAllDefects(z, dynamicsEvaluator);
            return LegendreGaussLobattoTranscription.MaxDefect(defects);
        }

        [TestMethod]
        public void InitialGuessCreatesLinearInterpolation()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 5);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var z = transcription.CreateInitialGuess(s_initialState, s_finalState, s_zeroControl);

            // Check initial state (first point)
            var state0 = transcription.GetState(z, 0);
            Assert.AreEqual(0.0, state0[0], Tolerance);
            Assert.AreEqual(1.0, state0[1], Tolerance);

            // Check final state (last point)
            var stateN = transcription.GetState(z, transcription.TotalPoints - 1);
            Assert.AreEqual(10.0, stateN[0], Tolerance);
            Assert.AreEqual(5.0, stateN[1], Tolerance);

            // Check that intermediate states are reasonable (between initial and final)
            for (var i = 1; i < transcription.TotalPoints - 1; i++)
            {
                var state = transcription.GetState(z, i);
                Assert.IsTrue(state[0] >= 0.0 && state[0] <= 10.0,
                    $"State[{i}][0] = {state[0]} should be between 0 and 10");
                Assert.IsTrue(state[1] >= 1.0 && state[1] <= 5.0,
                    $"State[{i}][1] = {state[1]} should be between 1 and 5");
            }

            // Check control is constant everywhere
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                var control = transcription.GetControl(z, i);
                Assert.AreEqual(0.0, control[0], Tolerance);
            }
        }

        [TestMethod]
        public void InitialGuessHandlesNaNInFinalState()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 3);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var finalStateWithNaN = new[] { 10.0, double.NaN }; // Second state is free
            var z = transcription.CreateInitialGuess(s_initialState, finalStateWithNaN, s_unitControl);

            // First state component should interpolate from 0 to 10
            var state0 = transcription.GetState(z, 0);
            Assert.AreEqual(0.0, state0[0], Tolerance);

            var stateN = transcription.GetState(z, transcription.TotalPoints - 1);
            Assert.AreEqual(10.0, stateN[0], Tolerance);

            // Second state component should stay constant at initial value (1.0)
            Assert.AreEqual(1.0, state0[1], Tolerance);
            Assert.AreEqual(1.0, stateN[1], Tolerance);
        }

        [TestMethod]
        public void RunningCostIntegrationIsAccurate()
        {
            // Test: integrate L(x,u,t) = u^2 over [0, 10] with u = 2
            // Analytical: ∫[0,10] 4 dt = 40
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 5);

            // Create solution with constant control u = 2
            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                transcription.SetState(z, i, s_zeroControl);
                transcription.SetControl(z, i, s_doubleControl);
            }

            // Running cost: L(x,u,t) = u^2
            double RunningCostEvaluator(double[] x, double[] u, double t)
            {
                return u[0] * u[0];
            }

            var cost = transcription.ComputeRunningCost(z, RunningCostEvaluator);

            // Should be very close to 40
            Assert.AreEqual(40.0, cost, 1e-10);
        }

        [TestMethod]
        public void RunningCostIntegrationForPolynomial()
        {
            // Test: integrate L(x,u,t) = t^2 over [0, 2]
            // Analytical: ∫[0,2] t^2 dt = [t^3/3]_0^2 = 8/3 ≈ 2.6667
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 2.0, 5);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 5);

            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                transcription.SetState(z, i, s_zeroControl);
                transcription.SetControl(z, i, s_zeroControl);
            }

            // Running cost: L(x,u,t) = t^2
            double RunningCostEvaluator(double[] x, double[] u, double t)
            {
                return t * t;
            }

            var cost = transcription.ComputeRunningCost(z, RunningCostEvaluator);
            var analyticalCost = 8.0 / 3.0;

            // LGL quadrature of order 5 integrates polynomials of degree ≤ 2*5-3 = 7 exactly
            // t^2 is degree 2, so should be exact
            Assert.AreEqual(analyticalCost, cost, 1e-10);
        }

        [TestMethod]
        public void TerminalCostIsCorrect()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 5.0, 10);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var z = transcription.CreateInitialGuess(s_initialState, s_finalState, s_zeroControl);

            // Terminal cost: Φ(x,t) = x[0]^2 + x[1]^2
            double TerminalCostEvaluator(double[] x, double t)
            {
                return x[0] * x[0] + x[1] * x[1];
            }

            var cost = transcription.ComputeTerminalCost(z, TerminalCostEvaluator);

            // Final state is [10, 5], so cost = 100 + 25 = 125
            Assert.AreEqual(125.0, cost, Tolerance);
        }

        [TestMethod]
        public void TotalCostCombinesBothTerms()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 4);

            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                transcription.SetState(z, i, s_constantState5);
                transcription.SetControl(z, i, s_doubleControl);
            }

            // Running cost: L = u^2 = 4, integrated over [0,1] → 4
            double RunningCostEvaluator(double[] x, double[] u, double t)
            {
                return u[0] * u[0];
            }

            // Terminal cost: Φ = x^2 = 25
            double TerminalCostEvaluator(double[] x, double t)
            {
                return x[0] * x[0];
            }

            var totalCost = transcription.ComputeTotalCost(z, RunningCostEvaluator, TerminalCostEvaluator);

            // Should be 4 + 25 = 29
            Assert.AreEqual(29.0, totalCost, 1e-10);
        }

        [TestMethod]
        public void TotalCostHandlesNullEvaluators()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 3);
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order: 3);

            var z = transcription.CreateInitialGuess(s_initialState1D, s_finalState1D, s_unitControl);

            // Null running cost evaluator
            var cost1 = transcription.ComputeTotalCost(z, null, (x, t) => 10.0);
            Assert.AreEqual(10.0, cost1, Tolerance);

            // Null terminal cost evaluator
            var cost2 = transcription.ComputeTotalCost(z, (x, u, t) => 5.0, null);
            Assert.IsTrue(cost2 > 0); // Should have some running cost

            // Both null
            var cost3 = transcription.ComputeTotalCost(z, null, null);
            Assert.AreEqual(0.0, cost3, Tolerance);
        }

        [TestMethod]
        public void MaxDefectReturnsMaximumAbsoluteValue()
        {
            var defects = new[] { -0.5, 0.2, -0.8, 0.3 };
            var max = LegendreGaussLobattoTranscription.MaxDefect(defects);
            Assert.AreEqual(0.8, max, Tolerance);
        }

        protected abstract ICollocationTranscription CreateTranscription(ControlProblem problem, CollocationGrid grid, int order);
    }
}
