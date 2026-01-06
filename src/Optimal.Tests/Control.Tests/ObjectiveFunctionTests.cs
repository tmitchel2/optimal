/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.Control.Optimization;
using Optimal.Control.Solvers;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Control.Tests
{
    [TestClass]
    public sealed class ObjectiveFunctionTests
    {
        private const double Tolerance = 1e-10;
        private static readonly double[] s_zeroControl = new[] { 0.0 };
        private static readonly double[] s_unitControl = new[] { 1.0 };
        private static readonly double[] s_zeroState = new[] { 0.0 };
        private static readonly double[] s_unitState = new[] { 1.0 };
        private static readonly double[] s_halfControl = new[] { 0.5 };
        private static readonly double[] s_twoControl = new[] { 2.0 };
        private static readonly double[] s_threeState = new[] { 3.0 };
        private static readonly double[] s_twoState = new[] { 2.0 };
        private static readonly double[] s_sevenState = new[] { 7.0 };

        [TestMethod]
        public void CanComputeRunningCostWithConstantControl()
        {
            // Test: ∫ 0.5*u² dt with u=1 from t=0 to t=10
            // Analytical: 0.5 * 1² * 10 = 5.0
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            // Create trajectory with constant control u=1
            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                transcription.SetState(z, k, s_zeroState);
                transcription.SetControl(z, k, s_unitControl);
            }

            // Cost function: 0.5 * u²
            double RunningCost(double[] x, double[] u, double t) => 0.5 * u[0] * u[0];

            var cost = transcription.ComputeRunningCost(z, RunningCost);

            Assert.AreEqual(5.0, cost, Tolerance);
        }

        [TestMethod]
        public void CanComputeRunningCostWithVaryingControl()
        {
            // Test: ∫ 0.5*u² dt with u=t from t=0 to t=2
            // Analytical: ∫[0,2] 0.5*t² dt = 0.5 * [t³/3]₀² = 0.5 * 8/3 = 4/3
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 2.0, 20); // Fine grid for accuracy
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            // Create trajectory with u(t) = t
            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                var t = grid.TimePoints[k];
                transcription.SetState(z, k, s_zeroState);
                transcription.SetControl(z, k, new[] { t });
            }

            double RunningCost(double[] x, double[] u, double t) => 0.5 * u[0] * u[0];

            var cost = transcription.ComputeRunningCost(z, RunningCost);
            var expected = 4.0 / 3.0;

            Assert.AreEqual(expected, cost, 1e-6); // Simpson's rule should be very accurate
        }

        [TestMethod]
        public void CanComputeTerminalCost()
        {
            // Test: terminal cost Φ(x) = 0.5 * (x - 5)²
            // With x_f = 3: Φ = 0.5 * (3 - 5)² = 0.5 * 4 = 2.0
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];
            transcription.SetState(z, grid.Segments, s_threeState);

            double TerminalCost(double[] x, double t)
            {
                var diff = x[0] - 5.0;
                return 0.5 * diff * diff;
            }

            var cost = transcription.ComputeTerminalCost(z, TerminalCost);

            Assert.AreEqual(2.0, cost, Tolerance);
        }

        [TestMethod]
        public void CanComputeTotalCostWithBothTerms()
        {
            // Test: J = ∫ u² dt + (x_f - 1)²
            // With u=0.5, t∈[0,10], x_f=2
            // Running: 0.5² * 10 = 2.5
            // Terminal: (2 - 1)² = 1.0
            // Total: 3.5
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                transcription.SetState(z, k, s_twoState);
                transcription.SetControl(z, k, s_halfControl);
            }

            double RunningCost(double[] x, double[] u, double t) => u[0] * u[0];
            double TerminalCost(double[] x, double t)
            {
                var diff = x[0] - 1.0;
                return diff * diff;
            }

            var cost = transcription.ComputeTotalCost(z, RunningCost, TerminalCost);

            Assert.AreEqual(3.5, cost, Tolerance);
        }

        [TestMethod]
        public void CanComputeTotalCostWithOnlyRunning()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 5.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                transcription.SetState(z, k, s_zeroState);
                transcription.SetControl(z, k, s_unitControl);
            }

            double RunningCost(double[] x, double[] u, double t) => u[0];

            var cost = transcription.ComputeTotalCost(z, RunningCost, null);

            Assert.AreEqual(5.0, cost, Tolerance);
        }

        [TestMethod]
        public void CanComputeTotalCostWithOnlyTerminal()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 10.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];
            transcription.SetState(z, grid.Segments, s_sevenState);

            double TerminalCost(double[] x, double t) => x[0];

            var cost = transcription.ComputeTotalCost(z, null, TerminalCost);

            Assert.AreEqual(7.0, cost, Tolerance);
        }

        [TestMethod]
        public void MinimumEnergyProblemCostIsCorrect()
        {
            // Classic minimum energy problem: min ∫ u² dt
            // Subject to: ẋ = u, x(0) = 0, x(T) = 1
            // Optimal control: u = 1/T (constant)
            // Optimal cost: (1/T)² * T = 1/T
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var T = 5.0;
            var grid = new CollocationGrid(0.0, T, 20);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            // Create trajectory with optimal constant control u = 1/T
            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                var t = grid.TimePoints[k];
                transcription.SetState(z, k, new[] { t / T });
                transcription.SetControl(z, k, new[] { 1.0 / T });
            }

            double RunningCost(double[] x, double[] u, double t) => u[0] * u[0];

            var cost = transcription.ComputeRunningCost(z, RunningCost);
            var expected = 1.0 / T;

            Assert.AreEqual(expected, cost, 1e-10);
        }

        [TestMethod]
        public void TimeOptimalProblemCostIsCorrect()
        {
            // Time-optimal problem: min ∫ 1 dt = min T
            // For T = 10: cost should be 10
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var T = 10.0;
            var grid = new CollocationGrid(0.0, T, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                transcription.SetState(z, k, s_zeroState);
                transcription.SetControl(z, k, s_zeroControl);
            }

            double RunningCost(double[] x, double[] u, double t) => 1.0;

            var cost = transcription.ComputeRunningCost(z, RunningCost);

            Assert.AreEqual(T, cost, Tolerance);
        }

        [TestMethod]
        public void QuadraticStateAndControlCost()
        {
            // Test: ∫ 0.5*(x² + u²) dt
            // With x=1, u=2, T=4: 0.5*(1 + 4)*4 = 10
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 4.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = new double[transcription.DecisionVectorSize];
            for (var k = 0; k <= grid.Segments; k++)
            {
                transcription.SetState(z, k, s_unitState);
                transcription.SetControl(z, k, s_twoControl);
            }

            double RunningCost(double[] x, double[] u, double t) => 0.5 * (x[0] * x[0] + u[0] * u[0]);

            var cost = transcription.ComputeRunningCost(z, RunningCost);

            Assert.AreEqual(10.0, cost, Tolerance);
        }
    }
}
