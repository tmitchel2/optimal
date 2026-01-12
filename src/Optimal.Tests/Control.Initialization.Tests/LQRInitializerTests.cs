/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace Optimal.Control.Initialization.Tests
{
    [TestClass]
    public sealed class LQRInitializerTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };
        private static readonly double[] s_oneZeroState2D = new[] { 1.0, 0.0 };
        private static readonly double[] s_Q1D = new[] { 1.0 };
        private static readonly double[] s_R1D = new[] { 1.0 };
        private static readonly double[] s_Q2D = new[] { 1.0, 1.0 };

        [TestMethod]
        public void GenerateInitialGuessThrowsWhenProblemIsNull()
        {
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = (states: new[] { s_zeroState1D }, controls: new[] { s_zeroState1D });

            Assert.ThrowsException<ArgumentNullException>(() =>
                LQRInitializer.GenerateInitialGuess(null!, grid, nominal, s_Q1D, s_R1D));
        }

        [TestMethod]
        public void GenerateInitialGuessThrowsWhenGridIsNull()
        {
            var problem = CreateSimpleProblem();
            var nominal = (states: new[] { s_zeroState1D }, controls: new[] { s_zeroState1D });

            Assert.ThrowsException<ArgumentNullException>(() =>
                LQRInitializer.GenerateInitialGuess(problem, null!, nominal, s_Q1D, s_R1D));
        }

        [TestMethod]
        public void GenerateInitialGuessReturnsValidDecisionVector()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, s_Q1D, s_R1D);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Length > 0);
        }

        [TestMethod]
        public void GenerateInitialGuessHandlesDoubleIntegrator()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_zeroState2D)
                .WithFinalCondition(s_oneZeroState2D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0, 1.0 };
                    gradients[1] = new[] { 0.0, 0.0 };
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 2.0, 10);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, s_Q2D, s_R1D);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Length > 0);
        }

        [TestMethod]
        public void CreateNominalTrajectoryReturnsCorrectSize()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 10);

            var (states, controls) = LQRInitializer.CreateNominalTrajectory(problem, grid);

            Assert.AreEqual(11, states.Length); // segments + 1
            Assert.AreEqual(11, controls.Length);
            Assert.AreEqual(1, states[0].Length); // 1D state
            Assert.AreEqual(1, controls[0].Length); // 1D control
        }

        [TestMethod]
        public void CreateNominalTrajectoryInterpolatesStates()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 10);

            var (states, controls) = LQRInitializer.CreateNominalTrajectory(problem, grid);

            // First state should be initial condition
            Assert.AreEqual(0.0, states[0][0], 1e-10);

            // Last state should be final condition
            Assert.AreEqual(1.0, states[10][0], 1e-10);

            // Middle should be interpolated
            Assert.AreEqual(0.5, states[5][0], 1e-10);
        }

        [TestMethod]
        public void CreateNominalTrajectoryHandlesMissingFinalState()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);

            var (states, controls) = LQRInitializer.CreateNominalTrajectory(problem, grid);

            // All states should be initial condition when no final state
            foreach (var state in states)
            {
                Assert.AreEqual(1.0, state[0], 1e-10);
            }
        }

        [TestMethod]
        public void CreateNominalTrajectoryReturnsZeroControls()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);

            var (_, controls) = LQRInitializer.CreateNominalTrajectory(problem, grid);

            foreach (var control in controls)
            {
                Assert.AreEqual(0.0, control[0], 1e-10);
            }
        }

        [TestMethod]
        public void GenerateInitialGuessWorksWithNumericalDynamicsGradients()
        {
            // Dynamics that don't return proper gradients to force numerical computation
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    // Return empty gradients to force numerical differentiation
                    var gradients = new double[2][];
                    gradients[0] = Array.Empty<double>();
                    gradients[1] = Array.Empty<double>();
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, s_Q1D, s_R1D);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Length > 0);
        }

        [TestMethod]
        public void GenerateInitialGuessHandlesMultipleControls()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(2)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState2D)
                .WithFinalCondition(s_oneZeroState2D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0], u[1] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0, 0.0 };
                    gradients[1] = new[] { 0.0, 0.0 };
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);
            var Q = new[] { 1.0, 1.0 };
            var R = new[] { 1.0, 1.0 };

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, Q, R);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Length > 0);
        }

        [TestMethod]
        public void CreateNominalTrajectoryHandlesMissingInitialState()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);

            var (states, _) = LQRInitializer.CreateNominalTrajectory(problem, grid);

            // States should be zero when no initial/final state provided
            foreach (var state in states)
            {
                Assert.AreEqual(0.0, state[0], 1e-10);
            }
        }

        [TestMethod]
        public void GenerateInitialGuessDecisionVectorHasCorrectSize()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, s_Q1D, s_R1D);

            // For HermiteSimpson: (nStates + nControls) * (segments + 1)
            // With 5 segments, 1 state, 1 control: (1 + 1) * 6 = 12
            Assert.AreEqual(12, result.Length);
        }

        private static ControlProblem CreateSimpleProblem()
        {
            return new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                });
        }
    }
}
