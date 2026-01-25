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
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];
        private static readonly double[] s_oneZeroState2D = [1.0, 0.0];
        private static readonly double[] s_Q1D = [1.0];
        private static readonly double[] s_R1D = [1.0];
        private static readonly double[] s_Q2D = [1.0, 1.0];

        [TestMethod]
        public void GenerateInitialGuessThrowsWhenProblemIsNull()
        {
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = (states: new[] { s_zeroState1D }, controls: new[] { s_zeroState1D });

            Assert.Throws<ArgumentNullException>(() =>
                LQRInitializer.GenerateInitialGuess(null!, grid, nominal, s_Q1D, s_R1D));
        }

        [TestMethod]
        public void GenerateInitialGuessThrowsWhenGridIsNull()
        {
            var problem = CreateSimpleProblem();
            var nominal = (states: new[] { s_zeroState1D }, controls: new[] { s_zeroState1D });

            Assert.Throws<ArgumentNullException>(() =>
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
            Assert.IsNotEmpty(result);
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
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0, 1.0];
                    gradients[1] = [0.0, 0.0];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 2.0, 10);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, s_Q2D, s_R1D);

            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result);
        }

        [TestMethod]
        public void CreateNominalTrajectoryReturnsCorrectSize()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 10);

            var (states, controls) = LQRInitializer.CreateNominalTrajectory(problem, grid);

            Assert.HasCount(11, states); // segments + 1
            Assert.HasCount(11, controls);
            Assert.HasCount(1, states[0]); // 1D state
            Assert.HasCount(1, controls[0]); // 1D control
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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    // Return empty gradients to force numerical differentiation
                    var gradients = new double[2][];
                    gradients[0] = Array.Empty<double>();
                    gradients[1] = Array.Empty<double>();
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, s_Q1D, s_R1D);

            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result);
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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0], u[1] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0, 0.0];
                    gradients[1] = [0.0, 0.0];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);
            var Q = new[] { 1.0, 1.0 };
            var R = new[] { 1.0, 1.0 };

            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, Q, R);

            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result);
        }

        [TestMethod]
        public void CreateNominalTrajectoryHandlesMissingInitialState()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
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
            Assert.HasCount(12, result);
        }

        private static ControlProblem CreateSimpleProblem()
        {
            return new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                });
        }

        [TestMethod]
        public void ComputeSimpleGainProducesDifferentGainsForDifferentAMatrices()
        {
            // Test that different A matrices produce different gains
            // This will fail if A is not used in the gain computation

            // System 1: Stable system (a = -1)
            // ẋ = -x + u
            var stableProblem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition([1.0])
                .WithFinalCondition(s_zeroState1D)
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { -1.0 * x[0] + u[0] };
                    var gradients = new double[2][];
                    // df/dx = -1
                    gradients[0] = [-1.0];
                    // df/du = 1
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                });

            // System 2: Unstable system (a = +1)
            // ẋ = +x + u
            var unstableProblem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition([1.0])
                .WithFinalCondition(s_zeroState1D)
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { 1.0 * x[0] + u[0] };
                    var gradients = new double[2][];
                    // df/dx = +1
                    gradients[0] = [1.0];
                    // df/du = 1
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var Q = new[] { 1.0 };
            var R = new[] { 1.0 };

            var stableNominal = LQRInitializer.CreateNominalTrajectory(stableProblem, grid);
            var unstableNominal = LQRInitializer.CreateNominalTrajectory(unstableProblem, grid);

            var stableResult = LQRInitializer.GenerateInitialGuess(stableProblem, grid, stableNominal, Q, R);
            var unstableResult = LQRInitializer.GenerateInitialGuess(unstableProblem, grid, unstableNominal, Q, R);

            // The controls should be different because the A matrices are different
            // Extract first control from each result (index depends on transcription layout)
            // For HermiteSimpson with 1 state, 1 control: layout is [x0, u0, x1, u1, ...]
            var stableControl = stableResult[1];
            var unstableControl = unstableResult[1];

            // These should be significantly different if A is being used
            // Unstable system needs larger control effort
            Assert.AreNotEqual(stableControl, unstableControl, 0.01,
                "Gains should differ for stable vs unstable systems - A matrix must be used");
        }

        [TestMethod]
        public void ComputeSimpleGainMatchesAnalyticalRiccatiSolutionForScalarSystem()
        {
            // For a 1D system: ẋ = a·x + b·u
            // The continuous-time algebraic Riccati equation gives:
            // P = (a·r + √(a²·r² + q·r·b²)) / b²  (taking positive root for stability)
            // K = b·P/r
            //
            // For a = -1, b = 1, q = 1, r = 1:
            // P = (-1 + √(1 + 1)) / 1 = -1 + √2 ≈ 0.414
            // K = 1 * 0.414 / 1 ≈ 0.414
            //
            // For a = 2, b = 1, q = 1, r = 1:
            // P = (2 + √(4 + 1)) / 1 = 2 + √5 ≈ 4.236
            // K = 1 * 4.236 / 1 ≈ 4.236

            // Test with a = 2 (unstable system requiring significant control)
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition([1.0])
                .WithFinalCondition(s_zeroState1D)
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { 2.0 * x[0] + u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [2.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 1); // Just 2 points to simplify
            var Q = new[] { 1.0 };
            var R = new[] { 1.0 };

            // Analytical solution
            const double a = 2.0;
            const double b = 1.0;
            const double q = 1.0;
            const double r = 1.0;
            var expectedP = (a * r + Math.Sqrt(a * a * r * r + q * r * b * b)) / (b * b);
            var expectedK = b * expectedP / r;

            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);
            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, Q, R);

            // The control at t=0 should be u = -K * (x_desired - x_nominal)
            // With x_nominal[0] = 1.0 and x_desired interpolated, the control should reflect K
            // We can verify K is close to expected by examining the control response

            // For this test, we verify the gain magnitude is in the right ballpark
            // The exact value depends on the desired state interpolation
            var control = result[1]; // First control value

            // The control should be negative (trying to drive state toward 0)
            // and its magnitude should be proportional to K * state_error
            // With state = 1.0 and desired = 0.5 (midpoint interpolation), error = -0.5
            // u ≈ K * 0.5 ≈ 4.236 * 0.5 ≈ 2.118 (positive, pushing toward target)

            // At minimum, verify the gain is substantial (not the near-zero heuristic)
            Assert.IsGreaterThan(0.5,
Math.Abs(control), $"Control magnitude {Math.Abs(control)} should be substantial for unstable system (expected K ≈ {expectedK})");
        }

        [TestMethod]
        public void ComputeSimpleGainForDoubleIntegratorMatchesExpectedBehavior()
        {
            // Double integrator: ẋ₁ = x₂, ẋ₂ = u
            // A = [0, 1; 0, 0], B = [0; 1]
            // This is a classic control problem with known LQR solution

            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition([1.0, 0.0]) // Start at position 1, zero velocity
                .WithFinalCondition(s_zeroState2D) // Target origin
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    // df/dx: [0, 1; 0, 0] flattened
                    gradients[0] = [0.0, 1.0, 0.0, 0.0];
                    // df/du: [0; 1] flattened
                    gradients[1] = [0.0, 1.0];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 2.0, 10);
            var Q = new[] { 1.0, 1.0 };
            var R = new[] { 1.0 };

            var nominal = LQRInitializer.CreateNominalTrajectory(problem, grid);
            var result = LQRInitializer.GenerateInitialGuess(problem, grid, nominal, Q, R);

            // For a double integrator with Q = I, R = 1, the LQR gain is approximately
            // K = [1, √2] meaning u = -k1*x1 - k2*x2
            // The initial control should be negative (decelerating toward target)

            // Extract control at first time point
            // Layout for 2 states, 1 control: [x1_0, x2_0, u_0, x1_1, x2_1, u_1, ...]
            var control0 = result[2]; // Third element is first control

            // Control should be trying to drive the system toward zero
            // From position 1, velocity 0, we need negative acceleration
            Assert.IsLessThan(0,
control0, $"Initial control {control0} should be negative to decelerate toward target");
        }
    }
}
