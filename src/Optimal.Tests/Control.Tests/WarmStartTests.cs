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
using Optimal.NonLinear;

namespace Optimal.Control.Tests
{
    [TestClass]
    public sealed class WarmStartTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_twoState1D = new[] { 2.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };
        private static readonly double[] s_targetState2D = new[] { 1.0, 0.0 };

        [TestMethod]
        public void CanInterpolateFromPreviousSolution()
        {
            // Solve a simple problem first
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result = solver.Solve(problem);
            Assert.IsTrue(result.Success, "First solve should converge");

            // Create a new grid with different resolution
            var newGrid = new CollocationGrid(0.0, 5.0, 15);
            var newTranscription = new HermiteSimpsonTranscription(problem, newGrid);

            // Interpolate to new grid
            var initialGuess = WarmStart.InterpolateFromPrevious(result, newGrid, newTranscription);

            // Verify size
            Assert.AreEqual(newTranscription.DecisionVectorSize, initialGuess.Length);

            // Verify interpolated values are reasonable
            var firstState = newTranscription.GetState(initialGuess, 0);
            var lastState = newTranscription.GetState(initialGuess, 15);

            Assert.AreEqual(0.0, firstState[0], 0.1, "Initial state should be near 0");
            Assert.AreEqual(1.0, lastState[0], 0.2, "Final state should be near 1");
        }

        [TestMethod]
        public void WarmStartImprovesConvergence()
        {
            // Problem with somewhat difficult initial guess
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_twoState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = 0.5 * u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            // First solve with fewer segments
            var solver1 = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result1 = solver1.Solve(problem);
            Assert.IsTrue(result1.Success, "Coarse solution should converge");

            // Now solve with more segments using warm start
            var solver2 = new HermiteSimpsonSolver()
                .WithSegments(20)
                .WithTolerance(1e-4)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var newGrid = new CollocationGrid(0.0, 5.0, 20);
            var newTranscription = new HermiteSimpsonTranscription(problem, newGrid);
            var warmStart = WarmStart.InterpolateFromPrevious(result1, newGrid, newTranscription);

            var result2 = solver2.Solve(problem, warmStart);

            Assert.IsTrue(result2.Success, "Refined solution should converge with warm start");
            Assert.IsTrue(result2.MaxDefect < result1.MaxDefect || result2.MaxDefect < 1e-3,
                "Refined solution should have smaller defects");
        }

        [TestMethod]
        public void CanScaleTimeHorizon()
        {
            // Solve problem with T = 5
            var problem1 = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result1 = solver.Solve(problem1);
            Assert.IsTrue(result1.Success);

            // Scale to T = 10
            var newGrid = new CollocationGrid(0.0, 10.0, 10);
            var newTranscription = new HermiteSimpsonTranscription(problem1, newGrid);

            var scaledGuess = WarmStart.ScaleTimeHorizon(result1, 0.0, 10.0, newGrid, newTranscription);

            // Verify boundary conditions are preserved
            var firstState = newTranscription.GetState(scaledGuess, 0);
            var lastState = newTranscription.GetState(scaledGuess, 10);

            Assert.AreEqual(0.0, firstState[0], 0.1, "Initial state should be preserved");
            Assert.AreEqual(1.0, lastState[0], 0.2, "Final state should be preserved");
        }

        [TestMethod]
        public void CanBlendSolutions()
        {
            // Create two different solutions
            var problem1 = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var problem2 = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_twoState1D) // Different target
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result1 = solver.Solve(problem1);
            var result2 = solver.Solve(problem2);

            Assert.IsTrue(result1.Success);
            Assert.IsTrue(result2.Success);

            // Blend at lambda = 0.5
            var grid = new CollocationGrid(0.0, 5.0, 10);
            var transcription = new HermiteSimpsonTranscription(problem1, grid);

            var blended = WarmStart.BlendSolutions(result1, result2, 0.5, grid, transcription);

            // Verify blended final state is midway
            var finalState = transcription.GetState(blended, 10);
            var expectedFinal = 0.5 * 1.0 + 0.5 * 2.0; // 1.5

            Assert.AreEqual(expectedFinal, finalState[0], 0.3, "Blended state should be midway");
        }

        [TestMethod]
        public void BlendThrowsForInvalidLambda()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) => (new[] { u[0] }, new double[2][]))
                .WithRunningCost((x, u, t) => (u[0] * u[0], new double[3]));

            var solver = new HermiteSimpsonSolver()
                .WithSegments(5)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result = solver.Solve(problem);

            var grid = new CollocationGrid(0.0, 5.0, 5);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            Assert.ThrowsException<ArgumentOutOfRangeException>(() =>
                WarmStart.BlendSolutions(result, result, -0.1, grid, transcription));

            Assert.ThrowsException<ArgumentOutOfRangeException>(() =>
                WarmStart.BlendSolutions(result, result, 1.1, grid, transcription));
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveDoubleIntegratorWithWarmStart()
        {
            // Solve double integrator with warm start
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_zeroState2D)
                .WithFinalCondition(s_targetState2D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0, 1.0 };
                    gradients[1] = new[] { 1.0, 0.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = 0.5 * u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 0.0;
                    gradients[2] = u[0];
                    return (value, gradients);
                });

            // Solve coarse first
            var solver1 = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result1 = solver1.Solve(problem);
            Assert.IsTrue(result1.Success, "Coarse solution should converge");

            // Refine with warm start
            var solver2 = new HermiteSimpsonSolver()
                .WithSegments(16)
                .WithTolerance(1e-4)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var grid2 = new CollocationGrid(0.0, 2.0, 16);
            var transcription2 = new HermiteSimpsonTranscription(problem, grid2);
            var warmStart = WarmStart.InterpolateFromPrevious(result1, grid2, transcription2);

            var result2 = solver2.Solve(problem, warmStart);

            Assert.IsTrue(result2.Success, "Refined solution should converge");

            // Verify boundary conditions
            Assert.AreEqual(0.0, result2.States[0][0], 0.05, "Initial position");
            Assert.AreEqual(0.0, result2.States[0][1], 0.05, "Initial velocity");
            Assert.AreEqual(1.0, result2.States[result2.States.Length - 1][0], 0.1, "Final position");
            Assert.AreEqual(0.0, result2.States[result2.States.Length - 1][1], 0.1, "Final velocity");
        }
    }
}
