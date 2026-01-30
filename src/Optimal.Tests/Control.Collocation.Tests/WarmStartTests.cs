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
using Optimal.Control.Core;
using Optimal.Control.Solvers;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Collocation.Tests
{
    [TestClass]
    public sealed class WarmStartTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_twoState1D = [2.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];
        private static readonly double[] s_targetState2D = [1.0, 0.0];

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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return new RunningCostResult(value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(problem, 10);
            var result = solver.Solve(problem, initialGuess);
            Assert.IsTrue(result.Success, "First solve should converge");

            // Create a new grid with different resolution
            var newGrid = new CollocationGrid(0.0, 5.0, 15);

            // Interpolate to new grid
            var warmStartGuess = WarmStart.InterpolateFromPrevious(result, newGrid);

            // Verify size
            Assert.AreEqual(16, warmStartGuess.Points); // 15 segments + 1

            // Verify interpolated values are reasonable
            var firstState = warmStartGuess.StateTrajectory[0];
            var lastState = warmStartGuess.StateTrajectory[15];

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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = 0.5 * u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = u[0];
                    gradients[2] = 0.0;
                    return new RunningCostResult(value, gradients);
                });

            // First solve with fewer segments
            var solver1 = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var initialGuess1 = InitialGuessFactory.CreateWithControlHeuristics(problem, 8);
            var result1 = solver1.Solve(problem, initialGuess1);
            Assert.IsTrue(result1.Success, "Coarse solution should converge");

            // Now solve with more segments using warm start
            var solver2 = new HermiteSimpsonSolver()
                .WithSegments(20)
                .WithTolerance(1e-4)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var newGrid = new CollocationGrid(0.0, 5.0, 20);
            var warmStart = WarmStart.InterpolateFromPrevious(result1, newGrid);

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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return new RunningCostResult(value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(problem1, 10);
            var result1 = solver.Solve(problem1, initialGuess);
            Assert.IsTrue(result1.Success);

            // Scale to T = 10
            var newGrid = new CollocationGrid(0.0, 10.0, 10);

            var scaledGuess = WarmStart.ScaleTimeHorizon(result1, 0.0, 10.0, newGrid);

            // Verify boundary conditions are preserved
            var firstState = scaledGuess.StateTrajectory[0];
            var lastState = scaledGuess.StateTrajectory[10];

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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return new RunningCostResult(value, gradients);
                });

            var problem2 = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_twoState1D) // Different target
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return new RunningCostResult(value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var initialGuess1 = InitialGuessFactory.CreateWithControlHeuristics(problem1, 10);
            var initialGuess2 = InitialGuessFactory.CreateWithControlHeuristics(problem2, 10);
            var result1 = solver.Solve(problem1, initialGuess1);
            var result2 = solver.Solve(problem2, initialGuess2);

            Assert.IsTrue(result1.Success);
            Assert.IsTrue(result2.Success);

            // Blend at lambda = 0.5
            var grid = new CollocationGrid(0.0, 5.0, 10);

            var blended = WarmStart.BlendSolutions(result1, result2, 0.5, grid);

            // Verify blended final state is midway
            var finalState = blended.StateTrajectory[10];
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
                .WithDynamics(input => new DynamicsResult(new[] { input.Control[0] }, new double[2][]))
                .WithRunningCost(input => new RunningCostResult(input.Control[0] * input.Control[0], new double[3]));

            var solver = new HermiteSimpsonSolver()
                .WithSegments(5)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(problem, 5);
            var result = solver.Solve(problem, initialGuess);

            var grid = new CollocationGrid(0.0, 5.0, 5);

            Assert.Throws<ArgumentOutOfRangeException>(() =>
                WarmStart.BlendSolutions(result, result, -0.1, grid));

            Assert.Throws<ArgumentOutOfRangeException>(() =>
                WarmStart.BlendSolutions(result, result, 1.1, grid));
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
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0, 1.0];
                    gradients[1] = [1.0, 0.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = 0.5 * u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 0.0;
                    gradients[2] = u[0];
                    return new RunningCostResult(value, gradients);
                });

            // Solve coarse first
            var solver1 = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var initialGuess1 = InitialGuessFactory.CreateWithControlHeuristics(problem, 8);
            var result1 = solver1.Solve(problem, initialGuess1);
            Assert.IsTrue(result1.Success, "Coarse solution should converge");

            // Refine with warm start
            var solver2 = new HermiteSimpsonSolver()
                .WithSegments(16)
                .WithTolerance(1e-4)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var grid2 = new CollocationGrid(0.0, 2.0, 16);
            var warmStart = WarmStart.InterpolateFromPrevious(result1, grid2);

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
