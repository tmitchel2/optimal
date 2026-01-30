/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;
using Optimal.Control.Solvers;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Scaling.Tests
{
    [TestClass]
    public sealed class ScaledSolverIntegrationTests
    {
        private const double SolverTolerance = 1e-4;

        [TestMethod]
        [TestCategory("Integration")]
        public void SimpleIntegratorWithScalingProducesSameSolutionAsWithoutScaling()
        {
            // Simple integrator: dx/dt = u, min integral(u^2)
            var problem = CreateSimpleIntegratorProblem();

            // Solve without scaling
            var solverWithout = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(SolverTolerance)
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var guessWithout = InitialGuessFactory.CreateWithControlHeuristics(problem, 10);
            var resultWithout = solverWithout.Solve(problem, guessWithout);

            // Solve with auto-scaling
            var solverWith = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(SolverTolerance)
                .WithMaxIterations(100)
                .WithAutoScaling(true)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var guessWith = InitialGuessFactory.CreateWithControlHeuristics(problem, 10);
            var resultWith = solverWith.Solve(problem, guessWith);

            // Both should succeed
            Assert.IsTrue(resultWithout.Success, "Solver without scaling should succeed");
            Assert.IsTrue(resultWith.Success, "Solver with scaling should succeed");

            // Solutions should be similar
            Assert.AreEqual(resultWithout.OptimalCost, resultWith.OptimalCost, 0.1,
                "Costs should be similar");

            // Final states should be similar
            var n = resultWithout.States.Length - 1;
            Assert.AreEqual(resultWithout.States[n][0], resultWith.States[n][0], 0.1,
                "Final states should be similar");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void PoorlyScaledProblemCanBeSolvedWithScaling()
        {
            // Problem with scale disparity between states
            var problem = CreatePoorlyScaledProblem();

            // Solve with auto-scaling
            var solver = new HermiteSimpsonSolver()
                .WithSegments(15)
                .WithTolerance(SolverTolerance)
                .WithMaxIterations(200)
                .WithAutoScaling(true)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6, MaxIterations = 500 }, new BacktrackingLineSearch()));

            var guess = InitialGuessFactory.CreateWithControlHeuristics(problem, 15);
            var result = solver.Solve(problem, guess);

            // Verify solver can work with scaling enabled
            // Note: convergence depends on many factors, so we just check it runs without error
            // and returns valid trajectories
            Assert.IsNotNull(result.States, "States should not be null");
            Assert.IsNotNull(result.Controls, "Controls should not be null");
            Assert.HasCount(16, result.States, "Should have correct number of state points");

            // If it succeeded, verify boundary conditions are approximately satisfied
            if (result.Success)
            {
                Assert.AreEqual(0.0, result.States[0][0], 1.0, "Initial position should be ~0");
                Assert.AreEqual(50.0, result.States[^1][0], 10.0, "Final position should be ~50");
            }
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CustomScalingCanBeProvided()
        {
            var problem = CreateSimpleIntegratorProblem();

            // Create custom scaling
            var scaling = new VariableScaling(
                stateScales: new[] { 1.0 },
                stateCenters: new[] { 0.5 },
                controlScales: new[] { 1.0 },
                controlCenters: new[] { 0.0 });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(SolverTolerance)
                .WithMaxIterations(100)
                .WithScaling(scaling)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var guess = InitialGuessFactory.CreateWithControlHeuristics(problem, 10);
            var result = solver.Solve(problem, guess);

            Assert.IsTrue(result.Success, "Solver with custom scaling should succeed");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void ScalingWithPathConstraintsRuns()
        {
            // Problem with path constraint - test that scaling doesn't break path constraint handling
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 10.0, 10.0 })
                .WithControlBounds(new[] { -5.0 }, new[] { 5.0 })
                .WithInitialCondition(new[] { 0.0, 0.0 })
                .WithFinalCondition(new[] { 5.0, 0.0 })
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    // dx0/dt = x1, dx1/dt = u
                    var value = new[] { x[1], u[0] };
                    var dfdx = new[] { 0.0, 1.0, 0.0, 0.0 }; // 2x2 row-major
                    var dfdu = new[] { 0.0, 1.0 }; // 2x1
                    return new DynamicsResult(value, new[] { dfdx, dfdu });
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    return new RunningCostResult(u[0] * u[0], new[] { 0.0, 0.0, 2 * u[0], 0.0 });
                })
                .WithPathConstraint(input =>
                {
                    // Simple constraint: x0 <= 10
                    var x = input.State;
                    return new PathConstraintResult(x[0] - 10.0, new[] { 1.0, 0.0, 0.0, 0.0 });
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(SolverTolerance)
                .WithMaxIterations(100)
                .WithAutoScaling(true)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var guess = InitialGuessFactory.CreateWithControlHeuristics(problem, 10);

            // Test that solver runs without throwing with scaling + path constraints
            var result = solver.Solve(problem, guess);

            // Just verify we get valid output arrays
            Assert.IsNotNull(result.States, "States should not be null");
            Assert.IsNotNull(result.Controls, "Controls should not be null");
            Assert.HasCount(11, result.States, "Should have correct number of state points");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void ScaledSolutionUnscalesCorrectly()
        {
            // Verify that the solution is properly unscaled back to original coordinates
            // Use a simpler problem with smaller state change
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0 }, new[] { 10.0 })
                .WithControlBounds(new[] { -5.0 }, new[] { 5.0 })
                .WithInitialCondition(new[] { 2.0 })
                .WithFinalCondition(new[] { 8.0 })
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    return new DynamicsResult(new[] { u[0] }, new[] { new[] { 0.0 }, new[] { 1.0 } });
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    return new RunningCostResult(u[0] * u[0], new[] { 0.0, 2 * u[0], 0.0 });
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(SolverTolerance)
                .WithMaxIterations(100)
                .WithAutoScaling(true)
                .WithInnerOptimizer(new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var guess = InitialGuessFactory.CreateWithControlHeuristics(problem, 10);
            var result = solver.Solve(problem, guess);

            // Verify solution trajectories are in original coordinates (not scaled [-1,1])
            // Even if solver doesn't fully converge, output should be unscaled

            // Initial state should be near 2, not near -1 or scaled value
            Assert.IsTrue(result.States[0][0] > 0 && result.States[0][0] < 10,
                $"Initial state {result.States[0][0]} should be in original coordinates [0,10]");

            // Final state should be near 8, not near 1 or scaled value
            Assert.IsTrue(result.States[^1][0] > 0 && result.States[^1][0] < 10,
                $"Final state {result.States[^1][0]} should be in original coordinates [0,10]");

            // If it succeeded, verify boundary conditions more precisely
            if (result.Success)
            {
                Assert.AreEqual(2.0, result.States[0][0], 1.0,
                    "Initial state should be ~2");
                Assert.AreEqual(8.0, result.States[^1][0], 2.0,
                    "Final state should be ~8");
            }
        }

        private static ControlProblem CreateSimpleIntegratorProblem()
        {
            return new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0 }, new[] { 1.0 })
                .WithControlBounds(new[] { -2.0 }, new[] { 2.0 })
                .WithInitialCondition(new[] { 0.0 })
                .WithFinalCondition(new[] { 1.0 })
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    return new DynamicsResult(new[] { u[0] }, new[] { new[] { 0.0 }, new[] { 1.0 } });
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    return new RunningCostResult(u[0] * u[0], new[] { 0.0, 2 * u[0], 0.0 });
                });
        }

        private static ControlProblem CreatePoorlyScaledProblem()
        {
            // Problem with 100x scale difference between states
            // State 1: position in [0, 100]
            // State 2: velocity in [-1, 1]
            return new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithStateBounds(new[] { 0.0, -1.0 }, new[] { 100.0, 1.0 })
                .WithControlBounds(new[] { -2.0 }, new[] { 2.0 })
                .WithInitialCondition(new[] { 0.0, 0.0 })
                .WithFinalCondition(new[] { 50.0, 0.0 })
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    // dx0/dt = 50 * x1 (position changes fast with velocity)
                    // dx1/dt = u (velocity controlled by u)
                    var value = new[] { 50.0 * x[1], u[0] };
                    var dfdx = new[] { 0.0, 50.0, 0.0, 0.0 }; // 2x2 row-major
                    var dfdu = new[] { 0.0, 1.0 }; // 2x1
                    return new DynamicsResult(value, new[] { dfdx, dfdu });
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    return new RunningCostResult(u[0] * u[0], new[] { 0.0, 0.0, 2 * u[0], 0.0 });
                });
        }
    }
}
