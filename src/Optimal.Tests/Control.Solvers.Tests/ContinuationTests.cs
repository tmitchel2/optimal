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
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public sealed class ContinuationTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];

        [TestMethod]
        public void CanSolveWithLinearContinuation()
        {
            // Use continuation to gradually increase the target position
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 10, Tolerance = 1e-3, MaxIterations = 50 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver)
                .WithLinearSteps(5); // 0.0, 0.25, 0.5, 0.75, 1.0

            var result = continuation.Solve(lambda =>
            {
                var target = lambda * 2.0; // Gradually move from 0 to 2

                return new ControlProblem()
                    .WithStateSize(1)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 5.0)
                    .WithInitialCondition(s_zeroState1D)
                    .WithFinalCondition([target])
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
            });

            Assert.IsTrue(result.Success, "Continuation should converge");
            Assert.AreEqual(0.0, result.States[0][0], 0.1, "Initial state");
            Assert.AreEqual(2.0, result.States[result.States.Length - 1][0], 0.2, "Final state should reach target");
        }

        [TestMethod]
        public void CanSolveWithCustomParameters()
        {
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 8, Tolerance = 1e-3, MaxIterations = 40 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver)
                .WithParameters(0.0, 0.2, 0.6, 1.0);

            var result = continuation.Solve(lambda =>
            {
                var costWeight = 1.0 + lambda * 9.0; // Cost weight from 1 to 10

                return new ControlProblem()
                    .WithStateSize(1)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 4.0)
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
                        var value = costWeight * u[0] * u[0];
                        var gradients = new double[3];
                        gradients[0] = 0.0;
                        gradients[1] = 2.0 * costWeight * u[0];
                        gradients[2] = 0.0;
                        return new RunningCostResult(value, gradients);
                    });
            });

            Assert.IsTrue(result.Success, "Continuation with custom parameters should converge");
        }

        [TestMethod]
        public void ContinuationHandlesFailure()
        {
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 5, Tolerance = 1e-6, MaxIterations = 5 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver)
                .WithLinearSteps(3);

            var result = continuation.Solve(lambda =>
            {
                return new ControlProblem()
                    .WithStateSize(1)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 5.0)
                    .WithInitialCondition(s_zeroState1D)
                    .WithFinalCondition([lambda * 5.0])
                    .WithDynamics(input =>
                    {
                        var u = input.Control;
                        var value = new[] { u[0] };
                        var gradients = new double[2][];
                        return new DynamicsResult(value, gradients);
                    })
                    .WithRunningCost(input =>
                    {
                        var u = input.Control;
                        var value = u[0] * u[0];
                        var gradients = new double[3];
                        return new RunningCostResult(value, gradients);
                    });
            });

            // Should return result (possibly failed) without throwing
            Assert.IsNotNull(result);
        }

        [TestMethod]
        public void CanSolveSequenceOfProblems()
        {
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 10, Tolerance = 1e-3 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver);

            // Create a sequence of problems with increasing target
            var problems = new ControlProblem[3];

            for (var i = 0; i < 3; i++)
            {
                var target = (i + 1) * 0.5; // 0.5, 1.0, 1.5

                problems[i] = new ControlProblem()
                    .WithStateSize(1)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 5.0)
                    .WithInitialCondition(s_zeroState1D)
                    .WithFinalCondition([target])
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
            }

            var results = continuation.SolveSequence(problems);

            Assert.HasCount(3, results);
            Assert.IsTrue(results[0].Success, "Problem 1 should converge");
            Assert.IsTrue(results[1].Success, "Problem 2 should converge with warm start");
            Assert.IsTrue(results[2].Success, "Problem 3 should converge with warm start");

            // Verify targets
            Assert.AreEqual(0.5, results[0].States[results[0].States.Length - 1][0], 0.1);
            Assert.AreEqual(1.0, results[1].States[results[1].States.Length - 1][0], 0.15);
            Assert.AreEqual(1.5, results[2].States[results[2].States.Length - 1][0], 0.2);
        }

        [TestMethod]
        [Ignore("Long running test - enable for full validation")]
        public void ContinuationHelpsWithDifficultProblem()
        {
            // Problem with nonlinear dynamics that's hard without continuation
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 15, Tolerance = 1e-3, MaxIterations = 60 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-4 }, new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver)
                .WithLinearSteps(6)
                .WithVerbose(false);

            var result = continuation.Solve(lambda =>
            {
                // Gradually introduce nonlinearity
                var nonlinearStrength = lambda * 2.0;

                return new ControlProblem()
                    .WithStateSize(2)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 3.0)
                    .WithInitialCondition(s_zeroState2D)
                    .WithFinalCondition([1.0, 0.0])
                    .WithDynamics(input =>
                    {
                        var x = input.State;
                        var u = input.Control;
                        // Van der Pol-like: ẋ₁ = x₂, ẋ₂ = -x₁ + μ(1-x₁²)x₂ + u
                        var mu = nonlinearStrength;
                        var value = new[]
                        {
                            x[1],
                            -x[0] + mu * (1.0 - x[0] * x[0]) * x[1] + u[0]
                        };
                        var gradients = new double[2][];
                        gradients[0] = [0.0, 1.0];
                        gradients[1] =
                        [
                            -1.0 - 2.0 * mu * x[0] * x[1],
                            mu * (1.0 - x[0] * x[0]),
                            1.0
                        ];
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
            });

            Assert.IsTrue(result.Success, "Continuation should help solve nonlinear problem");
            Assert.AreEqual(0.0, result.States[0][0], 0.1, "Initial state");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.3, "Final state");
        }

        [TestMethod]
        public void WithParametersValidatesInput()
        {
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 10 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver);

            // Invalid parameters outside [0, 1]
            Assert.Throws<ArgumentException>(() =>
                continuation.WithParameters(-0.1, 0.5, 1.0));

            Assert.Throws<ArgumentException>(() =>
                continuation.WithParameters(0.0, 0.5, 1.5));
        }

        [TestMethod]
        public void WithLinearStepsValidatesInput()
        {
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 10 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver);

            // Must have at least 2 steps
            Assert.Throws<ArgumentException>(() =>
                continuation.WithLinearSteps(1));

            Assert.Throws<ArgumentException>(() =>
                continuation.WithLinearSteps(0));
        }

        [TestMethod]
        public void SequenceReturnsEmptyForNoProblems()
        {
            var baseSolver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 10 },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));

            var continuation = new ContinuationSolver(baseSolver);

            var results = continuation.SolveSequence();

            Assert.IsEmpty(results);
        }
    }
}
