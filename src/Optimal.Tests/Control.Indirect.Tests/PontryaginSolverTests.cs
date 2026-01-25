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

namespace Optimal.Control.Indirect.Tests
{
    [TestClass]
    public sealed class PontryaginSolverTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];
        private static readonly double[] s_oneZeroState2D = [1.0, 0.0];
        private static readonly double[] s_initialCostates1D = [0.1];
        private static readonly double[] s_initialCostates2D = [0.1, 0.1];

        [TestMethod]
        public void WithMaxIterationsReturnsSelf()
        {
            var solver = new PontryaginSolver();
            var result = solver.WithMaxIterations(50);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithToleranceReturnsSelf()
        {
            var solver = new PontryaginSolver();
            var result = solver.WithTolerance(1e-5);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithVerboseReturnsSelf()
        {
            var solver = new PontryaginSolver();
            var result = solver.WithVerbose(true);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void SolveThrowsWhenProblemIsNull()
        {
            var solver = new PontryaginSolver();
            Func<double[], double[], double[], double, double[]> optimalControl = (_, _, _, _) => [0.0];

            Assert.Throws<ArgumentNullException>(() =>
                solver.Solve(null!, optimalControl, s_initialCostates1D));
        }

        [TestMethod]
        public void SolveThrowsWhenOptimalControlIsNull()
        {
            var problem = CreateSimpleProblem();
            var solver = new PontryaginSolver();

            Assert.Throws<ArgumentNullException>(() =>
                solver.Solve(problem, null!, s_initialCostates1D));
        }

        [TestMethod]
        public void SolveThrowsWhenInitialCostatesIsNull()
        {
            var problem = CreateSimpleProblem();
            var solver = new PontryaginSolver();
            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) => [lambda[0]];

            Assert.Throws<ArgumentNullException>(() =>
                solver.Solve(problem, optimalControl, null!));
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveSimpleIntegratorProblem()
        {
            // Simple integrator: ẋ = u, minimize ∫ u² dt
            // Optimal control from Pontryagin: u* = -λ/2

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

            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) =>
            {
                // For H = u² + λ·u, ∂H/∂u = 2u + λ = 0 → u = -λ/2
                return [-lambda[0] / 2.0];
            };

            var solver = new PontryaginSolver()
                .WithMaxIterations(100)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result.Times);
            Assert.HasCount(result.Times.Length, result.States);
            Assert.HasCount(result.Times.Length, result.Controls);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithTerminalCost()
        {
            // Problem with free final state and terminal cost

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
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
                })
                .WithTerminalCost(input =>
                {
                    var value = 0.5 * (input.State[0] - 1.0) * (input.State[0] - 1.0);
                    var gradients = new double[2];
                    gradients[0] = input.State[0] - 1.0;
                    return new TerminalCostResult(value, gradients);
                });

            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) =>
            {
                return [-lambda[0]];
            };

            var solver = new PontryaginSolver()
                .WithMaxIterations(100)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result.Times);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveDoubleIntegrator()
        {
            // Double integrator: ẋ₁ = x₂, ẋ₂ = u

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
                    gradients[0] = [0.0, 1.0, 0.0, 0.0];
                    gradients[1] = [0.0, 1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost(input =>
                {
                    var u = input.Control;
                    var value = u[0] * u[0];
                    var gradients = new double[4];
                    gradients[2] = 2.0 * u[0];
                    return new RunningCostResult(value, gradients);
                });

            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) =>
            {
                return [-lambda[1] / 2.0];
            };

            var solver = new PontryaginSolver()
                .WithMaxIterations(100)
                .WithTolerance(1e-2);

            var result = solver.Solve(problem, optimalControl, s_initialCostates2D);

            Assert.IsNotNull(result);
            Assert.IsNotEmpty(result.Times);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void ResultContainsValidTimeRange()
        {
            var problem = CreateSimpleProblem();

            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) =>
            {
                return [-lambda[0] / 2.0];
            };

            var solver = new PontryaginSolver()
                .WithMaxIterations(50)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            Assert.AreEqual(0.0, result.Times[0], 1e-10);
            Assert.AreEqual(1.0, result.Times[result.Times.Length - 1], 1e-10);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void ResultHasMonotonicallyIncreasingTimes()
        {
            var problem = CreateSimpleProblem();

            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) =>
            {
                return [-lambda[0] / 2.0];
            };

            var solver = new PontryaginSolver()
                .WithMaxIterations(50)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            for (var i = 1; i < result.Times.Length; i++)
            {
                Assert.IsGreaterThan(result.Times[i - 1],
result.Times[i], $"Times should be monotonically increasing: t[{i - 1}]={result.Times[i - 1]}, t[{i}]={result.Times[i]}");
            }
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void SolveWithVerboseProducesOutput()
        {
            var problem = CreateSimpleProblem();

            Func<double[], double[], double[], double, double[]> optimalControl = (_, lambda, _, _) =>
            {
                return [-lambda[0] / 2.0];
            };

            var solver = new PontryaginSolver()
                .WithMaxIterations(20)
                .WithTolerance(1e-3)
                .WithVerbose(true);

            // Just ensure it doesn't throw when verbose
            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);
            Assert.IsNotNull(result);
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
    }
}
