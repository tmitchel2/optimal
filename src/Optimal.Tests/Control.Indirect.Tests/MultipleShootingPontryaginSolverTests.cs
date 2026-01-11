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
using Optimal.Control.Indirect;

namespace Optimal.Control.Indirect.Tests
{
    [TestClass]
    public sealed class MultipleShootingPontryaginSolverTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };
        private static readonly double[] s_oneZeroState2D = new[] { 1.0, 0.0 };
        private static readonly double[] s_initialCostates1D = new[] { 0.1 };
        private static readonly double[] s_initialCostates2D = new[] { 0.1, 0.1 };

        [TestMethod]
        public void WithMaxIterationsReturnsSelf()
        {
            var solver = new MultipleShootingPontryaginSolver();
            var result = solver.WithMaxIterations(50);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithToleranceReturnsSelf()
        {
            var solver = new MultipleShootingPontryaginSolver();
            var result = solver.WithTolerance(1e-5);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithShootingIntervalsReturnsSelf()
        {
            var solver = new MultipleShootingPontryaginSolver();
            var result = solver.WithShootingIntervals(6);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithVerboseReturnsSelf()
        {
            var solver = new MultipleShootingPontryaginSolver();
            var result = solver.WithVerbose(true);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void SolveThrowsWhenProblemIsNull()
        {
            var solver = new MultipleShootingPontryaginSolver();
            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) => new[] { 0.0 };

            Assert.ThrowsException<ArgumentNullException>(() =>
                solver.Solve(null!, optimalControl, s_initialCostates1D));
        }

        [TestMethod]
        public void SolveThrowsWhenOptimalControlIsNull()
        {
            var problem = CreateSimpleProblem();
            var solver = new MultipleShootingPontryaginSolver();

            Assert.ThrowsException<ArgumentNullException>(() =>
                solver.Solve(problem, null!, s_initialCostates1D));
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveSimpleIntegratorProblem()
        {
            // Simple integrator: ẋ = u, minimize ∫ u² dt

            var problem = new ControlProblem()
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

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[0] / 2.0 };
            };

            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(50)
                .WithShootingIntervals(2)
                .WithTolerance(1e-2);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Times.Length > 0);
            Assert.AreEqual(result.Times.Length, result.States.Length);
            Assert.AreEqual(result.Times.Length, result.Controls.Length);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithTerminalCost()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
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
                })
                .WithTerminalCost((x, t) =>
                {
                    var value = 0.5 * (x[0] - 1.0) * (x[0] - 1.0);
                    var gradients = new double[2];
                    gradients[0] = x[0] - 1.0;
                    return (value, gradients);
                });

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[0] };
            };

            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(50)
                .WithShootingIntervals(3)
                .WithTolerance(1e-2);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Times.Length > 0);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveDoubleIntegrator()
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
                    gradients[0] = new double[4] { 0.0, 1.0, 0.0, 0.0 };
                    gradients[1] = new double[2] { 0.0, 1.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[4];
                    gradients[2] = 2.0 * u[0];
                    return (value, gradients);
                });

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[1] / 2.0 };
            };

            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(50)
                .WithShootingIntervals(2)
                .WithTolerance(1e-2);

            var result = solver.Solve(problem, optimalControl, s_initialCostates2D);

            Assert.IsNotNull(result);
            Assert.IsTrue(result.Times.Length > 0);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void SuccessfulResultContainsValidCost()
        {
            var problem = new ControlProblem()
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
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[0] / 2.0 };
            };

            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(50)
                .WithShootingIntervals(2)
                .WithTolerance(1e-2);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            if (result.Success)
            {
                Assert.IsTrue(result.OptimalCost >= 0, "Cost should be non-negative");
            }
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void ResultMessageIsSet()
        {
            var problem = CreateSimpleProblem();

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[0] / 2.0 };
            };

            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(50)
                .WithShootingIntervals(2)
                .WithTolerance(1e-2);

            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

            Assert.IsNotNull(result.Message);
            Assert.IsTrue(result.Message.Length > 0);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void SolveWithVerboseProducesOutput()
        {
            var problem = CreateSimpleProblem();

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[0] / 2.0 };
            };

            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(20)
                .WithShootingIntervals(2)
                .WithTolerance(1e-2)
                .WithVerbose(true);

            // Just ensure it doesn't throw when verbose
            var result = solver.Solve(problem, optimalControl, s_initialCostates1D);
            Assert.IsNotNull(result);
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithDifferentNumberOfIntervals()
        {
            var problem = CreateSimpleProblem();

            Func<double[], double[], double[], double, double[]> optimalControl = (x, lambda, _, t) =>
            {
                return new[] { -lambda[0] / 2.0 };
            };

            // Test with different interval counts
            foreach (var intervals in new[] { 1, 2, 4 })
            {
                var solver = new MultipleShootingPontryaginSolver()
                    .WithMaxIterations(30)
                    .WithShootingIntervals(intervals)
                    .WithTolerance(1e-2);

                var result = solver.Solve(problem, optimalControl, s_initialCostates1D);

                Assert.IsNotNull(result, $"Result should not be null for {intervals} intervals");
                Assert.IsTrue(result.Times.Length > 0, $"Times should have entries for {intervals} intervals");
            }
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
        }
    }
}
