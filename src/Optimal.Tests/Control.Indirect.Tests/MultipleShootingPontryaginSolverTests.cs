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
    public sealed class MultipleShootingPontryaginSolverTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_initialCostates1D = [0.1];

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
            Func<double[], double[], double[], double, double[]> optimalControl = (_, _, _, _) => [0.0];

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
        public void FluentMethodsChainingWorks()
        {
            var solver = new MultipleShootingPontryaginSolver()
                .WithMaxIterations(100)
                .WithTolerance(1e-4)
                .WithShootingIntervals(8)
                .WithVerbose(false);

            Assert.IsNotNull(solver);
        }

        private static ControlProblem CreateSimpleProblem()
        {
            return new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithDynamics((_, u, _) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return (value, gradients);
                });
        }
    }
}
