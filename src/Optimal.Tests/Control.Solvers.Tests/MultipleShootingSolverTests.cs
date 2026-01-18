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
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public sealed class MultipleShootingSolverTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_threeState1D = [3.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];
        private static readonly double[] s_oneZeroState2D = [1.0, 0.0];
        private static readonly double[] s_controlBoundsLower = [-1.0];
        private static readonly double[] s_controlBoundsUpper = [1.0];
        private static readonly double[] s_stateBoundsLower = [-0.5];
        private static readonly double[] s_stateBoundsUpper = [1.5];

        [TestMethod]
        public void WithSegmentsReturnsSelf()
        {
            var solver = new MultipleShootingSolver();
            var result = solver.WithSegments(5);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithSegmentsThrowsWhenZeroOrNegative()
        {
            var solver = new MultipleShootingSolver();
            Assert.ThrowsException<ArgumentException>(() => solver.WithSegments(0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithSegments(-1));
        }

        [TestMethod]
        public void WithShootingIntervalsReturnsSelf()
        {
            var solver = new MultipleShootingSolver();
            var result = solver.WithShootingIntervals(3);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithShootingIntervalsThrowsWhenZeroOrNegative()
        {
            var solver = new MultipleShootingSolver();
            Assert.ThrowsException<ArgumentException>(() => solver.WithShootingIntervals(0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithShootingIntervals(-1));
        }

        [TestMethod]
        public void WithToleranceReturnsSelf()
        {
            var solver = new MultipleShootingSolver();
            var result = solver.WithTolerance(1e-4);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithMaxIterationsReturnsSelf()
        {
            var solver = new MultipleShootingSolver();
            var result = solver.WithMaxIterations(200);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithInnerOptimizerReturnsSelf()
        {
            var solver = new MultipleShootingSolver();
            var optimizer = new LBFGSOptimizer();
            var result = solver.WithInnerOptimizer(optimizer);
            Assert.AreSame(solver, result);
        }

        [TestMethod]
        public void WithInnerOptimizerThrowsWhenNull()
        {
            var solver = new MultipleShootingSolver();
            Assert.ThrowsException<ArgumentNullException>(() => solver.WithInnerOptimizer(null!));
        }

        [TestMethod]
        public void SolveThrowsWhenProblemIsNull()
        {
            var solver = new MultipleShootingSolver();
            Assert.ThrowsException<ArgumentNullException>(() => solver.Solve(null!));
        }

        [TestMethod]
        public void SolveThrowsWhenDynamicsNotDefined()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D);

            var solver = new MultipleShootingSolver();
            Assert.ThrowsException<InvalidOperationException>(() => solver.Solve(problem));
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveSimpleIntegratorMinimumEnergy()
        {
            // Problem: min ∫ u² dt
            // Subject to: ẋ = u, x(0) = 0, x(5) = 1

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
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(5)
                .WithShootingIntervals(2)
                .WithTolerance(1e-3)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, $"Solver should converge: {result.Message}");
            Assert.AreEqual(0.0, result.States[0][0], 1e-2, "Initial state should be 0");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.1, "Final state should be 1");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveDoubleIntegratorMinimumEnergy()
        {
            // Problem: min ∫ u² dt
            // Subject to: ẍ = u, [x,v](0) = [0,0], [x,v](2) = [1,0]

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
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[4];
                    gradients[0] = 0.0;
                    gradients[1] = 0.0;
                    gradients[2] = 2.0 * u[0];
                    gradients[3] = 0.0;
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(8)
                .WithShootingIntervals(3)
                .WithTolerance(1e-3)
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, $"Solver should converge: {result.Message}");
            Assert.AreEqual(0.0, result.States[0][0], 1e-2, "Initial position should be 0");
            Assert.AreEqual(0.0, result.States[0][1], 1e-2, "Initial velocity should be 0");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithControlBounds()
        {
            // Problem with control bounds

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_threeState1D)
                .WithControlBounds(s_controlBoundsLower, s_controlBoundsUpper)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(5)
                .WithShootingIntervals(2)
                .WithTolerance(1e-3)
                .WithMaxIterations(100);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, $"Solver should converge: {result.Message}");
            foreach (var u in result.Controls)
            {
                Assert.IsTrue(u[0] >= -1.0 - 1e-6, "Control should be >= -1");
                Assert.IsTrue(u[0] <= 1.0 + 1e-6, "Control should be <= 1");
            }
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithStateBounds()
        {
            // Problem with state bounds

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithStateBounds(s_stateBoundsLower, s_stateBoundsUpper)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(5)
                .WithShootingIntervals(2)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, $"Solver should converge: {result.Message}");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithTerminalCost()
        {
            // Problem with terminal cost

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
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
                .WithRunningCost((_, u, _) =>
                {
                    var value = 0.5 * u[0] * u[0];
                    var gradients = new double[3];
                    gradients[1] = u[0];
                    return (value, gradients);
                })
                .WithTerminalCost(input =>
                {
                    var value = 0.5 * (input.State[0] - 1.0) * (input.State[0] - 1.0);
                    var gradients = new double[2];
                    gradients[0] = input.State[0] - 1.0;
                    return new TerminalCostResult(value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(5)
                .WithShootingIntervals(2)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, $"Solver should converge: {result.Message}");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void SolutionTimesAreContinuous()
        {
            var problem = new ControlProblem()
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
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(5)
                .WithShootingIntervals(4)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success);

            // Verify times are monotonically increasing
            for (var i = 1; i < result.Times.Length; i++)
            {
                Assert.IsTrue(result.Times[i] > result.Times[i - 1],
                    $"Times should be monotonically increasing: t[{i - 1}]={result.Times[i - 1]}, t[{i}]={result.Times[i]}");
            }

            // Verify time range
            Assert.AreEqual(0.0, result.Times[0], 1e-10, "First time should be 0");
            Assert.AreEqual(4.0, result.Times[result.Times.Length - 1], 1e-10, "Last time should be 4");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveWithSingleShootingInterval()
        {
            // Test edge case with just one shooting interval (reduces to direct collocation)

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
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
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(10)
                .WithShootingIntervals(1)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, $"Solver should converge: {result.Message}");
            Assert.AreEqual(0.0, result.States[0][0], 1e-2, "Initial state should be 0");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.1, "Final state should be 1");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void ResultContainsValidMetadata()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
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
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new MultipleShootingSolver()
                .WithSegments(5)
                .WithShootingIntervals(2)
                .WithTolerance(1e-3);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success);
            Assert.IsNotNull(result.Message);
            Assert.IsTrue(result.Times.Length > 0);
            Assert.AreEqual(result.Times.Length, result.States.Length);
            Assert.AreEqual(result.Times.Length, result.Controls.Length);
            Assert.IsTrue(result.OptimalCost >= 0);
        }
    }
}
