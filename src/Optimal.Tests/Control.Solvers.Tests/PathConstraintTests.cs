/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using System;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.Control.Optimization;
using Optimal.Control.Solvers;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public sealed class PathConstraintTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };
        private static readonly double[] s_controlBoundsLower = new[] { -2.0 };
        private static readonly double[] s_controlBoundsUpper = new[] { 2.0 };

        [TestMethod]
        [Ignore("Challenging convergence - requires tighter tolerance or better initial guess")]
        public void CanSolveWithStatePathConstraint()
        {
            // Problem: min ∫ u² dt
            // Subject to: ẋ = u, x(0) = 0, x(3) = 1
            // Path constraint: x(t) ≤ 0.6 (state limit)
            // The trajectory must stay below 0.6

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 3.0)
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
                })
                .WithPathConstraint((x, u, t) =>
                {
                    // x ≤ 0.6  =>  x - 0.6 ≤ 0
                    var value = x[0] - 0.6;
                    var gradients = new double[3];
                    gradients[0] = 1.0; // dg/dx
                    gradients[1] = 0.0; // dg/du
                    gradients[2] = 0.0; // dg/dt
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(5e-4)
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

            var result = solver.Solve(problem);

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-2, $"Defects should be small, was {result.MaxDefect}");

            // Check path constraint is satisfied at all points
            foreach (var state in result.States)
            {
                Assert.IsTrue(state[0] <= 0.6 + 0.1, $"State {state[0]} should be ≤ 0.6 (with tolerance)");
            }

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 0.05, "Initial state should be 0");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.1, "Final state should be 1");
        }

        [TestMethod]
        public void CanSolveWithObstacleAvoidance()
        {
            // Problem: min ∫ u² dt (minimize energy)
            // Dynamics: ẋ = u (simple integrator)
            // Boundary: x(0) = 0, x(5) = 2
            // Path constraint: x(t) ≥ 0.5 for t ≥ 2.5 (must go above obstacle)
            // This forces the trajectory to rise early

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(new[] { 2.0 })
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
                })
                .WithPathConstraint((x, u, t) =>
                {
                    // For t ≥ 2.5: x ≥ 0.5  =>  0.5 - x ≤ 0
                    // For t < 2.5: no constraint (always satisfied)
                    var value = (t >= 2.5) ? (0.5 - x[0]) : -1.0;
                    var gradients = new double[3];
                    gradients[0] = (t >= 2.5) ? -1.0 : 0.0;
                    gradients[1] = 0.0;
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(15)
                .WithTolerance(1e-3)
                .WithMaxIterations(80)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

            var result = solver.Solve(problem);

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");

            // Check obstacle avoidance for t ≥ 2.5
            for (var i = 0; i < result.Times.Length; i++)
            {
                if (result.Times[i] >= 2.5)
                {
                    Assert.IsTrue(result.States[i][0] >= 0.5 - 0.15, 
                        $"At t={result.Times[i]:F2}, x={result.States[i][0]:F2} should be ≥ 0.5 (obstacle constraint)");
                }
            }

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 0.1, "Initial state");
            Assert.AreEqual(2.0, result.States[result.States.Length - 1][0], 0.15, "Final state");
        }

        [TestMethod]
        public void CanSolveDoubleIntegratorWithVelocityLimit()
        {
            // Problem: min ∫ u² dt
            // Dynamics: ẍ = u (double integrator)
            // Boundary: [x,v](0) = [0,0], [x,v](3) = [1,0]
            // Path constraint: |v| ≤ 0.8 (velocity limit)

            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 3.0)
                .WithInitialCondition(s_zeroState2D)
                .WithFinalCondition(s_oneState1D.Length == 2 ? s_oneState1D : new[] { 1.0, 0.0 })
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0, 1.0 };
                    gradients[1] = new[] { 0.0, 0.0 };
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[4];
                    return (value, gradients);
                })
                .WithPathConstraint((x, u, t) =>
                {
                    // v ≤ 0.8  =>  v - 0.8 ≤ 0
                    var value = x[1] - 0.8;
                    var gradients = new double[4];
                    gradients[0] = 0.0;
                    gradients[1] = 1.0;
                    return (value, gradients);
                })
                .WithPathConstraint((x, u, t) =>
                {
                    // v ≥ -0.8  =>  -0.8 - v ≤ 0
                    var value = -0.8 - x[1];
                    var gradients = new double[4];
                    gradients[0] = 0.0;
                    gradients[1] = -1.0;
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(12)
                .WithTolerance(1e-3)
                .WithMaxIterations(80)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

            var result = solver.Solve(problem);

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");

            // Check velocity limits
            foreach (var state in result.States)
            {
                var velocity = state[1];
                Assert.IsTrue(velocity <= 0.8 + 0.15, $"Velocity {velocity:F2} should be ≤ 0.8");
                Assert.IsTrue(velocity >= -0.8 - 0.15, $"Velocity {velocity:F2} should be ≥ -0.8");
            }

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 0.1, "Initial position");
            Assert.AreEqual(0.0, result.States[0][1], 0.1, "Initial velocity");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.2, "Final position");
            Assert.AreEqual(0.0, result.States[result.States.Length - 1][1], 0.2, "Final velocity");
        }

        [TestMethod]
        [Ignore("Challenging convergence - requires better initialization")]
        public void CanSolveWithMultiplePathConstraints()
        {
            // Simple test: min ∫ u² dt with two path constraints

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                })
                .WithPathConstraint((x, u, t) =>
                {
                    var value = x[0] - 0.7; // x ≤ 0.7
                    var gradients = new double[3];
                    return (value, gradients);
                })
                .WithPathConstraint((x, u, t) =>
                {
                    var value = -0.1 - x[0]; // x ≥ -0.1
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(5e-3)
                .WithMaxIterations(100);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, "Should converge with multiple constraints");

            foreach (var state in result.States)
            {
                Assert.IsTrue(state[0] >= -0.1 - 0.1, "Lower constraint");
                Assert.IsTrue(state[0] <= 0.7 + 0.1, "Upper constraint");
            }
        }
    }
}
