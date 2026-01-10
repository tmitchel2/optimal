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

namespace Optimal.Control.Tests
{
    [TestClass]
    public sealed class HermiteSimpsonSolverTests
    {
        private const double Tolerance = 1e-3;
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_threeState1D = new[] { 3.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };
        private static readonly double[] s_oneZeroState2D = new[] { 1.0, 0.0 };
        private static readonly double[] s_controlBoundsLower = new[] { -1.0 };
        private static readonly double[] s_controlBoundsUpper = new[] { 1.0 };

        [TestMethod]
        [TestCategory("Integration")]
        public void CanSolveSimpleIntegratorMinimumEnergy()
        {
            // Problem: min ∫ u² dt
            // Subject to: ẋ = u, x(0) = 0, x(5) = 1
            // Optimal solution: u = 1/5 (constant), cost = 1/5

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
                    gradients[0] = new[] { 0.0 }; // df/dx
                    gradients[1] = new[] { 1.0 }; // df/du
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0; // dL/dx
                    gradients[1] = 2.0 * u[0]; // dL/du
                    gradients[2] = 0.0; // dL/dt
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(10)
                .WithTolerance(1e-4)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-6))
                .WithVerbose(true);

            var result = solver.Solve(problem);

            // Debug output
            Console.WriteLine($"HS Solver Test Results:");
            Console.WriteLine($"  Success: {result.Success}");
            Console.WriteLine($"  Cost: {result.OptimalCost}");
            Console.WriteLine($"  MaxDefect: {result.MaxDefect}");
            Console.WriteLine($"  Initial state: {result.States[0][0]}");
            Console.WriteLine($"  Final state: {result.States[result.States.Length - 1][0]}");
            Console.WriteLine($"  Avg control: {System.Linq.Enumerable.Average(result.Controls, u => u[0])}");

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-3, $"Defects should be small, was {result.MaxDefect}");

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 1e-3, "Initial state should be 0");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 1e-2, "Final state should be 1");

            // Check optimal cost (should be close to 1/5 = 0.2)
            var expectedCost = 0.2;
            Assert.AreEqual(expectedCost, result.OptimalCost, 0.1, "Cost should be near 0.2");

            // Check control is approximately constant
            var avgControl = 0.0;
            foreach (var u in result.Controls)
            {
                avgControl += u[0];
            }
            avgControl /= result.Controls.Length;

            Assert.AreEqual(0.2, avgControl, 0.1, "Average control should be ~0.2");
        }

        [TestMethod]
        public void CanSolveDoubleIntegratorMinimumEnergy()
        {
            // Problem: min ∫ u² dt
            // Subject to: ẍ = u, [x,v](0) = [0,0], [x,v](2) = [1,0]
            // Minimum energy with fixed endpoints

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
                    gradients[0] = new[] { 0.0, 1.0 }; // [df0/dx0, df0/dx1]
                    gradients[1] = new[] { 0.0, 0.0 }; // [df1/dx0, df1/dx1]
                    // Note: df/du would be [[0, 0], [0, 1]] but simplified
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[4];
                    gradients[0] = 0.0; // dL/dx0
                    gradients[1] = 0.0; // dL/dx1
                    gradients[2] = 2.0 * u[0]; // dL/du
                    gradients[3] = 0.0; // dL/dt
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(15)
                .WithTolerance(1e-4)
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-6));

            var result = solver.Solve(problem);

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-2, $"Defects should be small, was {result.MaxDefect}");

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 1e-2, "Initial position should be 0");
            Assert.AreEqual(0.0, result.States[0][1], 1e-2, "Initial velocity should be 0");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.1, "Final position should be 1");
            Assert.AreEqual(0.0, result.States[result.States.Length - 1][1], 0.1, "Final velocity should be 0");
        }

        [TestMethod]
        public void CanSolveWithControlBounds()
        {
            // Problem: min ∫ u² dt
            // Subject to: ẋ = u, x(0) = 0, x(5) = 3, -1 ≤ u ≤ 1
            // With saturation: optimal control will hit bounds

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_threeState1D)
                .WithControlBounds(s_controlBoundsLower, s_controlBoundsUpper)
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
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var result = solver.Solve(problem);

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");

            // Check final state is reached
            Assert.AreEqual(3.0, result.States[result.States.Length - 1][0], 0.2, "Final state should be 3");

            // Check controls respect bounds
            foreach (var u in result.Controls)
            {
                Assert.IsTrue(u[0] >= -1.0 - 1e-6, "Control should be >= -1");
                Assert.IsTrue(u[0] <= 1.0 + 1e-6, "Control should be <= 1");
            }
        }

        [TestMethod]
        public void CanSolveWithOnlyRunningCost()
        {
            // No terminal cost, only running cost

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
                .WithTolerance(1e-4);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-2, "Defects should be small");
        }

        [TestMethod]
        public void ThrowsWhenDynamicsNotDefined()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var solver = new HermiteSimpsonSolver();

            Assert.ThrowsException<InvalidOperationException>(() => solver.Solve(problem));
        }
    }
}
