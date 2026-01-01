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
    public sealed class LegendreGaussLobattoSolverTests
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

            var solver = new LegendreGaussLobattoSolver()
                .WithSegments(10)
                .WithOrder(4)
                .WithTolerance(1e-4)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-6));

            var result = solver.Solve(problem);

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
                    gradients[0] = new[] { 0.0, 1.0, 0.0, 0.0 }; // df/dx (flattened 2x2)
                    gradients[1] = new[] { 0.0, 1.0 }; // df/du (2x1)
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

            var solver = new LegendreGaussLobattoSolver()
                .WithSegments(10)
                .WithOrder(5)
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

            var solver = new LegendreGaussLobattoSolver()
                .WithSegments(10)
                .WithOrder(4)
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
        public void HigherOrderGivesBetterAccuracy()
        {
            // Test that higher LGL order provides better accuracy
            // Problem: ẋ = -x with x(0) = 1 (exponential decay)
            // Analytical solution: x(t) = e^(-t)
            // Running cost: minimize control effort (which should be zero for this problem)

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { -x[0] + u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { -1.0 };
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

            // Solve with order 3
            var solver3 = new LegendreGaussLobattoSolver()
                .WithSegments(5)
                .WithOrder(3)
                .WithTolerance(1e-5)
                .WithMaxIterations(100);

            var result3 = solver3.Solve(problem);

            // Solve with order 5
            var solver5 = new LegendreGaussLobattoSolver()
                .WithSegments(5)
                .WithOrder(5)
                .WithTolerance(1e-5)
                .WithMaxIterations(100);

            var result5 = solver5.Solve(problem);

            // Solve with order 7
            var solver7 = new LegendreGaussLobattoSolver()
                .WithSegments(5)
                .WithOrder(7)
                .WithTolerance(1e-5)
                .WithMaxIterations(100);

            var result7 = solver7.Solve(problem);

            // All should converge
            Assert.IsTrue(result3.Success, "Order 3 should converge");
            Assert.IsTrue(result5.Success, "Order 5 should converge");
            Assert.IsTrue(result7.Success, "Order 7 should converge");

            // Higher order should have smaller defects
            Assert.IsTrue(result5.MaxDefect < result3.MaxDefect,
                $"Order 5 defect ({result5.MaxDefect:E2}) should be < order 3 ({result3.MaxDefect:E2})");
            Assert.IsTrue(result7.MaxDefect < result5.MaxDefect,
                $"Order 7 defect ({result7.MaxDefect:E2}) should be < order 5 ({result5.MaxDefect:E2})");

            // Verify analytical solution at final time
            var tfinal = 2.0;
            var analyticalFinal = Math.Exp(-tfinal);

            var error3 = Math.Abs(result3.States[result3.States.Length - 1][0] - analyticalFinal);
            var error5 = Math.Abs(result5.States[result5.States.Length - 1][0] - analyticalFinal);
            var error7 = Math.Abs(result7.States[result7.States.Length - 1][0] - analyticalFinal);

            // Higher order should have smaller error
            Assert.IsTrue(error5 < error3, $"Order 5 error ({error5:E2}) should be < order 3 ({error3:E2})");
            Assert.IsTrue(error7 < error5, $"Order 7 error ({error7:E2}) should be < order 5 ({error5:E2})");
        }

        [TestMethod]
        public void CanSolveWithOnlyRunningCost()
        {
            // No terminal cost, only running cost
            // Problem: min ∫ u² dt
            // Subject to: ẋ = u, x(0) = 0, x(2) = 1

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
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new LegendreGaussLobattoSolver()
                .WithSegments(8)
                .WithOrder(4)
                .WithTolerance(1e-4)
                .WithMaxIterations(50);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-3, $"Defects should be small, was {result.MaxDefect}");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.1, "Final state should be 1");
        }

        [TestMethod]
        public void CanSolveWithOnlyTerminalCost()
        {
            // Only terminal cost, no running cost
            // Problem: min (x(2) - 1)²
            // Subject to: ẋ = u, x(0) = 0
            // Should drive state to 1 at final time

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_zeroState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                })
                .WithTerminalCost((x, t) =>
                {
                    var error = x[0] - 1.0;
                    var value = error * error;
                    var gradients = new double[2];
                    gradients[0] = 2.0 * error; // dΦ/dx
                    gradients[1] = 0.0; // dΦ/dt
                    return (value, gradients);
                });

            var solver = new LegendreGaussLobattoSolver()
                .WithSegments(8)
                .WithOrder(4)
                .WithTolerance(1e-4)
                .WithMaxIterations(50);

            var result = solver.Solve(problem);

            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.2,
                "Final state should be close to 1 (minimizing terminal cost)");
        }

        [TestMethod]
        public void ThrowsExceptionForInvalidOrder()
        {
            var solver = new LegendreGaussLobattoSolver();

            Assert.ThrowsException<ArgumentException>(() => solver.WithOrder(1));
            Assert.ThrowsException<ArgumentException>(() => solver.WithOrder(0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithOrder(-1));
        }

        [TestMethod]
        public void ThrowsExceptionForInvalidSegments()
        {
            var solver = new LegendreGaussLobattoSolver();

            Assert.ThrowsException<ArgumentException>(() => solver.WithSegments(0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithSegments(-5));
        }

        [TestMethod]
        public void ThrowsExceptionForInvalidTolerance()
        {
            var solver = new LegendreGaussLobattoSolver();

            Assert.ThrowsException<ArgumentException>(() => solver.WithTolerance(0.0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithTolerance(-1e-6));
        }

        [TestMethod]
        public void FluentAPIReturnsThis()
        {
            var solver = new LegendreGaussLobattoSolver();

            var result1 = solver.WithSegments(10);
            Assert.AreSame(solver, result1, "WithSegments should return this");

            var result2 = solver.WithOrder(5);
            Assert.AreSame(solver, result2, "WithOrder should return this");

            var result3 = solver.WithTolerance(1e-4);
            Assert.AreSame(solver, result3, "WithTolerance should return this");

            var result4 = solver.WithMaxIterations(100);
            Assert.AreSame(solver, result4, "WithMaxIterations should return this");

            var result5 = solver.WithVerbose(true);
            Assert.AreSame(solver, result5, "WithVerbose should return this");
        }
    }
}
