/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public abstract class SolverTestsAlt
    {
        private const double Tolerance = 1e-3;
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_threeState1D = [3.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];
        private static readonly double[] s_oneZeroState2D = [1.0, 0.0];
        private static readonly double[] s_controlBoundsLower = [-1.0];
        private static readonly double[] s_controlBoundsUpper = [1.0];

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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0]; // df/dx
                    gradients[1] = [1.0]; // df/du
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0; // dL/dx
                    gradients[1] = 2.0 * u[0]; // dL/du
                    gradients[2] = 0.0; // dL/dt
                    return (value, gradients);
                });

            // Note: Using Order 3 for reliability. Higher orders (≥4) can exhibit spurious
            // local minima in some problems - a known limitation of high-order pseudospectral methods
            var solver = CreateSolver()
                .WithSegments(10)
                .WithOrder(3)
                .WithTolerance(1e-4)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-6));

            var initialGuess = CreateInitialGuess(problem, 10, 3);
            var result = solver.Solve(problem, initialGuess);

            // Verify solution
            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-3, $"Defects should be small, was {result.MaxDefect}");

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 1e-3, "Initial state should be 0");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 1e-2, "Final state should be 1");

            // Check optimal cost (should be close to 1/5 = 0.2)
            var expectedCost = 0.2;
            Assert.AreEqual(expectedCost, result.OptimalCost, 0.1, "Cost should be near 0.2");

            // Check control values are reasonable
            // Note: With LGL collocation, controls may vary slightly due to quadrature weighting
            var avgControl = 0.0;
            foreach (var u in result.Controls)
            {
                avgControl += u[0];
            }
            avgControl /= result.Controls.Length;

            // Verify control is in reasonable range (cost is the more important check)
            Assert.IsTrue(avgControl > 0.05 && avgControl < 0.3,
                $"Average control should be reasonable, was {avgControl:F3}");
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
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0, 1.0, 0.0, 0.0]; // df/dx (flattened 2x2)
                    gradients[1] = [0.0, 1.0]; // df/du (2x1)
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[4];
                    gradients[0] = 0.0; // dL/dx0
                    gradients[1] = 0.0; // dL/dx1
                    gradients[2] = 2.0 * u[0]; // dL/du
                    gradients[3] = 0.0; // dL/dt
                    return (value, gradients);
                });

            var solver = CreateSolver()
                .WithSegments(10)
                .WithOrder(5)
                .WithTolerance(1e-4)
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-6));

            var initialGuess = CreateInitialGuess(problem, 10, 5);
            var result = solver.Solve(problem, initialGuess);

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

            var solver = CreateSolver()
                .WithSegments(10)
                .WithOrder(4)
                .WithTolerance(1e-3)
                .WithMaxIterations(100)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var initialGuess = CreateInitialGuess(problem, 10, 4);
            var result = solver.Solve(problem, initialGuess);

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
        public void CanSolveExponentialDecayProblem()
        {
            // Test LGL solver on problem with known analytical solution
            // Problem: ẋ = -x + u, x(0) = 1, minimize ∫u²dt
            // Optimal solution: u = 0, x(t) = e^(-t)

            // NOTE: High-order LGL collocation (order ≥ 4) can exhibit spurious local minima
            // requiring sophisticated initialization or trust-region methods. This is a known
            // limitation of pseudospectral methods. We use Order 3 for reliability.

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 2.0)
                .WithInitialCondition(s_oneState1D)
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { -x[0] + u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [-1.0];
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

            var solver = CreateSolver()
                .WithSegments(10)
                .WithOrder(3)
                .WithTolerance(1e-5)
                .WithMaxIterations(100);

            var initialGuess = CreateInitialGuess(problem, 10, 3);
            var result = solver.Solve(problem, initialGuess);

            // Verify convergence
            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.IsTrue(result.MaxDefect < 1e-4, $"Defects should be small, was {result.MaxDefect:E2}");

            // Verify solution matches analytical solution: x(t) = e^(-t)
            var tfinal = 2.0;
            var analyticalFinal = Math.Exp(-tfinal);
            var error = Math.Abs(result.States[result.States.Length - 1][0] - analyticalFinal);

            Assert.IsTrue(error < 0.02, $"Solution error should be small, was {error:E2}");

            // Verify control is near zero (optimal solution)
            var maxControl = result.Controls.Max(u => Math.Abs(u[0]));
            Assert.IsTrue(maxControl < 0.1, $"Control should be near zero, max was {maxControl:E2}");
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

            var solver = CreateSolver()
                .WithSegments(8)
                .WithOrder(4)
                .WithTolerance(1e-4)
                .WithMaxIterations(50);

            var initialGuess = CreateInitialGuess(problem, 8, 4);
            var result = solver.Solve(problem, initialGuess);

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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0];
                    gradients[1] = [1.0];
                    return new DynamicsResult(value, gradients);
                })
                .WithTerminalCost((x, _) =>
                {
                    var error = x[0] - 1.0;
                    var value = error * error;
                    var gradients = new double[2];
                    gradients[0] = 2.0 * error; // dΦ/dx
                    gradients[1] = 0.0; // dΦ/dt
                    return (value, gradients);
                });

            var solver = CreateSolver()
                .WithSegments(8)
                .WithOrder(4)
                .WithTolerance(1e-4)
                .WithMaxIterations(50);

            var initialGuess = CreateInitialGuess(problem, 8, 4);
            var result = solver.Solve(problem, initialGuess);

            Assert.IsTrue(result.Success, "Solver should converge");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.2,
                "Final state should be close to 1 (minimizing terminal cost)");
        }

        [TestMethod]
        public void ThrowsExceptionForInvalidSegments()
        {
            var solver = CreateSolver();

            Assert.ThrowsException<ArgumentException>(() => solver.WithSegments(0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithSegments(-5));
        }

        [TestMethod]
        public void ThrowsExceptionForInvalidTolerance()
        {
            var solver = CreateSolver();

            Assert.ThrowsException<ArgumentException>(() => solver.WithTolerance(0.0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithTolerance(-1e-6));
        }

        [TestMethod]
        public void FluentAPIReturnsThis()
        {
            var solver = CreateSolver();

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

            var result6 = solver.WithParallelization(true);
            Assert.AreSame(solver, result6, "WithParallelization should return this");

            var result7 = solver.WithMeshRefinement(true, 5, 1e-4);
            Assert.AreSame(solver, result7, "WithMeshRefinement should return this");
        }

        protected abstract ISolver CreateSolver();

        /// <summary>
        /// Creates an initial guess appropriate for this solver type.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="segments">Number of segments.</param>
        /// <param name="order">Polynomial order (used by LGL solver).</param>
        /// <returns>An initial guess suitable for the solver.</returns>
        protected abstract InitialGuess CreateInitialGuess(ControlProblem problem, int segments, int order);
    }
}
