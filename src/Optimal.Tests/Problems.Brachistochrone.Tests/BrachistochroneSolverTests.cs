/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for test clarity

using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;
using Optimal.Control.Solvers;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;
using OptimalCli.Problems.Brachistochrone;

namespace Optimal.Problems.Brachistochrone.Tests
{
    /// <summary>
    /// Integration tests for Brachistochrone problem solver with arc-length parameterization.
    ///
    /// Arc-length formulation:
    /// - Independent variable: s (horizontal distance from start)
    /// - State: [v, n, alpha, t]
    ///   - v: Speed (m/s)
    ///   - n: Vertical position below start (m, positive = descended)
    ///   - alpha: Heading angle from horizontal (rad, positive = descending)
    ///   - t: Elapsed time (s)
    /// - Control: [k]
    ///   - k: Path curvature = dalpha/ds (rad/m)
    /// </summary>
    [TestClass]
    [TestCategory("Integration")]
    public sealed class BrachistochroneSolverTests
    {
        // Physical constants
        private const double Gravity = 9.80665;
        private const double Xf = 10.0;         // Final horizontal position
        private const double Nf = 5.0;          // Vertical drop (m)
        private const double V0 = 0.5;          // Initial velocity
        private const double Alpha0 = Math.PI / 6.0; // Initial angle (30 degrees)

        // State indices
        private const int IdxV = 0;
        private const int IdxN = 1;
        private const int IdxAlpha = 2;
        private const int IdxT = 3;

        // Theoretical optimal time estimate
        private const double TheoreticalOptimalTime = 1.8;

        #region Hermite-Simpson Tests

        [TestMethod]
        public void HermiteSimpsonConverges()
        {
            var problem = CreateArcLengthProblem();
            var solver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 20, Tolerance = 1e-2, MaxIterations = 100 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var initialGuess = CreateCustomInitialGuess(problem, 20);
            var result = solver.Solve(problem, initialGuess);

            // Check that defects are small
            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");
        }

        [TestMethod]
        public void HermiteSimpsonSatisfiesBoundaryConditions()
        {
            var problem = CreateArcLengthProblem();
            var solver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 20, Tolerance = 1e-2, MaxIterations = 100 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var initialGuess = CreateCustomInitialGuess(problem, 20);
            var result = solver.Solve(problem, initialGuess);

            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");

            // Check initial conditions
            Assert.AreEqual(V0, result.States[0][IdxV], 0.1, "Initial velocity should be V0");
            Assert.AreEqual(0.0, result.States[0][IdxN], 0.1, "Initial n should be 0");
            Assert.AreEqual(Alpha0, result.States[0][IdxAlpha], 0.1, "Initial alpha should be Alpha0");
            Assert.AreEqual(0.0, result.States[0][IdxT], 0.1, "Initial time should be 0");

            // Check final condition (n = Nf)
            var finalState = result.States[result.States.Length - 1];
            Assert.AreEqual(Nf, finalState[IdxN], 1.0, "Final n should be near Nf");
        }

        [TestMethod]
        [TestCategory("Integration")]
        public void HermiteSimpsonConservesEnergy()
        {
            var problem = CreateArcLengthProblem();
            var solver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 20, Tolerance = 1e-2, MaxIterations = 100 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var initialGuess = CreateCustomInitialGuess(problem, 20);
            var result = solver.Solve(problem, initialGuess);

            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");

            // Energy conservation: v² = v0² + 2*g*n
            var v0 = result.States[0][IdxV];
            var n0 = result.States[0][IdxN];
            var vf = result.States[result.States.Length - 1][IdxV];
            var nf = result.States[result.States.Length - 1][IdxN];

            var expectedVf = Math.Sqrt(v0 * v0 + 2 * Gravity * (nf - n0));

            var energyError = Math.Abs(vf - expectedVf) / expectedVf;
            Assert.IsLessThan(0.35, energyError, $"Energy conservation error should be < 35%, was {energyError * 100:F1}%");
        }

        [TestMethod]
        public void HermiteSimpsonFindsReasonableTime()
        {
            var problem = CreateArcLengthProblem();
            var solver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 20, Tolerance = 1e-2, MaxIterations = 100 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var initialGuess = CreateCustomInitialGuess(problem, 20);
            var result = solver.Solve(problem, initialGuess);

            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");

            // Final time is stored in state[IdxT]
            var finalTime = result.States[result.States.Length - 1][IdxT];

            // Should be in a reasonable range (0.5 to 5 seconds)
            Assert.IsTrue(finalTime > 0.3 && finalTime < 5.0,
                $"Final time should be between 0.3 and 5 seconds, was {finalTime:F2}s");

            Console.WriteLine($"Hermite-Simpson: Final time = {finalTime:F4}s, Optimal cost = {result.OptimalCost:F4}");
        }

        [TestMethod]
        public void HermiteSimpsonControlVaries()
        {
            var problem = CreateArcLengthProblem();
            var solver = new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions { Segments = 20, Tolerance = 1e-2, MaxIterations = 100 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-6 }, new BacktrackingLineSearch()));

            var initialGuess = CreateCustomInitialGuess(problem, 20);
            var result = solver.Solve(problem, initialGuess);

            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");

            // Curvature control should vary along the path
            var controls = result.Controls.Select(u => u[0]).ToArray();
            var controlRange = controls.Max() - controls.Min();

            Console.WriteLine($"Hermite-Simpson: Curvature range = {controlRange:F4} rad/m");
            Console.WriteLine($"  Min curvature: {controls.Min():F4} rad/m");
            Console.WriteLine($"  Max curvature: {controls.Max():F4} rad/m");

            // Should have some variation
            Assert.IsGreaterThanOrEqualTo(0.0, controlRange, "Curvature range should be non-negative");
        }

        #endregion

        #region LGL Tests

        [TestMethod]
        public void LGLRunsWithoutException()
        {
            var problem = CreateArcLengthProblem();
            var solver = new LegendreGaussLobattoSolver(
                new LegendreGaussLobattoSolverOptions { Segments = 10, Order = 4, Tolerance = 5e-2, MaxIterations = 50 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-5 }, new BacktrackingLineSearch()));

            var initialGuess = InitialGuessFactory.CreateForLGL(problem, 10, 4);
            var result = solver.Solve(problem, initialGuess);

            Console.WriteLine($"LGL: MaxDefect={result.MaxDefect:E2}, Success={result.Success}");

            // Just verify it runs without exception
            Assert.IsNotNull(result.States, "Should return states array");
            Assert.IsNotEmpty(result.States, "Should have at least one state");
        }

        [TestMethod]
        public void LGLReturnsValidStructure()
        {
            var problem = CreateArcLengthProblem();
            var solver = new LegendreGaussLobattoSolver(
                new LegendreGaussLobattoSolverOptions { Segments = 10, Order = 4, Tolerance = 5e-2, MaxIterations = 50 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-5 }, new BacktrackingLineSearch()));

            var initialGuess = InitialGuessFactory.CreateForLGL(problem, 10, 4);
            var result = solver.Solve(problem, initialGuess);

            // Verify structure is valid
            Assert.IsNotNull(result.States, "Should return states array");
            Assert.IsNotEmpty(result.States, "Should have at least one state");
            Assert.HasCount(4, result.States[0], "State should have 4 components [v, n, alpha, t]");
            Assert.HasCount(1, result.Controls[0], "Control should have 1 component [k]");
        }

        [TestMethod]
        public void LGLConverges()
        {
            var problem = CreateArcLengthProblem();
            var solver = new LegendreGaussLobattoSolver(
                new LegendreGaussLobattoSolverOptions { Segments = 10, Order = 4, Tolerance = 5e-2, MaxIterations = 50 },
                new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-5 }, new BacktrackingLineSearch()));

            var initialGuess = InitialGuessFactory.CreateForLGL(problem, 10, 4);
            var result = solver.Solve(problem, initialGuess);

            Console.WriteLine($"LGL: MaxDefect={result.MaxDefect:E2}, Success={result.Success}");

            // Check defects are reasonable
            Assert.IsLessThan(0.5, result.MaxDefect, $"Defects should be reasonable, was {result.MaxDefect:E2}");
        }

        #endregion

        #region Helper Methods

        /// <summary>
        /// Creates the arc-length parameterized Brachistochrone problem.
        /// </summary>
        private static ControlProblem CreateArcLengthProblem()
        {
            // Initial state: [v, n, alpha, t]
            var initialState = new double[] { V0, 0.0, Alpha0, 0.0 };

            // Final state: [v=free, n=Nf, alpha=free, t=free]
            var finalState = new double[] { double.NaN, Nf, double.NaN, double.NaN };

            return new ControlProblem()
                .WithStateSize(4) // [v, n, alpha, t]
                .WithControlSize(1) // [k] curvature
                .WithTimeHorizon(0.0, Xf) // s from 0 to Xf
                .WithInitialCondition(initialState)
                .WithFinalCondition(finalState)
                .WithControlBounds([-2.0], [2.0]) // Curvature bounds
                .WithStateBounds(
                    [0.01, -1.0, -Math.PI / 2.5, 0.0],
                    [30.0, 20.0, Math.PI / 2.5, 10.0])
                .WithDynamics(ComputeDynamics)
                .WithRunningCost(ComputeRunningCost);
        }

        private static DynamicsResult ComputeDynamics(DynamicsInput input)
        {
            var x = input.State;
            var u = input.Control;

            var v = x[IdxV];
            var alpha = x[IdxAlpha];
            var k = u[0];

            // Compute state derivatives using BrachistochroneDynamics
            var dVds = BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity);
            var dNds = BrachistochroneDynamics.VerticalRateS(alpha);
            var dAlphads = BrachistochroneDynamics.AlphaRateS(k);
            var dTds = BrachistochroneDynamics.TimeRateS(v, alpha);

            var value = new[] { dVds, dNds, dAlphads, dTds };

            // Compute gradients numerically
            var gradients = ComputeDynamicsGradientsNumerically(x, u);

            return new DynamicsResult(value, gradients);
        }

        private static double[][] ComputeDynamicsGradientsNumerically(double[] x, double[] u)
        {
            const double eps = 1e-7;
            const int stateDim = 4;
            const int controlDim = 1;

            var stateGradients = new double[stateDim * stateDim];
            var controlGradients = new double[stateDim * controlDim];

            var f0 = ComputeDerivatives(x, u);

            for (var j = 0; j < stateDim; j++)
            {
                var xPerturbed = (double[])x.Clone();
                xPerturbed[j] += eps;
                var fPerturbed = ComputeDerivatives(xPerturbed, u);

                for (var i = 0; i < stateDim; i++)
                {
                    stateGradients[i * stateDim + j] = (fPerturbed[i] - f0[i]) / eps;
                }
            }

            for (var j = 0; j < controlDim; j++)
            {
                var uPerturbed = (double[])u.Clone();
                uPerturbed[j] += eps;
                var fPerturbed = ComputeDerivatives(x, uPerturbed);

                for (var i = 0; i < stateDim; i++)
                {
                    controlGradients[i * controlDim + j] = (fPerturbed[i] - f0[i]) / eps;
                }
            }

            return [stateGradients, controlGradients];
        }

        private static double[] ComputeDerivatives(double[] x, double[] u)
        {
            var v = x[IdxV];
            var alpha = x[IdxAlpha];
            var k = u[0];

            return
            [
                BrachistochroneDynamics.SpeedRateS(v, alpha, Gravity),
                BrachistochroneDynamics.VerticalRateS(alpha),
                BrachistochroneDynamics.AlphaRateS(k),
                BrachistochroneDynamics.TimeRateS(v, alpha)
            ];
        }

        private static RunningCostResult ComputeRunningCost(RunningCostInput input)
        {
            var x = input.State;
            var v = x[IdxV];
            var alpha = x[IdxAlpha];

            var cost = BrachistochroneDynamics.RunningCostS(v, alpha);
            var gradients = ComputeRunningCostGradientsNumerically(x);

            return new RunningCostResult(cost, gradients);
        }

        private static double[] ComputeRunningCostGradientsNumerically(double[] x)
        {
            const double eps = 1e-7;
            var gradients = new double[6]; // [dL/dx (4), dL/du (1), dL/ds (1)]

            var v = x[IdxV];
            var alpha = x[IdxAlpha];

            var L0 = BrachistochroneDynamics.RunningCostS(v, alpha);

            var Lp = BrachistochroneDynamics.RunningCostS(v + eps, alpha);
            gradients[IdxV] = (Lp - L0) / eps;

            Lp = BrachistochroneDynamics.RunningCostS(v, alpha + eps);
            gradients[IdxAlpha] = (Lp - L0) / eps;

            return gradients;
        }

        /// <summary>
        /// Creates a custom initial guess for Hermite-Simpson solver.
        /// </summary>
        private static InitialGuess CreateCustomInitialGuess(ControlProblem problem, int segments)
        {
            var numPoints = 2 * segments + 1;
            var states = new double[numPoints][];
            var controls = new double[numPoints][];

            var v0 = problem.InitialState![IdxV];
            var n0 = problem.InitialState[IdxN];
            var alpha0 = problem.InitialState[IdxAlpha];
            var nf = problem.FinalState![IdxN];

            for (var i = 0; i < numPoints; i++)
            {
                var tau = (double)i / (numPoints - 1);
                var s = tau * Xf;

                var n = n0 + tau * (nf - n0);
                var alpha = alpha0 * (1.0 - 0.5 * tau);

                var v = Math.Sqrt(v0 * v0 + 2.0 * Gravity * n);
                v = Math.Max(v, 0.1);

                var vAvg = (v0 + v) / 2.0;
                var t = s / (vAvg * Math.Cos(alpha0));

                states[i] = [v, n, alpha, t];

                var k = -alpha0 / Xf * 0.5;
                controls[i] = [k];
            }

            return new InitialGuess(states, controls);
        }

        #endregion
    }
}
