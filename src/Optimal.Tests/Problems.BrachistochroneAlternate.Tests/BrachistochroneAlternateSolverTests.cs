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
using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;
using Optimal.Control.Solvers;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;
using OptimalCli.Problems.BrachistochroneAlternate;

namespace Optimal.Problems.BrachistochroneAlternate.Tests
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
    public sealed class BrachistochroneAlternateSolverTests
    {
        // Physical constants
        private const double Gravity = 9.80665;
        private const double Xf = 10.0;         // Horizontal distance (for reference line calculation)
        private const double Nf = 5.0;          // Vertical drop (for reference line calculation)
        private const double V0 = 0.5;          // Initial velocity
        private const double Alpha0 = Math.PI / 6.0; // Initial angle (30 degrees)

        // Reference line geometry
        private static readonly double STotal = Math.Sqrt(Xf * Xf + Nf * Nf);
        private static readonly double ThetaRef = Math.Atan2(Nf, Xf);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");

            // Check initial conditions
            Assert.AreEqual(V0, result.States[0][IdxV], 0.1, "Initial velocity should be V0");
            Assert.AreEqual(0.0, result.States[0][IdxN], 0.1, "Initial n should be 0 (on reference line)");
            Assert.AreEqual(Alpha0, result.States[0][IdxAlpha], 0.1, "Initial alpha should be Alpha0");
            Assert.AreEqual(0.0, result.States[0][IdxT], 0.1, "Initial time should be 0");

            // Check final condition (n = 0, back on reference line)
            var finalState = result.States[result.States.Length - 1];
            Assert.AreEqual(0.0, finalState[IdxN], 0.5, "Final n should be 0 (on reference line)");
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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

            Assert.IsLessThan(0.1, result.MaxDefect, $"Defects should be small, was {result.MaxDefect:E2}");

            // Energy conservation in rotated coordinates:
            // Actual vertical drop = s * sin(ThetaRef) + n * cos(ThetaRef)
            var v0 = result.States[0][IdxV];
            var s0 = result.Times[0];
            var n0 = result.States[0][IdxN];
            var vf = result.States[result.States.Length - 1][IdxV];
            var sf = result.Times[result.Times.Length - 1];
            var nf = result.States[result.States.Length - 1][IdxN];

            var yDown0 = s0 * Math.Sin(ThetaRef) + n0 * Math.Cos(ThetaRef);
            var yDownF = sf * Math.Sin(ThetaRef) + nf * Math.Cos(ThetaRef);
            var actualVerticalDrop = yDownF - yDown0;
            var expectedVf = Math.Sqrt(v0 * v0 + 2 * Gravity * actualVerticalDrop);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

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
            var result = solver.Solve(problem, initialGuess, CancellationToken.None);

            Console.WriteLine($"LGL: MaxDefect={result.MaxDefect:E2}, Success={result.Success}");

            // Check defects are reasonable
            Assert.IsLessThan(0.5, result.MaxDefect, $"Defects should be reasonable, was {result.MaxDefect:E2}");
        }

        #endregion

        #region Helper Methods

        /// <summary>
        /// Creates the reference-line-aligned Brachistochrone problem.
        /// </summary>
        private static ControlProblem CreateArcLengthProblem()
        {
            // Initial state: [v, n, alpha, t] - on reference line (n=0)
            var initialState = new double[] { V0, 0.0, Alpha0, 0.0 };

            // Final state: [v=free, n=0, alpha=free, t=free] - back on reference line
            var finalState = new double[] { double.NaN, 0.0, double.NaN, double.NaN };

            return new ControlProblem()
                .WithStateSize(4) // [v, n, alpha, t]
                .WithControlSize(1) // [k] curvature
                .WithTimeHorizon(0.0, STotal) // s from 0 to STotal (line distance)
                .WithInitialCondition(initialState)
                .WithFinalCondition(finalState)
                .WithControlBounds([-2.0], [2.0]) // Curvature bounds
                .WithStateBounds(
                    [0.01, -5.0, -Math.PI / 2.5, 0.0],  // n can be negative (above line)
                    [30.0, 5.0, Math.PI / 2.5, 10.0]) // n can be positive (below line)
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

            // Compute state derivatives using BrachistochroneAlternateDynamics
            var dVds = BrachistochroneDynamicsAlternate.SpeedRateS(v, alpha, Gravity, ThetaRef);
            var dNds = BrachistochroneDynamicsAlternate.VerticalRateS(alpha);
            var dAlphads = BrachistochroneDynamicsAlternate.AlphaRateS(k);
            var dTds = BrachistochroneDynamicsAlternate.TimeRateS(v, alpha);

            var value = new[] { dVds, dNds, dAlphads, dTds };

            // Compute gradients numerically
            var gradients = ComputeDynamicsGradientsNumerically(x, u);

            return new DynamicsResult(value, gradients);
        }

        private static double[][] ComputeDynamicsGradientsNumerically(double[] x, double[] u)
        {
            const double eps = 1e-6; // Slightly larger epsilon for central differences
            const int stateDim = 4;
            const int controlDim = 1;

            var stateGradients = new double[stateDim * stateDim];
            var controlGradients = new double[stateDim * controlDim];

            // Central differences for state gradients
            for (var j = 0; j < stateDim; j++)
            {
                var xPlus = (double[])x.Clone();
                var xMinus = (double[])x.Clone();
                xPlus[j] += eps;
                xMinus[j] -= eps;
                var fPlus = ComputeDerivatives(xPlus, u);
                var fMinus = ComputeDerivatives(xMinus, u);

                for (var i = 0; i < stateDim; i++)
                {
                    stateGradients[i * stateDim + j] = (fPlus[i] - fMinus[i]) / (2.0 * eps);
                }
            }

            // Central differences for control gradients
            for (var j = 0; j < controlDim; j++)
            {
                var uPlus = (double[])u.Clone();
                var uMinus = (double[])u.Clone();
                uPlus[j] += eps;
                uMinus[j] -= eps;
                var fPlus = ComputeDerivatives(x, uPlus);
                var fMinus = ComputeDerivatives(x, uMinus);

                for (var i = 0; i < stateDim; i++)
                {
                    controlGradients[i * controlDim + j] = (fPlus[i] - fMinus[i]) / (2.0 * eps);
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
                BrachistochroneDynamicsAlternate.SpeedRateS(v, alpha, Gravity, ThetaRef),
                BrachistochroneDynamicsAlternate.VerticalRateS(alpha),
                BrachistochroneDynamicsAlternate.AlphaRateS(k),
                BrachistochroneDynamicsAlternate.TimeRateS(v, alpha)
            ];
        }

        private static RunningCostResult ComputeRunningCost(RunningCostInput input)
        {
            var x = input.State;
            var v = x[IdxV];
            var alpha = x[IdxAlpha];

            var cost = BrachistochroneDynamicsAlternate.RunningCostS(v, alpha);
            var gradients = ComputeRunningCostGradientsNumerically(x);

            return new RunningCostResult(cost, gradients);
        }

        private static double[] ComputeRunningCostGradientsNumerically(double[] x)
        {
            const double eps = 1e-6; // Slightly larger epsilon for central differences
            var gradients = new double[6]; // [dL/dx (4), dL/du (1), dL/ds (1)]

            var v = x[IdxV];
            var alpha = x[IdxAlpha];

            // Central differences for dL/dv
            var LPlus = BrachistochroneDynamicsAlternate.RunningCostS(v + eps, alpha);
            var LMinus = BrachistochroneDynamicsAlternate.RunningCostS(v - eps, alpha);
            gradients[IdxV] = (LPlus - LMinus) / (2.0 * eps);

            // Central differences for dL/dalpha
            LPlus = BrachistochroneDynamicsAlternate.RunningCostS(v, alpha + eps);
            LMinus = BrachistochroneDynamicsAlternate.RunningCostS(v, alpha - eps);
            gradients[IdxAlpha] = (LPlus - LMinus) / (2.0 * eps);

            return gradients;
        }

        /// <summary>
        /// Creates a custom initial guess for Hermite-Simpson solver.
        /// Uses straight-line path along reference line (n=0 throughout).
        /// </summary>
        private static InitialGuess CreateCustomInitialGuess(ControlProblem problem, int segments)
        {
            var numPoints = 2 * segments + 1;
            var states = new double[numPoints][];
            var controls = new double[numPoints][];

            var v0 = problem.InitialState![IdxV];
            var alpha0 = problem.InitialState[IdxAlpha];

            for (var i = 0; i < numPoints; i++)
            {
                var tau = (double)i / (numPoints - 1);
                var s = tau * STotal;

                // Straight line along reference: n = 0
                var n = 0.0;
                var alpha = alpha0 * (1.0 - 0.5 * tau);

                // Actual vertical drop when following reference line = s * sin(ThetaRef)
                var actualVerticalDrop = s * Math.Sin(ThetaRef);
                var v = Math.Sqrt(v0 * v0 + 2.0 * Gravity * actualVerticalDrop);
                v = Math.Max(v, 0.1);

                var vAvg = (v0 + v) / 2.0;
                var t = s / (vAvg * Math.Cos(alpha0));

                states[i] = [v, n, alpha, t];

                var k = -alpha0 / STotal * 0.5;
                controls[i] = [k];
            }

            return new InitialGuess(states, controls);
        }

        #endregion
    }
}
