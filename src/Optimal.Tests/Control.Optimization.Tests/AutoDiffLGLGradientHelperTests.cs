/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for test clarity

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace Optimal.Control.Optimization.Tests
{
    /// <summary>
    /// Tests for AutoDiffLGLGradientHelper to verify gradient computation correctness.
    /// This is Phase 3 of the Brachistochrone solver investigation - likely location of LGL bug.
    /// </summary>
    [TestClass]
    public sealed class AutoDiffLGLGradientHelperTests
    {
        private const double Epsilon = 1e-7;
        private const double GradientTolerance = 1e-4;

        #region Defect Gradient Tests

        [TestMethod]
        public void DefectGradientMatchesNumericalForSimpleIntegrator()
        {
            // Test: ẋ = u (simple integrator)
            // Verify analytical defect gradient matches numerical
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 2);
            var order = 4;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            // Create a test decision vector with non-trivial values
            var z = transcription.CreateInitialGuess(
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 0.5 });

            // Dynamics with gradients: ẋ = u
            (double[] value, double[][] gradients) DynamicsWithGradients(double[] x, double[] u, double t)
            {
                var value_ = new[] { u[0] };
                var gradients_ = new double[2][];
                gradients_[0] = new[] { 0.0 }; // df/dx = 0
                gradients_[1] = new[] { 1.0 }; // df/du = 1
                return (value_, gradients_);
            }

            // Dynamics for defect computation
            double[] Dynamics(double[] x, double[] u, double t) => new[] { u[0] };

            // Get differentiation matrix
            var diffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(order);

            // Test gradient at each defect constraint
            var numInteriorPerSegment = order - 2;
            var totalDefects = grid.Segments * numInteriorPerSegment * problem.StateDim;

            for (var defectIdx = 0; defectIdx < totalDefects; defectIdx++)
            {
                // Map defect index to segment and interior point
                var defectsPerSegment = numInteriorPerSegment * problem.StateDim;
                var segmentIndex = defectIdx / defectsPerSegment;
                var remainder = defectIdx % defectsPerSegment;
                var interiorPointIndex = 1 + (remainder / problem.StateDim);
                var stateComponentIndex = remainder % problem.StateDim;

                // Compute analytical gradient
                var analyticalGrad = AutoDiffLGLGradientHelper.ComputeDefectGradient(
                    problem, grid, z, order, diffMatrix,
                    transcription.GetState, transcription.GetControl,
                    segmentIndex, interiorPointIndex, stateComponentIndex,
                    DynamicsWithGradients);

                // Compute numerical gradient
                var numericalGrad = ComputeNumericalDefectGradient(
                    z, defectIdx, transcription, Dynamics);

                // Compare each element
                for (var j = 0; j < z.Length; j++)
                {
                    Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                        $"Defect {defectIdx}, gradient element {j}: numerical={numericalGrad[j]}, analytical={analyticalGrad[j]}");
                }
            }
        }

        [TestMethod]
        public void DefectGradientMatchesNumericalForDoubleIntegrator()
        {
            // Test: [ẋ, v̇] = [v, u] (double integrator)
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 2.0, 2);
            var order = 4;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = transcription.CreateInitialGuess(
                new[] { 0.0, 0.0 },
                new[] { 1.0, 0.0 },
                new[] { 0.5 });

            (double[] value, double[][] gradients) DynamicsWithGradients(double[] x, double[] u, double t)
            {
                var value_ = new[] { x[1], u[0] };
                var gradients_ = new double[2][];
                // df/dx: [df0/dx0, df0/dx1; df1/dx0, df1/dx1] = [0, 1; 0, 0] flattened row-major
                gradients_[0] = new[] { 0.0, 1.0, 0.0, 0.0 };
                // df/du: [df0/du0; df1/du0] = [0; 1]
                gradients_[1] = new[] { 0.0, 1.0 };
                return (value_, gradients_);
            }

            double[] Dynamics(double[] x, double[] u, double t) => new[] { x[1], u[0] };

            var diffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(order);
            var numInteriorPerSegment = order - 2;
            var totalDefects = grid.Segments * numInteriorPerSegment * problem.StateDim;

            for (var defectIdx = 0; defectIdx < totalDefects; defectIdx++)
            {
                var defectsPerSegment = numInteriorPerSegment * problem.StateDim;
                var segmentIndex = defectIdx / defectsPerSegment;
                var remainder = defectIdx % defectsPerSegment;
                var interiorPointIndex = 1 + (remainder / problem.StateDim);
                var stateComponentIndex = remainder % problem.StateDim;

                var analyticalGrad = AutoDiffLGLGradientHelper.ComputeDefectGradient(
                    problem, grid, z, order, diffMatrix,
                    transcription.GetState, transcription.GetControl,
                    segmentIndex, interiorPointIndex, stateComponentIndex,
                    DynamicsWithGradients);

                var numericalGrad = ComputeNumericalDefectGradient(
                    z, defectIdx, transcription, Dynamics);

                for (var j = 0; j < z.Length; j++)
                {
                    Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                        $"Double integrator defect {defectIdx}, element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
                }
            }
        }

        [TestMethod]
        public void DefectGradientMatchesNumericalForNonlinearDynamics()
        {
            // Test: ẋ = sin(u) + x^2 (nonlinear dynamics)
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 2);
            var order = 3;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = transcription.CreateInitialGuess(
                new[] { 0.1 },
                new[] { 0.5 },
                new[] { 0.3 });

            (double[] value, double[][] gradients) DynamicsWithGradients(double[] x, double[] u, double t)
            {
                var value_ = new[] { Math.Sin(u[0]) + x[0] * x[0] };
                var gradients_ = new double[2][];
                gradients_[0] = new[] { 2.0 * x[0] }; // df/dx = 2x
                gradients_[1] = new[] { Math.Cos(u[0]) }; // df/du = cos(u)
                return (value_, gradients_);
            }

            double[] Dynamics(double[] x, double[] u, double t) => new[] { Math.Sin(u[0]) + x[0] * x[0] };

            var diffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(order);
            var numInteriorPerSegment = order - 2;
            var totalDefects = grid.Segments * numInteriorPerSegment * problem.StateDim;

            for (var defectIdx = 0; defectIdx < totalDefects; defectIdx++)
            {
                var defectsPerSegment = numInteriorPerSegment * problem.StateDim;
                var segmentIndex = defectIdx / defectsPerSegment;
                var remainder = defectIdx % defectsPerSegment;
                var interiorPointIndex = 1 + (remainder / problem.StateDim);
                var stateComponentIndex = remainder % problem.StateDim;

                var analyticalGrad = AutoDiffLGLGradientHelper.ComputeDefectGradient(
                    problem, grid, z, order, diffMatrix,
                    transcription.GetState, transcription.GetControl,
                    segmentIndex, interiorPointIndex, stateComponentIndex,
                    DynamicsWithGradients);

                var numericalGrad = ComputeNumericalDefectGradient(
                    z, defectIdx, transcription, Dynamics);

                for (var j = 0; j < z.Length; j++)
                {
                    Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                        $"Nonlinear defect {defectIdx}, element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
                }
            }
        }

        #endregion

        #region Running Cost Gradient Tests

        [TestMethod]
        public void RunningCostGradientMatchesNumericalForQuadraticCost()
        {
            // L(x, u, t) = u^2
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 3);
            var order = 4;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = transcription.CreateInitialGuess(
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 0.5 });

            (double value, double[] gradients) RunningCostWithGradients(double[] x, double[] u, double t)
            {
                var value_ = u[0] * u[0];
                var gradients_ = new double[3]; // [dL/dx, dL/du, dL/dt]
                gradients_[0] = 0.0;
                gradients_[1] = 2.0 * u[0];
                gradients_[2] = 0.0;
                return (value_, gradients_);
            }

            double RunningCost(double[] x, double[] u, double t) => u[0] * u[0];

            var analyticalGrad = AutoDiffLGLGradientHelper.ComputeRunningCostGradient(
                problem, grid, z, order,
                transcription.GetState, transcription.GetControl,
                RunningCostWithGradients);

            var numericalGrad = ComputeNumericalRunningCostGradient(z, transcription, RunningCost);

            for (var j = 0; j < z.Length; j++)
            {
                Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                    $"Running cost gradient element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
            }
        }

        [TestMethod]
        public void RunningCostGradientMatchesNumericalForStateDependentCost()
        {
            // L(x, u, t) = x^2 + u^2
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 2.0, 2);
            var order = 3;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = transcription.CreateInitialGuess(
                new[] { 1.0 },
                new[] { 2.0 },
                new[] { 0.3 });

            (double value, double[] gradients) RunningCostWithGradients(double[] x, double[] u, double t)
            {
                var value_ = x[0] * x[0] + u[0] * u[0];
                var gradients_ = new double[3];
                gradients_[0] = 2.0 * x[0];
                gradients_[1] = 2.0 * u[0];
                gradients_[2] = 0.0;
                return (value_, gradients_);
            }

            double RunningCost(double[] x, double[] u, double t) => x[0] * x[0] + u[0] * u[0];

            var analyticalGrad = AutoDiffLGLGradientHelper.ComputeRunningCostGradient(
                problem, grid, z, order,
                transcription.GetState, transcription.GetControl,
                RunningCostWithGradients);

            var numericalGrad = ComputeNumericalRunningCostGradient(z, transcription, RunningCost);

            for (var j = 0; j < z.Length; j++)
            {
                Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                    $"State-dependent cost gradient element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
            }
        }

        #endregion

        #region Terminal Cost Gradient Tests

        [TestMethod]
        public void TerminalCostGradientMatchesNumerical()
        {
            // Φ(x, t) = x^2
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 3);
            var order = 4;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = transcription.CreateInitialGuess(
                new[] { 0.0 },
                new[] { 2.0 },
                new[] { 0.5 });

            (double value, double[] gradients) TerminalCostWithGradients(double[] x, double t)
            {
                var value_ = x[0] * x[0];
                var gradients_ = new double[2]; // [dΦ/dx, dΦ/dt]
                gradients_[0] = 2.0 * x[0];
                gradients_[1] = 0.0;
                return (value_, gradients_);
            }

            double TerminalCost(double[] x, double t) => x[0] * x[0];

            var analyticalGrad = AutoDiffLGLGradientHelper.ComputeTerminalCostGradient(
                problem, z, transcription.TotalPoints, grid.FinalTime,
                transcription.GetState,
                TerminalCostWithGradients);

            var numericalGrad = ComputeNumericalTerminalCostGradient(z, transcription, grid.FinalTime, TerminalCost);

            for (var j = 0; j < z.Length; j++)
            {
                Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                    $"Terminal cost gradient element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
            }
        }

        [TestMethod]
        public void TerminalCostGradientIsNonzeroOnlyAtFinalPoint()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var order = 4;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = transcription.CreateInitialGuess(
                new[] { 0.0, 0.0 },
                new[] { 1.0, 1.0 },
                new[] { 0.5 });

            (double value, double[] gradients) TerminalCostWithGradients(double[] x, double t)
            {
                var value_ = x[0] + x[1];
                var gradients_ = new double[3]; // [dΦ/dx0, dΦ/dx1, dΦ/dt]
                gradients_[0] = 1.0;
                gradients_[1] = 1.0;
                gradients_[2] = 0.0;
                return (value_, gradients_);
            }

            var gradient = AutoDiffLGLGradientHelper.ComputeTerminalCostGradient(
                problem, z, transcription.TotalPoints, grid.FinalTime,
                transcription.GetState,
                TerminalCostWithGradients);

            // Only the final point should have non-zero gradient
            var stateControlDim = problem.StateDim + problem.ControlDim;
            var finalPointOffset = (transcription.TotalPoints - 1) * stateControlDim;

            for (var j = 0; j < z.Length; j++)
            {
                if (j >= finalPointOffset && j < finalPointOffset + problem.StateDim)
                {
                    // Should be 1.0 for both state components
                    Assert.AreEqual(1.0, gradient[j], GradientTolerance,
                        $"Final state gradient element {j} should be 1.0");
                }
                else
                {
                    // Should be 0 for all other elements
                    Assert.AreEqual(0.0, gradient[j], GradientTolerance,
                        $"Non-final gradient element {j} should be 0");
                }
            }
        }

        #endregion

        #region Time-Scaled Dynamics Gradient Tests (Critical for Free Final Time)

        [TestMethod]
        public void DefectGradientCorrectForTimeScaledDynamics()
        {
            // Test time-scaled dynamics: dx/dτ = T_f · f(x, u)
            // State: [x, T_f], Control: [u]
            // Dynamics: [T_f * u, 0]
            var problem = new ControlProblem()
                .WithStateSize(2)  // [x, T_f]
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 2); // τ ∈ [0, 1]
            var order = 3;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            // Initial guess: x goes from 0 to 1, T_f = 2.0
            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                var alpha = (double)i / (transcription.TotalPoints - 1);
                transcription.SetState(z, i, new[] { alpha, 2.0 });
                transcription.SetControl(z, i, new[] { 0.5 });
            }

            // Time-scaled dynamics with gradients
            (double[] value, double[][] gradients) DynamicsWithGradients(double[] x, double[] u, double t)
            {
                var Tf = x[1];
                var physicalRate = u[0]; // ẋ_physical = u

                var value_ = new[] { Tf * physicalRate, 0.0 }; // [dx/dτ, dTf/dτ]
                var gradients_ = new double[2][];

                // df/dx: 2x2 matrix flattened
                // [∂f0/∂x, ∂f0/∂Tf; ∂f1/∂x, ∂f1/∂Tf] = [0, u; 0, 0]
                gradients_[0] = new[] { 0.0, physicalRate, 0.0, 0.0 };

                // df/du: 2x1 vector
                // [∂f0/∂u; ∂f1/∂u] = [Tf; 0]
                gradients_[1] = new[] { Tf, 0.0 };

                return (value_, gradients_);
            }

            double[] Dynamics(double[] x, double[] u, double t)
            {
                var Tf = x[1];
                return new[] { Tf * u[0], 0.0 };
            }

            var diffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(order);
            var numInteriorPerSegment = order - 2;
            var totalDefects = grid.Segments * numInteriorPerSegment * problem.StateDim;

            for (var defectIdx = 0; defectIdx < totalDefects; defectIdx++)
            {
                var defectsPerSegment = numInteriorPerSegment * problem.StateDim;
                var segmentIndex = defectIdx / defectsPerSegment;
                var remainder = defectIdx % defectsPerSegment;
                var interiorPointIndex = 1 + (remainder / problem.StateDim);
                var stateComponentIndex = remainder % problem.StateDim;

                var analyticalGrad = AutoDiffLGLGradientHelper.ComputeDefectGradient(
                    problem, grid, z, order, diffMatrix,
                    transcription.GetState, transcription.GetControl,
                    segmentIndex, interiorPointIndex, stateComponentIndex,
                    DynamicsWithGradients);

                var numericalGrad = ComputeNumericalDefectGradient(
                    z, defectIdx, transcription, Dynamics);

                // Log for debugging
                Console.WriteLine($"Time-scaled defect {defectIdx} (seg={segmentIndex}, pt={interiorPointIndex}, state={stateComponentIndex}):");

                for (var j = 0; j < z.Length; j++)
                {
                    if (Math.Abs(numericalGrad[j]) > 1e-10 || Math.Abs(analyticalGrad[j]) > 1e-10)
                    {
                        Console.WriteLine($"  z[{j}]: num={numericalGrad[j]:F6}, ana={analyticalGrad[j]:F6}");
                    }

                    Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                        $"Time-scaled defect {defectIdx}, element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
                }
            }
        }

        [TestMethod]
        public void ControlGradientIsNonzeroForTimeScaledDynamics()
        {
            // Verify that control gradient is computed correctly when dynamics depend on T_f
            var problem = new ControlProblem()
                .WithStateSize(2)  // [x, T_f]
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 1);
            var order = 3;
            var transcription = new LegendreGaussLobattoTranscription(problem, grid, order);

            var z = new double[transcription.DecisionVectorSize];
            for (var i = 0; i < transcription.TotalPoints; i++)
            {
                transcription.SetState(z, i, new[] { 0.5, 2.0 }); // x=0.5, T_f=2.0
                transcription.SetControl(z, i, new[] { 1.0 }); // u=1.0
            }

            // Dynamics: [T_f * u, 0]
            double[] Dynamics(double[] x, double[] u, double t)
            {
                var Tf = x[1];
                return new[] { Tf * u[0], 0.0 };
            }

            // Compute numerical gradient of defect w.r.t. control
            var defects = transcription.ComputeAllDefects(z, Dynamics);

            // Perturb control at first interior point
            var zPert = (double[])z.Clone();
            var controlOffset = 1 * (problem.StateDim + problem.ControlDim) + problem.StateDim;
            zPert[controlOffset] += Epsilon;

            var defectsPert = transcription.ComputeAllDefects(zPert, Dynamics);

            // First defect should change
            var gradWrtControl = (defectsPert[0] - defects[0]) / Epsilon;

            Console.WriteLine($"Numerical gradient of defect[0] w.r.t. control: {gradWrtControl}");

            // The gradient should be non-zero (approximately -T_f = -2.0)
            Assert.IsTrue(Math.Abs(gradWrtControl) > 0.1,
                $"Control should affect defect, got gradient = {gradWrtControl}");
        }

        #endregion

        #region Helper Methods

        private static double[] ComputeNumericalDefectGradient(
            double[] z,
            int defectIndex,
            LegendreGaussLobattoTranscription transcription,
            Func<double[], double[], double, double[]> dynamics)
        {
            var gradient = new double[z.Length];
            var defects = transcription.ComputeAllDefects(z, dynamics);
            var baseDefect = defects[defectIndex];

            for (var j = 0; j < z.Length; j++)
            {
                var zPert = (double[])z.Clone();
                zPert[j] += Epsilon;
                var defectsPert = transcription.ComputeAllDefects(zPert, dynamics);
                gradient[j] = (defectsPert[defectIndex] - baseDefect) / Epsilon;
            }

            return gradient;
        }

        private static double[] ComputeNumericalRunningCostGradient(
            double[] z,
            LegendreGaussLobattoTranscription transcription,
            Func<double[], double[], double, double> runningCost)
        {
            var gradient = new double[z.Length];
            var baseCost = transcription.ComputeRunningCost(z, runningCost);

            for (var j = 0; j < z.Length; j++)
            {
                var zPert = (double[])z.Clone();
                zPert[j] += Epsilon;
                var costPert = transcription.ComputeRunningCost(zPert, runningCost);
                gradient[j] = (costPert - baseCost) / Epsilon;
            }

            return gradient;
        }

        private static double[] ComputeNumericalTerminalCostGradient(
            double[] z,
            LegendreGaussLobattoTranscription transcription,
            double finalTime,
            Func<double[], double, double> terminalCost)
        {
            var gradient = new double[z.Length];
            var baseCost = transcription.ComputeTerminalCost(z, terminalCost);

            for (var j = 0; j < z.Length; j++)
            {
                var zPert = (double[])z.Clone();
                zPert[j] += Epsilon;
                var costPert = transcription.ComputeTerminalCost(zPert, terminalCost);
                gradient[j] = (costPert - baseCost) / Epsilon;
            }

            return gradient;
        }

        #endregion
    }
}
