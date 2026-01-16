/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable RCS1163 // Unused parameter

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace Optimal.Control.Optimization.Tests
{
    /// <summary>
    /// Tests for AutoDiffGradientHelper (Hermite-Simpson) to verify gradient computation correctness.
    /// </summary>
    [TestClass]
    public sealed class AutoDiffGradientHelperTests
    {
        private const double Epsilon = 1e-7;
        private const double GradientTolerance = 1e-4;

        #region Running Cost Gradient Tests

        [TestMethod]
        public void RunningCostGradientMatchesNumericalForQuadraticCost()
        {
            // L(x, u, t) = u^2
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 3);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(
                [0.0],
                [1.0],
                [0.5]);

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

            var analyticalGrad = AutoDiffGradientHelper.ComputeRunningCostGradient(
                problem, grid, z, transcription.GetState, transcription.GetControl,
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

            var grid = new CollocationGrid(0.0, 2.0, 4);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(
                [1.0],
                [3.0],
                [0.5]);

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

            var analyticalGrad = AutoDiffGradientHelper.ComputeRunningCostGradient(
                problem, grid, z, transcription.GetState, transcription.GetControl,
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
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(
                [0.0],
                [2.0],
                [0.5]);

            (double value, double[] gradients) TerminalCostWithGradients(double[] x, double t)
            {
                var value_ = x[0] * x[0];
                var gradients_ = new double[2]; // [dΦ/dx, dΦ/dt]
                gradients_[0] = 2.0 * x[0];
                gradients_[1] = 0.0;
                return (value_, gradients_);
            }

            double TerminalCost(double[] x, double t) => x[0] * x[0];

            var analyticalGrad = AutoDiffGradientHelper.ComputeTerminalCostGradient(
                problem, grid, z,
                transcription.GetState,
                TerminalCostWithGradients);

            var numericalGrad = ComputeNumericalTerminalCostGradient(z, transcription, TerminalCost);

            for (var j = 0; j < z.Length; j++)
            {
                Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                    $"Terminal cost gradient element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
            }
        }

        [TestMethod]
        public void TerminalCostGradientIsNonzeroOnlyAtFinalPoint()
        {
            // Φ(x, t) = x
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 4);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(
                [0.0],
                [5.0],
                [1.0]);

            (double value, double[] gradients) TerminalCostWithGradients(double[] x, double t)
            {
                var value_ = x[0];
                var gradients_ = new double[2];
                gradients_[0] = 1.0;
                gradients_[1] = 0.0;
                return (value_, gradients_);
            }

            var gradient = AutoDiffGradientHelper.ComputeTerminalCostGradient(
                problem, grid, z,
                transcription.GetState,
                TerminalCostWithGradients);

            var stateControlDim = problem.StateDim + problem.ControlDim;
            var finalStateOffset = grid.Segments * stateControlDim;

            for (var j = 0; j < z.Length; j++)
            {
                if (j == finalStateOffset)
                {
                    Assert.AreEqual(1.0, gradient[j], GradientTolerance,
                        $"Final state gradient should be 1.0");
                }
                else
                {
                    Assert.AreEqual(0.0, gradient[j], GradientTolerance,
                        $"Non-final gradient element {j} should be 0");
                }
            }
        }

        #endregion

        #region Defect Gradient Tests

        [TestMethod]
        public void DefectGradientMatchesNumericalForSimpleIntegrator()
        {
            // ẋ = u (simple integrator)
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 1.0, 2);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(
                [0.0],
                [1.0],
                [0.5]);

            (double[] value, double[][] gradients) DynamicsWithGradients(double[] x, double[] u, double t)
            {
                var value_ = new[] { u[0] };
                var gradients_ = new double[2][];
                gradients_[0] = [0.0]; // df/dx
                gradients_[1] = [1.0]; // df/du
                return (value_, gradients_);
            }

            double[] Dynamics(double[] x, double[] u, double t) => [u[0]];

            // Test first defect
            var analyticalGrad = AutoDiffGradientHelper.ComputeDefectGradient(
                problem, grid, z, transcription.GetState, transcription.GetControl,
                0, 0, DynamicsWithGradients);

            var numericalGrad = ComputeNumericalDefectGradient(z, 0, transcription, Dynamics);

            for (var j = 0; j < z.Length; j++)
            {
                Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                    $"Defect gradient element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
            }
        }

        [TestMethod]
        public void DefectGradientMatchesNumericalForDoubleIntegrator()
        {
            // State: [x, v], Control: u
            // Dynamics: [v, u] (double integrator)
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1);

            var grid = new CollocationGrid(0.0, 2.0, 3);
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            var z = transcription.CreateInitialGuess(
                [0.0, 0.0],
                [4.0, 2.0],
                [1.0]);

            (double[] value, double[][] gradients) DynamicsWithGradients(double[] x, double[] u, double t)
            {
                var value_ = new[] { x[1], u[0] };
                var gradients_ = new double[2][];
                // df/dx: 2x2 matrix flattened [df0/dx0, df0/dx1, df1/dx0, df1/dx1]
                gradients_[0] = [0.0, 1.0, 0.0, 0.0];
                // df/du: 2x1 vector [df0/du, df1/du]
                gradients_[1] = [0.0, 1.0];
                return (value_, gradients_);
            }

            double[] Dynamics(double[] x, double[] u, double t) => [x[1], u[0]];

            // Test all defects
            var totalDefects = grid.Segments * problem.StateDim;
            for (var defectIdx = 0; defectIdx < totalDefects; defectIdx++)
            {
                var segmentIndex = defectIdx / problem.StateDim;
                var stateComponentIndex = defectIdx % problem.StateDim;

                var analyticalGrad = AutoDiffGradientHelper.ComputeDefectGradient(
                    problem, grid, z, transcription.GetState, transcription.GetControl,
                    segmentIndex, stateComponentIndex, DynamicsWithGradients);

                var numericalGrad = ComputeNumericalDefectGradient(z, defectIdx, transcription, Dynamics);

                for (var j = 0; j < z.Length; j++)
                {
                    Assert.AreEqual(numericalGrad[j], analyticalGrad[j], GradientTolerance,
                        $"Defect {defectIdx} gradient element {j}: num={numericalGrad[j]}, ana={analyticalGrad[j]}");
                }
            }
        }

        #endregion

        #region Helper Methods

        private static double[] ComputeNumericalRunningCostGradient(
            double[] z,
            HermiteSimpsonTranscription transcription,
            Func<double[], double[], double, double> runningCost)
        {
            var gradient = new double[z.Length];

            double ComputeCost(double[] zz)
            {
                return transcription.ComputeRunningCost(zz, runningCost);
            }

            for (var i = 0; i < z.Length; i++)
            {
                var zPlus = (double[])z.Clone();
                var zMinus = (double[])z.Clone();
                zPlus[i] += Epsilon;
                zMinus[i] -= Epsilon;

                gradient[i] = (ComputeCost(zPlus) - ComputeCost(zMinus)) / (2 * Epsilon);
            }

            return gradient;
        }

        private static double[] ComputeNumericalTerminalCostGradient(
            double[] z,
            HermiteSimpsonTranscription transcription,
            Func<double[], double, double> terminalCost)
        {
            var gradient = new double[z.Length];

            double ComputeCost(double[] zz)
            {
                return transcription.ComputeTerminalCost(zz, terminalCost);
            }

            for (var i = 0; i < z.Length; i++)
            {
                var zPlus = (double[])z.Clone();
                var zMinus = (double[])z.Clone();
                zPlus[i] += Epsilon;
                zMinus[i] -= Epsilon;

                gradient[i] = (ComputeCost(zPlus) - ComputeCost(zMinus)) / (2 * Epsilon);
            }

            return gradient;
        }

        private static double[] ComputeNumericalDefectGradient(
            double[] z,
            int defectIndex,
            HermiteSimpsonTranscription transcription,
            Func<double[], double[], double, double[]> dynamics)
        {
            var gradient = new double[z.Length];

            double ComputeDefect(double[] zz)
            {
                var defects = transcription.ComputeAllDefects(zz, dynamics);
                return defects[defectIndex];
            }

            for (var i = 0; i < z.Length; i++)
            {
                var zPlus = (double[])z.Clone();
                var zMinus = (double[])z.Clone();
                zPlus[i] += Epsilon;
                zMinus[i] -= Epsilon;

                gradient[i] = (ComputeDefect(zPlus) - ComputeDefect(zMinus)) / (2 * Epsilon);
            }

            return gradient;
        }

        #endregion
    }
}
