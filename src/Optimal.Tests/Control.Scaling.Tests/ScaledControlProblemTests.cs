/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - test code using inline arrays is clearer

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;

namespace Optimal.Control.Scaling.Tests
{
    [TestClass]
    public sealed class ScaledControlProblemTests
    {
        private const double FiniteDiffEpsilon = 1e-7;
        private const double GradientTolerance = 1e-5;

        [TestMethod]
        public void ConstructorThrowsOnDimensionMismatch()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 })
                .WithControlBounds(new[] { -1.0 }, new[] { 1.0 });

            // Scaling with wrong state dimension
            var wrongScaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 1.0 }, // Only 1 state instead of 2
                new[] { -1.0 }, new[] { 1.0 });

            Assert.Throws<ArgumentException>(() => new ScaledControlProblem(problem, wrongScaling));
        }

        [TestMethod]
        public void InitialStateIsScaled()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 })
                .WithControlBounds(new[] { -1.0 }, new[] { 1.0 })
                .WithInitialCondition(new[] { 50.0, 0.0 }); // Center values

            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 },
                new[] { -1.0 }, new[] { 1.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            Assert.IsNotNull(scaled.InitialState);
            Assert.AreEqual(0.0, scaled.InitialState[0], 1e-10); // Center maps to 0
            Assert.AreEqual(0.0, scaled.InitialState[1], 1e-10);
        }

        [TestMethod]
        public void StateBoundsAreScaledToMinusOnePlusOne()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 1.0, -15.0 }, new[] { 70.0, 15.0 })
                .WithControlBounds(new[] { -0.5 }, new[] { 0.5 });

            var scaling = VariableScaling.FromBounds(
                new[] { 1.0, -15.0 }, new[] { 70.0, 15.0 },
                new[] { -0.5 }, new[] { 0.5 });

            var scaled = new ScaledControlProblem(problem, scaling);

            Assert.AreEqual(-1.0, scaled.StateLowerBounds![0], 1e-10);
            Assert.AreEqual(-1.0, scaled.StateLowerBounds[1], 1e-10);
            Assert.AreEqual(1.0, scaled.StateUpperBounds![0], 1e-10);
            Assert.AreEqual(1.0, scaled.StateUpperBounds[1], 1e-10);
        }

        [TestMethod]
        public void DynamicsValueIsCorrectlyTransformed()
        {
            // Simple dynamics: dx/dt = [x[0] + u[0], x[1]]
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 })
                .WithControlBounds(new[] { -1.0 }, new[] { 1.0 })
                .WithDynamics(input => new DynamicsResult(
                    new[] { input.State[0] + input.Control[0], input.State[1] },
                    new[] { new double[] { 1, 0, 0, 1 }, new double[] { 1, 0 } }));

            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 },
                new[] { -1.0 }, new[] { 1.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            // Test at a point: scaled state [0, 0] = original [50, 0], control [0] = original [0]
            var result = scaled.Dynamics!(new DynamicsInput(
                new[] { 0.0, 0.0 },
                new[] { 0.0 },
                0.5, 0, 1));

            // Original dynamics at (50, 0, 0): dx/dt = [50, 0]
            // Scaled: dx_s/dt = [50/50, 0/10] = [1, 0]
            Assert.AreEqual(1.0, result.Value[0], 1e-10);
            Assert.AreEqual(0.0, result.Value[1], 1e-10);
        }

        [TestMethod]
        public void DynamicsGradientMatchesFiniteDifference()
        {
            // Quadratic dynamics for more interesting gradients
            // dx/dt = [x[0]^2 + u[0], x[1] * x[0]]
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 1.0, -5.0 }, new[] { 10.0, 5.0 })
                .WithControlBounds(new[] { -2.0 }, new[] { 2.0 })
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { (x[0] * x[0]) + u[0], x[1] * x[0] };

                    // Gradients: df/dx is 2x2, df/du is 2x1 (flattened row-major)
                    // df0/dx0 = 2*x[0], df0/dx1 = 0
                    // df1/dx0 = x[1], df1/dx1 = x[0]
                    // df0/du0 = 1, df1/du0 = 0
                    var dfdx = new[] { 2 * x[0], 0.0, x[1], x[0] };
                    var dfdu = new[] { 1.0, 0.0 };

                    return new DynamicsResult(value, new[] { dfdx, dfdu });
                });

            var scaling = VariableScaling.FromBounds(
                new[] { 1.0, -5.0 }, new[] { 10.0, 5.0 },
                new[] { -2.0 }, new[] { 2.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            // Test point in scaled space
            var testState = new[] { 0.5, -0.3 };
            var testControl = new[] { 0.2 };
            var testTime = 0.5;

            var result = scaled.Dynamics!(new DynamicsInput(testState, testControl, testTime, 0, 1));

            // Verify gradients with finite differences
            var numDfdx = ComputeNumericalDynamicsStateGradient(scaled.Dynamics, testState, testControl, testTime);
            var numDfdu = ComputeNumericalDynamicsControlGradient(scaled.Dynamics, testState, testControl, testTime);

            // Compare analytical vs numerical (df/dx is 2x2 = 4 elements, df/du is 2x1 = 2 elements)
            for (var i = 0; i < 4; i++)
            {
                Assert.AreEqual(numDfdx[i], result.Gradients[0][i], GradientTolerance,
                    $"df/dx[{i}] mismatch: analytical={result.Gradients[0][i]}, numerical={numDfdx[i]}");
            }

            for (var i = 0; i < 2; i++)
            {
                Assert.AreEqual(numDfdu[i], result.Gradients[1][i], GradientTolerance,
                    $"df/du[{i}] mismatch: analytical={result.Gradients[1][i]}, numerical={numDfdu[i]}");
            }
        }

        [TestMethod]
        public void RunningCostGradientMatchesFiniteDifference()
        {
            // Running cost: L = x[0]^2 + x[1]^2 + u[0]^2
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 })
                .WithControlBounds(new[] { -5.0 }, new[] { 5.0 })
                .WithDynamics(_ => new DynamicsResult(new[] { 0.0, 0.0 }, new[] { new double[4], new double[2] }))
                .WithRunningCost(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = (x[0] * x[0]) + (x[1] * x[1]) + (u[0] * u[0]);
                    // dL/dx = [2*x[0], 2*x[1]], dL/du = [2*u[0]], dL/dt = 0
                    var gradients = new[] { 2 * x[0], 2 * x[1], 2 * u[0], 0.0 };
                    return new RunningCostResult(value, gradients);
                });

            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 },
                new[] { -5.0 }, new[] { 5.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            var testState = new[] { 0.3, 0.5 };
            var testControl = new[] { -0.2 };
            var testTime = 0.5;

            var result = scaled.RunningCost!(new RunningCostInput(testState, testControl, testTime));

            // Compute numerical gradients
            var numGrad = ComputeNumericalRunningCostGradient(scaled.RunningCost, testState, testControl, testTime);

            // Compare: gradients are [dL/dx0, dL/dx1, dL/du0, dL/dt]
            for (var i = 0; i < 3; i++) // Skip dL/dt
            {
                Assert.AreEqual(numGrad[i], result.Gradients[i], GradientTolerance,
                    $"Running cost gradient[{i}] mismatch: analytical={result.Gradients[i]}, numerical={numGrad[i]}");
            }
        }

        [TestMethod]
        public void PathConstraintGradientMatchesFiniteDifference()
        {
            // Path constraint: g = x[0]^2 + x[1]^2 - 1 <= 0 (unit circle)
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { -2.0, -2.0 }, new[] { 2.0, 2.0 })
                .WithControlBounds(new[] { -1.0 }, new[] { 1.0 })
                .WithDynamics(_ => new DynamicsResult(new[] { 0.0, 0.0 }, new[] { new double[4], new double[2] }))
                .WithPathConstraint(input =>
                {
                    var x = input.State;
                    var value = (x[0] * x[0]) + (x[1] * x[1]) - 1.0;
                    // dg/dx = [2*x[0], 2*x[1]], dg/du = [0], dg/dt = 0
                    var gradients = new[] { 2 * x[0], 2 * x[1], 0.0, 0.0 };
                    return new PathConstraintResult(value, gradients);
                });

            var scaling = VariableScaling.FromBounds(
                new[] { -2.0, -2.0 }, new[] { 2.0, 2.0 },
                new[] { -1.0 }, new[] { 1.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            var testState = new[] { 0.3, 0.4 };
            var testControl = new[] { 0.1 };
            var testTime = 0.5;

            var constraint = scaled.PathConstraints[0];
            var result = constraint(new PathConstraintInput(testState, testControl, testTime));

            // Compute numerical gradients
            var numGrad = ComputeNumericalPathConstraintGradient(constraint, testState, testControl, testTime);

            // Compare: gradients are [dg/dx0, dg/dx1, dg/du0, dg/dt]
            for (var i = 0; i < 3; i++) // Skip dg/dt
            {
                Assert.AreEqual(numGrad[i], result.Gradients[i], GradientTolerance,
                    $"Path constraint gradient[{i}] mismatch: analytical={result.Gradients[i]}, numerical={numGrad[i]}");
            }
        }

        [TestMethod]
        public void ToControlProblemCreatesValidProblem()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 10.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 })
                .WithControlBounds(new[] { -5.0 }, new[] { 5.0 })
                .WithInitialCondition(new[] { 50.0, 0.0 })
                .WithDynamics(_ => new DynamicsResult(new[] { 0.0, 0.0 }, new[] { new double[4], new double[2] }));

            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 },
                new[] { -5.0 }, new[] { 5.0 });

            var scaled = new ScaledControlProblem(problem, scaling);
            var controlProblem = scaled.ToControlProblem();

            Assert.AreEqual(2, controlProblem.StateDim);
            Assert.AreEqual(1, controlProblem.ControlDim);
            Assert.AreEqual(0.0, controlProblem.InitialTime);
            Assert.AreEqual(10.0, controlProblem.FinalTime);
            Assert.IsNotNull(controlProblem.Dynamics);
            Assert.IsNotNull(controlProblem.InitialState);
        }

        [TestMethod]
        public void ScaleInitialGuessTransformsCorrectly()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 })
                .WithControlBounds(new[] { -5.0 }, new[] { 5.0 });

            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 },
                new[] { -5.0 }, new[] { 5.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            // Original guess at centers
            var originalGuess = new InitialGuess(
                new[] { new[] { 50.0, 0.0 }, new[] { 50.0, 0.0 } },
                new[] { new[] { 0.0 }, new[] { 0.0 } });

            var scaledGuess = scaled.ScaleInitialGuess(originalGuess);

            // Centers should map to zeros
            Assert.AreEqual(0.0, scaledGuess.StateTrajectory[0][0], 1e-10);
            Assert.AreEqual(0.0, scaledGuess.StateTrajectory[0][1], 1e-10);
            Assert.AreEqual(0.0, scaledGuess.ControlTrajectory[0][0], 1e-10);
        }

        [TestMethod]
        public void UnscaleStatesTransformsCorrectly()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithStateBounds(new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 })
                .WithControlBounds(new[] { -5.0 }, new[] { 5.0 });

            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, -10.0 }, new[] { 100.0, 10.0 },
                new[] { -5.0 }, new[] { 5.0 });

            var scaled = new ScaledControlProblem(problem, scaling);

            // Scaled states at zeros (centers)
            var scaledStates = new[] { new[] { 0.0, 0.0 }, new[] { 1.0, -1.0 } };

            var unscaled = scaled.UnscaleStates(scaledStates);

            // Centers
            Assert.AreEqual(50.0, unscaled[0][0], 1e-10);
            Assert.AreEqual(0.0, unscaled[0][1], 1e-10);
            // Upper/lower bounds
            Assert.AreEqual(100.0, unscaled[1][0], 1e-10);
            Assert.AreEqual(-10.0, unscaled[1][1], 1e-10);
        }

        private static double[] ComputeNumericalDynamicsStateGradient(
            Func<DynamicsInput, DynamicsResult> dynamics,
            double[] state,
            double[] control,
            double time)
        {
            var stateDim = state.Length;
            var gradient = new double[stateDim * stateDim];

            for (var j = 0; j < stateDim; j++)
            {
                var perturbedPlus = (double[])state.Clone();
                var perturbedMinus = (double[])state.Clone();
                perturbedPlus[j] += FiniteDiffEpsilon;
                perturbedMinus[j] -= FiniteDiffEpsilon;

                var fPlus = dynamics(new DynamicsInput(perturbedPlus, control, time, 0, 1)).Value;
                var fMinus = dynamics(new DynamicsInput(perturbedMinus, control, time, 0, 1)).Value;

                for (var i = 0; i < stateDim; i++)
                {
                    gradient[(i * stateDim) + j] = (fPlus[i] - fMinus[i]) / (2 * FiniteDiffEpsilon);
                }
            }

            return gradient;
        }

        private static double[] ComputeNumericalDynamicsControlGradient(
            Func<DynamicsInput, DynamicsResult> dynamics,
            double[] state,
            double[] control,
            double time)
        {
            var stateDim = state.Length;
            var controlDim = control.Length;
            var gradient = new double[stateDim * controlDim];

            for (var j = 0; j < controlDim; j++)
            {
                var perturbedPlus = (double[])control.Clone();
                var perturbedMinus = (double[])control.Clone();
                perturbedPlus[j] += FiniteDiffEpsilon;
                perturbedMinus[j] -= FiniteDiffEpsilon;

                var fPlus = dynamics(new DynamicsInput(state, perturbedPlus, time, 0, 1)).Value;
                var fMinus = dynamics(new DynamicsInput(state, perturbedMinus, time, 0, 1)).Value;

                for (var i = 0; i < stateDim; i++)
                {
                    gradient[(i * controlDim) + j] = (fPlus[i] - fMinus[i]) / (2 * FiniteDiffEpsilon);
                }
            }

            return gradient;
        }

        private static double[] ComputeNumericalRunningCostGradient(
            Func<RunningCostInput, RunningCostResult> cost,
            double[] state,
            double[] control,
            double time)
        {
            var stateDim = state.Length;
            var controlDim = control.Length;
            var gradient = new double[stateDim + controlDim + 1];

            // State gradients
            for (var i = 0; i < stateDim; i++)
            {
                var perturbedPlus = (double[])state.Clone();
                var perturbedMinus = (double[])state.Clone();
                perturbedPlus[i] += FiniteDiffEpsilon;
                perturbedMinus[i] -= FiniteDiffEpsilon;

                var fPlus = cost(new RunningCostInput(perturbedPlus, control, time)).Value;
                var fMinus = cost(new RunningCostInput(perturbedMinus, control, time)).Value;

                gradient[i] = (fPlus - fMinus) / (2 * FiniteDiffEpsilon);
            }

            // Control gradients
            for (var i = 0; i < controlDim; i++)
            {
                var perturbedPlus = (double[])control.Clone();
                var perturbedMinus = (double[])control.Clone();
                perturbedPlus[i] += FiniteDiffEpsilon;
                perturbedMinus[i] -= FiniteDiffEpsilon;

                var fPlus = cost(new RunningCostInput(state, perturbedPlus, time)).Value;
                var fMinus = cost(new RunningCostInput(state, perturbedMinus, time)).Value;

                gradient[stateDim + i] = (fPlus - fMinus) / (2 * FiniteDiffEpsilon);
            }

            // Time gradient
            var timePlus = cost(new RunningCostInput(state, control, time + FiniteDiffEpsilon)).Value;
            var timeMinus = cost(new RunningCostInput(state, control, time - FiniteDiffEpsilon)).Value;
            gradient[stateDim + controlDim] = (timePlus - timeMinus) / (2 * FiniteDiffEpsilon);

            return gradient;
        }

        private static double[] ComputeNumericalPathConstraintGradient(
            Func<PathConstraintInput, PathConstraintResult> constraint,
            double[] state,
            double[] control,
            double time)
        {
            var stateDim = state.Length;
            var controlDim = control.Length;
            var gradient = new double[stateDim + controlDim + 1];

            // State gradients
            for (var i = 0; i < stateDim; i++)
            {
                var perturbedPlus = (double[])state.Clone();
                var perturbedMinus = (double[])state.Clone();
                perturbedPlus[i] += FiniteDiffEpsilon;
                perturbedMinus[i] -= FiniteDiffEpsilon;

                var fPlus = constraint(new PathConstraintInput(perturbedPlus, control, time)).Value;
                var fMinus = constraint(new PathConstraintInput(perturbedMinus, control, time)).Value;

                gradient[i] = (fPlus - fMinus) / (2 * FiniteDiffEpsilon);
            }

            // Control gradients
            for (var i = 0; i < controlDim; i++)
            {
                var perturbedPlus = (double[])control.Clone();
                var perturbedMinus = (double[])control.Clone();
                perturbedPlus[i] += FiniteDiffEpsilon;
                perturbedMinus[i] -= FiniteDiffEpsilon;

                var fPlus = constraint(new PathConstraintInput(state, perturbedPlus, time)).Value;
                var fMinus = constraint(new PathConstraintInput(state, perturbedMinus, time)).Value;

                gradient[stateDim + i] = (fPlus - fMinus) / (2 * FiniteDiffEpsilon);
            }

            // Time gradient
            var timePlus = constraint(new PathConstraintInput(state, control, time + FiniteDiffEpsilon)).Value;
            var timeMinus = constraint(new PathConstraintInput(state, control, time - FiniteDiffEpsilon)).Value;
            gradient[stateDim + controlDim] = (timePlus - timeMinus) / (2 * FiniteDiffEpsilon);

            return gradient;
        }
    }
}
