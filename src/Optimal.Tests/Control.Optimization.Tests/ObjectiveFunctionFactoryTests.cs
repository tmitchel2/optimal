/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace Optimal.Control.Optimization.Tests
{
    [TestClass]
    public sealed class ObjectiveFunctionFactoryTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];

        [TestMethod]
        public void ComputeTotalCostReturnsZeroForNoCosts()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
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
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var z = new double[transcription.DecisionVectorSize];

            var cost = ObjectiveFunctionFactory.ComputeTotalCost(problem, transcription, z);

            Assert.AreEqual(0.0, cost);
        }

        [TestMethod]
        public void ComputeTotalCostIncludesRunningCost()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
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
                .WithRunningCost((_, _, _) =>
                {
                    var value = 1.0; // Constant running cost
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var z = new double[transcription.DecisionVectorSize];

            var cost = ObjectiveFunctionFactory.ComputeTotalCost(problem, transcription, z);

            // Cost should be approximately 1.0 (integral of 1 over [0,1])
            Assert.IsTrue(cost > 0.5 && cost < 1.5, $"Running cost should be approximately 1.0, was {cost}");
        }

        [TestMethod]
        public void ComputeTotalCostIncludesTerminalCost()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
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
                .WithTerminalCost(_ =>
                {
                    var value = 10.0; // Constant terminal cost
                    var gradients = new double[2];
                    return new TerminalCostResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var z = new double[transcription.DecisionVectorSize];

            var cost = ObjectiveFunctionFactory.ComputeTotalCost(problem, transcription, z);

            Assert.AreEqual(10.0, cost);
        }

        [TestMethod]
        public void ComputeTotalCostSumsBothCosts()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
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
                .WithRunningCost((_, _, _) =>
                {
                    var value = 1.0;
                    var gradients = new double[3];
                    return (value, gradients);
                })
                .WithTerminalCost(_ =>
                {
                    var value = 10.0;
                    var gradients = new double[2];
                    return new TerminalCostResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var z = new double[transcription.DecisionVectorSize];

            var cost = ObjectiveFunctionFactory.ComputeTotalCost(problem, transcription, z);

            // Running cost ~1.0 + terminal cost 10.0
            Assert.IsTrue(cost > 10.5 && cost < 11.5, $"Total cost should be ~11.0, was {cost}");
        }

        [TestMethod]
        public void CanUseAnalyticalGradientsForCostsReturnsTrueWhenGradientsProvided()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
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
                    var gradients = new double[3]; // StateDim + ControlDim + 1 = 3
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var result = ObjectiveFunctionFactory.CanUseAnalyticalGradientsForCosts(problem);

            Assert.IsTrue(result);
        }

        [TestMethod]
        public void CanUseAnalyticalGradientsForCostsReturnsFalseWhenGradientsTooShort()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[1]; // Too short - needs 3
                    return (value, gradients);
                });

            var result = ObjectiveFunctionFactory.CanUseAnalyticalGradientsForCosts(problem);

            Assert.IsFalse(result);
        }

        [TestMethod]
        public void CanUseAnalyticalGradientsForCostsReturnsFalseWhenGradientsNull()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    double[]? gradients = null;
                    return (value, gradients!);
                });

            var result = ObjectiveFunctionFactory.CanUseAnalyticalGradientsForCosts(problem);

            Assert.IsFalse(result);
        }

        [TestMethod]
        public void CanUseAnalyticalGradientsForCostsReturnsTrueForNoCosts()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                });

            var result = ObjectiveFunctionFactory.CanUseAnalyticalGradientsForCosts(problem);

            Assert.IsTrue(result);
        }

        [TestMethod]
        public void CanUseAnalyticalGradientsForCostsChecksTerminalCost()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                })
                .WithTerminalCost(input =>
                {
                    var value = input.State[0];
                    var gradients = new double[2]; // StateDim + 1 = 2
                    gradients[0] = 1.0;
                    gradients[1] = 0.0;
                    return new TerminalCostResult(value, gradients);
                });

            var result = ObjectiveFunctionFactory.CanUseAnalyticalGradientsForCosts(problem);

            Assert.IsTrue(result);
        }

        [TestMethod]
        public void ComputeObjectiveGradientReturnsCorrectSize()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
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

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var z = new double[transcription.DecisionVectorSize];

            var gradient = ObjectiveFunctionFactory.ComputeObjectiveGradient(problem, grid, transcription, z);

            Assert.AreEqual(z.Length, gradient.Length);
        }

        [TestMethod]
        public void ComputeObjectiveGradientUsesNumericalGradientsWhenNoAnalytical()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                })
                .WithRunningCost((_, u, _) =>
                {
                    var value = u[0] * u[0];
                    double[]? gradients = null;
                    return (value, gradients!);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var z = new double[transcription.DecisionVectorSize];
            // Set some controls to non-zero
            for (var k = 0; k <= 5; k++)
            {
                z[k * 2 + 1] = 0.5; // Control at each node
            }

            var gradient = ObjectiveFunctionFactory.ComputeObjectiveGradient(problem, grid, transcription, z);

            Assert.AreEqual(z.Length, gradient.Length);
            // Gradient should be non-zero for control variables
        }
    }
}
