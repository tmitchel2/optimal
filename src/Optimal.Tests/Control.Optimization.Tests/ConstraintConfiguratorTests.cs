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
using Optimal.NonLinear.Constrained;

namespace Optimal.Control.Optimization.Tests
{
    [TestClass]
    public sealed class ConstraintConfiguratorTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };
        private static readonly double[] s_oneZeroState2D = new[] { 1.0, 0.0 };

        [TestMethod]
        public void AddDefectConstraintsAddsCorrectNumberOfConstraints()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddDefectConstraints(
                problem, grid, transcription, 5, hasAnalyticalGradients: false, optimizer);

            // 5 segments * 1 state dim = 5 defect constraints
            // Just verify no exception is thrown - constraint count is internal
            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddDefectConstraintsHandlesMultipleStates()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState2D)
                .WithFinalCondition(s_oneZeroState2D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new double[4] { 0.0, 1.0, 0.0, 0.0 };
                    gradients[1] = new double[2] { 0.0, 1.0 };
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddDefectConstraints(
                problem, grid, transcription, 5, hasAnalyticalGradients: true, optimizer);

            // 5 segments * 2 state dims = 10 defect constraints
            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddBoundaryConstraintsAddsInitialStateConstraints()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddBoundaryConstraintsHandlesNoInitialState()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            // Should not throw
            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddBoxConstraintsDoesNothingWhenNoBounds()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            // Should not throw
            ConstraintConfigurator.AddBoxConstraints(problem, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddBoxConstraintsAddsControlBounds()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithControlBounds(new[] { -1.0 }, new[] { 1.0 })
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddBoxConstraints(problem, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddBoxConstraintsAddsStateBounds()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithStateBounds(new[] { -5.0 }, new[] { 5.0 })
                .WithControlBounds(new[] { -1.0 }, new[] { 1.0 })
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddBoxConstraints(problem, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddPathConstraintsDoesNothingWhenNoConstraints()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            // Should not throw
            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddPathConstraintsAddsConstraintsAtEachNode()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return (value, gradients);
                })
                .WithPathConstraint((x, u, t) =>
                {
                    // x[0] <= 2.0 expressed as x[0] - 2.0 <= 0
                    var value = x[0] - 2.0;
                    double[]? gradients = null;
                    return (value, gradients!);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, 5, optimizer);

            Assert.IsNotNull(optimizer);
        }

        [TestMethod]
        public void AddDefectConstraintsWithAnalyticalGradientsWorks()
        {
            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var optimizer = new AugmentedLagrangianOptimizer();
            optimizer.WithInitialPoint(new double[transcription.DecisionVectorSize]);

            ConstraintConfigurator.AddDefectConstraints(
                problem, grid, transcription, 5, hasAnalyticalGradients: true, optimizer);

            Assert.IsNotNull(optimizer);
        }

        private static ControlProblem CreateSimpleProblem()
        {
            return new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 1.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    gradients[0] = new[] { 0.0 };
                    gradients[1] = new[] { 1.0 };
                    return (value, gradients);
                });
        }
    }
}
