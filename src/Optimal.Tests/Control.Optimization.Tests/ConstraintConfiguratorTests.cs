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
    public sealed class ConstraintConfiguratorTests
    {
        private static readonly double[] s_zeroState1D = [0.0];
        private static readonly double[] s_oneState1D = [1.0];
        private static readonly double[] s_zeroState2D = [0.0, 0.0];
        private static readonly double[] s_oneZeroState2D = [1.0, 0.0];

        [TestMethod]
        public void AddDefectConstraintsAddsCorrectNumberOfConstraints()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddDefectConstraints(
                problem, grid, transcription, 5, hasAnalyticalGradients: false, constraints);

            // 5 segments * 1 state dim = 5 defect constraints
            Assert.HasCount(5, constraints.EqualityConstraints);
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
                .WithDynamics(input =>
                {
                    var x = input.State;
                    var u = input.Control;
                    var value = new[] { x[1], u[0] };
                    var gradients = new double[2][];
                    gradients[0] = [0.0, 1.0, 0.0, 0.0];
                    gradients[1] = [0.0, 1.0];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddDefectConstraints(
                problem, grid, transcription, 5, hasAnalyticalGradients: true, constraints);

            // 5 segments * 2 state dims = 10 defect constraints
            Assert.HasCount(10, constraints.EqualityConstraints);
        }

        [TestMethod]
        public void AddBoundaryConstraintsAddsInitialStateConstraints()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, 5, constraints);

            // 1 initial + 1 final state constraint
            Assert.HasCount(2, constraints.EqualityConstraints);
        }

        [TestMethod]
        public void AddBoundaryConstraintsHandlesNoInitialState()
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

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            // Should not throw
            ConstraintConfigurator.AddBoundaryConstraints(problem, transcription, 5, constraints);

            // No constraints should be added when no initial/final state specified
            Assert.IsEmpty(constraints.EqualityConstraints);
        }

        [TestMethod]
        public void AddBoxConstraintsDoesNothingWhenNoBounds()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            // Should not throw
            ConstraintConfigurator.AddBoxConstraints(problem, transcription, 5, constraints);

            // No box constraints should be set
            Assert.IsNull(constraints.BoxConstraints);
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
                .WithControlBounds([-1.0], [1.0])
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddBoxConstraints(problem, transcription, 5, constraints);

            Assert.IsNotNull(constraints.BoxConstraints);
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
                .WithStateBounds([-5.0], [5.0])
                .WithControlBounds([-1.0], [1.0])
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddBoxConstraints(problem, transcription, 5, constraints);

            Assert.IsNotNull(constraints.BoxConstraints);
        }

        [TestMethod]
        public void AddPathConstraintsDoesNothingWhenNoConstraints()
        {
            var problem = CreateSimpleProblem();
            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            // Should not throw
            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, 5, constraints);

            // No inequality constraints should be added
            Assert.IsEmpty(constraints.InequalityConstraints);
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
                .WithDynamics(input =>
                {
                    var u = input.Control;
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return new DynamicsResult(value, gradients);
                })
                .WithPathConstraint(input =>
                {
                    // x[0] <= 2.0 expressed as x[0] - 2.0 <= 0
                    var value = input.State[0] - 2.0;
                    double[]? gradients = null;
                    return new PathConstraintResult(value, gradients!);
                });

            var grid = new CollocationGrid(0.0, 1.0, 5);
            var transcription = new ParallelHermiteSimpsonTranscription(problem, grid);
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddPathConstraints(problem, grid, transcription, 5, constraints);

            // 1 path constraint at each of 6 nodes (0 to 5 inclusive)
            Assert.HasCount(6, constraints.InequalityConstraints);
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
            var constraints = new ConstraintCollection();

            ConstraintConfigurator.AddDefectConstraints(
                problem, grid, transcription, 5, hasAnalyticalGradients: true, constraints);

            Assert.HasCount(5, constraints.EqualityConstraints);
        }

        private static ControlProblem CreateSimpleProblem()
        {
            return new ControlProblem()
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
        }
    }
}
