/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.NonLinear;

namespace Optimal.Control.Optimization
{
    /// <summary>
    /// Helper class for configuring constraints in collocation-based optimal control solvers.
    /// </summary>
    internal static class ConstraintConfigurator
    {
        /// <summary>
        /// Adds defect constraints to the optimizer for Hermite-Simpson collocation.
        /// </summary>
        public static void AddDefectConstraints(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelTranscription transcription,
            int segments,
            bool hasAnalyticalGradients,
            AugmentedLagrangianOptimizer optimizer)
        {
            double[] DynamicsValue(double[] x, double[] u, double t) => problem.Dynamics!(x, u, t).value;
            var totalDefects = segments * problem.StateDim;

            for (var i = 0; i < totalDefects; i++)
            {
                var defectIndex = i;
                optimizer.WithEqualityConstraint(z =>
                {
                    var allDefects = transcription.ComputeAllDefects(z, DynamicsValue);
                    var gradient = ComputeDefectGradient(problem, grid, transcription, z, defectIndex, hasAnalyticalGradients, DynamicsValue);
                    return (allDefects[defectIndex], gradient);
                });
            }
        }

        /// <summary>
        /// Adds boundary constraints (initial and final state) to the optimizer.
        /// </summary>
        public static void AddBoundaryConstraints(
            ControlProblem problem,
            ParallelTranscription transcription,
            int segments,
            AugmentedLagrangianOptimizer optimizer)
        {
            AddInitialStateConstraints(problem, transcription, optimizer);
            AddFinalStateConstraints(problem, transcription, segments, optimizer);
        }

        /// <summary>
        /// Adds box constraints (bounds on states and controls) to the optimizer.
        /// </summary>
        public static void AddBoxConstraints(
            ControlProblem problem,
            ParallelTranscription transcription,
            int segments,
            AugmentedLagrangianOptimizer optimizer)
        {
            if (problem.ControlLowerBounds == null || problem.ControlUpperBounds == null)
            {
                return;
            }

            var lowerBounds = new double[transcription.DecisionVectorSize];
            var upperBounds = new double[transcription.DecisionVectorSize];

            for (var k = 0; k <= segments; k++)
            {
                var offset = k * (problem.StateDim + problem.ControlDim);

                for (var i = 0; i < problem.StateDim; i++)
                {
                    lowerBounds[offset + i] = problem.StateLowerBounds?[i] ?? double.NegativeInfinity;
                    upperBounds[offset + i] = problem.StateUpperBounds?[i] ?? double.PositiveInfinity;
                }

                for (var i = 0; i < problem.ControlDim; i++)
                {
                    lowerBounds[offset + problem.StateDim + i] = problem.ControlLowerBounds[i];
                    upperBounds[offset + problem.StateDim + i] = problem.ControlUpperBounds[i];
                }
            }

            optimizer.WithBoxConstraints(lowerBounds, upperBounds);
        }

        /// <summary>
        /// Adds path constraints (constraints at each collocation point) to the optimizer.
        /// </summary>
        public static void AddPathConstraints(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelTranscription transcription,
            int segments,
            AugmentedLagrangianOptimizer optimizer)
        {
            if (problem.PathConstraints.Count == 0)
            {
                return;
            }

            for (var constraintIndex = 0; constraintIndex < problem.PathConstraints.Count; constraintIndex++)
            {
                var pathConstraint = problem.PathConstraints[constraintIndex];

                for (var k = 0; k <= segments; k++)
                {
                    var nodeIndex = k;
                    var timePoint = grid.TimePoints[k];

                    optimizer.WithInequalityConstraint(z =>
                    {
                        var x = transcription.GetState(z, nodeIndex);
                        var u = transcription.GetControl(z, nodeIndex);
                        var result = pathConstraint(x, u, timePoint);
                        var gradient = NumericalGradients.ComputeConstraintGradient(
                            zz =>
                            {
                                var xx = transcription.GetState(zz, nodeIndex);
                                var uu = transcription.GetControl(zz, nodeIndex);
                                return pathConstraint(xx, uu, timePoint).value;
                            }, z);
                        return (result.value, gradient);
                    });
                }
            }
        }

        private static double[] ComputeDefectGradient(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelTranscription transcription,
            double[] z,
            int defectIndex,
            bool hasAnalyticalGradients,
            Func<double[], double[], double, double[]> dynamicsValue)
        {
            if (!hasAnalyticalGradients)
            {
                return NumericalGradients.ComputeConstraintGradient(
                    zz => transcription.ComputeAllDefects(zz, dynamicsValue)[defectIndex], z);
            }

            var segmentIndex = defectIndex / problem.StateDim;
            var stateComponentIndex = defectIndex % problem.StateDim;

            try
            {
                return AutoDiffGradientHelper.ComputeDefectGradient(
                    problem, grid, z, transcription.GetState, transcription.GetControl,
                    segmentIndex, stateComponentIndex,
                    (x, u, t) =>
                    {
                        var res = problem.Dynamics!(x, u, t);
                        return (res.value, res.gradients);
                    });
            }
            catch
            {
                return NumericalGradients.ComputeConstraintGradient(
                    zz => transcription.ComputeAllDefects(zz, dynamicsValue)[defectIndex], z);
            }
        }

        private static void AddInitialStateConstraints(
            ControlProblem problem,
            ParallelTranscription transcription,
            AugmentedLagrangianOptimizer optimizer)
        {
            if (problem.InitialState == null)
            {
                return;
            }

            for (var i = 0; i < problem.StateDim; i++)
            {
                var stateIndex = i;
                var targetValue = problem.InitialState[i];
                optimizer.WithEqualityConstraint(z =>
                {
                    var x = transcription.GetState(z, 0);
                    var gradient = NumericalGradients.ComputeConstraintGradient(
                        zz => transcription.GetState(zz, 0)[stateIndex] - targetValue, z);
                    return (x[stateIndex] - targetValue, gradient);
                });
            }
        }

        private static void AddFinalStateConstraints(
            ControlProblem problem,
            ParallelTranscription transcription,
            int segments,
            AugmentedLagrangianOptimizer optimizer)
        {
            if (problem.FinalState == null)
            {
                return;
            }

            for (var i = 0; i < problem.StateDim; i++)
            {
                var stateIndex = i;
                var targetValue = problem.FinalState[i];

                if (double.IsNaN(targetValue))
                {
                    continue;
                }

                optimizer.WithEqualityConstraint(z =>
                {
                    var x = transcription.GetState(z, segments);
                    var gradient = NumericalGradients.ComputeConstraintGradient(
                        zz => transcription.GetState(zz, segments)[stateIndex] - targetValue, z);
                    return (x[stateIndex] - targetValue, gradient);
                });
            }
        }
    }
}
