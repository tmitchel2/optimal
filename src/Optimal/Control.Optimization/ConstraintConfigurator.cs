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

namespace Optimal.Control.Optimization
{
    /// <summary>
    /// Helper class for configuring constraints in collocation-based optimal control solvers.
    /// </summary>
    internal static class ConstraintConfigurator
    {
        /// <summary>
        /// Adds defect constraints to the constraint collection for Hermite-Simpson collocation.
        /// </summary>
        public static void AddDefectConstraints(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            bool hasAnalyticalGradients,
            ConstraintCollection constraints)
        {
            var dynamics = problem.Dynamics!;
            var totalDefects = segments * problem.StateDim;

            for (var i = 0; i < totalDefects; i++)
            {
                var defectIndex = i;
                constraints.AddEqualityConstraint(z =>
                {
                    var allDefects = transcription.ComputeAllDefects(z, dynamics);
                    var gradient = ComputeDefectGradient(problem, grid, transcription, z, defectIndex, hasAnalyticalGradients, dynamics);
                    return (allDefects[defectIndex], gradient);
                });
            }
        }

        /// <summary>
        /// Adds boundary constraints (initial and final state) to the constraint collection.
        /// </summary>
        public static void AddBoundaryConstraints(
            ControlProblem problem,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            ConstraintCollection constraints)
        {
            AddInitialStateConstraints(problem, transcription, constraints);
            AddFinalStateConstraints(problem, transcription, segments, constraints);
        }

        /// <summary>
        /// Adds box constraints (bounds on states and controls) to the constraint collection.
        /// </summary>
        public static void AddBoxConstraints(
            ControlProblem problem,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            ConstraintCollection constraints)
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

            constraints.SetBoxConstraints(lowerBounds, upperBounds);
        }

        /// <summary>
        /// Adds path constraints (constraints at each collocation point) to the constraint collection.
        /// </summary>
        public static void AddPathConstraints(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            ConstraintCollection constraints)
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

                    constraints.AddInequalityConstraint(z =>
                    {
                        var x = transcription.GetState(z, nodeIndex);
                        var u = transcription.GetControl(z, nodeIndex);
                        var input = new PathConstraintInput(x, u, timePoint);
                        var result = pathConstraint(input);
                        var gradient = NumericalGradients.ComputeConstraintGradient(
                            zz =>
                            {
                                var xx = transcription.GetState(zz, nodeIndex);
                                var uu = transcription.GetControl(zz, nodeIndex);
                                var inp = new PathConstraintInput(xx, uu, timePoint);
                                return pathConstraint(inp).Value;
                            }, z);
                        return (result.Value, gradient);
                    });
                }
            }
        }

        private static double[] ComputeDefectGradient(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            double[] z,
            int defectIndex,
            bool hasAnalyticalGradients,
            Func<DynamicsInput, DynamicsResult> dynamics)
        {
            if (!hasAnalyticalGradients)
            {
                return NumericalGradients.ComputeConstraintGradient(
                    zz => transcription.ComputeAllDefects(zz, dynamics)[defectIndex], z);
            }

            var segmentIndex = defectIndex / problem.StateDim;
            var stateComponentIndex = defectIndex % problem.StateDim;

            try
            {
                return AutoDiffGradientHelper.ComputeDefectGradient(
                    problem, grid, z, transcription.GetState, transcription.GetControl,
                    segmentIndex, stateComponentIndex,
                    (x, u, t, segIdx) =>
                    {
                        var res = dynamics(new DynamicsInput(x, u, t, segIdx, grid.Segments));
                        return (res.Value, res.Gradients);
                    });
            }
            catch
            {
                return NumericalGradients.ComputeConstraintGradient(
                    zz => transcription.ComputeAllDefects(zz, dynamics)[defectIndex], z);
            }
        }

        private static void AddInitialStateConstraints(
            ControlProblem problem,
            ParallelHermiteSimpsonTranscription transcription,
            ConstraintCollection constraints)
        {
            if (problem.InitialState == null)
            {
                return;
            }

            for (var i = 0; i < problem.StateDim; i++)
            {
                var stateIndex = i;
                var targetValue = problem.InitialState[i];

                if (double.IsNaN(targetValue))
                {
                    continue; // Skip free initial state components
                }

                constraints.AddEqualityConstraint(z =>
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
            ParallelHermiteSimpsonTranscription transcription,
            int segments,
            ConstraintCollection constraints)
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

                constraints.AddEqualityConstraint(z =>
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
