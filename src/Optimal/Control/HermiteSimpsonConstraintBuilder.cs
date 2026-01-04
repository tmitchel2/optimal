/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear;

namespace Optimal.Control
{
    /// <summary>
    /// Builds constraints for Hermite-Simpson collocation problems.
    /// </summary>
    internal static class HermiteSimpsonConstraintBuilder
    {
        /// <summary>
        /// Adds all defect constraints to the optimizer.
        /// </summary>
        public static void AddDefectConstraints(
            AugmentedLagrangianOptimizer optimizer,
            ControlProblem problem,
            CollocationGrid grid,
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            bool hasAnalyticalGradients,
            int segments,
            bool verbose,
            double[] initialGuess)
        {
            var totalDefects = segments * problem.StateDim;

            if (verbose)
            {
                LogDefectDiagnostics(transcription, dynamicsEvaluator, initialGuess, problem, segments);
            }

            for (var i = 0; i < totalDefects; i++)
            {
                var defectConstraint = CreateDefectConstraint(
                    i, problem, grid, transcription, dynamicsEvaluator, hasAnalyticalGradients);
                optimizer.WithEqualityConstraint(defectConstraint);
            }
        }

        /// <summary>
        /// Adds initial and final boundary condition constraints to the optimizer.
        /// </summary>
        public static void AddBoundaryConstraints(
            AugmentedLagrangianOptimizer optimizer,
            ControlProblem problem,
            ITranscription transcription,
            int segments)
        {
            if (problem.InitialState != null)
            {
                AddInitialStateConstraints(optimizer, problem, transcription);
            }

            if (problem.FinalState != null)
            {
                AddFinalStateConstraints(optimizer, problem, transcription, segments);
            }
        }

        /// <summary>
        /// Adds box constraints for states and controls to the optimizer.
        /// </summary>
        public static void AddBoxConstraints(
            AugmentedLagrangianOptimizer optimizer,
            ControlProblem problem,
            ITranscription transcription,
            int segments)
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

                // State bounds (if specified)
                for (var i = 0; i < problem.StateDim; i++)
                {
                    lowerBounds[offset + i] = problem.StateLowerBounds?[i] ?? double.NegativeInfinity;
                    upperBounds[offset + i] = problem.StateUpperBounds?[i] ?? double.PositiveInfinity;
                }

                // Control bounds
                for (var i = 0; i < problem.ControlDim; i++)
                {
                    lowerBounds[offset + problem.StateDim + i] = problem.ControlLowerBounds[i];
                    upperBounds[offset + problem.StateDim + i] = problem.ControlUpperBounds[i];
                }
            }

            optimizer.WithBoxConstraints(lowerBounds, upperBounds);
        }

        /// <summary>
        /// Adds path constraints at all collocation nodes to the optimizer.
        /// </summary>
        public static void AddPathConstraints(
            AugmentedLagrangianOptimizer optimizer,
            ControlProblem problem,
            CollocationGrid grid,
            ITranscription transcription,
            int segments)
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

                        double ConstraintValue(double[] zz)
                        {
                            var xx = transcription.GetState(zz, nodeIndex);
                            var uu = transcription.GetControl(zz, nodeIndex);
                            var res = pathConstraint(xx, uu, timePoint);
                            return res.value;
                        }

                        var gradient = NumericalGradients.ComputeConstraintGradient(ConstraintValue, z);
                        return (result.value, gradient);
                    });
                }
            }
        }

        /// <summary>
        /// Creates a defect constraint function for a specific defect index.
        /// </summary>
        private static Func<double[], (double value, double[] gradient)> CreateDefectConstraint(
            int defectIndex,
            ControlProblem problem,
            CollocationGrid grid,
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            bool hasAnalyticalGradients)
        {
            return z =>
            {
                var parallelTranscription = (ParallelTranscription)transcription;
                var allDefects = parallelTranscription.ComputeAllDefects(z, dynamicsEvaluator);
                var segmentIndex = defectIndex / problem.StateDim;
                var stateComponentIndex = defectIndex % problem.StateDim;

                double[] gradient;

                if (hasAnalyticalGradients)
                {
                    try
                    {
                        gradient = AutoDiffGradientHelper.ComputeDefectGradient(
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
                        gradient = ComputeNumericalDefectGradient(
                            parallelTranscription, dynamicsEvaluator, z, defectIndex);
                    }
                }
                else
                {
                    gradient = ComputeNumericalDefectGradient(
                        parallelTranscription, dynamicsEvaluator, z, defectIndex);
                }

                return (allDefects[defectIndex], gradient);
            };
        }

        /// <summary>
        /// Adds initial state constraints.
        /// </summary>
        private static void AddInitialStateConstraints(
            AugmentedLagrangianOptimizer optimizer,
            ControlProblem problem,
            ITranscription transcription)
        {
            for (var i = 0; i < problem.StateDim; i++)
            {
                var stateIndex = i;
                var targetValue = problem.InitialState![i];

                optimizer.WithEqualityConstraint(z =>
                {
                    var x = transcription.GetState(z, 0);

                    double BoundaryValue(double[] zz)
                    {
                        var xx = transcription.GetState(zz, 0);
                        return xx[stateIndex] - targetValue;
                    }

                    var gradient = NumericalGradients.ComputeConstraintGradient(BoundaryValue, z);
                    return (x[stateIndex] - targetValue, gradient);
                });
            }
        }

        /// <summary>
        /// Adds final state constraints.
        /// </summary>
        private static void AddFinalStateConstraints(
            AugmentedLagrangianOptimizer optimizer,
            ControlProblem problem,
            ITranscription transcription,
            int segments)
        {
            for (var i = 0; i < problem.StateDim; i++)
            {
                var stateIndex = i;
                var targetValue = problem.FinalState![i];

                // Skip NaN values (free terminal conditions)
                if (double.IsNaN(targetValue))
                {
                    continue;
                }

                optimizer.WithEqualityConstraint(z =>
                {
                    var x = transcription.GetState(z, segments);

                    double BoundaryValue(double[] zz)
                    {
                        var xx = transcription.GetState(zz, segments);
                        return xx[stateIndex] - targetValue;
                    }

                    var gradient = NumericalGradients.ComputeConstraintGradient(BoundaryValue, z);
                    return (x[stateIndex] - targetValue, gradient);
                });
            }
        }

        /// <summary>
        /// Computes defect gradient numerically.
        /// </summary>
        private static double[] ComputeNumericalDefectGradient(
            ParallelTranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            double[] z,
            int defectIndex)
        {
            double DefectValue(double[] zz)
            {
                var defects = transcription.ComputeAllDefects(zz, dynamicsEvaluator);
                return defects[defectIndex];
            }

            return NumericalGradients.ComputeConstraintGradient(DefectValue, z);
        }

        /// <summary>
        /// Logs diagnostic information about defects on the initial guess.
        /// </summary>
        private static void LogDefectDiagnostics(
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            double[] initialGuess,
            ControlProblem problem,
            int segments)
        {
            var parallelTranscription = (ParallelTranscription)transcription;
            var testDefects = parallelTranscription.ComputeAllDefects(initialGuess, dynamicsEvaluator);

            var defectHasNaN = false;
            for (var i = 0; i < testDefects.Length; i++)
            {
                if (double.IsNaN(testDefects[i]))
                {
                    Console.WriteLine($"WARNING: Defect {i} is NaN on initial guess!");
                    defectHasNaN = true;
                    break;
                }
            }

            if (!defectHasNaN)
            {
                Console.WriteLine($"All {testDefects.Length} defects are finite on initial guess");
            }

            // Report max defect per state component
            var maxDefectPerState = new double[problem.StateDim];
            for (var seg = 0; seg < segments; seg++)
            {
                for (var state = 0; state < problem.StateDim; state++)
                {
                    var defectIdx = seg * problem.StateDim + state;
                    var absDefect = Math.Abs(testDefects[defectIdx]);
                    if (absDefect > maxDefectPerState[state])
                    {
                        maxDefectPerState[state] = absDefect;
                    }
                }
            }

            var formattedDefects = string.Join(", ",
                System.Linq.Enumerable.Select(maxDefectPerState,
                    d => d.ToString("E3", System.Globalization.CultureInfo.InvariantCulture)));
            Console.WriteLine($"Max defect per state component (initial guess): [{formattedDefects}]");
        }
    }
}
