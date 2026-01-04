/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control
{
    /// <summary>
    /// Builds the objective function for Hermite-Simpson collocation problems.
    /// </summary>
    internal static class HermiteSimpsonObjectiveBuilder
    {
        /// <summary>
        /// Builds the NLP objective function with progress callback support.
        /// </summary>
        public static Func<double[], (double value, double[] gradient)> Build(
            ControlProblem problem,
            CollocationGrid grid,
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            bool useAnalyticalGradients,
            int segments,
            bool verbose,
            ProgressCallback? progressCallback,
            double tolerance,
            IterationCounter iterationCounter)
        {
            return z =>
            {
                var cost = ComputeTotalCost(problem, transcription, z);

                if (verbose && double.IsNaN(cost))
                {
                    Console.WriteLine("WARNING: Objective cost is NaN!");
                }

                var gradient = useAnalyticalGradients
                    ? ComputeAnalyticalGradient(problem, grid, transcription, z)
                    : ComputeNumericalGradient(problem, transcription, z);

                if (verbose && gradient.Length > 0 && double.IsNaN(gradient[0]))
                {
                    Console.WriteLine("WARNING: Gradient is NaN!");
                }

                InvokeProgressCallback(
                    progressCallback, iterationCounter, cost, z, transcription,
                    grid, segments, dynamicsEvaluator, tolerance);

                return (cost, gradient);
            };
        }

        /// <summary>
        /// Computes the total cost (running + terminal).
        /// </summary>
        private static double ComputeTotalCost(
            ControlProblem problem,
            ITranscription transcription,
            double[] z)
        {
            var cost = 0.0;

            if (problem.RunningCost != null)
            {
                var parallelTranscription = (ParallelTranscription)transcription;
                cost += parallelTranscription.ComputeRunningCost(z, (x, u, t) =>
                {
                    var result = problem.RunningCost(x, u, t);
                    return result.value;
                });
            }

            if (problem.TerminalCost != null)
            {
                var parallelTranscription = (ParallelTranscription)transcription;
                cost += parallelTranscription.ComputeTerminalCost(z, (x, t) =>
                {
                    var result = problem.TerminalCost(x, t);
                    return result.value;
                });
            }

            return cost;
        }

        /// <summary>
        /// Computes the gradient using analytical differentiation.
        /// </summary>
        private static double[] ComputeAnalyticalGradient(
            ControlProblem problem,
            CollocationGrid grid,
            ITranscription transcription,
            double[] z)
        {
            var gradient = new double[z.Length];

            if (problem.RunningCost != null)
            {
                var runningGrad = AutoDiffGradientHelper.ComputeRunningCostGradient(
                    problem, grid, z, transcription.GetState, transcription.GetControl,
                    (x, u, t) =>
                    {
                        var res = problem.RunningCost!(x, u, t);
                        return (res.value, res.gradients!);
                    });

                for (var i = 0; i < gradient.Length; i++)
                {
                    gradient[i] += runningGrad[i];
                }
            }

            if (problem.TerminalCost != null)
            {
                var terminalGrad = AutoDiffGradientHelper.ComputeTerminalCostGradient(
                    problem, grid, z, transcription.GetState,
                    (x, t) =>
                    {
                        var res = problem.TerminalCost!(x, t);
                        return (res.value, res.gradients!);
                    });

                for (var i = 0; i < gradient.Length; i++)
                {
                    gradient[i] += terminalGrad[i];
                }
            }

            return gradient;
        }

        /// <summary>
        /// Computes the gradient using numerical finite differences.
        /// </summary>
        private static double[] ComputeNumericalGradient(
            ControlProblem problem,
            ITranscription transcription,
            double[] z)
        {
            double ObjectiveValue(double[] zz)
            {
                return ComputeTotalCost(problem, transcription, zz);
            }

            return NumericalGradients.ComputeGradient(ObjectiveValue, z);
        }

        /// <summary>
        /// Invokes the progress callback if provided.
        /// </summary>
        private static void InvokeProgressCallback(
            ProgressCallback? progressCallback,
            IterationCounter iterationCounter,
            double cost,
            double[] z,
            ITranscription transcription,
            CollocationGrid grid,
            int segments,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            double tolerance)
        {
            if (progressCallback == null)
            {
                return;
            }

            iterationCounter.Increment();
            var states = new double[segments + 1][];
            var controls = new double[segments + 1][];

            for (var k = 0; k <= segments; k++)
            {
                states[k] = transcription.GetState(z, k);
                controls[k] = transcription.GetControl(z, k);
            }

            var maxViolation = ComputeMaxConstraintViolation(
                z, transcription, dynamicsEvaluator);

            progressCallback(iterationCounter.Value, cost, states, controls, grid.TimePoints, maxViolation, tolerance);
        }

        /// <summary>
        /// Wrapper class for iteration counter to allow modification in lambda.
        /// </summary>
        public sealed class IterationCounter
        {
            public int Value { get; private set; }

            public void Increment()
            {
                Value++;
            }
        }

        /// <summary>
        /// Computes the maximum constraint violation from defects.
        /// </summary>
        private static double ComputeMaxConstraintViolation(
            double[] z,
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator)
        {
            var parallelTranscription = (ParallelTranscription)transcription;
            var allDefects = parallelTranscription.ComputeAllDefects(z, dynamicsEvaluator);
            var maxViolation = 0.0;

            foreach (var d in allDefects)
            {
                var abs = Math.Abs(d);
                if (abs > maxViolation)
                {
                    maxViolation = abs;
                }
            }

            return maxViolation;
        }
    }
}
