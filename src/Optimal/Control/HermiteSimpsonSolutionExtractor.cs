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
    /// Extracts and formats the solution from Hermite-Simpson optimization results.
    /// </summary>
    internal static class HermiteSimpsonSolutionExtractor
    {
        /// <summary>
        /// Extracts the collocation result from the NLP optimization result.
        /// </summary>
        /// <param name="nlpResult">The optimization result.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="transcription">The transcription object.</param>
        /// <param name="dynamicsEvaluator">Function to evaluate dynamics.</param>
        /// <param name="segments">Number of collocation segments.</param>
        /// <returns>The formatted collocation result.</returns>
        public static CollocationResult Extract(
            OptimizerResult nlpResult,
            CollocationGrid grid,
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator,
            int segments)
        {
            var optimalPoint = nlpResult.OptimalPoint;
            var times = grid.TimePoints;
            var states = new double[segments + 1][];
            var controls = new double[segments + 1][];

            for (var k = 0; k <= segments; k++)
            {
                states[k] = transcription.GetState(optimalPoint, k);
                controls[k] = transcription.GetControl(optimalPoint, k);
            }

            var maxDefect = ComputeMaxDefect(optimalPoint, transcription, dynamicsEvaluator);

            return new CollocationResult
            {
                Success = nlpResult.Success,
                Message = nlpResult.Message,
                Times = times,
                States = states,
                Controls = controls,
                OptimalCost = nlpResult.OptimalValue,
                Iterations = nlpResult.Iterations,
                MaxDefect = maxDefect,
                GradientNorm = nlpResult.GradientNorm
            };
        }

        /// <summary>
        /// Computes the maximum defect across all collocation constraints.
        /// </summary>
        private static double ComputeMaxDefect(
            double[] decisionVector,
            ITranscription transcription,
            Func<double[], double[], double, double[]> dynamicsEvaluator)
        {
            if (transcription is not ParallelTranscription parallelTranscription)
            {
                return 0.0;
            }

            var defects = parallelTranscription.ComputeAllDefects(decisionVector, dynamicsEvaluator);
            var maxDefect = 0.0;

            foreach (var d in defects)
            {
                var abs = Math.Abs(d);
                if (abs > maxDefect)
                {
                    maxDefect = abs;
                }
            }

            return maxDefect;
        }
    }
}
