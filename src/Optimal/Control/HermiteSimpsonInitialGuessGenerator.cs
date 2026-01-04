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
    /// Generates initial guesses for Hermite-Simpson collocation problems.
    /// </summary>
    internal static class HermiteSimpsonInitialGuessGenerator
    {
        /// <summary>
        /// Generates an initial guess for the decision variables.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="transcription">The transcription object.</param>
        /// <param name="providedGuess">Optional user-provided initial guess.</param>
        /// <param name="verbose">Whether to output verbose information.</param>
        /// <returns>The initial guess vector.</returns>
        public static double[] Generate(
            ControlProblem problem,
            ITranscription transcription,
            double[]? providedGuess,
            bool verbose)
        {
            // Use provided guess if it's the right size
            if (providedGuess != null && providedGuess.Length == transcription.DecisionVectorSize)
            {
                return providedGuess;
            }

            var x0 = problem.InitialState ?? new double[problem.StateDim];
            var xf = problem.FinalState ?? new double[problem.StateDim];
            var u0 = ComputeInitialControl(problem, x0, xf);

            var guess = transcription.CreateInitialGuess(x0, xf, u0);

            if (verbose)
            {
                var hasNaN = CheckForNaN(guess);
                Console.WriteLine($"Initial guess created: size={guess.Length}, hasNaN={hasNaN}");
            }

            return guess;
        }

        /// <summary>
        /// Computes a reasonable initial control guess based on start and end states.
        /// </summary>
        private static double[] ComputeInitialControl(ControlProblem problem, double[] x0, double[] xf)
        {
            var u0 = new double[problem.ControlDim];

            // For single-control, 2+ state problems, initialize control to point toward target
            if (problem.ControlDim == 1 && problem.StateDim >= 2 &&
                problem.InitialState != null && problem.FinalState != null)
            {
                var dx = xf[0] - x0[0];
                var dy = xf[1] - x0[1];

                if (Math.Abs(dx) > 1e-10 || Math.Abs(dy) > 1e-10)
                {
                    // For Brachistochrone-like problems, angle is from horizontal, positive for descent
                    // ẏ = -v·sin(θ), so when descending (dy < 0), we need θ > 0
                    // Use absolute value of descent angle and clamp to [0, π/2]
                    var angle = Math.Atan2(-dy, dx);  // Negate dy since we measure descent angle
                    u0[0] = Math.Clamp(angle, 0.0, Math.PI / 2.0);
                }
            }

            return u0;
        }

        /// <summary>
        /// Checks if the guess contains any NaN values.
        /// </summary>
        private static bool CheckForNaN(double[] guess)
        {
            for (var i = 0; i < guess.Length; i++)
            {
                if (double.IsNaN(guess[i]))
                {
                    return true;
                }
            }

            return false;
        }
    }

    /// <summary>
    /// Interface for transcription objects that can create initial guesses.
    /// </summary>
    internal interface ITranscription
    {
        int DecisionVectorSize { get; }
        double[] CreateInitialGuess(double[] x0, double[] xf, double[] u0);
        double[] GetState(double[] decisionVector, int nodeIndex);
        double[] GetControl(double[] decisionVector, int nodeIndex);
        void SetState(double[] decisionVector, int nodeIndex, double[] state);
        void SetControl(double[] decisionVector, int nodeIndex, double[] control);
    }
}
