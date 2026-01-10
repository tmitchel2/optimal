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

namespace Optimal.Control.Initialization
{
    /// <summary>
    /// Linear Quadratic Regulator (LQR) solver for generating initial guesses.
    /// Solves the linearized optimal control problem around a reference trajectory.
    /// </summary>
    public static class LQRInitializer
    {
        /// <summary>
        /// Generates an initial guess by solving a linearized LQR problem.
        /// Linearizes around a nominal trajectory and solves the resulting LQR problem.
        /// </summary>
        /// <param name="problem">The nonlinear control problem.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="nominalTrajectory">Nominal trajectory to linearize around (states and controls at each time point).</param>
        /// <param name="Q">State cost weight matrix (diagonal).</param>
        /// <param name="R">Control cost weight matrix (diagonal).</param>
        /// <returns>Initial guess as decision vector.</returns>
        public static double[] GenerateInitialGuess(
            ControlProblem problem,
            CollocationGrid grid,
            (double[][] states, double[][] controls) nominalTrajectory,
            double[] Q,
            double[] R)
        {
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(grid);

            var nStates = problem.StateDim;
            var nControls = problem.ControlDim;
            var nPoints = grid.Segments + 1;

            // Create transcription to get decision vector layout
            var transcription = new HermiteSimpsonTranscription(problem, grid);
            var initialGuess = new double[transcription.DecisionVectorSize];

            // For each time point, compute linearization and solve local LQR
            for (var k = 0; k < nPoints; k++)
            {
                var t = grid.TimePoints[k];
                var xNominal = nominalTrajectory.states[k];
                var uNominal = nominalTrajectory.controls[k];

                // Linearize dynamics: ẋ ≈ A·(x - x̄) + B·(u - ū)
                var (A, B) = LinearizeDynamics(problem.Dynamics!, xNominal, uNominal, t);

                // Solve discrete-time LQR for this time point
                // For simplicity, use a heuristic: u = ū - K·(x - x̄)
                // Where K is chosen to stabilize toward target
                var K = ComputeSimpleGain(A, B, Q, R, nStates, nControls);

                // Apply feedback control
                var xDesired = GetDesiredState(problem, k, nPoints);
                var deltaX = new double[nStates];
                for (var i = 0; i < nStates; i++)
                {
                    deltaX[i] = xDesired[i] - xNominal[i];
                }

                var control = new double[nControls];
                for (var i = 0; i < nControls; i++)
                {
                    control[i] = uNominal[i];
                    for (var j = 0; j < nStates; j++)
                    {
                        control[i] += K[i, j] * deltaX[j];
                    }
                }

                // Integrate forward one step to get state
                double[] state;
                if (k == 0)
                {
                    state = problem.InitialState ?? xNominal;
                }
                else
                {
                    var dt = grid.TimePoints[k] - grid.TimePoints[k - 1];
                    var xPrev = transcription.GetState(initialGuess, k - 1);
                    var uPrev = transcription.GetControl(initialGuess, k - 1);
                    
                    // Simple forward Euler integration
                    var (f, _) = problem.Dynamics!(xPrev, uPrev, grid.TimePoints[k - 1]);
                    state = new double[nStates];
                    for (var i = 0; i < nStates; i++)
                    {
                        state[i] = xPrev[i] + dt * f[i];
                    }
                }

                // Set in decision vector
                transcription.SetState(initialGuess, k, state);
                transcription.SetControl(initialGuess, k, control);
            }

            return initialGuess;
        }

        /// <summary>
        /// Linearizes dynamics around a nominal point using finite differences.
        /// </summary>
        private static (double[,] A, double[,] B) LinearizeDynamics(
            Func<double[], double[], double, (double[] value, double[][] gradients)> dynamics,
            double[] xNominal,
            double[] uNominal,
            double t)
        {
            var (f, gradients) = dynamics(xNominal, uNominal, t);
            var nStates = xNominal.Length;
            var nControls = uNominal.Length;

            var A = new double[nStates, nStates];
            var B = new double[nStates, nControls];

            // Extract A matrix from gradients[0]
            if (gradients[0] != null && gradients[0].Length == nStates * nStates)
            {
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        A[i, j] = gradients[0][i * nStates + j];
                    }
                }
            }
            else
            {
                // Compute numerically if not provided
                var epsilon = 1e-6;
                for (var j = 0; j < nStates; j++)
                {
                    var xPerturb = (double[])xNominal.Clone();
                    xPerturb[j] += epsilon;
                    var (fPerturb, _) = dynamics(xPerturb, uNominal, t);
                    
                    for (var i = 0; i < nStates; i++)
                    {
                        A[i, j] = (fPerturb[i] - f[i]) / epsilon;
                    }
                }
            }

            // Extract B matrix from gradients[1]
            if (gradients[1] != null && gradients[1].Length == nStates * nControls)
            {
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nControls; j++)
                    {
                        B[i, j] = gradients[1][i * nControls + j];
                    }
                }
            }
            else
            {
                // Compute numerically if not provided
                var epsilon = 1e-6;
                for (var j = 0; j < nControls; j++)
                {
                    var uPerturb = (double[])uNominal.Clone();
                    uPerturb[j] += epsilon;
                    var (fPerturb, _) = dynamics(xNominal, uPerturb, t);
                    
                    for (var i = 0; i < nStates; i++)
                    {
                        B[i, j] = (fPerturb[i] - f[i]) / epsilon;
                    }
                }
            }

            return (A, B);
        }

        /// <summary>
        /// Computes a simple proportional gain matrix K.
        /// For full LQR, would solve Riccati equation. Here we use a heuristic.
        /// </summary>
        private static double[,] ComputeSimpleGain(
            double[,] A,
            double[,] B,
            double[] Q,
            double[] R,
            int nStates,
            int nControls)
        {
            var K = new double[nControls, nStates];

            // Simple heuristic: K = -R^{-1} * B^T * Q
            // This is not the full LQR solution but gives reasonable gains
            for (var i = 0; i < nControls; i++)
            {
                for (var j = 0; j < nStates; j++)
                {
                    var gain = 0.0;
                    for (var k = 0; k < nStates; k++)
                    {
                        gain += B[k, i] * Q[k];
                    }
                    K[i, j] = -gain / (R[i] + 1e-6);
                }
            }

            // Scale down for stability
            var scale = 0.1;
            for (var i = 0; i < nControls; i++)
            {
                for (var j = 0; j < nStates; j++)
                {
                    K[i, j] *= scale;
                }
            }

            return K;
        }

        /// <summary>
        /// Gets the desired state at a given time point.
        /// Interpolates between initial and final conditions.
        /// </summary>
        private static double[] GetDesiredState(ControlProblem problem, int k, int nPoints)
        {
            var alpha = (double)k / (nPoints - 1);
            var nStates = problem.StateDim;
            var desired = new double[nStates];

            var x0 = problem.InitialState;
            var xf = problem.FinalState;

            if (x0 != null && xf != null)
            {
                for (var i = 0; i < nStates; i++)
                {
                    desired[i] = (1 - alpha) * x0[i] + alpha * xf[i];
                }
            }
            else if (x0 != null)
            {
                for (var i = 0; i < nStates; i++)
                {
                    desired[i] = x0[i];
                }
            }
            else
            {
                // Default to zeros
                for (var i = 0; i < nStates; i++)
                {
                    desired[i] = 0.0;
                }
            }

            return desired;
        }

        /// <summary>
        /// Creates a simple nominal trajectory for linearization.
        /// Uses linear interpolation between initial and final states.
        /// </summary>
        public static (double[][] states, double[][] controls) CreateNominalTrajectory(
            ControlProblem problem,
            CollocationGrid grid)
        {
            var nPoints = grid.Segments + 1;
            var states = new double[nPoints][];
            var controls = new double[nPoints][];

            for (var k = 0; k < nPoints; k++)
            {
                var alpha = (double)k / (nPoints - 1);
                
                // Interpolate state
                states[k] = new double[problem.StateDim];
                if (problem.InitialState != null && problem.FinalState != null)
                {
                    for (var i = 0; i < problem.StateDim; i++)
                    {
                        states[k][i] = (1 - alpha) * problem.InitialState[i] + alpha * problem.FinalState[i];
                    }
                }
                else if (problem.InitialState != null)
                {
                    for (var i = 0; i < problem.StateDim; i++)
                    {
                        states[k][i] = problem.InitialState[i];
                    }
                }

                // Zero control as nominal
                controls[k] = new double[problem.ControlDim];
            }

            return (states, controls);
        }
    }
}
