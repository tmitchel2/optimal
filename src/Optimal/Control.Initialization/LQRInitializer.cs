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

                // Compute state at this time point first (needed for feedback)
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
                    var dynResult = problem.Dynamics!(new DynamicsInput(xPrev, uPrev, grid.TimePoints[k - 1]));
                    var f = dynResult.Value;
                    state = new double[nStates];
                    for (var i = 0; i < nStates; i++)
                    {
                        state[i] = xPrev[i] + (dt * f[i]);
                    }
                }

                // Linearize dynamics: ẋ ≈ A·(x - x̄) + B·(u - ū)
                var (A, B) = LinearizeDynamics(problem.Dynamics!, xNominal, uNominal, t);

                // Solve continuous-time algebraic Riccati equation for LQR gain
                // K = R⁻¹B'P where P solves the CARE
                var K = ComputeSimpleGain(A, B, Q, R, nStates, nControls);

                // Apply LQR feedback control: u = -K * (x - xTarget)
                // Use final state as target for regulation
                var xTarget = problem.FinalState ?? new double[nStates];
                var control = new double[nControls];
                for (var i = 0; i < nControls; i++)
                {
                    control[i] = uNominal[i];
                    for (var j = 0; j < nStates; j++)
                    {
                        // Feedback drives state toward target: u = -K * (state - target)
                        control[i] -= K[i, j] * (state[j] - xTarget[j]);
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
            Func<DynamicsInput, DynamicsResult> dynamics,
            double[] xNominal,
            double[] uNominal,
            double t)
        {
            var result = dynamics(new DynamicsInput(xNominal, uNominal, t));
            var f = result.Value;
            var gradients = result.Gradients;
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
                    var fPerturb = dynamics(new DynamicsInput(xPerturb, uNominal, t)).Value;

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
                    var fPerturb = dynamics(new DynamicsInput(xNominal, uPerturb, t)).Value;

                    for (var i = 0; i < nStates; i++)
                    {
                        B[i, j] = (fPerturb[i] - f[i]) / epsilon;
                    }
                }
            }

            return (A, B);
        }

        /// <summary>
        /// Computes the LQR gain matrix K by solving the continuous-time algebraic Riccati equation.
        /// CARE: A'P + PA - PBR⁻¹B'P + Q = 0
        /// Gain: K = R⁻¹B'P
        /// </summary>
        private static double[,] ComputeSimpleGain(
            double[,] A,
            double[,] B,
            double[] Q,
            double[] R,
            int nStates,
            int nControls)
        {
            // Solve CARE and compute gain
            var P = SolveContinuousRiccati(A, B, Q, R, nStates, nControls);

            // Compute K = R^{-1} * B' * P
            var K = new double[nControls, nStates];
            for (var i = 0; i < nControls; i++)
            {
                for (var j = 0; j < nStates; j++)
                {
                    var sum = 0.0;
                    for (var k = 0; k < nStates; k++)
                    {
                        sum += B[k, i] * P[k, j];
                    }

                    K[i, j] = sum / (R[i] + 1e-10);
                }
            }

            return K;
        }

        /// <summary>
        /// Solves the continuous-time algebraic Riccati equation using iterative method.
        /// For scalar systems, uses closed-form solution.
        /// </summary>
        private static double[,] SolveContinuousRiccati(
            double[,] A,
            double[,] B,
            double[] Q,
            double[] R,
            int nStates,
            int nControls)
        {
            var P = new double[nStates, nStates];

            // For scalar (1D) case, use closed-form solution
            if (nStates == 1 && nControls == 1)
            {
                var a = A[0, 0];
                var b = B[0, 0];
                var q = Q[0];
                var r = R[0];

                // CARE for scalar: 2aP - P²b²/r + q = 0
                // Solving: P = (ar + sqrt(a²r² + qrb²)) / b²
                // Taking positive root for stability
                var bSq = b * b;
                if (Math.Abs(bSq) < 1e-10)
                {
                    // Degenerate case: no control authority
                    P[0, 0] = q;
                }
                else
                {
                    var discriminant = a * a * r * r + q * r * bSq;
                    P[0, 0] = (a * r + Math.Sqrt(discriminant)) / bSq;
                }

                return P;
            }

            // For multi-dimensional case, use iterative solver
            // Initialize P with diagonal Q
            for (var i = 0; i < nStates; i++)
            {
                P[i, i] = Q[i];
            }

            // Precompute R inverse diagonal
            var RInv = new double[nControls];
            for (var i = 0; i < nControls; i++)
            {
                RInv[i] = 1.0 / (R[i] + 1e-10);
            }

            // Iterative solution using fixed-point iteration
            const int maxIterations = 100;
            const double tolerance = 1e-8;

            for (var iter = 0; iter < maxIterations; iter++)
            {
                var PNew = new double[nStates, nStates];

                // Compute P_new = A'P + PA - PBR^{-1}B'P + Q
                // This is the Riccati residual set to zero, we iterate toward the solution

                // For stability, we use the Kleinman iteration:
                // Given current P, compute K = R^{-1}B'P
                // Then solve Lyapunov equation: (A-BK)'P_new + P_new(A-BK) + K'RK + Q = 0
                // Simplified: we use gradient descent on P

                // Compute BRinvBT
                var BRinvBT = new double[nStates, nStates];
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        for (var k = 0; k < nControls; k++)
                        {
                            BRinvBT[i, j] += B[i, k] * RInv[k] * B[j, k];
                        }
                    }
                }

                // Compute A'P
                var ATP = new double[nStates, nStates];
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        for (var k = 0; k < nStates; k++)
                        {
                            ATP[i, j] += A[k, i] * P[k, j];
                        }
                    }
                }

                // Compute PA
                var PA = new double[nStates, nStates];
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        for (var k = 0; k < nStates; k++)
                        {
                            PA[i, j] += P[i, k] * A[k, j];
                        }
                    }
                }

                // Compute PBRinvBTP
                var PBRinvBTP = new double[nStates, nStates];
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        for (var k = 0; k < nStates; k++)
                        {
                            PBRinvBTP[i, j] += P[i, k] * BRinvBT[k, j];
                        }
                    }
                }

                // Final multiplication with P on the right
                var PBRinvBTPP = new double[nStates, nStates];
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        for (var k = 0; k < nStates; k++)
                        {
                            PBRinvBTPP[i, j] += PBRinvBTP[i, k] * P[k, j];
                        }
                    }
                }

                // Newton-like update: P_new = (A'P + PA + Q + PBR^{-1}B'P) / 2 with adjustment
                // Using a simpler fixed-point: solve for steady-state
                // We use: P_new_ij = (A'P + PA + Q - PBR^{-1}B'P) elements scaled appropriately

                // Actually, let's use direct iteration with damping
                var maxChange = 0.0;
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        // Residual of Riccati equation
                        var residual = ATP[i, j] + PA[i, j] - PBRinvBTPP[i, j];
                        if (i == j)
                        {
                            residual += Q[i];
                        }

                        // For the Riccati equation at equilibrium, residual should be zero
                        // Update P in direction that reduces residual
                        // Using gradient step: P_new = P + alpha * residual (simplified)
                        var alpha = 0.5;
                        PNew[i, j] = P[i, j] + alpha * residual;

                        maxChange = Math.Max(maxChange, Math.Abs(PNew[i, j] - P[i, j]));
                    }
                }

                // Ensure symmetry
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = i + 1; j < nStates; j++)
                    {
                        var avg = (PNew[i, j] + PNew[j, i]) / 2.0;
                        PNew[i, j] = avg;
                        PNew[j, i] = avg;
                    }
                }

                P = PNew;

                if (maxChange < tolerance)
                {
                    break;
                }
            }

            return P;
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
