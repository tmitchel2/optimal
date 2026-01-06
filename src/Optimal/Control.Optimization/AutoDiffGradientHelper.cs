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
    /// Helper methods for computing gradients using AutoDiff-generated derivatives
    /// in the context of collocation transcriptions.
    /// </summary>
    public static class AutoDiffGradientHelper
    {
        /// <summary>
        /// Computes the gradient of the running cost with respect to the decision vector.
        /// Uses AutoDiff gradients if available, falls back to numerical gradients.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="z">The decision vector.</param>
        /// <param name="getState">Function to extract state from decision vector.</param>
        /// <param name="getControl">Function to extract control from decision vector.</param>
        /// <param name="runningCostWithGradients">Running cost function that returns (value, gradients).</param>
        /// <returns>Gradient of running cost with respect to decision vector.</returns>
        public static double[] ComputeRunningCostGradient(
            ControlProblem problem,
            CollocationGrid grid,
            double[] z,
            Func<double[], int, double[]> getState,
            Func<double[], int, double[]> getControl,
            Func<double[], double[], double, (double value, double[] gradients)>? runningCostWithGradients)
        {
            var n = z.Length;
            var gradient = new double[n];

            if (runningCostWithGradients == null)
            {
                return gradient;
            }

            // Iterate over all segments
            for (var k = 0; k < grid.Segments; k++)
            {
                var t_k = grid.TimePoints[k];
                var t_kp1 = grid.TimePoints[k + 1];
                var h = t_kp1 - t_k;

                // Get states and controls at endpoints and midpoint
                var x_k = getState(z, k);
                var u_k = getControl(z, k);
                var x_kp1 = getState(z, k + 1);
                var u_kp1 = getControl(z, k + 1);

                // Compute midpoint values
                var x_mid = new double[problem.StateDim];
                var u_mid = new double[problem.ControlDim];
                for (var i = 0; i < problem.StateDim; i++)
                {
                    x_mid[i] = 0.5 * (x_k[i] + x_kp1[i]);
                }
                for (var i = 0; i < problem.ControlDim; i++)
                {
                    u_mid[i] = 0.5 * (u_k[i] + u_kp1[i]);
                }

                var t_mid = 0.5 * (t_k + t_kp1);

                // Evaluate cost and gradients at endpoints and midpoint
                var (L_k, grad_k) = runningCostWithGradients(x_k, u_k, t_k);
                var (L_mid, grad_mid) = runningCostWithGradients(x_mid, u_mid, t_mid);
                var (L_kp1, grad_kp1) = runningCostWithGradients(x_kp1, u_kp1, t_kp1);

                // Simpson's rule weights: h/6 * (f_k + 4*f_mid + f_kp1)
                var weight_k = h / 6.0;
                var weight_mid = 4.0 * h / 6.0;
                var weight_kp1 = h / 6.0;

                // Add contributions to gradient
                // grad_k: [∂L/∂x (StateDim), ∂L/∂u (ControlDim), ∂L/∂t]
                var stateControlDim = problem.StateDim + problem.ControlDim;

                // Contribution from k-th node
                var offset_k = k * stateControlDim;
                for (var i = 0; i < problem.StateDim; i++)
                {
                    // Direct contribution from L_k
                    gradient[offset_k + i] += weight_k * grad_k[i];
                    // Contribution from L_mid (midpoint depends on both endpoints)
                    gradient[offset_k + i] += weight_mid * grad_mid[i] * 0.5;
                }
                for (var i = 0; i < problem.ControlDim; i++)
                {
                    gradient[offset_k + problem.StateDim + i] += weight_k * grad_k[problem.StateDim + i];
                    gradient[offset_k + problem.StateDim + i] += weight_mid * grad_mid[problem.StateDim + i] * 0.5;
                }

                // Contribution from (k+1)-th node
                var offset_kp1 = (k + 1) * stateControlDim;
                for (var i = 0; i < problem.StateDim; i++)
                {
                    gradient[offset_kp1 + i] += weight_kp1 * grad_kp1[i];
                    gradient[offset_kp1 + i] += weight_mid * grad_mid[i] * 0.5;
                }
                for (var i = 0; i < problem.ControlDim; i++)
                {
                    gradient[offset_kp1 + problem.StateDim + i] += weight_kp1 * grad_kp1[problem.StateDim + i];
                    gradient[offset_kp1 + problem.StateDim + i] += weight_mid * grad_mid[problem.StateDim + i] * 0.5;
                }
            }

            return gradient;
        }

        /// <summary>
        /// Computes the gradient of terminal cost with respect to the decision vector.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="z">The decision vector.</param>
        /// <param name="getState">Function to extract state from decision vector.</param>
        /// <param name="terminalCostWithGradients">Terminal cost function that returns (value, gradients).</param>
        /// <returns>Gradient of terminal cost with respect to decision vector.</returns>
        public static double[] ComputeTerminalCostGradient(
            ControlProblem problem,
            CollocationGrid grid,
            double[] z,
            Func<double[], int, double[]> getState,
            Func<double[], double, (double value, double[] gradients)>? terminalCostWithGradients)
        {
            var n = z.Length;
            var gradient = new double[n];

            if (terminalCostWithGradients == null)
            {
                return gradient;
            }

            // Get final state
            var x_final = getState(z, grid.Segments);
            var t_final = grid.TimePoints[^1];

            // Evaluate terminal cost and gradient
            var (_, grad) = terminalCostWithGradients(x_final, t_final);

            // grad: [∂Φ/∂x (StateDim), ∂Φ/∂t]
            // Only affects the final state in the decision vector
            var stateControlDim = problem.StateDim + problem.ControlDim;
            var offset_final = grid.Segments * stateControlDim;

            for (var i = 0; i < problem.StateDim; i++)
            {
                gradient[offset_final + i] = grad[i];
            }

            return gradient;
        }

        /// <summary>
        /// Computes the gradient of a defect constraint with respect to the decision vector.
        /// Uses AutoDiff gradients from dynamics if available.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="z">The decision vector.</param>
        /// <param name="getState">Function to extract state from decision vector.</param>
        /// <param name="getControl">Function to extract control from decision vector.</param>
        /// <param name="segmentIndex">Index of the segment.</param>
        /// <param name="stateComponentIndex">Index of the state component.</param>
        /// <param name="dynamicsWithGradients">Dynamics function that returns (value, gradients).</param>
        /// <returns>Gradient of defect constraint with respect to decision vector.</returns>
        public static double[] ComputeDefectGradient(
            ControlProblem problem,
            CollocationGrid grid,
            double[] z,
            Func<double[], int, double[]> getState,
            Func<double[], int, double[]> getControl,
            int segmentIndex,
            int stateComponentIndex,
            Func<double[], double[], double, (double[] value, double[][]? gradients)>? dynamicsWithGradients)
        {
            var n = z.Length;
            var gradient = new double[n];

            if (dynamicsWithGradients == null)
            {
                return gradient;
            }

            var k = segmentIndex;
            var i = stateComponentIndex;

            var t_k = grid.TimePoints[k];
            var t_kp1 = grid.TimePoints[k + 1];
            var h = t_kp1 - t_k;

            // Get states and controls
            var x_k = getState(z, k);
            var u_k = getControl(z, k);
            var x_kp1 = getState(z, k + 1);
            var u_kp1 = getControl(z, k + 1);

            // Evaluate dynamics at endpoints first (needed for Hermite interpolation)
            var (f_k, grad_k) = dynamicsWithGradients(x_k, u_k, t_k);
            var (f_kp1, grad_kp1) = dynamicsWithGradients(x_kp1, u_kp1, t_kp1);

            // Compute midpoint using HERMITE interpolation (must match HermiteSimpsonTranscription)
            // x_mid = (x_k + x_{k+1})/2 + (h/8)(f_k - f_{k+1})
            var x_mid = new double[problem.StateDim];
            var u_mid = new double[problem.ControlDim];
            for (var j = 0; j < problem.StateDim; j++)
            {
                x_mid[j] = 0.5 * (x_k[j] + x_kp1[j]) + (h / 8.0) * (f_k[j] - f_kp1[j]);
            }
            for (var j = 0; j < problem.ControlDim; j++)
            {
                u_mid[j] = 0.5 * (u_k[j] + u_kp1[j]);
            }

            var t_mid = 0.5 * (t_k + t_kp1);

            // Evaluate dynamics at midpoint
            var (f_mid, grad_mid) = dynamicsWithGradients(x_mid, u_mid, t_mid);

            // Defect for state component i:
            // c_i = x_{k+1,i} - x_{k,i} - h/6 * (f_{k,i} + 4*f_{mid,i} + f_{k+1,i})

            var stateControlDim = problem.StateDim + problem.ControlDim;
            var offset_k = k * stateControlDim;
            var offset_kp1 = (k + 1) * stateControlDim;

            // ∂c_i/∂x_k = -I - h/6 * ∂f_k/∂x
            //           - 4h/6 * ∂f_mid/∂x_mid * (∂x_mid/∂x_k + ∂x_mid/∂f_k * ∂f_k/∂x_k)
            // where ∂x_mid/∂x_k = 0.5*I and ∂x_mid/∂f_k = (h/8)*I
            if (grad_k != null && grad_k.Length >= 2 && grad_k[0] != null)
            {
                var df_k_dx = grad_k[0]; // [∂f/∂x] is flattened: [∂f_0/∂x_0, ∂f_0/∂x_1, ..., ∂f_1/∂x_0, ...]
                for (var j = 0; j < problem.StateDim; j++)
                {
                    var df_dx_ij = df_k_dx[i * problem.StateDim + j];
                    // Direct term: -h/6 * ∂f_k,i/∂x_k,j
                    gradient[offset_k + j] += -h / 6.0 * df_dx_ij;

                    // Hermite chain term: -4h/6 * Σ_m (∂f_mid,i/∂x_mid,m) * (h/8) * (∂f_k,m/∂x_k,j)
                    if (grad_mid != null && grad_mid[0] != null)
                    {
                        var df_mid_dx = grad_mid[0];
                        for (var m = 0; m < problem.StateDim; m++)
                        {
                            var df_mid_dx_im = df_mid_dx[i * problem.StateDim + m];
                            var df_k_dx_mj = df_k_dx[m * problem.StateDim + j];
                            gradient[offset_k + j] += -4.0 * h / 6.0 * df_mid_dx_im * (h / 8.0) * df_k_dx_mj;
                        }
                    }
                }
                gradient[offset_k + i] += -1.0;
            }

            // Direct x_mid contribution: ∂f_mid/∂x_mid * ∂x_mid/∂x_k = ∂f_mid/∂x_mid * 0.5
            if (grad_mid != null && grad_mid.Length >= 2 && grad_mid[0] != null)
            {
                var df_mid_dx = grad_mid[0];
                for (var j = 0; j < problem.StateDim; j++)
                {
                    var df_dx_ij = df_mid_dx[i * problem.StateDim + j];
                    gradient[offset_k + j] += -4.0 * h / 6.0 * df_dx_ij * 0.5;
                    gradient[offset_kp1 + j] += -4.0 * h / 6.0 * df_dx_ij * 0.5;
                }
            }

            // ∂c_i/∂x_{k+1} = I - h/6 * ∂f_{k+1}/∂x
            //              - 4h/6 * ∂f_mid/∂x_mid * (∂x_mid/∂x_{k+1} + ∂x_mid/∂f_{k+1} * ∂f_{k+1}/∂x_{k+1})
            // where ∂x_mid/∂x_{k+1} = 0.5*I and ∂x_mid/∂f_{k+1} = (-h/8)*I
            if (grad_kp1 != null && grad_kp1.Length >= 2 && grad_kp1[0] != null)
            {
                var df_kp1_dx = grad_kp1[0];
                for (var j = 0; j < problem.StateDim; j++)
                {
                    var df_dx_ij = df_kp1_dx[i * problem.StateDim + j];
                    // Direct term: -h/6 * ∂f_{k+1,i}/∂x_{k+1,j}
                    gradient[offset_kp1 + j] += -h / 6.0 * df_dx_ij;

                    // Hermite chain term: -4h/6 * Σ_m (∂f_mid,i/∂x_mid,m) * (-h/8) * (∂f_{k+1,m}/∂x_{k+1,j})
                    if (grad_mid != null && grad_mid[0] != null)
                    {
                        var df_mid_dx = grad_mid[0];
                        for (var m = 0; m < problem.StateDim; m++)
                        {
                            var df_mid_dx_im = df_mid_dx[i * problem.StateDim + m];
                            var df_kp1_dx_mj = df_kp1_dx[m * problem.StateDim + j];
                            gradient[offset_kp1 + j] += -4.0 * h / 6.0 * df_mid_dx_im * (-h / 8.0) * df_kp1_dx_mj;
                        }
                    }
                }
                gradient[offset_kp1 + i] += 1.0;
            }

            // ∂c_i/∂u_k: direct term + Hermite chain rule term
            if (grad_k != null && grad_k.Length >= 2 && grad_k[1] != null)
            {
                var df_k_du = grad_k[1];
                for (var j = 0; j < problem.ControlDim; j++)
                {
                    var df_du_ij = df_k_du[i * problem.ControlDim + j];
                    // Direct: -h/6 * ∂f_k/∂u_k
                    gradient[offset_k + problem.StateDim + j] += -h / 6.0 * df_du_ij;

                    // Hermite chain: -4h/6 * ∂f_mid/∂x_mid * (h/8) * ∂f_k/∂u_k
                    // Sum over all state components m that x_mid depends on
                    if (grad_mid != null && grad_mid[0] != null)
                    {
                        var df_mid_dx = grad_mid[0];
                        for (var m = 0; m < problem.StateDim; m++)
                        {
                            // ∂f_{mid,i}/∂x_{mid,m}
                            var df_mid_dx_im = df_mid_dx[i * problem.StateDim + m];
                            // ∂x_{mid,m}/∂f_{k,m} = h/8 (only diagonal)
                            // ∂f_{k,m}/∂u_{k,j}
                            var df_k_du_mj = df_k_du[m * problem.ControlDim + j];
                            gradient[offset_k + problem.StateDim + j] += -4.0 * h / 6.0 * df_mid_dx_im * (h / 8.0) * df_k_du_mj;
                        }
                    }
                }
            }

            // u_mid contribution to f_mid (direct: ∂f_mid/∂u_mid * ∂u_mid/∂u_k)
            if (grad_mid != null && grad_mid.Length >= 2 && grad_mid[1] != null)
            {
                var df_mid_du = grad_mid[1];
                for (var j = 0; j < problem.ControlDim; j++)
                {
                    var df_du_ij = df_mid_du[i * problem.ControlDim + j];
                    gradient[offset_k + problem.StateDim + j] += -4.0 * h / 6.0 * df_du_ij * 0.5;
                    gradient[offset_kp1 + problem.StateDim + j] += -4.0 * h / 6.0 * df_du_ij * 0.5;
                }
            }

            // ∂c_i/∂u_{k+1}: direct term + Hermite chain rule term
            if (grad_kp1 != null && grad_kp1.Length >= 2 && grad_kp1[1] != null)
            {
                var df_kp1_du = grad_kp1[1];
                for (var j = 0; j < problem.ControlDim; j++)
                {
                    var df_du_ij = df_kp1_du[i * problem.ControlDim + j];
                    // Direct: -h/6 * ∂f_{k+1}/∂u_{k+1}
                    gradient[offset_kp1 + problem.StateDim + j] += -h / 6.0 * df_du_ij;

                    // Hermite chain: -4h/6 * ∂f_mid/∂x_mid * (-h/8) * ∂f_{k+1}/∂u_{k+1}
                    if (grad_mid != null && grad_mid[0] != null)
                    {
                        var df_mid_dx = grad_mid[0];
                        for (var m = 0; m < problem.StateDim; m++)
                        {
                            var df_mid_dx_im = df_mid_dx[i * problem.StateDim + m];
                            var df_kp1_du_mj = df_kp1_du[m * problem.ControlDim + j];
                            // Note: ∂x_mid/∂f_{k+1} = -h/8 (negative!)
                            gradient[offset_kp1 + problem.StateDim + j] += -4.0 * h / 6.0 * df_mid_dx_im * (-h / 8.0) * df_kp1_du_mj;
                        }
                    }
                }
            }

            return gradient;
        }
    }
}
