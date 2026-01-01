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
    /// Helper methods for computing gradients using AutoDiff-generated derivatives
    /// for Legendre-Gauss-Lobatto collocation transcriptions.
    /// </summary>
    public static class AutoDiffLGLGradientHelper
    {
        /// <summary>
        /// Computes the gradient of the running cost with respect to the decision vector.
        /// Uses LGL quadrature and AutoDiff gradients.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="z">The decision vector.</param>
        /// <param name="order">LGL order (number of points per segment).</param>
        /// <param name="getState">Function to extract state from decision vector.</param>
        /// <param name="getControl">Function to extract control from decision vector.</param>
        /// <param name="runningCostWithGradients">Running cost function that returns (value, gradients).</param>
        /// <returns>Gradient of running cost with respect to decision vector.</returns>
        public static double[] ComputeRunningCostGradient(
            ControlProblem problem,
            CollocationGrid grid,
            double[] z,
            int order,
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

            var (lglPoints, lglWeights) = LegendreGaussLobatto.GetPointsAndWeights(order);
            var totalPoints = grid.Segments * (order - 1) + 1;

            // Iterate over all segments
            for (var k = 0; k < grid.Segments; k++)
            {
                var t_k = grid.TimePoints[k];
                var h = grid.GetTimeStep(k);

                // Iterate over all LGL points in this segment
                for (var localIdx = 0; localIdx < order; localIdx++)
                {
                    // Skip shared endpoint (already counted in previous segment)
                    if (k > 0 && localIdx == 0)
                    {
                        continue;
                    }

                    var globalIdx = k * (order - 1) + localIdx;
                    var tau = lglPoints[localIdx];
                    var t = t_k + (tau + 1.0) * h / 2.0;

                    var x = getState(z, globalIdx);
                    var u = getControl(z, globalIdx);

                    // Evaluate cost and gradients
                    var (L, grad) = runningCostWithGradients(x, u, t);

                    // LGL quadrature weight scaled for physical interval
                    var weight = (h / 2.0) * lglWeights[localIdx];

                    // Compute gradient contribution
                    // grad = [dL/dx_0, dL/dx_1, ..., dL/du_0, dL/du_1, ..., dL/dt]
                    var offset = globalIdx * (problem.StateDim + problem.ControlDim);

                    // State gradients
                    for (var i = 0; i < problem.StateDim; i++)
                    {
                        gradient[offset + i] += weight * grad[i];
                    }

                    // Control gradients
                    for (var i = 0; i < problem.ControlDim; i++)
                    {
                        gradient[offset + problem.StateDim + i] += weight * grad[problem.StateDim + i];
                    }
                }
            }

            return gradient;
        }

        /// <summary>
        /// Computes the gradient of the terminal cost with respect to the decision vector.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="z">The decision vector.</param>
        /// <param name="totalPoints">Total number of collocation points.</param>
        /// <param name="getState">Function to extract state from decision vector.</param>
        /// <param name="terminalCostWithGradients">Terminal cost function that returns (value, gradients).</param>
        /// <returns>Gradient of terminal cost with respect to decision vector.</returns>
        public static double[] ComputeTerminalCostGradient(
            ControlProblem problem,
            double[] z,
            int totalPoints,
            double finalTime,
            Func<double[], int, double[]> getState,
            Func<double[], double, (double value, double[] gradients)>? terminalCostWithGradients)
        {
            var n = z.Length;
            var gradient = new double[n];

            if (terminalCostWithGradients == null)
            {
                return gradient;
            }

            var finalPointIndex = totalPoints - 1;
            var x_f = getState(z, finalPointIndex);

            var (phi, grad) = terminalCostWithGradients(x_f, finalTime);

            // grad = [dPhi/dx_0, dPhi/dx_1, ..., dPhi/dt]
            var offset = finalPointIndex * (problem.StateDim + problem.ControlDim);

            for (var i = 0; i < problem.StateDim; i++)
            {
                gradient[offset + i] = grad[i];
            }

            return gradient;
        }

        /// <summary>
        /// Computes the gradient of a defect constraint with respect to the decision vector.
        /// For LGL collocation: defect = (2/h) * Σ_j D_ij * x_j - f(x_i, u_i, t_i)
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="z">The decision vector.</param>
        /// <param name="order">LGL order.</param>
        /// <param name="diffMatrix">LGL differentiation matrix.</param>
        /// <param name="getState">Function to extract state from decision vector.</param>
        /// <param name="getControl">Function to extract control from decision vector.</param>
        /// <param name="segmentIndex">Segment index.</param>
        /// <param name="interiorPointIndex">Interior point index within segment (1 to order-2).</param>
        /// <param name="stateComponentIndex">State component index.</param>
        /// <param name="dynamicsWithGradients">Dynamics function that returns (value, gradients).</param>
        /// <returns>Gradient of defect constraint with respect to decision vector.</returns>
        public static double[] ComputeDefectGradient(
            ControlProblem problem,
            CollocationGrid grid,
            double[] z,
            int order,
            double[,] diffMatrix,
            Func<double[], int, double[]> getState,
            Func<double[], int, double[]> getControl,
            int segmentIndex,
            int interiorPointIndex,
            int stateComponentIndex,
            Func<double[], double[], double, (double[] value, double[][] gradients)> dynamicsWithGradients)
        {
            var n = z.Length;
            var gradient = new double[n];

            var h = grid.GetTimeStep(segmentIndex);
            var alpha = 2.0 / h;

            // Global index of the interior point where we're evaluating the defect
            var globalIdx = segmentIndex * (order - 1) + interiorPointIndex;

            var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(order);
            var t_k = grid.TimePoints[segmentIndex];
            var tau = lglPoints[interiorPointIndex];
            var t = t_k + (tau + 1.0) * h / 2.0;

            var x_i = getState(z, globalIdx);
            var u_i = getControl(z, globalIdx);

            // Evaluate dynamics and gradients at the interior point
            var (f, f_grad) = dynamicsWithGradients(x_i, u_i, t);
            // f_grad[0] = df/dx, f_grad[1] = df/du

            // Gradient from differentiation matrix term: d/dz [(2/h) * Σ_j D_ij * x_j[s]]
            for (var j = 0; j < order; j++)
            {
                var globalIdx_j = segmentIndex * (order - 1) + j;
                var offset_j = globalIdx_j * (problem.StateDim + problem.ControlDim);

                // Contribution from x_j[stateComponentIndex]
                gradient[offset_j + stateComponentIndex] += alpha * diffMatrix[interiorPointIndex, j];
            }

            // Gradient from dynamics term: -d/dz [f(x_i, u_i, t)[s]]
            var offset_i = globalIdx * (problem.StateDim + problem.ControlDim);

            // df/dx contribution
            for (var j = 0; j < problem.StateDim; j++)
            {
                gradient[offset_i + j] -= f_grad[0][stateComponentIndex * problem.StateDim + j];
            }

            // df/du contribution
            for (var j = 0; j < problem.ControlDim; j++)
            {
                gradient[offset_i + problem.StateDim + j] -= f_grad[1][stateComponentIndex * problem.ControlDim + j];
            }

            return gradient;
        }
    }
}
