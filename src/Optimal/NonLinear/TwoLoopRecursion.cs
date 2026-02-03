/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Buffers;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Implements the L-BFGS two-loop recursion algorithm for computing search directions.
    /// </summary>
    public static class TwoLoopRecursion
    {
        /// <summary>
        /// Computes the L-BFGS search direction using the two-loop recursion algorithm.
        /// </summary>
        /// <param name="gradient">Current gradient vector.</param>
        /// <param name="memory">L-BFGS memory containing correction pairs.</param>
        /// <returns>Search direction (approximately -H * gradient, where H is inverse Hessian approximation).</returns>
        public static double[] ComputeDirection(double[] gradient, LBFGSMemory memory)
        {
            return ComputeDirection(gradient, memory, preconditioner: null);
        }

        /// <summary>
        /// Computes the L-BFGS search direction using the two-loop recursion algorithm with optional preconditioning.
        /// </summary>
        /// <param name="gradient">Current gradient vector.</param>
        /// <param name="memory">L-BFGS memory containing correction pairs.</param>
        /// <param name="preconditioner">Optional preconditioner for ill-conditioned problems.</param>
        /// <returns>Search direction (approximately -H * gradient, where H is inverse Hessian approximation).</returns>
        public static double[] ComputeDirection(double[] gradient, LBFGSMemory memory, IPreconditioner? preconditioner)
        {
            var n = gradient.Length;
            var m = memory.Count;

            if (m == 0)
            {
                // No history available, return steepest descent
                var direction = new double[n];
                VectorOps.Negate(gradient, direction);
                return direction;
            }

            var pool = ArrayPool<double>.Shared;

            // Allocate temporary storage for alpha values
            var alpha = pool.Rent(m);
            // Initialize q = gradient
            var q = pool.Rent(n);

            try
            {
                var qSpan = q.AsSpan(0, n);
                VectorOps.Copy(gradient, qSpan);

                // First loop: iterate backwards through memory (newest to oldest)
                for (var i = m - 1; i >= 0; i--)
                {
                    var (s, y, rho) = memory.GetPair(i);

                    // alpha_i = rho_i * s_i^T * q
                    alpha[i] = rho * VectorOps.Dot(s, qSpan);

                    // q = q - alpha_i * y_i
                    VectorOps.AddScaled(qSpan, -alpha[i], y, qSpan);
                }

                // Initialize Hessian approximation using most recent correction pair
                var (s_k, y_k, _) = memory.GetPair(m - 1);
                var gamma = VectorOps.Dot(s_k, y_k) / VectorOps.Dot(y_k, y_k);

                // r = H_0 * q, where H_0 = gamma * I (or preconditioned variant)
                var r = new double[n];
                if (preconditioner != null)
                {
                    // Use preconditioner for H_0: r = P^{-1} * q where P approximates Hessian
                    // The preconditioner returns M^{-1} * q, so we scale by gamma for consistency
                    var preconditionedQ = preconditioner.ApplyToGradient(q.AsSpan(0, n).ToArray());
                    VectorOps.Scale(preconditionedQ, gamma, r);
                }
                else
                {
                    VectorOps.Scale(qSpan, gamma, r);
                }

                // Second loop: iterate forwards through memory (oldest to newest)
                for (var i = 0; i < m; i++)
                {
                    var (s, y, rho) = memory.GetPair(i);

                    // beta = rho_i * y_i^T * r
                    var beta = rho * VectorOps.Dot(y, r);

                    // r = r + s_i * (alpha_i - beta)
                    VectorOps.AddScaled(r, alpha[i] - beta, s, r);
                }

                // Return -r as the search direction (negative because we want to minimize)
                VectorOps.Negate(r, r);

                return r;
            }
            finally
            {
                pool.Return(alpha);
                pool.Return(q);
            }
        }
    }
}
