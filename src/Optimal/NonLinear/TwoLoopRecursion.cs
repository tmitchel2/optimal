/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

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
            var n = gradient.Length;
            var m = memory.Count;

            if (m == 0)
            {
                // No history available, return steepest descent
                var direction = new double[n];
                for (var i = 0; i < n; i++)
                {
                    direction[i] = -gradient[i];
                }
                return direction;
            }

            // Allocate temporary storage for alpha values
            var alpha = new double[m];

            // Initialize q = gradient
            var q = new double[n];
            Array.Copy(gradient, q, n);

            // First loop: iterate backwards through memory (newest to oldest)
            for (var i = m - 1; i >= 0; i--)
            {
                var (s, y, rho) = memory.GetPair(i);

                // alpha_i = rho_i * s_i^T * q
                alpha[i] = rho * DotProduct(s, q);

                // q = q - alpha_i * y_i
                for (var j = 0; j < n; j++)
                {
                    q[j] -= alpha[i] * y[j];
                }
            }

            // Initialize Hessian approximation using most recent correction pair
            var (s_k, y_k, _) = memory.GetPair(m - 1);
            var gamma = DotProduct(s_k, y_k) / DotProduct(y_k, y_k);

            // r = H_0 * q, where H_0 = gamma * I
            var r = new double[n];
            for (var i = 0; i < n; i++)
            {
                r[i] = gamma * q[i];
            }

            // Second loop: iterate forwards through memory (oldest to newest)
            for (var i = 0; i < m; i++)
            {
                var (s, y, rho) = memory.GetPair(i);

                // beta = rho_i * y_i^T * r
                var beta = rho * DotProduct(y, r);

                // r = r + s_i * (alpha_i - beta)
                for (var j = 0; j < n; j++)
                {
                    r[j] += s[j] * (alpha[i] - beta);
                }
            }

            // Return -r as the search direction (negative because we want to minimize)
            for (var i = 0; i < n; i++)
            {
                r[i] = -r[i];
            }

            return r;
        }

        private static double DotProduct(double[] a, double[] b)
        {
            var result = 0.0;
            for (var i = 0; i < a.Length; i++)
            {
                result += a[i] * b[i];
            }
            return result;
        }
    }
}
