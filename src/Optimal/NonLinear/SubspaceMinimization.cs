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
    /// Performs subspace minimization for L-BFGS-B within the space of free variables.
    /// This is the second phase of the L-BFGS-B algorithm, applied after the Cauchy point
    /// is computed to further improve the solution within the subspace of free variables.
    /// </summary>
    public static class SubspaceMinimization
    {
        /// <summary>
        /// Computes the search direction using L-BFGS in the reduced subspace of free variables.
        /// Variables that are at bounds (active) have their direction components set to zero.
        /// </summary>
        /// <param name="x">Current point (typically the Cauchy point).</param>
        /// <param name="gradient">Full gradient at current point.</param>
        /// <param name="freeVariables">Boolean mask of free (non-active) variables.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <param name="memory">L-BFGS memory.</param>
        /// <returns>Search direction in full space (zeros for active variables).</returns>
        public static double[] ComputeSubspaceDirection(
            double[] x,
            double[] gradient,
            bool[] freeVariables,
            double[] lower,
            double[] upper,
            LBFGSMemory memory)
        {
            var n = gradient.Length;

            // Count free variables
            var numFree = 0;
            for (var i = 0; i < n; i++)
            {
                if (freeVariables[i])
                {
                    numFree++;
                }
            }

            // If no free variables, return zero direction
            if (numFree == 0)
            {
                return new double[n];
            }

            // If all variables are free, use standard two-loop recursion
            if (numFree == n)
            {
                var direction = TwoLoopRecursion.ComputeDirection(gradient, memory);
                return ClipDirectionToBounds(direction, x, lower, upper);
            }

            // Create index mapping: reduced space -> full space
            var reducedToFull = new int[numFree];
            var reducedIndex = 0;

            for (var i = 0; i < n; i++)
            {
                if (freeVariables[i])
                {
                    reducedToFull[reducedIndex] = i;
                    reducedIndex++;
                }
            }

            // Extract reduced gradient
            var reducedGradient = new double[numFree];
            for (var i = 0; i < numFree; i++)
            {
                reducedGradient[i] = gradient[reducedToFull[i]];
            }

            // Apply two-loop recursion in reduced space
            var reducedDirection = ComputeReducedDirection(
                reducedGradient,
                memory,
                reducedToFull,
                numFree);

            // Map back to full space
            var fullDirection = new double[n];
            for (var i = 0; i < numFree; i++)
            {
                fullDirection[reducedToFull[i]] = reducedDirection[i];
            }

            // Clip direction to ensure we don't violate bounds
            return ClipDirectionToBounds(fullDirection, x, lower, upper);
        }

        /// <summary>
        /// Computes the L-BFGS direction in the reduced space.
        /// Uses the stored correction pairs but projects them onto the free variables.
        /// </summary>
        private static double[] ComputeReducedDirection(
            double[] reducedGradient,
            LBFGSMemory memory,
            int[] reducedToFull,
            int numFree)
        {
            var m = memory.Count;

            if (m == 0 || numFree == 0)
            {
                // No history, return steepest descent in reduced space
                var direction = new double[numFree];
                VectorOps.Negate(reducedGradient, direction);
                return direction;
            }

            var pool = ArrayPool<double>.Shared;

            // Extract reduced s and y vectors from memory
            var reducedS = new double[m][];
            var reducedY = new double[m][];
            var reducedRho = new double[m];

            for (var k = 0; k < m; k++)
            {
                var (s, y, _) = memory.GetPair(k);

                reducedS[k] = new double[numFree];
                reducedY[k] = new double[numFree];

                for (var i = 0; i < numFree; i++)
                {
                    reducedS[k][i] = s[reducedToFull[i]];
                    reducedY[k][i] = y[reducedToFull[i]];
                }

                // Recompute rho for reduced vectors using SIMD
                var yTs = VectorOps.Dot(reducedY[k], reducedS[k]);
                reducedRho[k] = Math.Abs(yTs) > 1e-16 ? 1.0 / yTs : 0.0;
            }

            // Two-loop recursion in reduced space with pooled arrays
            var alpha = pool.Rent(m);
            var q = pool.Rent(numFree);

            try
            {
                var qSpan = q.AsSpan(0, numFree);
                VectorOps.Copy(reducedGradient, qSpan);

                // First loop: backwards through memory
                for (var i = m - 1; i >= 0; i--)
                {
                    if (reducedRho[i] == 0.0)
                    {
                        continue;
                    }

                    alpha[i] = reducedRho[i] * VectorOps.Dot(reducedS[i], qSpan);
                    VectorOps.AddScaled(qSpan, -alpha[i], reducedY[i], qSpan);
                }

                // Initial Hessian: H_0 = gamma * I
                // Find the most recent pair with valid rho
                var gamma = 1.0;
                for (var i = m - 1; i >= 0; i--)
                {
                    if (reducedRho[i] != 0.0)
                    {
                        var yTy = VectorOps.Dot(reducedY[i], reducedY[i]);
                        if (yTy > 1e-16)
                        {
                            gamma = 1.0 / (reducedRho[i] * yTy);
                        }
                        break;
                    }
                }

                var r = new double[numFree];
                VectorOps.Scale(qSpan, gamma, r);

                // Second loop: forwards through memory
                for (var i = 0; i < m; i++)
                {
                    if (reducedRho[i] == 0.0)
                    {
                        continue;
                    }

                    var beta = reducedRho[i] * VectorOps.Dot(reducedY[i], r);
                    VectorOps.AddScaled(r, alpha[i] - beta, reducedS[i], r);
                }

                // Negate for descent direction
                VectorOps.Negate(r, r);

                return r;
            }
            finally
            {
                pool.Return(alpha);
                pool.Return(q);
            }
        }

        /// <summary>
        /// Clips the direction vector to ensure that moving along it won't immediately
        /// violate bounds. This is a safeguard for numerical stability.
        /// </summary>
        private static double[] ClipDirectionToBounds(
            double[] direction,
            double[] x,
            double[] lower,
            double[] upper)
        {
            var n = direction.Length;
            var clipped = new double[n];

            for (var i = 0; i < n; i++)
            {
                var d = direction[i];

                // If at lower bound, don't allow negative direction
                if (x[i] <= lower[i] + 1e-10 && d < 0)
                {
                    clipped[i] = 0;
                }
                // If at upper bound, don't allow positive direction
                else if (x[i] >= upper[i] - 1e-10 && d > 0)
                {
                    clipped[i] = 0;
                }
                else
                {
                    clipped[i] = d;
                }
            }

            return clipped;
        }

        /// <summary>
        /// Computes a simple projected gradient direction for the free variables.
        /// This is a fallback when L-BFGS memory is empty or produces invalid results.
        /// </summary>
        /// <param name="gradient">Full gradient.</param>
        /// <param name="freeVariables">Boolean mask of free variables.</param>
        /// <param name="x">Current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>Projected gradient direction.</returns>
        public static double[] ComputeProjectedGradientDirection(
            double[] gradient,
            bool[] freeVariables,
            double[] x,
            double[] lower,
            double[] upper)
        {
            var n = gradient.Length;
            var direction = new double[n];

            for (var i = 0; i < n; i++)
            {
                if (!freeVariables[i])
                {
                    direction[i] = 0;
                    continue;
                }

                var d = -gradient[i];

                // Clip to respect bounds
                if (x[i] <= lower[i] + 1e-10 && d < 0)
                {
                    direction[i] = 0;
                }
                else if (x[i] >= upper[i] - 1e-10 && d > 0)
                {
                    direction[i] = 0;
                }
                else
                {
                    direction[i] = d;
                }
            }

            return direction;
        }
    }
}
