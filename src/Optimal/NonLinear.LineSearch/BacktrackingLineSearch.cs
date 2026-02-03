/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Buffers;

namespace Optimal.NonLinear.LineSearch
{
    /// <summary>
    /// Backtracking line search using the Armijo condition for sufficient decrease.
    /// </summary>
    public sealed class BacktrackingLineSearch : ILineSearch
    {
        private readonly double _c1;
        private readonly double _rho;
        private readonly int _maxIterations;

        /// <summary>
        /// Initializes a new instance of the <see cref="BacktrackingLineSearch"/> class.
        /// </summary>
        /// <param name="c1">The Armijo condition parameter (typically 1e-4). Must be in (0, 1).</param>
        /// <param name="rho">The backtracking reduction factor (typically 0.5). Must be in (0, 1).</param>
        /// <param name="maxIterations">Maximum number of backtracking iterations (default 50).</param>
        public BacktrackingLineSearch(double c1 = 1e-4, double rho = 0.5, int maxIterations = 50)
        {
            if (c1 <= 0 || c1 >= 1)
            {
                throw new ArgumentException("c1 must be in (0, 1)", nameof(c1));
            }

            if (rho <= 0 || rho >= 1)
            {
                throw new ArgumentException("rho must be in (0, 1)", nameof(rho));
            }

            if (maxIterations <= 0)
            {
                throw new ArgumentException("maxIterations must be positive", nameof(maxIterations));
            }

            _c1 = c1;
            _rho = rho;
            _maxIterations = maxIterations;
        }

        /// <inheritdoc/>
        public double FindStepSize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] x,
            double fx,
            double[] gradient,
            double[] direction,
            double initialStepSize)
        {
            var n = x.Length;
            var alpha = initialStepSize;

            // Compute the directional derivative: grad^T * direction using SIMD
            var directionalDerivative = VectorOps.Dot(gradient, direction);

            // If we're not moving downhill, return 0 (no step)
            if (directionalDerivative >= 0)
            {
                return 0.0;
            }

            // Rent temporary array for evaluating candidate points
            var pool = ArrayPool<double>.Shared;
            var xNew = pool.Rent(n);

            try
            {
                var xNewSpan = xNew.AsSpan(0, n);

                // Backtracking loop
                for (var iter = 0; iter < _maxIterations; iter++)
                {
                    // Compute candidate point: x_new = x + alpha * direction using SIMD
                    VectorOps.AddScaled(x, alpha, direction, xNewSpan);

                    // Evaluate objective at candidate point
                    var (fNew, _) = objective(xNew.AsSpan(0, n).ToArray());

                    // Check Armijo condition: f(x + alpha*d) <= f(x) + c1 * alpha * grad^T * d
                    var armijoRhs = fx + _c1 * alpha * directionalDerivative;
                    if (fNew <= armijoRhs)
                    {
                        // Sufficient decrease achieved
                        return alpha;
                    }

                    // Reduce step size
                    alpha *= _rho;

                    // Check if step size became too small
                    if (alpha < 1e-16)
                    {
                        return 0.0;
                    }
                }

                // Maximum iterations reached, return current alpha
                return alpha;
            }
            finally
            {
                pool.Return(xNew);
            }
        }
    }
}
