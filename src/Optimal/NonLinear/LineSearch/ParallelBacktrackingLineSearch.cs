/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Threading.Tasks;

namespace Optimal.NonLinear.LineSearch
{
    /// <summary>
    /// Parallel backtracking line search using the Armijo condition.
    /// Evaluates multiple candidate step sizes simultaneously to reduce wall-clock time.
    /// </summary>
    public sealed class ParallelBacktrackingLineSearch : ILineSearch
    {
        private readonly double _c1;
        private readonly double _rho;
        private readonly int _maxIterations;
        private readonly int _parallelBatchSize;
        private readonly bool _enableParallelization;

        /// <summary>
        /// Initializes a new instance of the <see cref="ParallelBacktrackingLineSearch"/> class.
        /// </summary>
        /// <param name="c1">The Armijo condition parameter (typically 1e-4). Must be in (0, 1).</param>
        /// <param name="rho">The backtracking reduction factor (typically 0.5). Must be in (0, 1).</param>
        /// <param name="maxIterations">Maximum number of backtracking iterations (default 50).</param>
        /// <param name="parallelBatchSize">Number of candidate step sizes to evaluate in parallel (default 4).</param>
        /// <param name="enableParallelization">Enable parallel evaluation (default true).</param>
        public ParallelBacktrackingLineSearch(
            double c1 = 1e-4,
            double rho = 0.5,
            int maxIterations = 50,
            int parallelBatchSize = 4,
            bool enableParallelization = true)
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

            if (parallelBatchSize <= 0)
            {
                throw new ArgumentException("parallelBatchSize must be positive", nameof(parallelBatchSize));
            }

            _c1 = c1;
            _rho = rho;
            _maxIterations = maxIterations;
            _parallelBatchSize = parallelBatchSize;
            _enableParallelization = enableParallelization;
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

            // Compute the directional derivative: grad^T * direction
            var directionalDerivative = ComputeDotProduct(gradient, direction);

            // If we're not moving downhill, return 0 (no step)
            if (directionalDerivative >= 0)
            {
                return 0.0;
            }

            if (!_enableParallelization)
            {
                // Fall back to sequential backtracking
                return SequentialBacktracking(objective, x, fx, directionalDerivative, direction, initialStepSize, n);
            }

            // Parallel backtracking: evaluate multiple candidate step sizes in batches
            var alpha = initialStepSize;
            var iter = 0;

            while (iter < _maxIterations)
            {
                // Generate batch of candidate step sizes: alpha, alpha*rho, alpha*rho^2, ...
                var batchSize = Math.Min(_parallelBatchSize, _maxIterations - iter);
                var candidateAlphas = new double[batchSize];
                var candidateResults = new (double alpha, double fNew, bool satisfies)[batchSize];

                for (var i = 0; i < batchSize; i++)
                {
                    candidateAlphas[i] = alpha * Math.Pow(_rho, i);
                }

                // Evaluate all candidates in parallel
                Parallel.For(0, batchSize, i =>
                {
                    var candidateAlpha = candidateAlphas[i];

                    // Check if step size is too small
                    if (candidateAlpha < 1e-16)
                    {
                        candidateResults[i] = (candidateAlpha, double.PositiveInfinity, false);
                        return;
                    }

                    // Compute candidate point: x_new = x + alpha * direction
                    var xNew = new double[n];
                    for (var j = 0; j < n; j++)
                    {
                        xNew[j] = x[j] + candidateAlpha * direction[j];
                    }

                    // Evaluate objective at candidate point
                    var (fNew, _) = objective(xNew);

                    // Check Armijo condition: f(x + alpha*d) <= f(x) + c1 * alpha * grad^T * d
                    var armijoRhs = fx + _c1 * candidateAlpha * directionalDerivative;
                    var satisfiesArmijo = fNew <= armijoRhs;

                    candidateResults[i] = (candidateAlpha, fNew, satisfiesArmijo);
                });

                // Find the largest alpha that satisfies the Armijo condition
                for (var i = 0; i < batchSize; i++)
                {
                    if (candidateResults[i].satisfies)
                    {
                        return candidateResults[i].alpha;
                    }

                    if (candidateResults[i].alpha < 1e-16)
                    {
                        return 0.0;
                    }
                }

                // None of the candidates satisfied the condition, advance to next batch
                alpha = candidateAlphas[batchSize - 1] * _rho;
                iter += batchSize;

                if (alpha < 1e-16)
                {
                    return 0.0;
                }
            }

            // Maximum iterations reached
            return alpha;
        }

        private double SequentialBacktracking(
            Func<double[], (double value, double[] gradient)> objective,
            double[] x,
            double fx,
            double directionalDerivative,
            double[] direction,
            double initialStepSize,
            int n)
        {
            var alpha = initialStepSize;
            var xNew = new double[n];

            for (var iter = 0; iter < _maxIterations; iter++)
            {
                // Compute candidate point: x_new = x + alpha * direction
                for (var i = 0; i < n; i++)
                {
                    xNew[i] = x[i] + alpha * direction[i];
                }

                // Evaluate objective at candidate point
                var (fNew, _) = objective(xNew);

                // Check Armijo condition
                var armijoRhs = fx + _c1 * alpha * directionalDerivative;
                if (fNew <= armijoRhs)
                {
                    return alpha;
                }

                // Reduce step size
                alpha *= _rho;

                if (alpha < 1e-16)
                {
                    return 0.0;
                }
            }

            return alpha;
        }

        private static double ComputeDotProduct(double[] a, double[] b)
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
