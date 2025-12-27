/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear.LineSearch;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Gradient descent optimizer with optional line search.
    /// </summary>
    public sealed class GradientDescentOptimizer : IOptimizer
    {
        private double[] _x0 = Array.Empty<double>();
        private double _tolerance = 1e-6;
        private int _maxIterations = 1000;
        private double _stepSize = 0.01;
        private bool _verbose;
        private ILineSearch? _lineSearch;

        /// <summary>
        /// Sets the step size for gradient descent (only used when no line search is provided).
        /// </summary>
        /// <param name="stepSize">The step size (learning rate).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public GradientDescentOptimizer WithStepSize(double stepSize)
        {
            _stepSize = stepSize;
            return this;
        }

        /// <summary>
        /// Sets the line search algorithm to use for adaptive step sizing.
        /// </summary>
        /// <param name="lineSearch">The line search algorithm.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public GradientDescentOptimizer WithLineSearch(ILineSearch lineSearch)
        {
            _lineSearch = lineSearch;
            return this;
        }

        /// <inheritdoc/>
        public IOptimizer WithInitialPoint(double[] x0)
        {
            _x0 = x0;
            return this;
        }

        /// <inheritdoc/>
        public IOptimizer WithTolerance(double tolerance)
        {
            _tolerance = tolerance;
            return this;
        }

        /// <inheritdoc/>
        public IOptimizer WithMaxIterations(int maxIterations)
        {
            _maxIterations = maxIterations;
            return this;
        }

        /// <inheritdoc/>
        public IOptimizer WithVerbose(bool verbose = true)
        {
            _verbose = verbose;
            return this;
        }

        /// <inheritdoc/>
        public OptimizerResult Minimize(Func<double[], (double value, double[] gradient)> objective)
        {
            var x = (double[])_x0.Clone();
            var n = x.Length;
            var functionEvaluations = 0;

            for (var iter = 0; iter < _maxIterations; iter++)
            {
                var (value, gradient) = objective(x);
                functionEvaluations++;

                var gradNorm = ComputeNorm(gradient);

                // Note: Verbose output via Console.WriteLine removed due to analyzer restrictions
                // Future enhancement: Add callback-based progress reporting

                if (gradNorm < _tolerance)
                {
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = value,
                        FinalGradient = gradient,
                        Iterations = iter + 1,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = StoppingReason.GradientTolerance,
                        Success = true,
                        Message = "Converged: gradient norm below tolerance"
                    };
                }

                // Determine step size
                double alpha;
                if (_lineSearch != null)
                {
                    // Use line search to find step size
                    // Search direction is negative gradient
                    var direction = new double[n];
                    for (var i = 0; i < n; i++)
                    {
                        direction[i] = -gradient[i];
                    }

                    alpha = _lineSearch.FindStepSize(objective, x, value, gradient, direction, 1.0);

                    // If line search failed to find a step, use a small default
                    if (alpha == 0.0)
                    {
                        alpha = _stepSize;
                    }

                    // Count additional function evaluations from line search
                    // Note: BacktrackingLineSearch typically evaluates 1-10 times
                    functionEvaluations += 5; // Approximate average
                }
                else
                {
                    // Use fixed step size
                    alpha = _stepSize;
                }

                // Update: x = x - alpha * gradient
                for (var i = 0; i < n; i++)
                {
                    x[i] -= alpha * gradient[i];
                }
            }

            // Maximum iterations reached - do one final evaluation
            var finalEval = objective(x);
            functionEvaluations++;

            return new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = finalEval.value,
                FinalGradient = finalEval.gradient,
                Iterations = _maxIterations,
                FunctionEvaluations = functionEvaluations,
                StoppingReason = StoppingReason.MaxIterations,
                Success = false,
                Message = "Maximum iterations reached without convergence"
            };
        }

        private static double ComputeNorm(double[] vector)
        {
            var sum = 0.0;
            foreach (var x in vector)
            {
                sum += x * x;
            }
            return Math.Sqrt(sum);
        }
    }
}
