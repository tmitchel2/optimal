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
    /// Gradient descent optimizer using fixed step size.
    /// </summary>
    public sealed class GradientDescentOptimizer : IOptimizer
    {
        private double[] _x0 = Array.Empty<double>();
        private double _tolerance = 1e-6;
        private int _maxIterations = 1000;
        private double _stepSize = 0.01;
        private bool _verbose;

        /// <summary>
        /// Sets the step size for gradient descent.
        /// </summary>
        /// <param name="stepSize">The step size (learning rate).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public GradientDescentOptimizer WithStepSize(double stepSize)
        {
            _stepSize = stepSize;
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

                // Update: x = x - stepSize * gradient
                for (var i = 0; i < n; i++)
                {
                    x[i] -= _stepSize * gradient[i];
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
