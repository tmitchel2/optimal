/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Gradient descent optimizer with optional line search.
    /// </summary>
    public sealed class GradientDescentOptimizer : IOptimizer
    {
        private readonly GradientDescentOptions _options;
        private readonly ILineSearch? _lineSearch;

        /// <summary>
        /// Creates a new Gradient Descent optimizer with the specified options and optional line search.
        /// </summary>
        /// <param name="options">Optimizer options.</param>
        /// <param name="lineSearch">Optional line search algorithm for adaptive step sizing.</param>
        public GradientDescentOptimizer(
            GradientDescentOptions options,
            ILineSearch? lineSearch = null)
        {
            _options = options;
            _lineSearch = lineSearch;
        }

        /// <inheritdoc/>
        public OptimizerResult Minimize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] initialPoint)
        {
            var x = (double[])initialPoint.Clone();
            var n = x.Length;
            var functionEvaluations = 0;

            // Create convergence monitor
            // Use maxFunctionEvaluations if set, otherwise use maxIterations * 20 (generous for line search)
            var effectiveMaxFunctionEvals = _options.MaxFunctionEvaluations > 0
                ? _options.MaxFunctionEvaluations
                : _options.MaxIterations * 20;
            var monitor = new ConvergenceMonitor(
                gradientTolerance: _options.Tolerance,
                functionTolerance: _options.FunctionTolerance,
                parameterTolerance: _options.ParameterTolerance,
                maxIterations: _options.MaxIterations,
                maxFunctionEvaluations: effectiveMaxFunctionEvals,
                stallIterations: 10);

            if (_options.Verbose)
            {
                var lineSearchMode = _lineSearch != null ? "with line search" : $"fixed step α={_options.StepSize}";
                Console.WriteLine($"Gradient Descent Optimizer: {lineSearchMode}");
            }

            for (var iter = 0; iter < _options.MaxIterations; iter++)
            {
                var (value, gradient) = objective(x);
                functionEvaluations++;

                // Check for numerical errors
                if (double.IsNaN(value) || double.IsInfinity(value))
                {
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = value,
                        FinalGradient = gradient,
                        Iterations = iter + 1,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = StoppingReason.NumericalError,
                        Success = false,
                        Message = "Numerical error: objective function returned NaN or Infinity",
                        GradientNorm = double.NaN,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }

                for (var i = 0; i < n; i++)
                {
                    if (double.IsNaN(gradient[i]) || double.IsInfinity(gradient[i]))
                    {
                        return new OptimizerResult
                        {
                            OptimalPoint = x,
                            OptimalValue = value,
                            FinalGradient = gradient,
                            Iterations = iter + 1,
                            FunctionEvaluations = functionEvaluations,
                            StoppingReason = StoppingReason.NumericalError,
                            Success = false,
                            Message = "Numerical error: gradient contains NaN or Infinity",
                            GradientNorm = double.NaN,
                            FunctionChange = 0.0,
                            ParameterChange = 0.0
                        };
                    }
                }

                // Check convergence
                var convergence = monitor.CheckConvergence(iter, functionEvaluations, x, value, gradient);

                if (_options.Verbose && (iter % 10 == 0 || iter < 5))
                {
                    Console.WriteLine($"  Iter {iter + 1:D4}: f={value:E6}, ||grad||={convergence.GradientNorm:E3}");
                }

                if (convergence.HasConverged || convergence.Reason.HasValue)
                {
                    if (_options.Verbose)
                    {
                        Console.WriteLine($"Gradient Descent Converged: {convergence.Message}");
                    }
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = value,
                        FinalGradient = gradient,
                        Iterations = iter + 1,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = convergence.Reason ?? StoppingReason.MaxIterations,
                        Success = convergence.HasConverged,
                        Message = convergence.Message,
                        GradientNorm = convergence.GradientNorm,
                        FunctionChange = convergence.FunctionChange,
                        ParameterChange = convergence.ParameterChange
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
                        alpha = _options.StepSize;
                    }

                    // Count additional function evaluations from line search
                    // Note: BacktrackingLineSearch typically evaluates 1-10 times
                    functionEvaluations += 5; // Approximate average
                }
                else
                {
                    // Use fixed step size
                    alpha = _options.StepSize;
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

            // Check for numerical errors
            if (double.IsNaN(finalEval.value) || double.IsInfinity(finalEval.value))
            {
                return new OptimizerResult
                {
                    OptimalPoint = x,
                    OptimalValue = finalEval.value,
                    FinalGradient = finalEval.gradient,
                    Iterations = _options.MaxIterations,
                    FunctionEvaluations = functionEvaluations,
                    StoppingReason = StoppingReason.NumericalError,
                    Success = false,
                    Message = "Numerical error: objective function returned NaN or Infinity",
                    GradientNorm = double.NaN,
                    FunctionChange = 0.0,
                    ParameterChange = 0.0
                };
            }

            for (var i = 0; i < n; i++)
            {
                if (double.IsNaN(finalEval.gradient[i]) || double.IsInfinity(finalEval.gradient[i]))
                {
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = finalEval.value,
                        FinalGradient = finalEval.gradient,
                        Iterations = _options.MaxIterations,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = StoppingReason.NumericalError,
                        Success = false,
                        Message = "Numerical error: gradient contains NaN or Infinity",
                        GradientNorm = double.NaN,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }
            }

            var finalConvergence = monitor.CheckConvergence(_options.MaxIterations, functionEvaluations, x, finalEval.value, finalEval.gradient);

            return new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = finalEval.value,
                FinalGradient = finalEval.gradient,
                Iterations = _options.MaxIterations,
                FunctionEvaluations = functionEvaluations,
                StoppingReason = StoppingReason.MaxIterations,
                Success = false,
                Message = "Maximum iterations reached without convergence",
                GradientNorm = finalConvergence.GradientNorm,
                FunctionChange = finalConvergence.FunctionChange,
                ParameterChange = finalConvergence.ParameterChange
            };
        }
    }
}
