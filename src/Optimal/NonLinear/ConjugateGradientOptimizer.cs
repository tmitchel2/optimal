/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Threading;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Nonlinear conjugate gradient optimizer with line search.
    /// More efficient than steepest descent for high-dimensional problems.
    /// </summary>
    public sealed class ConjugateGradientOptimizer : IOptimizer
    {
        private readonly ConjugateGradientOptions _options;
        private readonly ILineSearch _lineSearch;

        /// <summary>
        /// Creates a new Conjugate Gradient optimizer with the specified options and line search.
        /// </summary>
        /// <param name="options">Optimizer options.</param>
        /// <param name="lineSearch">Line search algorithm.</param>
        public ConjugateGradientOptimizer(
            ConjugateGradientOptions options,
            ILineSearch lineSearch)
        {
            _options = options;
            _lineSearch = lineSearch;
        }

        /// <inheritdoc/>
        public OptimizerResult Minimize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] initialPoint,
            CancellationToken cancellationToken)
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

            // Evaluate at initial point
            var (value, gradient) = objective(x);
            functionEvaluations++;

            // Check for numerical errors at initial point
            if (double.IsNaN(value) || double.IsInfinity(value))
            {
                return new OptimizerResult
                {
                    OptimalPoint = x,
                    OptimalValue = value,
                    FinalGradient = gradient,
                    Iterations = 0,
                    FunctionEvaluations = functionEvaluations,
                    StoppingReason = StoppingReason.NumericalError,
                    Success = false,
                    Message = "Numerical error: initial objective evaluation returned NaN or Infinity",
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
                        Iterations = 0,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = StoppingReason.NumericalError,
                        Success = false,
                        Message = "Numerical error: initial gradient contains NaN or Infinity",
                        GradientNorm = double.NaN,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }
            }

            if (_options.Verbose)
            {
                var gradNormSq = 0.0;
                for (var i = 0; i < n; i++)
                {
                    gradNormSq += gradient[i] * gradient[i];
                }
                var gradNorm = Math.Sqrt(gradNormSq);
                Console.WriteLine($"Conjugate Gradient Optimizer: formula={_options.Formula}, Initial f={value:E6}, ||grad||={gradNorm:E3}");
            }

            // Initialize search direction to negative gradient (steepest descent)
            var direction = new double[n];
            for (var i = 0; i < n; i++)
            {
                direction[i] = -gradient[i];
            }

            var previousGradient = new double[n];

            for (var iter = 0; iter < _options.MaxIterations; iter++)
            {
                // Check for cancellation
                if (cancellationToken.IsCancellationRequested)
                {
                    var cancelConvergence = monitor.CheckConvergence(iter, functionEvaluations, x, value, gradient);
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = value,
                        FinalGradient = gradient,
                        Iterations = iter,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = StoppingReason.UserRequested,
                        Success = false,
                        Message = "Optimization cancelled by user request",
                        GradientNorm = cancelConvergence.GradientNorm,
                        FunctionChange = cancelConvergence.FunctionChange,
                        ParameterChange = cancelConvergence.ParameterChange
                    };
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
                        Console.WriteLine($"Conjugate Gradient Converged: {convergence.Message}");
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

                // Find step size using line search
                var alpha = _lineSearch.FindStepSize(objective, x, value, gradient, direction, 1.0);

                // If line search failed, try steepest descent direction
                if (alpha == 0.0)
                {
                    for (var i = 0; i < n; i++)
                    {
                        direction[i] = -gradient[i];
                    }
                    alpha = _lineSearch.FindStepSize(objective, x, value, gradient, direction, 1.0);

                    if (alpha == 0.0)
                    {
                        // Both CG and steepest descent failed
                        var errorConvergence = monitor.CheckConvergence(iter + 1, functionEvaluations, x, value, gradient);
                        return new OptimizerResult
                        {
                            OptimalPoint = x,
                            OptimalValue = value,
                            FinalGradient = gradient,
                            Iterations = iter + 1,
                            FunctionEvaluations = functionEvaluations,
                            StoppingReason = StoppingReason.NumericalError,
                            Success = false,
                            Message = "Line search failed to find descent direction",
                            GradientNorm = errorConvergence.GradientNorm,
                            FunctionChange = errorConvergence.FunctionChange,
                            ParameterChange = errorConvergence.ParameterChange
                        };
                    }
                }

                functionEvaluations += 5; // Approximate line search evaluations

                // Save current gradient for beta calculation
                Array.Copy(gradient, previousGradient, n);

                // Update position: x = x + alpha * direction
                for (var i = 0; i < n; i++)
                {
                    x[i] += alpha * direction[i];
                }

                // Evaluate at new point
                (value, gradient) = objective(x);
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

                // Check if we should restart (periodic or at beginning)
                var shouldRestart = (_options.RestartInterval > 0 && (iter + 1) % _options.RestartInterval == 0);

                if (shouldRestart)
                {
                    // Restart with steepest descent
                    for (var i = 0; i < n; i++)
                    {
                        direction[i] = -gradient[i];
                    }
                }
                else
                {
                    // Compute beta using the selected formula
                    var beta = ComputeBeta(gradient, previousGradient, direction);

                    // Polak-Ribiere variant: if beta < 0, restart with steepest descent
                    if (_options.Formula == ConjugateGradientFormula.PolakRibiere && beta < 0)
                    {
                        beta = 0.0;
                    }

                    // Update search direction: d = -grad + beta * d_old
                    for (var i = 0; i < n; i++)
                    {
                        direction[i] = -gradient[i] + beta * direction[i];
                    }
                }
            }

            // Maximum iterations reached
            var finalConvergence = monitor.CheckConvergence(_options.MaxIterations, functionEvaluations, x, value, gradient);
            return new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = value,
                FinalGradient = gradient,
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

        private double ComputeBeta(double[] gradientNew, double[] gradientOld, double[] directionOld)
        {
            var n = gradientNew.Length;

            switch (_options.Formula)
            {
                case ConjugateGradientFormula.FletcherReeves:
                    {
                        // beta = ||grad_new||^2 / ||grad_old||^2
                        var numerator = 0.0;
                        var denominator = 0.0;
                        for (var i = 0; i < n; i++)
                        {
                            numerator += gradientNew[i] * gradientNew[i];
                            denominator += gradientOld[i] * gradientOld[i];
                        }
                        return denominator > 1e-16 ? numerator / denominator : 0.0;
                    }

                case ConjugateGradientFormula.PolakRibiere:
                    {
                        // beta = grad_new^T * (grad_new - grad_old) / ||grad_old||^2
                        var numerator = 0.0;
                        var denominator = 0.0;
                        for (var i = 0; i < n; i++)
                        {
                            numerator += gradientNew[i] * (gradientNew[i] - gradientOld[i]);
                            denominator += gradientOld[i] * gradientOld[i];
                        }
                        return denominator > 1e-16 ? numerator / denominator : 0.0;
                    }

                case ConjugateGradientFormula.HestenesStiefel:
                    {
                        // beta = grad_new^T * (grad_new - grad_old) / (d_old^T * (grad_new - grad_old))
                        var numerator = 0.0;
                        var denominator = 0.0;
                        for (var i = 0; i < n; i++)
                        {
                            var gradDiff = gradientNew[i] - gradientOld[i];
                            numerator += gradientNew[i] * gradDiff;
                            denominator += directionOld[i] * gradDiff;
                        }
                        return denominator > 1e-16 ? numerator / denominator : 0.0;
                    }

                default:
                    return 0.0; // Fallback to steepest descent
            }
        }
    }
}
