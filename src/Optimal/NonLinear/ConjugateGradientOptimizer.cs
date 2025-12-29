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
    /// Nonlinear conjugate gradient optimizer with line search.
    /// More efficient than steepest descent for high-dimensional problems.
    /// </summary>
    public sealed class ConjugateGradientOptimizer : IOptimizer
    {
        private double[] _x0 = Array.Empty<double>();
        private double _tolerance = 1e-6;
        private double _functionTolerance = 1e-8;
        private double _parameterTolerance = 1e-8;
        private int _maxIterations = 1000;
        private int _maxFunctionEvaluations;
        private bool _verbose;
        private ILineSearch _lineSearch = new BacktrackingLineSearch();
        private ConjugateGradientFormula _formula = ConjugateGradientFormula.FletcherReeves;
        private int _restartInterval; // 0 = no periodic restart, n = restart every n iterations

        /// <summary>
        /// Sets the conjugate gradient formula to use for beta calculation.
        /// </summary>
        /// <param name="formula">The formula (Fletcher-Reeves, Polak-Ribiere, or Hestenes-Stiefel).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public ConjugateGradientOptimizer WithFormula(ConjugateGradientFormula formula)
        {
            _formula = formula;
            return this;
        }

        /// <summary>
        /// Sets the line search algorithm to use for adaptive step sizing.
        /// </summary>
        /// <param name="lineSearch">The line search algorithm.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public ConjugateGradientOptimizer WithLineSearch(ILineSearch lineSearch)
        {
            _lineSearch = lineSearch;
            return this;
        }

        /// <summary>
        /// Sets the restart interval. If > 0, resets to steepest descent every n iterations.
        /// </summary>
        /// <param name="interval">Restart interval (0 = no periodic restart).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public ConjugateGradientOptimizer WithRestartInterval(int interval)
        {
            _restartInterval = interval;
            return this;
        }

        /// <summary>
        /// Sets the function value change tolerance for convergence.
        /// </summary>
        /// <param name="tolerance">The function change tolerance.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public ConjugateGradientOptimizer WithFunctionTolerance(double tolerance)
        {
            _functionTolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the parameter change tolerance for convergence.
        /// </summary>
        /// <param name="tolerance">The parameter change tolerance.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public ConjugateGradientOptimizer WithParameterTolerance(double tolerance)
        {
            _parameterTolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the maximum number of function evaluations.
        /// </summary>
        /// <param name="maxEvaluations">The maximum number of function evaluations.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public ConjugateGradientOptimizer WithMaxFunctionEvaluations(int maxEvaluations)
        {
            _maxFunctionEvaluations = maxEvaluations;
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

            // Create convergence monitor
            // Use maxFunctionEvaluations if set, otherwise use maxIterations * 20 (generous for line search)
            var effectiveMaxFunctionEvals = _maxFunctionEvaluations > 0 ? _maxFunctionEvaluations : _maxIterations * 20;
            var monitor = new ConvergenceMonitor(
                gradientTolerance: _tolerance,
                functionTolerance: _functionTolerance,
                parameterTolerance: _parameterTolerance,
                maxIterations: _maxIterations,
                maxFunctionEvaluations: effectiveMaxFunctionEvals,
                stallIterations: 10);

            // Evaluate at initial point
            var (value, gradient) = objective(x);
            functionEvaluations++;

            if (_verbose)
            {
                var gradNormSq = 0.0;
                for (var i = 0; i < n; i++)
                {
                    gradNormSq += gradient[i] * gradient[i];
                }
                var gradNorm = Math.Sqrt(gradNormSq);
                Console.WriteLine($"Conjugate Gradient Optimizer: formula={_formula}, Initial f={value:E6}, ||grad||={gradNorm:E3}");
            }

            // Initialize search direction to negative gradient (steepest descent)
            var direction = new double[n];
            for (var i = 0; i < n; i++)
            {
                direction[i] = -gradient[i];
            }

            var previousGradient = new double[n];

            for (var iter = 0; iter < _maxIterations; iter++)
            {
                // Check convergence
                var convergence = monitor.CheckConvergence(iter, functionEvaluations, x, value, gradient);

                if (_verbose && (iter % 10 == 0 || iter < 5))
                {
                    Console.WriteLine($"  Iter {iter + 1:D4}: f={value:E6}, ||grad||={convergence.GradientNorm:E3}");
                }

                if (convergence.HasConverged || convergence.Reason.HasValue)
                {
                    if (_verbose)
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

                // Check if we should restart (periodic or at beginning)
                var shouldRestart = (_restartInterval > 0 && (iter + 1) % _restartInterval == 0);

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
                    if (_formula == ConjugateGradientFormula.PolakRibiere && beta < 0)
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
            var finalConvergence = monitor.CheckConvergence(_maxIterations, functionEvaluations, x, value, gradient);
            return new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = value,
                FinalGradient = gradient,
                Iterations = _maxIterations,
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

            switch (_formula)
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
