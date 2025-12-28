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
    /// Limited-memory BFGS (L-BFGS) quasi-Newton optimizer.
    /// Highly efficient for large-scale optimization problems.
    /// </summary>
    public sealed class LBFGSOptimizer : IOptimizer
    {
        private double[] _x0 = Array.Empty<double>();
        private double _tolerance = 1e-6;
        private double _functionTolerance = 1e-8;
        private double _parameterTolerance = 1e-8;
        private int _maxIterations = 1000;
        private int _maxFunctionEvaluations = 10000;
        private bool _verbose;
        private ILineSearch _lineSearch = new BacktrackingLineSearch();
        private int _memorySize = 10; // Number of correction pairs to store (m parameter)

        /// <summary>
        /// Sets the line search algorithm to use for adaptive step sizing.
        /// </summary>
        /// <param name="lineSearch">The line search algorithm.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public LBFGSOptimizer WithLineSearch(ILineSearch lineSearch)
        {
            _lineSearch = lineSearch;
            return this;
        }

        /// <summary>
        /// Sets the memory size (number of correction pairs to store).
        /// </summary>
        /// <param name="memorySize">Memory size (typically 3-20, default 10).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public LBFGSOptimizer WithMemorySize(int memorySize)
        {
            if (memorySize <= 0)
            {
                throw new ArgumentException("Memory size must be positive", nameof(memorySize));
            }

            _memorySize = memorySize;
            return this;
        }

        /// <summary>
        /// Sets the function value change tolerance for convergence.
        /// </summary>
        /// <param name="tolerance">The function change tolerance.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public LBFGSOptimizer WithFunctionTolerance(double tolerance)
        {
            _functionTolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the parameter change tolerance for convergence.
        /// </summary>
        /// <param name="tolerance">The parameter change tolerance.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public LBFGSOptimizer WithParameterTolerance(double tolerance)
        {
            _parameterTolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Sets the maximum number of function evaluations.
        /// </summary>
        /// <param name="maxEvaluations">The maximum number of function evaluations.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public LBFGSOptimizer WithMaxFunctionEvaluations(int maxEvaluations)
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

            // Initialize L-BFGS memory
            var memory = new LBFGSMemory(_memorySize, n);

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

            // Storage for previous position and gradient
            var xPrev = new double[n];
            var gradientPrev = new double[n];

            for (var iter = 0; iter < _maxIterations; iter++)
            {
                // Check convergence
                var convergence = monitor.CheckConvergence(iter, functionEvaluations, x, value, gradient);

                if (convergence.HasConverged || convergence.Reason.HasValue)
                {
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

                // Save current state before updating
                Array.Copy(x, xPrev, n);
                Array.Copy(gradient, gradientPrev, n);

                // Compute search direction using L-BFGS two-loop recursion
                var direction = TwoLoopRecursion.ComputeDirection(gradient, memory);

                // Find step size using line search
                var alpha = _lineSearch.FindStepSize(objective, x, value, gradient, direction, 1.0);

                // If line search failed, try steepest descent
                if (alpha == 0.0)
                {
                    for (var i = 0; i < n; i++)
                    {
                        direction[i] = -gradient[i];
                    }
                    alpha = _lineSearch.FindStepSize(objective, x, value, gradient, direction, 1.0);

                    if (alpha == 0.0)
                    {
                        // Both L-BFGS and steepest descent failed
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

                    // Clear memory and restart when falling back to steepest descent
                    memory.Clear();
                }

                functionEvaluations += 5; // Approximate line search evaluations

                // Update position: x = x + alpha * direction
                for (var i = 0; i < n; i++)
                {
                    x[i] += alpha * direction[i];
                }

                // Evaluate at new point
                (value, gradient) = objective(x);
                functionEvaluations++;

                // Compute correction pair (s, y) and add to memory
                var s = new double[n];
                var y = new double[n];
                for (var i = 0; i < n; i++)
                {
                    s[i] = x[i] - xPrev[i]; // Position change
                    y[i] = gradient[i] - gradientPrev[i]; // Gradient change
                }

                // Add to memory (will check curvature condition internally)
                memory.Push(s, y);
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
    }
}
