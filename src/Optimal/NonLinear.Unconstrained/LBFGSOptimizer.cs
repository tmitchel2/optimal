/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Buffers;
using System.Threading;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Monitoring;

namespace Optimal.NonLinear.Unconstrained
{
    /// <summary>
    /// Limited-memory BFGS (L-BFGS) quasi-Newton optimizer.
    /// Highly efficient for large-scale optimization problems.
    /// </summary>
    public sealed class LBFGSOptimizer : IOptimizer
    {
        private readonly LBFGSOptions _options;
        private readonly ILineSearch _lineSearch;
        private readonly OptimisationMonitor? _monitor;

        /// <summary>
        /// Creates a new L-BFGS optimizer with the specified options and line search.
        /// </summary>
        /// <param name="options">Optimizer options.</param>
        /// <param name="lineSearch">Line search algorithm.</param>
        /// <param name="monitor">Optional optimization monitor for conditioning and gradient analysis.</param>
        public LBFGSOptimizer(
            LBFGSOptions options,
            ILineSearch lineSearch,
            OptimisationMonitor? monitor = null)
        {
            _options = options;
            _lineSearch = lineSearch;
            _monitor = monitor;
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

            // Initialize L-BFGS memory
            var memory = new LBFGSMemory(_options.MemorySize, n);

            // Initialize preconditioner if configured
            IPreconditioner? preconditioner = CreatePreconditioner(n);

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
                var gradNorm = VectorOps.Norm2(gradient);
                Console.WriteLine($"L-BFGS Optimizer: Initial f={value:E6}, ||grad||={gradNorm:E3}");
            }

            // Storage for previous position and gradient
            var xPrev = new double[n];
            var gradientPrev = new double[n];

            // Pool arrays for correction pairs
            var pool = ArrayPool<double>.Shared;
            var s = pool.Rent(n);
            var y = pool.Rent(n);

            try
            {
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

                    if (convergence.HasConverged || convergence.Reason.HasValue)
                    {
                        if (_options.Verbose)
                        {
                            Console.WriteLine($"L-BFGS Converged: {convergence.Message}");
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

                    // Save current state before updating using Span copy
                    x.AsSpan().CopyTo(xPrev);
                    gradient.AsSpan().CopyTo(gradientPrev);

                    // Compute search direction using L-BFGS two-loop recursion
                    var direction = TwoLoopRecursion.ComputeDirection(gradient, memory, preconditioner);

                    // Find step size using line search
                    var alpha = _lineSearch.FindStepSize(objective, x, value, gradient, direction, 1.0);

                    // If line search failed, try steepest descent
                    if (alpha == 0.0)
                    {
                        VectorOps.Negate(gradient, direction);
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

                        // Clear memory when falling back to steepest descent
                        memory.Clear();
                    }

                    functionEvaluations += 5; // Approximate line search evaluations

                    // Update position: x = x + alpha * direction using SIMD
                    VectorOps.AddScaled(x, alpha, direction, x);

                    // Evaluate at new point
                    (value, gradient) = objective(x);
                    functionEvaluations++;

                    // Check for numerical errors
                    if (double.IsNaN(value) || double.IsInfinity(value))
                    {
                        return new OptimizerResult
                        {
                            OptimalPoint = xPrev,
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
                                OptimalPoint = xPrev,
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

                    if (_options.Verbose && (iter % 10 == 0 || iter < 5))
                    {
                        var gradNorm = VectorOps.Norm2(gradient);
                        Console.WriteLine($"  Iter {iter + 1:D4}: f={value:E6}, ||grad||={gradNorm:E3}, α={alpha:F6}");
                    }

                    // Compute correction pair (s, y) using SIMD
                    var sSpan = s.AsSpan(0, n);
                    var ySpan = y.AsSpan(0, n);
                    VectorOps.Subtract(x, xPrev, sSpan);       // s = x - xPrev
                    VectorOps.Subtract(gradient, gradientPrev, ySpan); // y = gradient - gradientPrev

                    // Compute rho = 1 / (y^T * s) for conditioning monitoring
                    var yTs = VectorOps.Dot(ySpan, sSpan);

                    // Add to memory (will check curvature condition internally)
                    memory.Push(sSpan.ToArray(), ySpan.ToArray());

                    // Notify monitor if curvature condition is satisfied
                    if (Math.Abs(yTs) >= 1e-16)
                    {
                        _monitor?.OnCorrectionPairAdded(sSpan.ToArray(), ySpan.ToArray(), 1.0 / yTs);

                        // Update preconditioner with new curvature information
                        preconditioner?.Update(sSpan.ToArray(), ySpan.ToArray());
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
            finally
            {
                pool.Return(s);
                pool.Return(y);
            }
        }

        /// <summary>
        /// Creates a preconditioner based on the options.
        /// </summary>
        private IPreconditioner? CreatePreconditioner(int dimension)
        {
            var options = _options.Preconditioning;
            if (options == null || options.Type == PreconditioningType.None)
            {
                return null;
            }

            return options.Type switch
            {
                PreconditioningType.Diagonal => new DiagonalPreconditioner(
                    dimension,
                    options.DiagonalUpdateStrategy,
                    options.DecayFactor,
                    options.MinScaling,
                    options.MaxScaling),

                PreconditioningType.Regularization => new RegularizationPreconditioner(
                    dimension,
                    options.RegularizationParameter),

                PreconditioningType.Combined => new CombinedPreconditioner(
                    new DiagonalPreconditioner(
                        dimension,
                        options.DiagonalUpdateStrategy,
                        options.DecayFactor,
                        options.MinScaling,
                        options.MaxScaling),
                    options.RegularizationParameter),

                _ => null
            };
        }
    }
}
