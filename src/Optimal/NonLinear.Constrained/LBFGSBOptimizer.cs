/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Constrained
{
    /// <summary>
    /// Limited-memory BFGS with Box constraints (L-BFGS-B) optimizer.
    /// Handles bound constraints directly in the optimization loop using
    /// projected gradient and subspace minimization.
    /// </summary>
    public sealed class LBFGSBOptimizer : IOptimizer
    {
        private readonly LBFGSBOptions _options;

        /// <summary>
        /// Creates a new L-BFGS-B optimizer with the specified options.
        /// </summary>
        /// <param name="options">Optimizer options including bounds.</param>
        public LBFGSBOptimizer(LBFGSBOptions options)
        {
            _options = options;
        }

        /// <inheritdoc/>
        public OptimizerResult Minimize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] initialPoint)
        {
            var n = initialPoint.Length;

            // Get bounds from options, defaulting to infinite bounds if not set
            double[] lower;
            double[] upper;
            if (_options.LowerBounds.Length == 0)
            {
                // No bounds set - use infinite bounds
                lower = new double[n];
                upper = new double[n];
                for (var i = 0; i < n; i++)
                {
                    lower[i] = double.NegativeInfinity;
                    upper[i] = double.PositiveInfinity;
                }
            }
            else
            {
                if (_options.LowerBounds.Length != n)
                {
                    throw new InvalidOperationException($"Bounds dimension ({_options.LowerBounds.Length}) does not match initial point dimension ({n})");
                }
                lower = _options.LowerBounds;
                upper = _options.UpperBounds;
            }

            // Project initial point onto feasible region
            var x = ProjectedGradient.Project(initialPoint, lower, upper);
            var functionEvaluations = 0;

            // Initialize L-BFGS memory
            var memory = new LBFGSMemory(_options.MemorySize, n);

            // Create convergence monitor
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
                var projGradNorm = ProjectedGradient.ProjectedGradientNormInf(x, gradient, lower, upper);
                Console.WriteLine($"L-BFGS-B Optimizer: Initial f={value:E6}, ||pg||_inf={projGradNorm:E3}");
            }

            // Storage for previous position and gradient
            var xPrev = new double[n];
            var gradientPrev = new double[n];

            for (var iter = 0; iter < _options.MaxIterations; iter++)
            {
                // Check convergence using projected gradient norm (L-BFGS-B specific)
                var projGradNorm = ProjectedGradient.ProjectedGradientNormInf(x, gradient, lower, upper);

                if (projGradNorm < _options.Tolerance)
                {
                    if (_options.Verbose)
                    {
                        Console.WriteLine($"L-BFGS-B Converged: Projected gradient norm {projGradNorm:E6} < {_options.Tolerance:E6}");
                    }
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = value,
                        FinalGradient = gradient,
                        Iterations = iter + 1,
                        FunctionEvaluations = functionEvaluations,
                        StoppingReason = StoppingReason.GradientTolerance,
                        Success = true,
                        Message = $"Converged: projected gradient norm {projGradNorm:E6} < {_options.Tolerance:E6}",
                        GradientNorm = projGradNorm,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }

                // Check other convergence criteria
                var convergence = monitor.CheckConvergence(iter, functionEvaluations, x, value, gradient);
                if (convergence.HasConverged || convergence.Reason.HasValue)
                {
                    if (_options.Verbose)
                    {
                        Console.WriteLine($"L-BFGS-B Converged: {convergence.Message}");
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
                        GradientNorm = projGradNorm,
                        FunctionChange = convergence.FunctionChange,
                        ParameterChange = convergence.ParameterChange
                    };
                }

                // Save current state
                Array.Copy(x, xPrev, n);
                Array.Copy(gradient, gradientPrev, n);

                // Step 1: Compute the generalized Cauchy point
                var cauchyResult = CauchyPointCalculator.Compute(x, gradient, lower, upper, memory);

                // Step 2: Subspace minimization - compute search direction for free variables
                var direction = SubspaceMinimization.ComputeSubspaceDirection(
                    cauchyResult.CauchyPoint,
                    gradient,
                    cauchyResult.FreeVariables,
                    lower,
                    upper,
                    memory);

                // Compute the full direction from x to (cauchy point + subspace direction)
                var fullDirection = new double[n];
                for (var i = 0; i < n; i++)
                {
                    fullDirection[i] = cauchyResult.CauchyPoint[i] - x[i] + direction[i];
                }

                // Check if direction is valid descent direction
                var gradDotDir = DotProduct(gradient, fullDirection);
                if (gradDotDir >= 0 || AllZero(fullDirection))
                {
                    // Not a descent direction - try projected gradient
                    fullDirection = SubspaceMinimization.ComputeProjectedGradientDirection(
                        gradient, cauchyResult.FreeVariables, x, lower, upper);

                    gradDotDir = DotProduct(gradient, fullDirection);
                    if (gradDotDir >= 0 || AllZero(fullDirection))
                    {
                        // Still not descent - we may be at optimum
                        if (_options.Verbose)
                        {
                            Console.WriteLine("L-BFGS-B: No descent direction found");
                        }
                        return new OptimizerResult
                        {
                            OptimalPoint = x,
                            OptimalValue = value,
                            FinalGradient = gradient,
                            Iterations = iter + 1,
                            FunctionEvaluations = functionEvaluations,
                            StoppingReason = StoppingReason.GradientTolerance,
                            Success = true,
                            Message = "No descent direction found - may be at constrained optimum",
                            GradientNorm = projGradNorm,
                            FunctionChange = 0.0,
                            ParameterChange = 0.0
                        };
                    }

                    // Clear memory when falling back to projected gradient
                    memory.Clear();
                }

                // Step 3: Bounded line search
                var maxStep = ProjectedGradient.MaxFeasibleStep(x, fullDirection, lower, upper);
                var initialStep = Math.Min(1.0, maxStep * 0.99);

                var alpha = BoundedLineSearch(objective, x, value, gradient, fullDirection, initialStep, maxStep, ref functionEvaluations);

                if (alpha <= 0)
                {
                    // Line search failed - try smaller step or fall back
                    alpha = Math.Min(0.01, maxStep * 0.5);
                    if (alpha <= 1e-16)
                    {
                        if (_options.Verbose)
                        {
                            Console.WriteLine("L-BFGS-B: Line search failed");
                        }
                        return new OptimizerResult
                        {
                            OptimalPoint = x,
                            OptimalValue = value,
                            FinalGradient = gradient,
                            Iterations = iter + 1,
                            FunctionEvaluations = functionEvaluations,
                            StoppingReason = StoppingReason.NumericalError,
                            Success = false,
                            Message = "Line search failed to find descent",
                            GradientNorm = projGradNorm,
                            FunctionChange = 0.0,
                            ParameterChange = 0.0
                        };
                    }
                }

                // Update position
                for (var i = 0; i < n; i++)
                {
                    x[i] += alpha * fullDirection[i];
                }

                // Project to ensure strict feasibility (numerical safety)
                ProjectedGradient.ProjectInPlace(x, lower, upper);

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
                    projGradNorm = ProjectedGradient.ProjectedGradientNormInf(x, gradient, lower, upper);
                    Console.WriteLine($"  Iter {iter + 1:D4}: f={value:E6}, ||pg||={projGradNorm:E3}, α={alpha:F6}");
                }

                // Compute correction pair for L-BFGS memory
                var s = new double[n];
                var y = new double[n];
                for (var i = 0; i < n; i++)
                {
                    s[i] = x[i] - xPrev[i];
                    y[i] = gradient[i] - gradientPrev[i];
                }

                // Add to memory (checks curvature condition internally)
                memory.Push(s, y);
            }

            // Maximum iterations reached
            var finalProjGradNorm = ProjectedGradient.ProjectedGradientNormInf(x, gradient, lower, upper);
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
                GradientNorm = finalProjGradNorm,
                FunctionChange = finalConvergence.FunctionChange,
                ParameterChange = finalConvergence.ParameterChange
            };
        }

        /// <summary>
        /// Performs backtracking line search with bounds enforcement.
        /// </summary>
        private static double BoundedLineSearch(
            Func<double[], (double value, double[] gradient)> objective,
            double[] x,
            double fx,
            double[] gradient,
            double[] direction,
            double initialStep,
            double maxStep,
            ref int functionEvaluations)
        {
            const double c1 = 1e-4; // Armijo parameter
            const double rho = 0.5; // Backtracking factor
            const int maxIter = 50;

            var gradDotDir = DotProduct(gradient, direction);
            if (gradDotDir >= 0)
            {
                return 0; // Not a descent direction
            }

            var alpha = Math.Min(initialStep, maxStep);
            var n = x.Length;
            var xTrial = new double[n];

            for (var i = 0; i < maxIter; i++)
            {
                // Compute trial point
                for (var j = 0; j < n; j++)
                {
                    xTrial[j] = x[j] + alpha * direction[j];
                }

                // Evaluate at trial point
                var (fTrial, _) = objective(xTrial);
                functionEvaluations++;

                // Check Armijo condition: f(x + alpha*d) <= f(x) + c1 * alpha * grad' * d
                if (fTrial <= fx + c1 * alpha * gradDotDir)
                {
                    return alpha;
                }

                // Backtrack
                alpha *= rho;

                if (alpha < 1e-16)
                {
                    return 0; // Step too small
                }
            }

            return 0; // Failed to find acceptable step
        }

        private static double DotProduct(double[] a, double[] b)
        {
            var sum = 0.0;
            for (var i = 0; i < a.Length; i++)
            {
                sum += a[i] * b[i];
            }
            return sum;
        }

        private static bool AllZero(double[] d)
        {
            for (var i = 0; i < d.Length; i++)
            {
                if (Math.Abs(d[i]) > 1e-16)
                {
                    return false;
                }
            }
            return true;
        }
    }
}
