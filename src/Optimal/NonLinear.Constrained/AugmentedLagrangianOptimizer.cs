/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Optimal.NonLinear.Constraints;
using Optimal.NonLinear.Monitoring;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Constrained
{
    /// <summary>
    /// Augmented Lagrangian optimizer for constrained optimization problems.
    /// Solves: minimize f(x) subject to h_j(x) = 0, g_k(x) &lt;= 0
    /// </summary>
    public sealed class AugmentedLagrangianOptimizer : IOptimizer
    {
        private readonly AugmentedLagrangianOptions _options;
        private readonly IOptimizer _unconstrainedOptimizer;
        private readonly OptimisationMonitor? _monitor;
        private readonly double _penaltyIncrease = 10.0;
        private readonly double _maxPenaltyParameter = 1e8;

        /// <summary>
        /// Creates a new Augmented Lagrangian optimizer with the specified options and dependencies.
        /// </summary>
        /// <param name="options">Optimizer options including constraints.</param>
        /// <param name="unconstrainedOptimizer">The inner optimizer for subproblems.</param>
        /// <param name="monitor">Optional optimization monitor for gradient verification.</param>
        public AugmentedLagrangianOptimizer(
            AugmentedLagrangianOptions options,
            IOptimizer unconstrainedOptimizer,
            OptimisationMonitor? monitor = null)
        {
            _options = options;
            _unconstrainedOptimizer = unconstrainedOptimizer;
            _monitor = monitor;
        }

        /// <inheritdoc/>
        public OptimizerResult Minimize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] initialPoint)
        {
            var n = initialPoint.Length;
            var x = (double[])initialPoint.Clone();

            // Build constraints list from options
            var constraints = new List<IConstraint>();
            foreach (var eq in _options.EqualityConstraints)
            {
                constraints.Add(new EqualityConstraint(eq));
            }
            foreach (var ineq in _options.InequalityConstraints)
            {
                constraints.Add(new InequalityConstraint(ineq));
            }

            // Check if the inner optimizer natively supports box constraints
            var innerSupportsBoxConstraints = _unconstrainedOptimizer is LBFGSBOptimizer;

            // If inner optimizer supports bounds and we have box constraints, configure them
            if (innerSupportsBoxConstraints && _options.BoxConstraints != null)
            {
                // Note: With the new pattern, we can't reconfigure the optimizer after construction
                // The user should pass a properly configured LBFGSBOptimizer
            }

            // Project onto box constraints if present (needed for initial point and when inner doesn't support bounds)
            if (_options.BoxConstraints != null)
            {
                x = _options.BoxConstraints.Project(x);
            }

            // Initialize Lagrange multipliers
            var lambda = new double[constraints.Count];
            var mu = _options.PenaltyParameter;
            var prevMaxViolation = double.PositiveInfinity;

            var totalFunctionEvaluations = 0;
            var totalIterations = 0;

            if (_options.Verbose)
            {
                Console.WriteLine($"Augmented Lagrangian: {constraints.Count} constraints, initial penalty μ={mu:E2}, max penalty μ_max={_maxPenaltyParameter:E2}");
            }

            // Initialize optimization monitor if attached
            if (_monitor != null)
            {
                _monitor.WithObjective(objective);
                if (constraints.Count > 0)
                {
                    _monitor.WithConstraints(constraints);
                }
                _monitor.OnOptimizationStart(x);
            }

            for (var outerIter = 0; outerIter < _options.MaxIterations; outerIter++)
            {
                // Build augmented Lagrangian function
                (double value, double[] gradient) AugmentedLagrangian(double[] xAug)
                {
                    // Evaluate objective
                    var (fValue, fGrad) = objective(xAug);

                    // Check for numerical errors in objective
                    if (double.IsNaN(fValue) || double.IsInfinity(fValue))
                    {
                        return (double.PositiveInfinity, fGrad);
                    }

                    for (var j = 0; j < n; j++)
                    {
                        if (double.IsNaN(fGrad[j]) || double.IsInfinity(fGrad[j]))
                        {
                            return (double.PositiveInfinity, fGrad);
                        }
                    }

                    var augValue = fValue;
                    var augGrad = (double[])fGrad.Clone();

                    // Add constraint terms
                    for (var i = 0; i < constraints.Count; i++)
                    {
                        var constraint = constraints[i];
                        var (cValue, cGrad) = constraint.Evaluate(xAug);

                        if (constraint.Type == ConstraintType.Equality)
                        {
                            // Equality: L = f + λ*h + (μ/2)*h²
                            var penaltyTerm = lambda[i] * cValue + 0.5 * mu * cValue * cValue;

                            // Check for numerical overflow
                            if (double.IsInfinity(penaltyTerm) || double.IsNaN(penaltyTerm))
                            {
                                return (double.PositiveInfinity, augGrad);
                            }

                            augValue += penaltyTerm;

                            for (var j = 0; j < n; j++)
                            {
                                augGrad[j] += (lambda[i] + mu * cValue) * cGrad[j];
                            }
                        }
                        else // Inequality
                        {
                            // Inequality: L = f + max(0, λ + μ*g)²/(2μ)
                            var lambdaPlusMuG = lambda[i] + mu * cValue;
                            if (lambdaPlusMuG > 0)
                            {
                                var penaltyTerm = lambdaPlusMuG * lambdaPlusMuG / (2.0 * mu);

                                // Check for numerical overflow
                                if (double.IsInfinity(penaltyTerm) || double.IsNaN(penaltyTerm))
                                {
                                    return (double.PositiveInfinity, augGrad);
                                }

                                augValue += penaltyTerm;

                                for (var j = 0; j < n; j++)
                                {
                                    augGrad[j] += lambdaPlusMuG * cGrad[j];
                                }
                            }
                        }
                    }

                    return (augValue, augGrad);
                }

                // Solve unconstrained subproblem
                var subResult = _unconstrainedOptimizer.Minimize(AugmentedLagrangian, x);

                // Check if inner optimizer encountered numerical error
                if (subResult.StoppingReason == StoppingReason.NumericalError)
                {
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = double.NaN,
                        FinalGradient = Array.Empty<double>(),
                        Iterations = outerIter + 1,
                        FunctionEvaluations = totalFunctionEvaluations + subResult.FunctionEvaluations,
                        StoppingReason = StoppingReason.NumericalError,
                        Success = false,
                        Message = "Numerical error in inner optimizer: " + subResult.Message,
                        GradientNorm = double.NaN,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }

                x = subResult.OptimalPoint;

                // Project onto box constraints if present (skip if inner optimizer handles bounds)
                if (_options.BoxConstraints != null && !innerSupportsBoxConstraints)
                {
                    x = _options.BoxConstraints.Project(x);
                }

                totalFunctionEvaluations += subResult.FunctionEvaluations;
                totalIterations += subResult.Iterations;

                // Evaluate constraints at current point
                var maxViolation = 0.0;
                var constraintValues = new double[constraints.Count];

                for (var i = 0; i < constraints.Count; i++)
                {
                    var (cValue, _) = constraints[i].Evaluate(x);
                    constraintValues[i] = cValue;

                    if (constraints[i].Type == ConstraintType.Equality)
                    {
                        maxViolation = Math.Max(maxViolation, Math.Abs(cValue));
                    }
                    else // Inequality
                    {
                        maxViolation = Math.Max(maxViolation, Math.Max(0, cValue));
                    }
                }

                // Check box constraint violation
                if (_options.BoxConstraints != null)
                {
                    maxViolation = Math.Max(maxViolation, _options.BoxConstraints.MaxViolation(x));
                }

                if (_options.Verbose)
                {
                    var (fVal, _) = objective(x);
                    Console.WriteLine($"  Outer iter {outerIter + 1}: f={fVal:E6}, max_violation={maxViolation:E3}, μ={mu:E2}, inner_iters={subResult.Iterations}");
                }

                // Check convergence
                if (maxViolation < _options.ConstraintTolerance)
                {
                    if (_options.Verbose)
                    {
                        Console.WriteLine($"Augmented Lagrangian Converged: max constraint violation {maxViolation:E3} < {_options.ConstraintTolerance:E3}");
                    }
                    var (finalValue, finalGrad) = objective(x);

                    // Check for numerical errors
                    if (double.IsNaN(finalValue) || double.IsInfinity(finalValue))
                    {
                        return new OptimizerResult
                        {
                            OptimalPoint = x,
                            OptimalValue = finalValue,
                            FinalGradient = finalGrad,
                            Iterations = outerIter + 1,
                            FunctionEvaluations = totalFunctionEvaluations,
                            StoppingReason = StoppingReason.NumericalError,
                            Success = false,
                            Message = "Numerical error: objective function returned NaN or Infinity",
                            GradientNorm = double.NaN,
                            FunctionChange = 0.0,
                            ParameterChange = 0.0
                        };
                    }

                    for (var j = 0; j < n; j++)
                    {
                        if (double.IsNaN(finalGrad[j]) || double.IsInfinity(finalGrad[j]))
                        {
                            return new OptimizerResult
                            {
                                OptimalPoint = x,
                                OptimalValue = finalValue,
                                FinalGradient = finalGrad,
                                Iterations = outerIter + 1,
                                FunctionEvaluations = totalFunctionEvaluations,
                                StoppingReason = StoppingReason.NumericalError,
                                Success = false,
                                Message = "Numerical error: gradient contains NaN or Infinity",
                                GradientNorm = double.NaN,
                                FunctionChange = 0.0,
                                ParameterChange = 0.0
                            };
                        }
                    }

                    var gradNorm = ComputeNorm(finalGrad);
                    var successResult = new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = finalValue,
                        FinalGradient = finalGrad,
                        Iterations = outerIter + 1,
                        FunctionEvaluations = totalFunctionEvaluations,
                        StoppingReason = StoppingReason.GradientTolerance,
                        Success = true,
                        Message = $"Converged: max constraint violation {maxViolation:E3} < {_options.ConstraintTolerance:E3}",
                        GradientNorm = gradNorm,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                    _monitor?.OnOptimizationEnd(successResult);
                    return successResult;
                }

                // Update Lagrange multipliers
                for (var i = 0; i < constraints.Count; i++)
                {
                    if (constraints[i].Type == ConstraintType.Equality)
                    {
                        lambda[i] += mu * constraintValues[i];
                    }
                    else // Inequality
                    {
                        lambda[i] = Math.Max(0, lambda[i] + mu * constraintValues[i]);
                    }
                }

                // Increase penalty parameter adaptively
                // Only increase if we're making progress, and cap at max value
                var violationRatio = maxViolation / prevMaxViolation;
                if (violationRatio < 0.9 && mu < _maxPenaltyParameter)
                {
                    // Good progress - increase penalty moderately
                    mu = Math.Min(mu * _penaltyIncrease, _maxPenaltyParameter);
                }
                else if (violationRatio < 1.0 && mu < _maxPenaltyParameter)
                {
                    // Some progress - increase penalty conservatively
                    mu = Math.Min(mu * Math.Sqrt(_penaltyIncrease), _maxPenaltyParameter);
                }
                else if (mu < _maxPenaltyParameter)
                {
                    // No progress - still increase penalty slowly to avoid getting stuck
                    mu = Math.Min(mu * 1.5, _maxPenaltyParameter);
                }

                prevMaxViolation = maxViolation;
            }

            // Maximum iterations reached
            var (endValue, endGrad) = objective(x);

            // Check for numerical errors
            if (double.IsNaN(endValue) || double.IsInfinity(endValue))
            {
                return new OptimizerResult
                {
                    OptimalPoint = x,
                    OptimalValue = endValue,
                    FinalGradient = endGrad,
                    Iterations = _options.MaxIterations,
                    FunctionEvaluations = totalFunctionEvaluations,
                    StoppingReason = StoppingReason.NumericalError,
                    Success = false,
                    Message = "Numerical error: objective function returned NaN or Infinity",
                    GradientNorm = double.NaN,
                    FunctionChange = 0.0,
                    ParameterChange = 0.0
                };
            }

            for (var j = 0; j < n; j++)
            {
                if (double.IsNaN(endGrad[j]) || double.IsInfinity(endGrad[j]))
                {
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = endValue,
                        FinalGradient = endGrad,
                        Iterations = _options.MaxIterations,
                        FunctionEvaluations = totalFunctionEvaluations,
                        StoppingReason = StoppingReason.NumericalError,
                        Success = false,
                        Message = "Numerical error: gradient contains NaN or Infinity",
                        GradientNorm = double.NaN,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }
            }

            var endGradNorm = ComputeNorm(endGrad);
            var maxIterResult = new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = endValue,
                FinalGradient = endGrad,
                Iterations = _options.MaxIterations,
                FunctionEvaluations = totalFunctionEvaluations,
                StoppingReason = StoppingReason.MaxIterations,
                Success = false,
                Message = "Maximum outer iterations reached without satisfying constraints",
                GradientNorm = endGradNorm,
                FunctionChange = 0.0,
                ParameterChange = 0.0
            };
            _monitor?.OnOptimizationEnd(maxIterResult);
            return maxIterResult;
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
