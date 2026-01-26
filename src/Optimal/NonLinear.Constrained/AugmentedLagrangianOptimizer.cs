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
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Constrained
{
    /// <summary>
    /// Augmented Lagrangian optimizer for constrained optimization problems.
    /// Solves: minimize f(x) subject to h_j(x) = 0, g_k(x) ≤ 0
    /// </summary>
    public sealed class AugmentedLagrangianOptimizer : IOptimizer
    {
        private double[] _x0 = Array.Empty<double>();
        private double _tolerance = 1e-6;
        private int _maxIterations = 50;
        private bool _verbose;
        private IOptimizer _unconstrainedOptimizer = new LBFGSOptimizer();
        private readonly List<IConstraint> _constraints = new List<IConstraint>();
        private BoxConstraints? _boxConstraints;
        private double _penaltyParameter = 1.0;
        private readonly double _penaltyIncrease = 10.0;
        private readonly double _maxPenaltyParameter = 1e8;
        private double _constraintTolerance = 1e-6;

        /// <summary>
        /// Sets the unconstrained optimizer to use for subproblems.
        /// </summary>
        /// <param name="optimizer">Unconstrained optimizer (L-BFGS, CG, GD).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public AugmentedLagrangianOptimizer WithUnconstrainedOptimizer(IOptimizer optimizer)
        {
            _unconstrainedOptimizer = optimizer ?? throw new ArgumentNullException(nameof(optimizer));
            return this;
        }

        /// <summary>
        /// Adds an equality constraint h(x) = 0.
        /// </summary>
        /// <param name="constraint">Equality constraint function.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public AugmentedLagrangianOptimizer WithEqualityConstraint(Func<double[], (double value, double[] gradient)> constraint)
        {
            _constraints.Add(new EqualityConstraint(constraint));
            return this;
        }

        /// <summary>
        /// Adds an inequality constraint g(x) ≤ 0.
        /// </summary>
        /// <param name="constraint">Inequality constraint function.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public AugmentedLagrangianOptimizer WithInequalityConstraint(Func<double[], (double value, double[] gradient)> constraint)
        {
            _constraints.Add(new InequalityConstraint(constraint));
            return this;
        }

        /// <summary>
        /// Sets box constraints for all variables.
        /// </summary>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public AugmentedLagrangianOptimizer WithBoxConstraints(double[] lower, double[] upper)
        {
            _boxConstraints = new BoxConstraints(lower, upper);
            return this;
        }

        /// <summary>
        /// Sets the initial penalty parameter.
        /// </summary>
        /// <param name="penalty">Initial penalty parameter (default 1.0).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public AugmentedLagrangianOptimizer WithPenaltyParameter(double penalty)
        {
            _penaltyParameter = penalty;
            return this;
        }

        /// <summary>
        /// Sets the constraint tolerance.
        /// </summary>
        /// <param name="tolerance">Constraint violation tolerance.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        public AugmentedLagrangianOptimizer WithConstraintTolerance(double tolerance)
        {
            _constraintTolerance = tolerance;
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
            var n = _x0.Length;
            var x = (double[])_x0.Clone();

            // Check if the inner optimizer natively supports box constraints
            var innerSupportsBoxConstraints = _unconstrainedOptimizer is IBoxConstrainedOptimizer;

            // If inner optimizer supports bounds and we have box constraints, pass them through
            if (innerSupportsBoxConstraints && _boxConstraints != null)
            {
                var boxOptimizer = (IBoxConstrainedOptimizer)_unconstrainedOptimizer;
                boxOptimizer.WithBounds(_boxConstraints.Lower, _boxConstraints.Upper);
            }

            // Project onto box constraints if present (needed for initial point and when inner doesn't support bounds)
            if (_boxConstraints != null)
            {
                x = _boxConstraints.Project(x);
            }

            // Initialize Lagrange multipliers
            var lambda = new double[_constraints.Count];
            var mu = _penaltyParameter;
            var prevMaxViolation = double.PositiveInfinity;

            var totalFunctionEvaluations = 0;
            var totalIterations = 0;

            if (_verbose)
            {
                Console.WriteLine($"Augmented Lagrangian: {_constraints.Count} constraints, initial penalty μ={mu:E2}, max penalty μ_max={_maxPenaltyParameter:E2}");
            }

            for (var outerIter = 0; outerIter < _maxIterations; outerIter++)
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
                    for (var i = 0; i < _constraints.Count; i++)
                    {
                        var constraint = _constraints[i];
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
                _unconstrainedOptimizer.WithInitialPoint(x);
                // _unconstrainedOptimizer.WithTolerance(_tolerance / 10.0); // Tighter inner tolerance
                _unconstrainedOptimizer.WithTolerance(_tolerance); // Tighter inner tolerance
                // _unconstrainedOptimizer.WithMaxIterations(1000);

                var subResult = _unconstrainedOptimizer.Minimize(AugmentedLagrangian);

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
                if (_boxConstraints != null && !innerSupportsBoxConstraints)
                {
                    x = _boxConstraints.Project(x);
                }

                totalFunctionEvaluations += subResult.FunctionEvaluations;
                totalIterations += subResult.Iterations;

                // Evaluate constraints at current point
                var maxViolation = 0.0;
                var constraintValues = new double[_constraints.Count];

                for (var i = 0; i < _constraints.Count; i++)
                {
                    var (cValue, _) = _constraints[i].Evaluate(x);
                    constraintValues[i] = cValue;

                    if (_constraints[i].Type == ConstraintType.Equality)
                    {
                        maxViolation = Math.Max(maxViolation, Math.Abs(cValue));
                    }
                    else // Inequality
                    {
                        maxViolation = Math.Max(maxViolation, Math.Max(0, cValue));
                    }
                }

                // Check box constraint violation
                if (_boxConstraints != null)
                {
                    maxViolation = Math.Max(maxViolation, _boxConstraints.MaxViolation(x));
                }

                if (_verbose)
                {
                    var (fVal, _) = objective(x);
                    Console.WriteLine($"  Outer iter {outerIter + 1}: f={fVal:E6}, max_violation={maxViolation:E3}, μ={mu:E2}, inner_iters={subResult.Iterations}");
                }

                // Check convergence
                if (maxViolation < _constraintTolerance)
                {
                    if (_verbose)
                    {
                        Console.WriteLine($"Augmented Lagrangian Converged: max constraint violation {maxViolation:E3} < {_constraintTolerance:E3}");
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
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = finalValue,
                        FinalGradient = finalGrad,
                        Iterations = outerIter + 1,
                        FunctionEvaluations = totalFunctionEvaluations,
                        StoppingReason = StoppingReason.GradientTolerance,
                        Success = true,
                        Message = $"Converged: max constraint violation {maxViolation:E3} < {_constraintTolerance:E3}",
                        GradientNorm = gradNorm,
                        FunctionChange = 0.0,
                        ParameterChange = 0.0
                    };
                }

                // Update Lagrange multipliers
                for (var i = 0; i < _constraints.Count; i++)
                {
                    if (_constraints[i].Type == ConstraintType.Equality)
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
                    Iterations = _maxIterations,
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
                        Iterations = _maxIterations,
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
            return new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = endValue,
                FinalGradient = endGrad,
                Iterations = _maxIterations,
                FunctionEvaluations = totalFunctionEvaluations,
                StoppingReason = StoppingReason.MaxIterations,
                Success = false,
                Message = "Maximum outer iterations reached without satisfying constraints",
                GradientNorm = endGradNorm,
                FunctionChange = 0.0,
                ParameterChange = 0.0
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
