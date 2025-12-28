/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear
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
        private double _penaltyIncrease = 10.0;
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

            // Project onto box constraints if present
            if (_boxConstraints != null)
            {
                x = _boxConstraints.Project(x);
            }

            // Initialize Lagrange multipliers
            var lambda = new double[_constraints.Count];
            var mu = _penaltyParameter;

            var totalFunctionEvaluations = 0;
            var totalIterations = 0;

            for (var outerIter = 0; outerIter < _maxIterations; outerIter++)
            {
                // Build augmented Lagrangian function
                (double value, double[] gradient) AugmentedLagrangian(double[] xAug)
                {
                    // Evaluate objective
                    var (fValue, fGrad) = objective(xAug);
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
                            augValue += lambda[i] * cValue + 0.5 * mu * cValue * cValue;

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
                                augValue += lambdaPlusMuG * lambdaPlusMuG / (2.0 * mu);

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
                _unconstrainedOptimizer.WithTolerance(_tolerance / 10.0); // Tighter inner tolerance
                _unconstrainedOptimizer.WithMaxIterations(1000);

                var subResult = _unconstrainedOptimizer.Minimize(AugmentedLagrangian);
                x = subResult.OptimalPoint;

                // Project onto box constraints if present
                if (_boxConstraints != null)
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

                // Check convergence
                if (maxViolation < _constraintTolerance)
                {
                    var (finalValue, finalGrad) = objective(x);
                    return new OptimizerResult
                    {
                        OptimalPoint = x,
                        OptimalValue = finalValue,
                        FinalGradient = finalGrad,
                        Iterations = outerIter + 1,
                        FunctionEvaluations = totalFunctionEvaluations,
                        StoppingReason = StoppingReason.GradientTolerance,
                        Success = true,
                        Message = $"Converged: max constraint violation {maxViolation:E3} < {_constraintTolerance:E3}"
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

                // Increase penalty parameter
                mu *= _penaltyIncrease;
            }

            // Maximum iterations reached
            var (endValue, endGrad) = objective(x);
            return new OptimizerResult
            {
                OptimalPoint = x,
                OptimalValue = endValue,
                FinalGradient = endGrad,
                Iterations = _maxIterations,
                FunctionEvaluations = totalFunctionEvaluations,
                StoppingReason = StoppingReason.MaxIterations,
                Success = false,
                Message = "Maximum outer iterations reached without satisfying constraints"
            };
        }
    }
}
