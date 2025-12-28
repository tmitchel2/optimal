/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Monitors optimization progress and checks multiple convergence criteria.
    /// </summary>
    public sealed class ConvergenceMonitor
    {
        private readonly double _gradientTolerance;
        private readonly double _functionTolerance;
        private readonly double _parameterTolerance;
        private readonly int _maxIterations;
        private readonly int _maxFunctionEvaluations;
        private readonly int _stallIterations;
        private readonly List<IterationState> _history;

        /// <summary>
        /// Initializes a new instance of the <see cref="ConvergenceMonitor"/> class.
        /// </summary>
        /// <param name="gradientTolerance">Gradient norm tolerance (default 1e-6).</param>
        /// <param name="functionTolerance">Function change tolerance (default 1e-8).</param>
        /// <param name="parameterTolerance">Parameter change tolerance (default 1e-8).</param>
        /// <param name="maxIterations">Maximum iterations (default 1000).</param>
        /// <param name="maxFunctionEvaluations">Maximum function evaluations (default 10000).</param>
        /// <param name="stallIterations">Iterations to detect stall (default 10, 0 to disable).</param>
        public ConvergenceMonitor(
            double gradientTolerance = 1e-6,
            double functionTolerance = 1e-8,
            double parameterTolerance = 1e-8,
            int maxIterations = 1000,
            int maxFunctionEvaluations = 10000,
            int stallIterations = 10)
        {
            _gradientTolerance = gradientTolerance;
            _functionTolerance = functionTolerance;
            _parameterTolerance = parameterTolerance;
            _maxIterations = maxIterations;
            _maxFunctionEvaluations = maxFunctionEvaluations;
            _stallIterations = stallIterations;
            _history = new List<IterationState>();
        }

        /// <summary>
        /// Records an iteration and checks for convergence.
        /// </summary>
        /// <param name="iteration">Current iteration number.</param>
        /// <param name="functionEvaluations">Total function evaluations so far.</param>
        /// <param name="x">Current parameter values.</param>
        /// <param name="fValue">Current objective function value.</param>
        /// <param name="gradient">Current gradient vector.</param>
        /// <returns>Convergence result indicating if converged and diagnostics.</returns>
        public ConvergenceResult CheckConvergence(
            int iteration,
            int functionEvaluations,
            double[] x,
            double fValue,
            double[] gradient)
        {
            var gradientNorm = ComputeNorm(gradient);
            var functionChange = 0.0;
            var parameterChange = 0.0;

            // Record current state
            var currentState = new IterationState
            {
                Iteration = iteration,
                Parameters = (double[])x.Clone(),
                FunctionValue = fValue,
                Gradient = (double[])gradient.Clone(),
                GradientNorm = gradientNorm
            };

            // Compute changes from last iteration if history exists
            if (_history.Count > 0)
            {
                var lastState = _history[^1];
                functionChange = Math.Abs(fValue - lastState.FunctionValue);
                parameterChange = ComputeDistance(x, lastState.Parameters);
            }

            _history.Add(currentState);

            // Check gradient tolerance
            if (gradientNorm < _gradientTolerance)
            {
                return new ConvergenceResult
                {
                    HasConverged = true,
                    Reason = StoppingReason.GradientTolerance,
                    GradientNorm = gradientNorm,
                    FunctionChange = functionChange,
                    ParameterChange = parameterChange,
                    Message = $"Converged: gradient norm {gradientNorm:E3} < {_gradientTolerance:E3}"
                };
            }

            // Check function value change tolerance (only after first iteration)
            if (_history.Count > 1 && functionChange < _functionTolerance)
            {
                return new ConvergenceResult
                {
                    HasConverged = true,
                    Reason = StoppingReason.FunctionTolerance,
                    GradientNorm = gradientNorm,
                    FunctionChange = functionChange,
                    ParameterChange = parameterChange,
                    Message = $"Converged: function change {functionChange:E3} < {_functionTolerance:E3}"
                };
            }

            // Check parameter change tolerance (only after first iteration)
            if (_history.Count > 1 && parameterChange < _parameterTolerance)
            {
                return new ConvergenceResult
                {
                    HasConverged = true,
                    Reason = StoppingReason.ParameterTolerance,
                    GradientNorm = gradientNorm,
                    FunctionChange = functionChange,
                    ParameterChange = parameterChange,
                    Message = $"Converged: parameter change {parameterChange:E3} < {_parameterTolerance:E3}"
                };
            }

            // Check for stall (no progress for N iterations)
            if (_stallIterations > 0 && _history.Count > _stallIterations)
            {
                var stallStart = _history.Count - _stallIterations - 1;
                var fValueAtStallStart = _history[stallStart].FunctionValue;
                var maxChangeInStallWindow = 0.0;

                for (var i = stallStart + 1; i < _history.Count; i++)
                {
                    var change = Math.Abs(_history[i].FunctionValue - fValueAtStallStart);
                    maxChangeInStallWindow = Math.Max(maxChangeInStallWindow, change);
                }

                if (maxChangeInStallWindow < _functionTolerance * 10.0) // Stall threshold
                {
                    return new ConvergenceResult
                    {
                        HasConverged = true,
                        Reason = StoppingReason.FunctionTolerance,
                        GradientNorm = gradientNorm,
                        FunctionChange = functionChange,
                        ParameterChange = parameterChange,
                        Message = $"Converged: stalled for {_stallIterations} iterations (max change {maxChangeInStallWindow:E3})"
                    };
                }
            }

            // Check maximum function evaluations
            if (functionEvaluations >= _maxFunctionEvaluations)
            {
                return new ConvergenceResult
                {
                    HasConverged = false,
                    Reason = StoppingReason.MaxFunctionEvaluations,
                    GradientNorm = gradientNorm,
                    FunctionChange = functionChange,
                    ParameterChange = parameterChange,
                    Message = $"Maximum function evaluations ({_maxFunctionEvaluations}) reached"
                };
            }

            // Check maximum iterations
            if (iteration >= _maxIterations)
            {
                return new ConvergenceResult
                {
                    HasConverged = false,
                    Reason = StoppingReason.MaxIterations,
                    GradientNorm = gradientNorm,
                    FunctionChange = functionChange,
                    ParameterChange = parameterChange,
                    Message = $"Maximum iterations ({_maxIterations}) reached"
                };
            }

            // No convergence yet
            return new ConvergenceResult
            {
                HasConverged = false,
                Reason = null,
                GradientNorm = gradientNorm,
                FunctionChange = functionChange,
                ParameterChange = parameterChange,
                Message = "Optimization in progress"
            };
        }

        /// <summary>
        /// Gets the optimization history.
        /// </summary>
        /// <returns>Read-only list of iteration states.</returns>
        public IReadOnlyList<IterationState> GetHistory() => _history.AsReadOnly();

        /// <summary>
        /// Clears the optimization history.
        /// </summary>
        public void Reset()
        {
            _history.Clear();
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

        private static double ComputeDistance(double[] x1, double[] x2)
        {
            var sum = 0.0;
            for (var i = 0; i < x1.Length; i++)
            {
                var diff = x1[i] - x2[i];
                sum += diff * diff;
            }
            return Math.Sqrt(sum);
        }
    }

    /// <summary>
    /// Represents the state of the optimization at a particular iteration.
    /// </summary>
    public sealed class IterationState
    {
        /// <summary>
        /// Gets or sets the iteration number.
        /// </summary>
        public int Iteration { get; set; }

        /// <summary>
        /// Gets or sets the parameter values at this iteration.
        /// </summary>
        public double[] Parameters { get; set; } = Array.Empty<double>();

        /// <summary>
        /// Gets or sets the objective function value at this iteration.
        /// </summary>
        public double FunctionValue { get; set; }

        /// <summary>
        /// Gets or sets the gradient vector at this iteration.
        /// </summary>
        public double[] Gradient { get; set; } = Array.Empty<double>();

        /// <summary>
        /// Gets or sets the L2 norm of the gradient at this iteration.
        /// </summary>
        public double GradientNorm { get; set; }
    }

    /// <summary>
    /// Result of a convergence check.
    /// </summary>
    public sealed class ConvergenceResult
    {
        /// <summary>
        /// Gets or sets a value indicating whether convergence has been achieved.
        /// </summary>
        public bool HasConverged { get; set; }

        /// <summary>
        /// Gets or sets the stopping reason (null if not converged yet).
        /// </summary>
        public StoppingReason? Reason { get; set; }

        /// <summary>
        /// Gets or sets the L2 norm of the gradient.
        /// </summary>
        public double GradientNorm { get; set; }

        /// <summary>
        /// Gets or sets the absolute change in function value.
        /// </summary>
        public double FunctionChange { get; set; }

        /// <summary>
        /// Gets or sets the L2 norm of the parameter change.
        /// </summary>
        public double ParameterChange { get; set; }

        /// <summary>
        /// Gets or sets a human-readable message describing the convergence status.
        /// </summary>
        public string Message { get; set; } = string.Empty;
    }
}
