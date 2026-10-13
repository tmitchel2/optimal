/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Numerics.Tensors;
using System.Threading;
using Optimal.NonLinear.Mma;

namespace Optimal.NonLinear.OptimalityCriteria
{
    /// <summary>
    /// The optimality-criteria method for one volume-type constraint and bounds, the cheap update of the classic
    /// topology-optimisation codes (top88, Andreassen et al. 2011; top99neo, Ferrari and Sigmund 2020). It suits a
    /// compliance-like objective, whose gradient is non-positive, with a linear constraint vᵀx - V &lt;= 0 with v &gt; 0.
    /// </summary>
    /// <remarks>
    /// The problem must have exactly one constraint. Each iteration uses the constraint's linearisation at the
    /// iterate, g(x) + ∇gᵀ(x_new - x) &lt;= 0, which is exact for a linear constraint. The variables must be
    /// non-negative: the update is multiplicative. A variable whose objective derivative is positive moves to its lower
    /// move limit. The run stops when the KKT residual's largest component reaches
    /// <see cref="OptimizerOptions.Tolerance"/>, when the scaled change reaches <see cref="OcOptions.ChangeTolerance"/>,
    /// after <see cref="OptimizerOptions.MaxIterations"/>, or on cancellation.
    /// </remarks>
    public sealed class OcOptimizer : IInequalityOptimizer
    {
        private readonly OcOptions _options;

        /// <summary>
        /// Initializes a new instance of the <see cref="OcOptimizer"/> class.
        /// </summary>
        /// <param name="options">The options.</param>
        public OcOptimizer(OcOptions options)
        {
            _options = options ?? throw new ArgumentNullException(nameof(options));
            if (!(options.MoveLimit > 0.0) || !(options.DampingExponent > 0.0) || !(options.BisectionTolerance > 0.0)
                || !(options.ChangeTolerance >= 0.0) || !(options.Tolerance >= 0.0))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(options),
                    "MoveLimit, DampingExponent and BisectionTolerance must be positive; ChangeTolerance and Tolerance non-negative.");
            }

            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(options.MaxBisections);
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(options.MaxIterations);
        }

        /// <inheritdoc/>
        public InequalityOptimizerResult Minimize(InequalityProblem problem, double[] initialPoint, CancellationToken cancellationToken)
        {
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(initialPoint);
            problem.Validate(initialPoint);
            var evaluator = new ProblemEvaluator(problem);
            if (evaluator.Matrix.RowCount != 1)
            {
                throw new ArgumentException($"OC takes exactly one constraint; the problem has {evaluator.Matrix.RowCount}.", nameof(problem));
            }

            var n = problem.VariableCount;
            var lower = problem.LowerBounds;
            var upper = problem.UpperBounds;
            var x = new double[n];
            for (var j = 0; j < n; j++)
            {
                if (!(lower[j] >= 0.0) || !(lower[j] < upper[j]))
                {
                    throw new ArgumentException($"Variable {j}: OC needs 0 <= lower < upper (got [{lower[j]}, {upper[j]}]).", nameof(problem));
                }

                x[j] = Math.Clamp(initialPoint[j], lower[j], upper[j]);
            }

            var state = new OcState(problem, evaluator, x);
            (double Norm, double Max) noKkt = (double.NaN, double.NaN);
            if (!state.Evaluate())
            {
                return state.Result(0, double.NaN, noKkt, 0.0, StoppingReason.NumericalError, "The problem is not finite at the initial point.");
            }

            var xNew = new double[n];
            var p = new double[n];
            var lo = new double[n];
            var hi = new double[n];
            var lambda = 0.0;
            var kkt = noKkt;
            var change = 0.0;
            for (var iteration = 1; iteration <= _options.MaxIterations; iteration++)
            {
                if (cancellationToken.IsCancellationRequested)
                {
                    return state.Result(iteration - 1, lambda, kkt, change, StoppingReason.UserRequested, "Optimization cancelled by user request");
                }

                var weights = state.ConstraintGradient;
                var nonPositive = Array.FindIndex(weights, w => !(w > 0.0));
                if (nonPositive >= 0)
                {
                    return state.Result(iteration - 1, lambda, kkt, change, StoppingReason.NumericalError, $"OC needs a positive constraint gradient; ∂g/∂x_{nonPositive} = {weights[nonPositive]}.");
                }

                MoveLimits(x, lower, upper, lo, hi);
                var target = TensorPrimitives.Dot(weights, x) - state.G[0];
                var (newLambda, bisections, _) = OcUpdate.Apply(
                    x, state.Gradient, weights, target, lo, hi, _options.DampingExponent, _options.BisectionTolerance, _options.MaxBisections, p, xNew);
                lambda = newLambda;
                change = ScaledChange(x, xNew, lower, upper);
                xNew.CopyTo(x, 0);
                if (!state.Evaluate())
                {
                    return state.Result(iteration, lambda, noKkt, change, StoppingReason.NumericalError, $"The problem is not finite at iteration {iteration}.");
                }

                kkt = state.KktResidual(lambda);
                Report(state, iteration, kkt, change, bisections);
                if (kkt.Max <= _options.Tolerance)
                {
                    return state.Result(iteration, lambda, kkt, change, StoppingReason.KktTolerance, $"Converged after {iteration} iterations: KKT residual {kkt.Max:E3}.");
                }

                if (_options.ChangeTolerance > 0.0 && change <= _options.ChangeTolerance)
                {
                    return state.Result(iteration, lambda, kkt, change, StoppingReason.ParameterTolerance, $"Converged after {iteration} iterations: change {change:E3}.");
                }
            }

            return state.Result(_options.MaxIterations, lambda, kkt, change, StoppingReason.MaxIterations, $"Maximum iterations reached: KKT residual {kkt.Max:E3}.");
        }

        private void MoveLimits(ReadOnlySpan<double> x, ReadOnlySpan<double> lower, ReadOnlySpan<double> upper, Span<double> lo, Span<double> hi)
        {
            for (var j = 0; j < x.Length; j++)
            {
                var move = _options.MoveLimit * (upper[j] - lower[j]);
                lo[j] = Math.Max(lower[j], x[j] - move);
                hi[j] = Math.Min(upper[j], x[j] + move);
            }
        }

        private static double ScaledChange(ReadOnlySpan<double> x, ReadOnlySpan<double> xNew, ReadOnlySpan<double> lower, ReadOnlySpan<double> upper)
        {
            var change = 0.0;
            for (var j = 0; j < x.Length; j++)
            {
                change = Math.Max(change, Math.Abs(xNew[j] - x[j]) / (upper[j] - lower[j]));
            }

            return change;
        }

        private void Report(OcState state, int iteration, (double Norm, double Max) kkt, double change, int bisections)
        {
            if (_options.Verbose)
            {
                Console.WriteLine($"  iter {iteration}: f={state.F0:E6}, g={state.G[0]:E3}, kkt={kkt.Max:E3}, change={change:E3}, bisections={bisections}");
            }

            _options.ProgressCallback?.Invoke(new InequalityIteration
            {
                Iteration = iteration,
                ObjectiveValue = state.F0,
                MaxConstraintViolation = Math.Max(0.0, state.G[0]),
                KktResidualNorm = kkt.Norm,
                KktResidualMax = kkt.Max,
                ParameterChange = change,
                SubproblemIterations = bisections,
                LinearIterations = 0,
                InnerIterations = 0,
                FunctionEvaluations = state.Evaluations,
                Point = state.X,
            });
        }
    }
}
