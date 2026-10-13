/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Threading;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// The globally convergent MMA (Svanberg 2002, "A class of globally convergent optimization methods based on
    /// conservative convex separable approximations", SIAM J. Optim. 12:555–573; in the form of Svanberg 2007). Each
    /// outer iteration solves MMA subproblems in inner iterations, raising the curvature terms ρ_0 and ρ_i until the
    /// approximations are conservative at the subproblem's solution (f_i(x̂) &lt;= f̃_i(x̂) for i = 0..m). Then, from a
    /// feasible start, the objective never increases and every outer iterate stays feasible.
    /// </summary>
    /// <remarks>
    /// Inner iterations evaluate the problem without gradients (an empty gradient span), so an adjoint can be skipped;
    /// the accepted point is then evaluated again with gradients. Stopping rules are those of <see cref="MmaOptimizer"/>.
    /// </remarks>
    public sealed class GcmmaOptimizer : IInequalityOptimizer
    {
        private const double RhoCoefficientMinimum = 1e-12;

        private readonly GcmmaOptions _options;

        /// <summary>
        /// Initializes a new instance of the <see cref="GcmmaOptimizer"/> class.
        /// </summary>
        /// <param name="options">The options.</param>
        public GcmmaOptimizer(GcmmaOptions options)
        {
            _options = options ?? throw new ArgumentNullException(nameof(options));
            ArgumentOutOfRangeException.ThrowIfNegative(options.MaxInnerIterations);
            if (!(options.MinimumRho > 0.0) || !(options.ConservativeTolerance >= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(options), "MinimumRho must be positive and ConservativeTolerance non-negative.");
            }
        }

        /// <inheritdoc/>
        public InequalityOptimizerResult Minimize(InequalityProblem problem, double[] initialPoint, CancellationToken cancellationToken)
        {
            var run = new MovingAsymptotesRun(problem, _options, initialPoint);
            (double Norm, double Max) noKkt = (double.NaN, double.NaN);
            if (!run.EvaluateIterate())
            {
                return run.Result(0, 0, 0, noKkt, StoppingReason.NumericalError, "The problem is not finite at the initial point.");
            }

            var m = run.G.Length;
            var rho = new double[m];
            var trialG = new double[m];
            var approximatedG = new double[m];
            var kkt = noKkt;
            var innerTotal = 0;
            var nonConservative = 0;
            for (var iteration = 1; iteration <= _options.MaxIterations; iteration++)
            {
                if (cancellationToken.IsCancellationRequested)
                {
                    return run.Result(iteration - 1, innerTotal, nonConservative, kkt, StoppingReason.UserRequested, "Optimization cancelled by user request");
                }

                run.PlaceAsymptotes(iteration);
                var rho0 = InitialRho(run, rho);
                var newton = 0;
                var linear = 0;
                var inner = 0;
                var conservative = false;
                while (true)
                {
                    if (!run.ApproximateAndSolve(rho0, rho))
                    {
                        return run.Result(iteration - 1, innerTotal, nonConservative, kkt, StoppingReason.NumericalError, $"The GCMMA subproblem failed at iteration {iteration}.");
                    }

                    newton += run.Subproblem.NewtonIterations;
                    linear += run.Subproblem.LinearIterations;
                    var trial = run.Subproblem.X;
                    if (!run.Evaluator.Evaluate(trial, Span<double>.Empty, trialG, Span<double>.Empty, out var trialF0))
                    {
                        return run.Result(iteration - 1, innerTotal, nonConservative, kkt, StoppingReason.NumericalError, $"The problem is not finite at a trial point of iteration {iteration}.");
                    }

                    var approximatedF0 = run.Approximation.Objective(trial, run.Low, run.Upp);
                    run.Approximation.Constraints(trial, run.Low, run.Upp, approximatedG);
                    conservative = IsConservative(trialF0, approximatedF0, trialG, approximatedG);
                    if (conservative || inner == _options.MaxInnerIterations)
                    {
                        break;
                    }

                    inner++;
                    rho0 = RaiseRho(run, trialF0, approximatedF0, trialG, approximatedG, rho0, rho);
                }

                innerTotal += inner;
                nonConservative += conservative ? 0 : 1;
                if (!run.Advance())
                {
                    return run.Result(iteration, innerTotal, nonConservative, noKkt, StoppingReason.NumericalError, $"The problem is not finite at iteration {iteration}.");
                }

                kkt = run.KktResidual();
                run.Report(iteration, kkt, newton, linear, inner);
                if (run.Converged(kkt) is { } stopped)
                {
                    return run.Result(iteration, innerTotal, nonConservative, kkt, stopped, $"Converged after {iteration} iterations: KKT residual {kkt.Max:E3}, change {run.ParameterChange:E3}.");
                }
            }

            return run.Result(_options.MaxIterations, innerTotal, nonConservative, kkt, StoppingReason.MaxIterations, $"Maximum iterations reached: KKT residual {kkt.Max:E3}.");
        }

        /// <summary>
        /// Svanberg's asymp: ρ_0 = max(ρ_min, (0.1/n) Σ_j |∂f_0/∂x_j| range_j), and ρ_i likewise from ∂f_i/∂x.
        /// </summary>
        private double InitialRho(MovingAsymptotesRun run, Span<double> rho)
        {
            var n = run.X.Length;
            var range = run.Range;
            var gradient = run.ObjectiveGradient;
            var sum = 0.0;
            for (var j = 0; j < n; j++)
            {
                sum += Math.Abs(gradient[j]) * range[j];
            }

            run.Evaluator.Matrix.RowWeightedAbsoluteSums(run.ConstraintJacobian, range, rho);
            for (var i = 0; i < rho.Length; i++)
            {
                rho[i] = Math.Max(_options.MinimumRho, 0.1 / n * rho[i]);
            }

            return Math.Max(_options.MinimumRho, 0.1 / n * sum);
        }

        private bool IsConservative(double f0, double approximatedF0, ReadOnlySpan<double> g, ReadOnlySpan<double> approximatedG)
        {
            var tolerance = _options.ConservativeTolerance;
            if (!(approximatedF0 + tolerance >= f0))
            {
                return false;
            }

            for (var i = 0; i < g.Length; i++)
            {
                if (!(approximatedG[i] + tolerance >= g[i]))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Svanberg's raaupdate: each non-conservative ρ grows by the shortfall over
        /// Σ_j (x̂_j - x_j)²/((U_j - x̂_j)(x̂_j - L_j)) · (U_j - L_j)/range_j, then by 10%, and at most tenfold.
        /// </summary>
        private double RaiseRho(
            MovingAsymptotesRun run,
            double f0,
            double approximatedF0,
            ReadOnlySpan<double> g,
            ReadOnlySpan<double> approximatedG,
            double rho0,
            Span<double> rho)
        {
            var trial = run.Subproblem.X;
            var x = run.X;
            var range = run.Range;
            var coefficient = 0.0;
            for (var j = 0; j < x.Length; j++)
            {
                var step = trial[j] - x[j];
                coefficient += step * step / ((run.Upp[j] - trial[j]) * (trial[j] - run.Low[j])) * (run.Upp[j] - run.Low[j]) / range[j];
            }

            coefficient = Math.Max(coefficient, RhoCoefficientMinimum);
            var slack = 0.5 * _options.ConservativeTolerance;
            for (var i = 0; i < rho.Length; i++)
            {
                if (g[i] > approximatedG[i] + slack)
                {
                    rho[i] = Raise(rho[i], g[i] - approximatedG[i], coefficient);
                }
            }

            return f0 > approximatedF0 + slack ? Raise(rho0, f0 - approximatedF0, coefficient) : rho0;
        }

        private static double Raise(double rho, double shortfall, double coefficient) =>
            Math.Min(1.1 * (rho + (shortfall / coefficient)), 10.0 * rho);
    }
}
