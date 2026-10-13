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
    /// The Method of Moving Asymptotes (Svanberg 1987, "The method of moving asymptotes — a new method for structural
    /// optimization", IJNME 24:359–373; in the form of Svanberg 2007, "MMA and GCMMA – two methods for nonlinear
    /// optimization"). Each iteration replaces the objective and constraints by separable convex approximations around
    /// moving asymptotes and solves that subproblem by a primal–dual interior-point method, whose reduced Newton
    /// system is m×m when m &lt; n, so many variables and few constraints is cheap; a sparse constraint Jacobian keeps
    /// many local constraints cheap too.
    /// </summary>
    /// <remarks>
    /// The run stops when the KKT residual's largest component reaches <see cref="OptimizerOptions.Tolerance"/>, when the
    /// scaled change reaches <see cref="MovingAsymptotesOptions.ChangeTolerance"/> (if set), after
    /// <see cref="OptimizerOptions.MaxIterations"/> iterations, on cancellation, or on a non-finite evaluation. Results
    /// are deterministic: the same problem and options give the same iterates, bit for bit, on one machine.
    /// </remarks>
    public sealed class MmaOptimizer : IInequalityOptimizer
    {
        private readonly MmaOptions _options;

        /// <summary>
        /// Initializes a new instance of the <see cref="MmaOptimizer"/> class.
        /// </summary>
        /// <param name="options">The options.</param>
        public MmaOptimizer(MmaOptions options)
        {
            _options = options ?? throw new ArgumentNullException(nameof(options));
            if (!(options.Regularization > 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(options), options.Regularization, "Regularization must be positive.");
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

            var rho = new double[run.G.Length];
            Array.Fill(rho, _options.Regularization);
            var kkt = noKkt;
            for (var iteration = 1; iteration <= _options.MaxIterations; iteration++)
            {
                if (cancellationToken.IsCancellationRequested)
                {
                    return run.Result(iteration - 1, 0, 0, kkt, StoppingReason.UserRequested, "Optimization cancelled by user request");
                }

                run.PlaceAsymptotes(iteration);
                if (!run.ApproximateAndSolve(_options.Regularization, rho))
                {
                    return run.Result(iteration - 1, 0, 0, kkt, StoppingReason.NumericalError, $"The MMA subproblem failed at iteration {iteration}.");
                }

                if (!run.Advance())
                {
                    return run.Result(iteration, 0, 0, noKkt, StoppingReason.NumericalError, $"The problem is not finite at iteration {iteration}.");
                }

                kkt = run.KktResidual();
                run.Report(iteration, kkt, run.Subproblem.NewtonIterations, run.Subproblem.LinearIterations, 0);
                if (run.Converged(kkt) is { } stopped)
                {
                    return run.Result(iteration, 0, 0, kkt, stopped, $"Converged after {iteration} iterations: KKT residual {kkt.Max:E3}, change {run.ParameterChange:E3}.");
                }
            }

            return run.Result(_options.MaxIterations, 0, 0, kkt, StoppingReason.MaxIterations, $"Maximum iterations reached: KKT residual {kkt.Max:E3}.");
        }
    }
}
