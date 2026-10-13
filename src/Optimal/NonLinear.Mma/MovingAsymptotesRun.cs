/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// The state MMA and GCMMA share over one run: the iterate and its two predecessors, the asymptotes, the box, the
    /// problem's values and gradients at the iterate, the approximation and the subproblem solver. Every buffer is
    /// allocated here, once.
    /// </summary>
    internal sealed class MovingAsymptotesRun
    {
        private readonly InequalityProblem _problem;
        private readonly MovingAsymptotesOptions _options;
        private readonly double[] _xOld1;
        private readonly double[] _xOld2;
        private readonly double[] _range;
        private readonly double[] _rangeInverse;
        private readonly double[] _df0;
        private readonly double[] _dg;
        private readonly double[] _work;

        public MovingAsymptotesRun(InequalityProblem problem, MovingAsymptotesOptions options, double[] initialPoint)
        {
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(initialPoint);
            problem.Validate(initialPoint);
            Asymptotes.Validate(options);
            _problem = problem;
            _options = options;
            Evaluator = new ProblemEvaluator(problem);
            var n = problem.VariableCount;
            var m = Evaluator.Matrix.RowCount;
            if (m == 0)
            {
                throw new ArgumentException("MMA needs at least one constraint; use LBFGSBOptimizer for bounds alone.", nameof(problem));
            }

            Settings = SubproblemSettings.From(options, m);
            Approximation = new MmaApproximation(Evaluator.Matrix);
            Subproblem = new MmaSubproblemSolver(Evaluator.Matrix, Settings);
            X = new double[n];
            _xOld1 = new double[n];
            _xOld2 = new double[n];
            Low = new double[n];
            Upp = new double[n];
            Alpha = new double[n];
            Beta = new double[n];
            _range = new double[n];
            _rangeInverse = new double[n];
            _df0 = new double[n];
            G = new double[m];
            _dg = new double[Evaluator.Matrix.StoredCount];
            _work = new double[n];
            for (var j = 0; j < n; j++)
            {
                if (!(problem.LowerBounds[j] < problem.UpperBounds[j]))
                {
                    throw new ArgumentException($"Variable {j}: MMA needs lower < upper (got [{problem.LowerBounds[j]}, {problem.UpperBounds[j]}]).", nameof(problem));
                }

                X[j] = Math.Clamp(initialPoint[j], problem.LowerBounds[j], problem.UpperBounds[j]);
                _range[j] = Math.Max(problem.UpperBounds[j] - problem.LowerBounds[j], 1e-5);
                _rangeInverse[j] = 1.0 / _range[j];
            }

            X.CopyTo(_xOld1, 0);
            X.CopyTo(_xOld2, 0);
        }

        public ProblemEvaluator Evaluator { get; }

        public SubproblemSettings Settings { get; }

        public MmaApproximation Approximation { get; }

        public MmaSubproblemSolver Subproblem { get; }

        /// <summary>Gets the current iterate.</summary>
        public double[] X { get; }

        public double[] Low { get; }

        public double[] Upp { get; }

        public double[] Alpha { get; }

        public double[] Beta { get; }

        /// <summary>Gets f0 at the iterate.</summary>
        public double F0 { get; private set; }

        /// <summary>Gets g at the iterate.</summary>
        public double[] G { get; }

        /// <summary>Gets ∇f0 at the iterate.</summary>
        public ReadOnlySpan<double> ObjectiveGradient => _df0;

        /// <summary>Gets the constraint Jacobian at the iterate, in the matrix's layout.</summary>
        public ReadOnlySpan<double> ConstraintJacobian => _dg;

        /// <summary>Gets the variables' ranges, at least 1e-5.</summary>
        public ReadOnlySpan<double> Range => _range;

        /// <summary>Gets the largest scaled change over the last step.</summary>
        public double ParameterChange { get; private set; }

        /// <summary>
        /// Evaluates the problem, with gradients, at the current iterate. Returns false if anything is not finite.
        /// </summary>
        public bool EvaluateIterate()
        {
            var finite = Evaluator.Evaluate(X, _df0, G, _dg, out var f0);
            F0 = f0;
            return finite;
        }

        /// <summary>
        /// Moves the asymptotes for this (1-based) iteration and sets the subproblem's box.
        /// </summary>
        public void PlaceAsymptotes(int iteration)
        {
            Asymptotes.Update(iteration, X, _xOld1, _xOld2, _range, Low, Upp, _options);
            Asymptotes.Box(X, Low, Upp, _problem.LowerBounds, _problem.UpperBounds, _range, Alpha, Beta, _options);
        }

        /// <summary>
        /// Builds the approximations at the iterate with curvature terms ρ_0 and ρ_i, and solves the subproblem.
        /// Returns false on a numerical failure.
        /// </summary>
        public bool ApproximateAndSolve(double rho0, ReadOnlySpan<double> rho)
        {
            Approximation.Build(X, _rangeInverse, Low, Upp, F0, _df0, G, _dg, rho0, rho);
            return Subproblem.Solve(Approximation, Low, Upp, Alpha, Beta);
        }

        /// <summary>
        /// Makes the subproblem's solution the iterate, keeping the two before it, and evaluates the problem there with
        /// gradients. Returns false if anything is not finite.
        /// </summary>
        public bool Advance()
        {
            _xOld1.CopyTo(_xOld2, 0);
            X.CopyTo(_xOld1, 0);
            Subproblem.X.CopyTo(X, 0);
            var change = 0.0;
            for (var j = 0; j < X.Length; j++)
            {
                change = Math.Max(change, Math.Abs(X[j] - _xOld1[j]) * _rangeInverse[j]);
            }

            ParameterChange = change;
            return EvaluateIterate();
        }

        /// <summary>
        /// The residual of the KKT conditions of Svanberg's artificial-variable form at the iterate, with the last
        /// subproblem's λ, y and z: x_j - clamp(x_j - ∂L/∂x_j, lower_j, upper_j) for each variable, the same projected
        /// form for y_i &gt;= 0 and z &gt;= 0, then max(0, s_i) and λ_i s_i for s_i = g_i - a_i z - y_i. Svanberg's kktcheck
        /// uses the subproblem's barrier multipliers of the move-limit box for the bounds, which leaves a floor of about
        /// ε·(x_j - lower_j)/(x_j - α_j); the projected form has none. z is taken as 0 when every a_i is 0, since it then
        /// has no effect. Returns the residual's Euclidean norm and its largest component.
        /// </summary>
        public (double Norm, double Max) KktResidual()
        {
            var sub = Subproblem;
            var settings = Settings;
            var lower = _problem.LowerBounds;
            var upper = _problem.UpperBounds;
            var hasA = Array.Exists(settings.A, value => value != 0.0);
            var z = hasA ? sub.Z : 0.0;
            _df0.CopyTo(_work, 0);
            Evaluator.Matrix.MultiplyTransposeAdd(_dg, sub.Lambda, _work);
            var sum = 0.0;
            var max = 0.0;
            for (var j = 0; j < X.Length; j++)
            {
                var rex = Projected(X[j], _work[j], lower[j], upper[j]);
                sum += rex * rex;
                max = Math.Max(max, Math.Abs(rex));
            }

            var aLambda = 0.0;
            for (var i = 0; i < G.Length; i++)
            {
                var y = sub.Y[i];
                var rey = Projected(y, settings.C[i] + (settings.D[i] * y) - sub.Lambda[i], 0.0, double.PositiveInfinity);
                var slack = G[i] - (settings.A[i] * z) - y;
                var feasibility = Math.Max(0.0, slack);
                var complementarity = sub.Lambda[i] * slack;
                aLambda += settings.A[i] * sub.Lambda[i];
                sum += (rey * rey) + (feasibility * feasibility) + (complementarity * complementarity);
                max = Math.Max(max, Math.Max(Math.Abs(rey), Math.Max(feasibility, Math.Abs(complementarity))));
            }

            var rez = Projected(z, settings.A0 - aLambda, 0.0, double.PositiveInfinity);
            sum += rez * rez;
            max = Math.Max(max, Math.Abs(rez));
            return (Math.Sqrt(sum), max);
        }

        /// <summary>
        /// The projected-gradient residual of one variable with derivative <paramref name="derivative"/> on [low, high]:
        /// zero exactly when the derivative vanishes inside, or pushes outward at a bound.
        /// </summary>
        private static double Projected(double value, double derivative, double low, double high) =>
            value - Math.Clamp(value - derivative, low, high);

        /// <summary>
        /// Gets max(0, g_i) over the constraints at the iterate.
        /// </summary>
        public double MaxViolation()
        {
            var violation = 0.0;
            foreach (var value in G)
            {
                violation = Math.Max(violation, value);
            }

            return violation;
        }

        /// <summary>
        /// Reports progress to the callback, if there is one.
        /// </summary>
        public void Report(int iteration, (double Norm, double Max) kkt, int subproblemIterations, int linearIterations, int innerIterations)
        {
            var callback = _options.ProgressCallback;
            if (callback is null && !_options.Verbose)
            {
                return;
            }

            var progress = new InequalityIteration
            {
                Iteration = iteration,
                ObjectiveValue = F0,
                MaxConstraintViolation = MaxViolation(),
                KktResidualNorm = kkt.Norm,
                KktResidualMax = kkt.Max,
                ParameterChange = ParameterChange,
                SubproblemIterations = subproblemIterations,
                LinearIterations = linearIterations,
                InnerIterations = innerIterations,
                FunctionEvaluations = Evaluator.Evaluations,
                Point = X,
            };
            if (_options.Verbose)
            {
                Console.WriteLine(
                    $"  iter {iteration}: f={F0:E6}, max_violation={progress.MaxConstraintViolation:E3}, kkt={kkt.Max:E3}, change={ParameterChange:E3}, newton={subproblemIterations}, cg={linearIterations}, inner={innerIterations}");
            }

            callback?.Invoke(progress);
        }

        /// <summary>
        /// Builds the result at the iterate.
        /// </summary>
        public InequalityOptimizerResult Result(
            int iterations,
            int innerIterations,
            int nonConservativeIterations,
            (double Norm, double Max) kkt,
            StoppingReason reason,
            string message)
        {
            var success = reason is StoppingReason.KktTolerance or StoppingReason.ParameterTolerance;
            return new InequalityOptimizerResult
            {
                OptimalPoint = (double[])X.Clone(),
                OptimalValue = F0,
                ConstraintValues = (double[])G.Clone(),
                Multipliers = (double[])Subproblem.Lambda.Clone(),
                MaxConstraintViolation = MaxViolation(),
                KktResidualNorm = kkt.Norm,
                KktResidualMax = kkt.Max,
                ParameterChange = ParameterChange,
                Iterations = iterations,
                InnerIterations = innerIterations,
                NonConservativeIterations = nonConservativeIterations,
                FunctionEvaluations = Evaluator.Evaluations,
                StoppingReason = reason,
                Success = success,
                Message = message,
            };
        }

        /// <summary>
        /// The stopping test after an iteration: the KKT residual first, then the change.
        /// </summary>
        public StoppingReason? Converged((double Norm, double Max) kkt)
        {
            if (kkt.Max <= _options.Tolerance)
            {
                return StoppingReason.KktTolerance;
            }

            return _options.ChangeTolerance > 0.0 && ParameterChange <= _options.ChangeTolerance
                ? StoppingReason.ParameterTolerance
                : null;
        }
    }
}
