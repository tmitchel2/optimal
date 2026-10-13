/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear.Mma;

namespace Optimal.NonLinear.OptimalityCriteria
{
    /// <summary>
    /// The OC iterate and the problem's values and gradients there.
    /// </summary>
    internal sealed class OcState
    {
        private static readonly double[] s_one = [1.0];

        private readonly InequalityProblem _problem;
        private readonly ProblemEvaluator _evaluator;
        private readonly double[] _jacobian;

        public OcState(InequalityProblem problem, ProblemEvaluator evaluator, double[] x)
        {
            _problem = problem;
            _evaluator = evaluator;
            var n = problem.VariableCount;
            X = x;
            G = new double[1];
            Gradient = new double[n];
            ConstraintGradient = new double[n];
            _jacobian = new double[evaluator.Matrix.StoredCount];
        }

        public double[] X { get; }

        public double[] G { get; }

        public double[] Gradient { get; }

        /// <summary>Gets the constraint's gradient over all n variables.</summary>
        public double[] ConstraintGradient { get; }

        public double F0 { get; private set; }

        public int Evaluations => _evaluator.Evaluations;

        /// <summary>
        /// Evaluates with gradients and spreads the constraint's gradient over all n variables.
        /// </summary>
        public bool Evaluate()
        {
            var finite = _evaluator.Evaluate(X, Gradient, G, _jacobian, out var f0);
            F0 = f0;
            Array.Clear(ConstraintGradient);
            _evaluator.Matrix.MultiplyTransposeAdd(_jacobian, s_one, ConstraintGradient);
            return finite;
        }

        /// <summary>
        /// The projected-gradient KKT residual: x_j - clamp(x_j - (∂f/∂x_j + λ ∂g/∂x_j), lower_j, upper_j) for each j,
        /// then max(0, g) and |λg|.
        /// </summary>
        public (double Norm, double Max) KktResidual(double lambda)
        {
            var sum = 0.0;
            var max = 0.0;
            for (var j = 0; j < X.Length; j++)
            {
                var stationarity = Gradient[j] + (lambda * ConstraintGradient[j]);
                var residual = X[j] - Math.Clamp(X[j] - stationarity, _problem.LowerBounds[j], _problem.UpperBounds[j]);
                sum += residual * residual;
                max = Math.Max(max, Math.Abs(residual));
            }

            var violation = Math.Max(0.0, G[0]);
            var complementarity = Math.Abs(lambda * G[0]);
            sum += (violation * violation) + (complementarity * complementarity);
            max = Math.Max(max, Math.Max(violation, complementarity));
            return (Math.Sqrt(sum), max);
        }

        public InequalityOptimizerResult Result(int iterations, double lambda, (double Norm, double Max) kkt, double change, StoppingReason reason, string message)
        {
            return new InequalityOptimizerResult
            {
                OptimalPoint = (double[])X.Clone(),
                OptimalValue = F0,
                ConstraintValues = (double[])G.Clone(),
                Multipliers = [lambda],
                MaxConstraintViolation = Math.Max(0.0, G[0]),
                KktResidualNorm = kkt.Norm,
                KktResidualMax = kkt.Max,
                ParameterChange = change,
                Iterations = iterations,
                FunctionEvaluations = Evaluations,
                StoppingReason = reason,
                Success = reason is StoppingReason.KktTolerance or StoppingReason.ParameterTolerance,
                Message = message,
            };
        }
    }
}
