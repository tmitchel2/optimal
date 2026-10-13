/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear
{
    /// <summary>
    /// The result of an <see cref="IInequalityOptimizer"/> run.
    /// </summary>
    public sealed record InequalityOptimizerResult
    {
        /// <summary>
        /// Gets the final point.
        /// </summary>
        public double[] OptimalPoint { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the objective at the final point.
        /// </summary>
        public double OptimalValue { get; init; }

        /// <summary>
        /// Gets the constraint values g_i at the final point.
        /// </summary>
        public double[] ConstraintValues { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the Lagrange multipliers λ_i of the constraints, from the last subproblem (MMA, GCMMA) or bisection (OC).
        /// </summary>
        public double[] Multipliers { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the largest constraint violation max(0, g_i) at the final point.
        /// </summary>
        public double MaxConstraintViolation { get; init; }

        /// <summary>
        /// Gets the Euclidean norm of the KKT residual at the final point.
        /// </summary>
        public double KktResidualNorm { get; init; }

        /// <summary>
        /// Gets the largest absolute component of the KKT residual at the final point.
        /// </summary>
        public double KktResidualMax { get; init; }

        /// <summary>
        /// Gets the largest change max_j |x_j - x_j,previous| / (upper_j - lower_j) over the last iteration.
        /// </summary>
        public double ParameterChange { get; init; }

        /// <summary>
        /// Gets the number of outer iterations.
        /// </summary>
        public int Iterations { get; init; }

        /// <summary>
        /// Gets the number of inner (GCMMA conservativeness) iterations over the run; zero for the other optimisers.
        /// </summary>
        public int InnerIterations { get; init; }

        /// <summary>
        /// Gets the GCMMA outer iterations that reached the inner-iteration cap and accepted a point whose
        /// approximations were not shown conservative; zero for the other optimisers.
        /// </summary>
        public int NonConservativeIterations { get; init; }

        /// <summary>
        /// Gets the number of evaluations of the problem (objective and every block, with or without gradients).
        /// </summary>
        public int FunctionEvaluations { get; init; }

        /// <summary>
        /// Gets the reason the run stopped.
        /// </summary>
        public StoppingReason StoppingReason { get; init; }

        /// <summary>
        /// Gets a value indicating whether the run converged.
        /// </summary>
        public bool Success { get; init; }

        /// <summary>
        /// Gets a human-readable message describing the outcome.
        /// </summary>
        public string Message { get; init; } = string.Empty;
    }
}
