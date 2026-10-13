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
    /// The progress an <see cref="IInequalityOptimizer"/> reports after each outer iteration.
    /// </summary>
    public sealed record InequalityIteration
    {
        /// <summary>
        /// Gets the outer iteration (1-based).
        /// </summary>
        public required int Iteration { get; init; }

        /// <summary>
        /// Gets the objective at the new point.
        /// </summary>
        public required double ObjectiveValue { get; init; }

        /// <summary>
        /// Gets the largest constraint violation max(0, g_i) at the new point.
        /// </summary>
        public required double MaxConstraintViolation { get; init; }

        /// <summary>
        /// Gets the Euclidean norm of the KKT residual at the new point.
        /// </summary>
        public required double KktResidualNorm { get; init; }

        /// <summary>
        /// Gets the largest absolute component of the KKT residual at the new point.
        /// </summary>
        public required double KktResidualMax { get; init; }

        /// <summary>
        /// Gets the largest scaled change max_j |Δx_j| / (upper_j - lower_j).
        /// </summary>
        public required double ParameterChange { get; init; }

        /// <summary>
        /// Gets the work inside the iteration: interior-point Newton steps (MMA, GCMMA, summed over inner iterations)
        /// or bisections (OC).
        /// </summary>
        public required int SubproblemIterations { get; init; }

        /// <summary>
        /// Gets the conjugate-gradient iterations spent on the subproblem's reduced systems; zero when solved directly.
        /// </summary>
        public required int LinearIterations { get; init; }

        /// <summary>
        /// Gets the GCMMA inner iterations taken to reach a conservative approximation; zero for the other optimisers.
        /// </summary>
        public required int InnerIterations { get; init; }

        /// <summary>
        /// Gets the problem evaluations so far.
        /// </summary>
        public required int FunctionEvaluations { get; init; }

        /// <summary>
        /// Gets the new point. It is a view of the optimizer's buffer, valid only during the callback: copy it to keep it.
        /// </summary>
        public required ReadOnlyMemory<double> Point { get; init; }
    }
}
