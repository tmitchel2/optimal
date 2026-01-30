/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear
{
    /// <summary>
    /// Extended tolerance options for optimizers with multiple convergence criteria.
    /// </summary>
    public record ExtendedToleranceOptions : OptimizerOptions
    {
        /// <summary>
        /// Function value change tolerance for convergence.
        /// </summary>
        public double FunctionTolerance { get; init; } = 1e-8;

        /// <summary>
        /// Parameter change tolerance for convergence.
        /// </summary>
        public double ParameterTolerance { get; init; } = 1e-8;

        /// <summary>
        /// Maximum number of function evaluations (0 = unlimited).
        /// </summary>
        public int MaxFunctionEvaluations { get; init; }
    }
}
