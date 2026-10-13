/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.OptimalityCriteria
{
    /// <summary>
    /// Options for <see cref="OcOptimizer"/>. <see cref="OptimizerOptions.Tolerance"/> is the tolerance on the largest
    /// component of the KKT residual; <see cref="OptimizerOptions.MaxIterations"/> caps the iterations.
    /// </summary>
    public record OcOptions : OptimizerOptions
    {
        /// <summary>
        /// Gets the move limit, as a fraction of each variable's range (top88 and top99neo: 0.2 on [0, 1]).
        /// </summary>
        public double MoveLimit { get; init; } = 0.2;

        /// <summary>
        /// Gets the damping exponent η of the update x_j ← x_j·B_j^η (0.5 in the classic codes).
        /// </summary>
        public double DampingExponent { get; init; } = 0.5;

        /// <summary>
        /// Gets the bisection's stopping test on the bracket [l1, l2] of l = λ^η: (l2 - l1)/(l1 + l2) &lt;= tolerance
        /// (top99neo: 1e-4).
        /// </summary>
        public double BisectionTolerance { get; init; } = 1e-4;

        /// <summary>
        /// Gets the cap on bisections per iteration.
        /// </summary>
        public int MaxBisections { get; init; } = 200;

        /// <summary>
        /// Gets the tolerance on the scaled change max_j |Δx_j| / (upper_j - lower_j) that also ends the run; 0 turns
        /// it off.
        /// </summary>
        public double ChangeTolerance { get; init; } = 1e-4;

        /// <summary>
        /// Gets an optional callback invoked after each iteration; its SubproblemIterations are the bisections.
        /// </summary>
        public Action<InequalityIteration>? ProgressCallback { get; init; }
    }
}
