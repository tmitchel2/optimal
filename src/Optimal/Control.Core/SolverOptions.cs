/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.Control.Core
{
    /// <summary>
    /// Base options for optimal control problem solvers.
    /// </summary>
    public record SolverOptions
    {
        /// <summary>
        /// Gets the number of collocation segments.
        /// </summary>
        public int Segments { get; init; } = 20;

        /// <summary>
        /// Gets the convergence tolerance.
        /// </summary>
        public double Tolerance { get; init; } = 1e-6;

        /// <summary>
        /// Gets the maximum number of iterations.
        /// </summary>
        public int MaxIterations { get; init; } = 100;

        /// <summary>
        /// Gets whether verbose output is enabled.
        /// </summary>
        public bool Verbose { get; init; }

        /// <summary>
        /// Gets whether parallelization is enabled.
        /// </summary>
        public bool EnableParallelization { get; init; } = true;

        /// <summary>
        /// Gets the progress callback function.
        /// </summary>
        public ProgressCallback? ProgressCallback { get; init; }
    }
}
