/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control.Core
{
    /// <summary>
    /// Result of an optimal control problem solved via collocation.
    /// </summary>
    public sealed record CollocationResult
    {
        /// <summary>
        /// Gets a value indicating whether the optimization succeeded.
        /// </summary>
        public bool Success { get; init; }

        /// <summary>
        /// Gets a human-readable message describing the optimization outcome.
        /// </summary>
        public string Message { get; init; } = string.Empty;

        /// <summary>
        /// Gets the time points (nodes) of the solution trajectory.
        /// Length = Segments + 1
        /// </summary>
        public double[] Times { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the state trajectory at each time point.
        /// Dimensions: [TimePoints][StateDim]
        /// </summary>
        public double[][] States { get; init; } = Array.Empty<double[]>();

        /// <summary>
        /// Gets the control trajectory at each time point.
        /// Dimensions: [TimePoints][ControlDim]
        /// </summary>
        public double[][] Controls { get; init; } = Array.Empty<double[]>();

        /// <summary>
        /// Gets the optimal cost (objective function value).
        /// </summary>
        public double OptimalCost { get; init; }

        /// <summary>
        /// Gets the number of NLP iterations performed.
        /// </summary>
        public int Iterations { get; init; }

        /// <summary>
        /// Gets the maximum defect (constraint violation) in the solution.
        /// </summary>
        public double MaxDefect { get; init; }

        /// <summary>
        /// Gets the gradient norm at the optimal solution.
        /// </summary>
        public double GradientNorm { get; init; }
    }
}
