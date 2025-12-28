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
    /// Result of an optimization process.
    /// </summary>
    public sealed record OptimizerResult
    {
        /// <summary>
        /// Gets the optimal point found by the optimizer.
        /// </summary>
        public double[] OptimalPoint { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the optimal objective function value at the optimal point.
        /// </summary>
        public double OptimalValue { get; init; }

        /// <summary>
        /// Gets the gradient vector at the optimal point.
        /// </summary>
        public double[] FinalGradient { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gets the number of iterations performed.
        /// </summary>
        public int Iterations { get; init; }

        /// <summary>
        /// Gets the number of objective function evaluations performed.
        /// </summary>
        public int FunctionEvaluations { get; init; }

        /// <summary>
        /// Gets the reason why the optimization process terminated.
        /// </summary>
        public StoppingReason StoppingReason { get; init; }

        /// <summary>
        /// Gets a value indicating whether the optimization succeeded.
        /// </summary>
        public bool Success { get; init; }

        /// <summary>
        /// Gets a human-readable message describing the optimization outcome.
        /// </summary>
        public string Message { get; init; } = string.Empty;

        /// <summary>
        /// Gets the L2 norm of the gradient at the optimal point.
        /// </summary>
        public double GradientNorm { get; init; }

        /// <summary>
        /// Gets the absolute change in function value from the last iteration.
        /// </summary>
        public double FunctionChange { get; init; }

        /// <summary>
        /// Gets the L2 norm of the parameter change from the last iteration.
        /// </summary>
        public double ParameterChange { get; init; }
    }
}
