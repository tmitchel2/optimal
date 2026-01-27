/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Monitoring
{
    /// <summary>
    /// Information about a single step during line search.
    /// </summary>
    public sealed record LineSearchStepInfo
    {
        /// <summary>
        /// Step size (alpha) along the search direction.
        /// </summary>
        public double Alpha { get; init; }

        /// <summary>
        /// Function value at x + alpha * direction.
        /// </summary>
        public double FunctionValue { get; init; }

        /// <summary>
        /// Gradient at x + alpha * direction.
        /// </summary>
        public double[] Gradient { get; init; } = Array.Empty<double>();
    }

    /// <summary>
    /// Information about a detected C0 (continuity) violation.
    /// </summary>
    public sealed record C0ViolationInfo
    {
        /// <summary>
        /// Step size where discontinuity was detected.
        /// </summary>
        public double Alpha { get; init; }

        /// <summary>
        /// Expected function value from interpolation.
        /// </summary>
        public double ExpectedValue { get; init; }

        /// <summary>
        /// Actual function value observed.
        /// </summary>
        public double ActualValue { get; init; }

        /// <summary>
        /// Relative error between expected and actual.
        /// </summary>
        public double RelativeError { get; init; }

        /// <summary>
        /// Base point of the line search (x).
        /// </summary>
        public double[] BasePoint { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Search direction (d).
        /// </summary>
        public double[] Direction { get; init; } = Array.Empty<double>();
    }

    /// <summary>
    /// Information about a detected C1 (smoothness) violation.
    /// </summary>
    public sealed record C1ViolationInfo
    {
        /// <summary>
        /// Step size where non-smoothness was detected.
        /// </summary>
        public double Alpha { get; init; }

        /// <summary>
        /// Expected directional derivative from interpolation.
        /// </summary>
        public double ExpectedDirectionalDerivative { get; init; }

        /// <summary>
        /// Actual directional derivative observed.
        /// </summary>
        public double ActualDirectionalDerivative { get; init; }

        /// <summary>
        /// Relative error between expected and actual.
        /// </summary>
        public double RelativeError { get; init; }

        /// <summary>
        /// Base point of the line search (x).
        /// </summary>
        public double[] BasePoint { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Search direction (d).
        /// </summary>
        public double[] Direction { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Gradient at the point of violation.
        /// </summary>
        public double[] Gradient { get; init; } = Array.Empty<double>();
    }
}
