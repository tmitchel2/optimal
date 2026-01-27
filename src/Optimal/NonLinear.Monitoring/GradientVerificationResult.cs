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
    /// Result of gradient verification for a single function.
    /// </summary>
    public sealed record GradientVerificationResult
    {
        /// <summary>
        /// True if gradient appears incorrect (relative error exceeds tolerance).
        /// </summary>
        public bool IsSuspicious { get; init; }

        /// <summary>
        /// Function index: -1 for objective, 0+ for constraints.
        /// </summary>
        public int FunctionIndex { get; init; }

        /// <summary>
        /// Index of the variable with the largest gradient error.
        /// </summary>
        public int WorstVariableIndex { get; init; }

        /// <summary>
        /// The point where the gradient was tested.
        /// </summary>
        public double[] TestPoint { get; init; } = Array.Empty<double>();

        /// <summary>
        /// The analytic gradient provided by the user.
        /// </summary>
        public double[] AnalyticGradient { get; init; } = Array.Empty<double>();

        /// <summary>
        /// The numerically computed gradient (4-point central difference).
        /// </summary>
        public double[] NumericalGradient { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Relative error for each gradient component.
        /// </summary>
        public double[] RelativeErrors { get; init; } = Array.Empty<double>();

        /// <summary>
        /// Maximum relative error across all components.
        /// </summary>
        public double MaxRelativeError { get; init; }
    }
}
