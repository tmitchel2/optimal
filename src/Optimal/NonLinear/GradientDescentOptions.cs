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
    /// Options for the Gradient Descent optimizer.
    /// </summary>
    public record GradientDescentOptions : ExtendedToleranceOptions
    {
        /// <summary>
        /// Fixed step size (learning rate) when no line search is used.
        /// </summary>
        public double StepSize { get; init; } = 0.01;
    }
}
