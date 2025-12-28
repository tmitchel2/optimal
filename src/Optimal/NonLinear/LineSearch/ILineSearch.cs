/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.LineSearch
{
    /// <summary>
    /// Interface for line search algorithms that find an appropriate step size.
    /// </summary>
    public interface ILineSearch
    {
        /// <summary>
        /// Finds an appropriate step size along the search direction.
        /// </summary>
        /// <param name="objective">
        /// The objective function to minimize. Takes a parameter vector and returns
        /// the function value and gradient vector.
        /// </param>
        /// <param name="x">The current point.</param>
        /// <param name="fx">The function value at the current point.</param>
        /// <param name="gradient">The gradient at the current point.</param>
        /// <param name="direction">The search direction (typically negative gradient).</param>
        /// <param name="initialStepSize">The initial step size to try.</param>
        /// <returns>The step size to use, or 0 if no suitable step size was found.</returns>
        double FindStepSize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] x,
            double fx,
            double[] gradient,
            double[] direction,
            double initialStepSize);
    }
}
