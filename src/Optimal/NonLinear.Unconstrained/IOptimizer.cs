/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Unconstrained
{
    /// <summary>
    /// Interface for nonlinear optimization algorithms.
    /// </summary>
    public interface IOptimizer
    {
        /// <summary>
        /// Minimizes the objective function starting from the given initial point.
        /// </summary>
        /// <param name="objective">
        /// The objective function to minimize. Takes a parameter vector and returns
        /// the function value and gradient vector.
        /// </param>
        /// <param name="initialPoint">The initial parameter vector.</param>
        /// <returns>The optimization result.</returns>
        OptimizerResult Minimize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] initialPoint);
    }
}
