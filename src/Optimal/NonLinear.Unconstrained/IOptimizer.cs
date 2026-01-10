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
        /// Minimizes the objective function.
        /// </summary>
        /// <param name="objective">
        /// The objective function to minimize. Takes a parameter vector and returns
        /// the function value and gradient vector.
        /// </param>
        /// <returns>The optimization result.</returns>
        OptimizerResult Minimize(Func<double[], (double value, double[] gradient)> objective);

        /// <summary>
        /// Sets the initial point for the optimization.
        /// </summary>
        /// <param name="x0">The initial parameter vector.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        IOptimizer WithInitialPoint(double[] x0);

        /// <summary>
        /// Sets the gradient norm tolerance for convergence.
        /// </summary>
        /// <param name="tolerance">The gradient norm tolerance.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        IOptimizer WithTolerance(double tolerance);

        /// <summary>
        /// Sets the maximum number of iterations.
        /// </summary>
        /// <param name="maxIterations">The maximum number of iterations.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        IOptimizer WithMaxIterations(int maxIterations);

        /// <summary>
        /// Enables or disables verbose output.
        /// </summary>
        /// <param name="verbose">True to enable verbose output, false otherwise.</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        IOptimizer WithVerbose(bool verbose = true);
    }
}
