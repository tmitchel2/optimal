/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Constrained
{
    /// <summary>
    /// Interface for optimizers that natively support box constraints.
    /// Optimizers implementing this interface handle bounds internally rather than
    /// requiring an external projection approach.
    /// </summary>
    public interface IBoxConstrainedOptimizer : IOptimizer
    {
        /// <summary>
        /// Sets the box constraints (bounds) for all variables.
        /// </summary>
        /// <param name="lower">Lower bounds for each variable (use double.NegativeInfinity for unbounded).</param>
        /// <param name="upper">Upper bounds for each variable (use double.PositiveInfinity for unbounded).</param>
        /// <returns>This optimizer instance for method chaining.</returns>
        IBoxConstrainedOptimizer WithBounds(double[] lower, double[] upper);
    }
}
