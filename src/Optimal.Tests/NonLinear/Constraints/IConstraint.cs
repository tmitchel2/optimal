/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Constraints
{
    /// <summary>
    /// Base interface for optimization constraints.
    /// </summary>
    public interface IConstraint
    {
        /// <summary>
        /// Evaluates the constraint function and its gradient at the given point.
        /// </summary>
        /// <param name="x">Parameter vector.</param>
        /// <returns>Tuple of (constraint value, gradient).</returns>
        (double value, double[] gradient) Evaluate(double[] x);

        /// <summary>
        /// Gets the type of constraint.
        /// </summary>
        ConstraintType Type { get; }
    }

    /// <summary>
    /// Type of constraint.
    /// </summary>
    public enum ConstraintType
    {
        /// <summary>
        /// Equality constraint: h(x) = 0
        /// </summary>
        Equality,

        /// <summary>
        /// Inequality constraint: g(x) ≤ 0
        /// </summary>
        Inequality
    }
}
