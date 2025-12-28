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
    /// Equality constraint: h(x) = 0
    /// </summary>
    public sealed class EqualityConstraint : IConstraint
    {
        private readonly Func<double[], (double value, double[] gradient)> _constraint;

        /// <summary>
        /// Initializes a new instance of the <see cref="EqualityConstraint"/> class.
        /// </summary>
        /// <param name="constraint">
        /// Constraint function that returns (h(x), ∇h(x)).
        /// The constraint is satisfied when h(x) = 0.
        /// </param>
        public EqualityConstraint(Func<double[], (double value, double[] gradient)> constraint)
        {
            _constraint = constraint ?? throw new ArgumentNullException(nameof(constraint));
        }

        /// <inheritdoc/>
        public (double value, double[] gradient) Evaluate(double[] x)
        {
            return _constraint(x);
        }

        /// <inheritdoc/>
        public ConstraintType Type => ConstraintType.Equality;
    }
}
