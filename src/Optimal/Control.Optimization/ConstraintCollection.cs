/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Optimal.NonLinear.Constraints;

namespace Optimal.Control.Optimization
{
    /// <summary>
    /// Collects constraints for building AugmentedLagrangianOptions.
    /// </summary>
    internal sealed class ConstraintCollection
    {
        private readonly List<Func<double[], (double value, double[] gradient)>> _equalityConstraints = new();
        private readonly List<Func<double[], (double value, double[] gradient)>> _inequalityConstraints = new();
        private BoxConstraints? _boxConstraints;

        /// <summary>
        /// Gets the collected equality constraints.
        /// </summary>
        public IReadOnlyList<Func<double[], (double value, double[] gradient)>> EqualityConstraints => _equalityConstraints;

        /// <summary>
        /// Gets the collected inequality constraints.
        /// </summary>
        public IReadOnlyList<Func<double[], (double value, double[] gradient)>> InequalityConstraints => _inequalityConstraints;

        /// <summary>
        /// Gets the box constraints.
        /// </summary>
        public BoxConstraints? BoxConstraints => _boxConstraints;

        /// <summary>
        /// Adds an equality constraint h(x) = 0.
        /// </summary>
        public void AddEqualityConstraint(Func<double[], (double value, double[] gradient)> constraint)
        {
            _equalityConstraints.Add(constraint);
        }

        /// <summary>
        /// Adds an inequality constraint g(x) &lt;= 0.
        /// </summary>
        public void AddInequalityConstraint(Func<double[], (double value, double[] gradient)> constraint)
        {
            _inequalityConstraints.Add(constraint);
        }

        /// <summary>
        /// Sets the box constraints.
        /// </summary>
        public void SetBoxConstraints(double[] lowerBounds, double[] upperBounds)
        {
            _boxConstraints = new BoxConstraints(lowerBounds, upperBounds);
        }
    }
}
