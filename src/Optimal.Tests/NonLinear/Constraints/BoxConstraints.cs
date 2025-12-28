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
    /// Box constraints: lower ≤ x ≤ upper for each variable.
    /// Implemented as inequality constraints.
    /// </summary>
    public sealed class BoxConstraints
    {
        private readonly double[] _lower;
        private readonly double[] _upper;

        /// <summary>
        /// Initializes a new instance of the <see cref="BoxConstraints"/> class.
        /// </summary>
        /// <param name="lower">Lower bounds for each variable (use double.NegativeInfinity for unbounded).</param>
        /// <param name="upper">Upper bounds for each variable (use double.PositiveInfinity for unbounded).</param>
        public BoxConstraints(double[] lower, double[] upper)
        {
            if (lower.Length != upper.Length)
            {
                throw new ArgumentException("Lower and upper bounds must have same length");
            }

            _lower = (double[])lower.Clone();
            _upper = (double[])upper.Clone();
        }

        /// <summary>
        /// Projects a point onto the feasible region defined by the box constraints.
        /// </summary>
        /// <param name="x">Point to project.</param>
        /// <returns>Projected point satisfying the box constraints.</returns>
        public double[] Project(double[] x)
        {
            var projected = new double[x.Length];
            for (var i = 0; i < x.Length; i++)
            {
                projected[i] = Math.Max(_lower[i], Math.Min(_upper[i], x[i]));
            }
            return projected;
        }

        /// <summary>
        /// Checks if a point satisfies the box constraints within tolerance.
        /// </summary>
        /// <param name="x">Point to check.</param>
        /// <param name="tolerance">Tolerance for constraint violation.</param>
        /// <returns>True if point is feasible.</returns>
        public bool IsFeasible(double[] x, double tolerance = 1e-10)
        {
            for (var i = 0; i < x.Length; i++)
            {
                if (x[i] < _lower[i] - tolerance || x[i] > _upper[i] + tolerance)
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Computes the maximum constraint violation.
        /// </summary>
        /// <param name="x">Point to check.</param>
        /// <returns>Maximum violation (0 if feasible).</returns>
        public double MaxViolation(double[] x)
        {
            var maxViolation = 0.0;
            for (var i = 0; i < x.Length; i++)
            {
                var lowerViolation = Math.Max(0, _lower[i] - x[i]);
                var upperViolation = Math.Max(0, x[i] - _upper[i]);
                maxViolation = Math.Max(maxViolation, Math.Max(lowerViolation, upperViolation));
            }
            return maxViolation;
        }

        /// <summary>
        /// Gets the lower bounds.
        /// </summary>
        public double[] Lower => (double[])_lower.Clone();

        /// <summary>
        /// Gets the upper bounds.
        /// </summary>
        public double[] Upper => (double[])_upper.Clone();
    }
}
