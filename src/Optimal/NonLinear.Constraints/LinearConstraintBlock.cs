/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Numerics.Tensors;

namespace Optimal.NonLinear.Constraints
{
    /// <summary>
    /// One linear constraint wᵀx - limit &lt;= 0, as a dense one-row block: a volume or mass constraint, with w the
    /// element volumes (or masses) and limit the allowed total.
    /// </summary>
    public sealed class LinearConstraintBlock : IConstraintBlock
    {
        private readonly double[] _weights;

        /// <summary>
        /// Initializes a new instance of the <see cref="LinearConstraintBlock"/> class.
        /// </summary>
        /// <param name="weights">The weights w (copied).</param>
        /// <param name="limit">The allowed total.</param>
        public LinearConstraintBlock(ReadOnlySpan<double> weights, double limit)
        {
            if (weights.IsEmpty)
            {
                throw new ArgumentException("A linear constraint needs at least one weight.", nameof(weights));
            }

            if (!double.IsFinite(limit))
            {
                throw new ArgumentOutOfRangeException(nameof(limit), limit, "The limit must be finite.");
            }

            _weights = weights.ToArray();
            Limit = limit;
            Sparsity = ConstraintSparsity.Dense(1, _weights.Length);
        }

        /// <inheritdoc/>
        public ConstraintSparsity Sparsity { get; }

        /// <summary>
        /// Gets the weights w.
        /// </summary>
        public ReadOnlySpan<double> Weights => _weights;

        /// <summary>
        /// Gets the allowed total.
        /// </summary>
        public double Limit { get; }

        /// <inheritdoc/>
        public void Evaluate(ReadOnlySpan<double> x, Span<double> values, Span<double> jacobian)
        {
            values[0] = TensorPrimitives.Dot(_weights, x) - Limit;
            if (!jacobian.IsEmpty)
            {
                _weights.CopyTo(jacobian);
            }
        }
    }
}
