/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.NonLinear.Constraints
{
    /// <summary>
    /// A constraint block defined by its Jacobian structure and an evaluator.
    /// </summary>
    public sealed class ConstraintBlock : IConstraintBlock
    {
        private readonly ConstraintBlockEvaluator _evaluate;

        /// <summary>
        /// Initializes a new instance of the <see cref="ConstraintBlock"/> class.
        /// </summary>
        /// <param name="sparsity">The Jacobian's structure (dense or CSR).</param>
        /// <param name="evaluate">Writes the values and, unless the span is empty, the Jacobian in the structure's order.</param>
        public ConstraintBlock(ConstraintSparsity sparsity, ConstraintBlockEvaluator evaluate)
        {
            Sparsity = sparsity ?? throw new ArgumentNullException(nameof(sparsity));
            _evaluate = evaluate ?? throw new ArgumentNullException(nameof(evaluate));
        }

        /// <inheritdoc/>
        public ConstraintSparsity Sparsity { get; }

        /// <summary>
        /// Creates a dense block from constraint functions of the shape the Augmented Lagrangian optimizer's options
        /// take: each returns (g(x), ∇g(x)) and is satisfied when g(x) &lt;= 0. Each evaluation copies x into an array,
        /// so this suits small problems; large ones should write a span evaluator.
        /// </summary>
        /// <param name="variableCount">The number of design variables.</param>
        /// <param name="constraints">The constraint functions, one row each.</param>
        /// <returns>The dense block.</returns>
        public static ConstraintBlock FromFunctions(
            int variableCount,
            IReadOnlyList<Func<double[], (double value, double[] gradient)>> constraints)
        {
            ArgumentNullException.ThrowIfNull(constraints);
            var sparsity = ConstraintSparsity.Dense(constraints.Count, variableCount);
            return new ConstraintBlock(sparsity, (x, values, jacobian) =>
            {
                var point = x.ToArray();
                for (var i = 0; i < constraints.Count; i++)
                {
                    var (value, gradient) = constraints[i](point);
                    values[i] = value;
                    if (!jacobian.IsEmpty)
                    {
                        CopyGradient(gradient, jacobian.Slice(i * variableCount, variableCount), i);
                    }
                }
            });
        }

        /// <inheritdoc/>
        public void Evaluate(ReadOnlySpan<double> x, Span<double> values, Span<double> jacobian)
        {
            _evaluate(x, values, jacobian);
        }

        private static void CopyGradient(double[] gradient, Span<double> row, int index)
        {
            if (gradient.Length != row.Length)
            {
                throw new InvalidOperationException(
                    $"Constraint {index} returned a gradient of length {gradient.Length}; expected {row.Length}.");
            }

            gradient.CopyTo(row);
        }
    }
}
