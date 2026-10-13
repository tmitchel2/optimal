/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear
{
    /// <summary>
    /// minimize f(x) subject to g_i(x) &lt;= 0 (i = 1..m) and lower_j &lt;= x_j &lt;= upper_j: the problem structural
    /// optimisation poses, with n design variables under simple bounds and m inequality constraints grouped in blocks.
    /// </summary>
    /// <remarks>
    /// At each point the optimisers evaluate the objective first and then every block in order, at the same x. A block
    /// may therefore reuse state the objective computed at that point (a finite element solve, say).
    /// </remarks>
    public sealed record InequalityProblem
    {
        /// <summary>
        /// Gets the objective f.
        /// </summary>
        public required ObjectiveEvaluator Objective { get; init; }

        /// <summary>
        /// Gets the constraint blocks; their rows, in order, are the constraints g_1..g_m.
        /// </summary>
        public required IReadOnlyList<IConstraintBlock> Constraints { get; init; }

        /// <summary>
        /// Gets the lower bounds (finite).
        /// </summary>
        public required double[] LowerBounds { get; init; }

        /// <summary>
        /// Gets the upper bounds (finite, and no less than the lower bounds).
        /// </summary>
        public required double[] UpperBounds { get; init; }

        /// <summary>
        /// Gets the number of design variables n.
        /// </summary>
        public int VariableCount => LowerBounds.Length;

        /// <summary>
        /// Gets the number of constraints m, summed over the blocks.
        /// </summary>
        public int ConstraintCount => Constraints.Sum(block => block.Sparsity.RowCount);

        /// <summary>
        /// Throws if the problem is malformed: bounds of different lengths, not finite or crossed, or a block whose
        /// column count is not n.
        /// </summary>
        /// <param name="initialPoint">The starting point, which must have n entries.</param>
        public void Validate(ReadOnlySpan<double> initialPoint)
        {
            ArgumentNullException.ThrowIfNull(Objective);
            ArgumentNullException.ThrowIfNull(Constraints);
            ArgumentNullException.ThrowIfNull(LowerBounds);
            ArgumentNullException.ThrowIfNull(UpperBounds);
            var n = LowerBounds.Length;
            if (n == 0 || UpperBounds.Length != n || initialPoint.Length != n)
            {
                throw new ArgumentException(
                    $"Bounds and the initial point must have the same, nonzero length (lower {n}, upper {UpperBounds.Length}, initial {initialPoint.Length}).");
            }

            for (var j = 0; j < n; j++)
            {
                if (!double.IsFinite(LowerBounds[j]) || !double.IsFinite(UpperBounds[j]) || LowerBounds[j] > UpperBounds[j])
                {
                    throw new ArgumentException(
                        $"Variable {j}: bounds must be finite with lower <= upper (got [{LowerBounds[j]}, {UpperBounds[j]}]).");
                }

                if (!double.IsFinite(initialPoint[j]))
                {
                    throw new ArgumentException($"Variable {j}: the initial point is not finite ({initialPoint[j]}).");
                }
            }

            for (var b = 0; b < Constraints.Count; b++)
            {
                var sparsity = Constraints[b].Sparsity;
                if (sparsity.ColumnCount != n)
                {
                    throw new ArgumentException($"Constraint block {b} has {sparsity.ColumnCount} columns; the problem has {n} variables.");
                }
            }
        }
    }
}
