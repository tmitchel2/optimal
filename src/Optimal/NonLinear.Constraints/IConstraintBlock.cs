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
    /// A block of inequality constraints g_i(x) &lt;= 0 that are evaluated together, with a Jacobian whose structure
    /// (<see cref="Sparsity"/>) is fixed for the whole run: a few dense rows (a volume constraint, an aggregated stress
    /// or frequency constraint) or many sparse rows (local constraints).
    /// </summary>
    public interface IConstraintBlock
    {
        /// <summary>
        /// Gets the structure of the block's Jacobian. Its row count is the number of constraints in the block.
        /// </summary>
        ConstraintSparsity Sparsity { get; }

        /// <summary>
        /// Evaluates the constraints and, unless <paramref name="jacobian"/> is empty, their Jacobian.
        /// </summary>
        /// <param name="x">The design variables.</param>
        /// <param name="values">Receives g_i(x), one value per row; the constraint is satisfied when g_i(x) &lt;= 0.</param>
        /// <param name="jacobian">
        /// Receives ∂g_i/∂x_j in the order of <see cref="Sparsity"/>: row-major when dense, in CSR order when sparse.
        /// Empty when only the values are wanted (GCMMA's inner iterations), so a block can skip its adjoint solves.
        /// </param>
        void Evaluate(ReadOnlySpan<double> x, Span<double> values, Span<double> jacobian);
    }
}
