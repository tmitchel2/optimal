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
    /// Evaluates a block of constraints g_i(x) &lt;= 0 and, unless <paramref name="jacobian"/> is empty, their Jacobian
    /// in the order of the block's <see cref="ConstraintSparsity"/>.
    /// </summary>
    /// <param name="x">The design variables.</param>
    /// <param name="values">Receives one value per constraint.</param>
    /// <param name="jacobian">Receives the Jacobian's stored values, or is empty when only values are wanted.</param>
    public delegate void ConstraintBlockEvaluator(ReadOnlySpan<double> x, Span<double> values, Span<double> jacobian);
}
