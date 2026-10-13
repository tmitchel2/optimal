/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// How a piecewise operation chooses the operand whose derivative it follows: <see cref="Left"/> ⋚
    /// <see cref="Right"/> (by the select <see cref="Op"/>) ? <see cref="WhenTrue"/> : <see cref="WhenFalse"/>.
    /// </summary>
    public sealed record PiecewiseChoice(ScalarOp Op, int Left, int Right, int WhenTrue, int WhenFalse);
}
