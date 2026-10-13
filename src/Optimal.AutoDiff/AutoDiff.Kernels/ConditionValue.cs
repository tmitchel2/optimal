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
    /// A run-time comparison <see cref="Left"/> ⋚ <see cref="Right"/> by the select <see cref="Op"/>, inverted when
    /// <see cref="Negated"/>. It can only choose between the branches of a conditional expression.
    /// </summary>
    public sealed record ConditionValue(ScalarOp Op, int Left, int Right, bool Negated) : KernelValue;
}
