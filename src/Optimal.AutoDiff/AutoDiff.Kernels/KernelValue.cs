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
    /// A value as the <see cref="LoopUnroller"/> interprets a kernel. Integers and booleans are compile-time values
    /// (<see cref="IntValue"/>, <see cref="BoolValue"/>); doubles are graph nodes (<see cref="ScalarValue"/>); a
    /// comparison of doubles not decided at compile time is a <see cref="ConditionValue"/>; a span is an
    /// <see cref="ArrayValue"/>.
    /// </summary>
    public abstract record KernelValue;
}
