/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>A <c>double</c>: node <see cref="Id"/> of the graph.</summary>
    public sealed record ScalarValue(int Id) : KernelValue;
}
