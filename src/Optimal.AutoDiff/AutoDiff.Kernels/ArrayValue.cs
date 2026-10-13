/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>A span: <see cref="Length"/> elements of <see cref="Storage"/> from <see cref="Offset"/>.</summary>
    public sealed record ArrayValue(KernelArray Storage, int Offset, int Length) : KernelValue;
}
