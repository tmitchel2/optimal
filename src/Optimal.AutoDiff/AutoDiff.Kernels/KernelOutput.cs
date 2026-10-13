/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>A span parameter of a generated method, whose element k is written from graph node <see cref="Ids"/>[k].</summary>
    public sealed record KernelOutput(string Name, ImmutableArray<int> Ids);
}
