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
    /// <summary>
    /// A kernel flattened to SSA: <see cref="OutputIds"/>[i] is the graph node written to the output's element i.
    /// Nodes below <see cref="PrimalCount"/> are the kernel's own; derivative transforms append theirs above it, and
    /// several transforms may share one graph.
    /// </summary>
    public sealed record UnrolledKernel(
        ScalarGraph Graph,
        ImmutableArray<KernelParameter> Parameters,
        KernelParameter Output,
        ImmutableArray<int> OutputIds,
        int PrimalCount);
}
