/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>One in-process generator run: its diagnostics, its sources by hint name, and the compilation with them.</summary>
    public sealed record GeneratorRun(
        ImmutableArray<Diagnostic> Diagnostics,
        ImmutableDictionary<string, string> Sources,
        Compilation Output);
}
