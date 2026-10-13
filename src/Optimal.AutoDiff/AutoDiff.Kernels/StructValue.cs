/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// A value of a small struct, flattened: one value per instance field of <see cref="Type"/>, in the order of
    /// <see cref="KernelStructs.Fields"/>. A field is a <see cref="ScalarValue"/> (a double), another
    /// <see cref="StructValue"/>, or null while unassigned. The value is immutable: a field write makes a new one.
    /// </summary>
    public sealed record StructValue(INamedTypeSymbol Type, ImmutableArray<KernelValue?> Fields) : KernelValue
    {
        /// <summary>This value with field <paramref name="index"/> replaced.</summary>
        public StructValue With(int index, KernelValue? value)
        {
            return this with { Fields = Fields.SetItem(index, value) };
        }
    }
}
