/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The storage behind a span the unroller tracks: an input, the output, a local array or a static table. Each
    /// element is a <see cref="ScalarValue"/> (a double) or an <see cref="IntValue"/> (an int), or null while an output
    /// element is unwritten. A rectangular table has <see cref="Columns"/> &gt; 0 and is indexed row-major.
    /// </summary>
    public sealed class KernelArray
    {
        private readonly KernelValue?[] _elements;

        public KernelArray(string name, bool isIntegral, int columns, bool isReadOnly, IEnumerable<KernelValue?> elements)
        {
            Name = name;
            IsIntegral = isIntegral;
            Columns = columns;
            IsReadOnly = isReadOnly;
            _elements = elements.ToArray();
            if (columns > 0 && _elements.Length % columns != 0)
            {
                throw new ArgumentException($"{_elements.Length} elements do not fill rows of {columns}.", nameof(elements));
            }
        }

        /// <summary>The name used in diagnostics: the parameter, local or table the array was created for.</summary>
        public string Name { get; }

        /// <summary>Whether the elements are <c>int</c>s; otherwise they are <c>double</c>s.</summary>
        public bool IsIntegral { get; }

        /// <summary>The row length of a rectangular table, or 0 for a one-dimensional array.</summary>
        public int Columns { get; }

        /// <summary>Whether the kernel may only read the array: an input span or a static table.</summary>
        public bool IsReadOnly { get; }

        public int Length => _elements.Length;

        public KernelValue? this[int index]
        {
            get => _elements[index];
            set => _elements[index] = IsReadOnly ? throw new InvalidOperationException($"'{Name}' is read-only.") : value;
        }

        /// <summary>The index of the first element never written, or −1 if every element holds a value.</summary>
        public int FirstUnwritten()
        {
            return Array.IndexOf(_elements, null);
        }
    }
}
