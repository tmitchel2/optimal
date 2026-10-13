/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.CodeAnalysis;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Why a kernel cannot be generated. It carries its diagnostic and where to report it: a source
    /// <see cref="Location"/> when it was raised from syntax, or the IR <see cref="Node"/> it was raised at, which the
    /// <see cref="KernelFunctionLibrary"/> maps back to source.
    /// </summary>
    public sealed class KernelException : Exception
    {
        public KernelException(DiagnosticDescriptor descriptor, Location location, string detail)
            : base(detail)
        {
            Descriptor = descriptor;
            Location = location;
        }

        public KernelException(DiagnosticDescriptor descriptor, IRNode node, string detail)
            : base(detail)
        {
            Descriptor = descriptor;
            Node = node;
        }

        public DiagnosticDescriptor Descriptor { get; }

        public Location? Location { get; }

        public IRNode? Node { get; }
    }
}
