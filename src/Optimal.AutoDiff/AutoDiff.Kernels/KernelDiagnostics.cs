/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.CodeAnalysis;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The errors the kernel generator reports. Each names the kernel and says what to change; none is a warning,
    /// because a kernel that cannot be differentiated exactly must not be differentiated at all.
    /// </summary>
    public static class KernelDiagnostics
    {
        public const string Category = "Optimal.AutoDiff";

        /// <summary>A loop bound, index, length or statement condition that is not known at compile time.</summary>
        public static readonly DiagnosticDescriptor ControlFlowNotConstant = Create(
            "OPTAD100",
            "Kernel control flow must be decided at compile time",
            "Kernel '{0}': {1}");

        /// <summary>A statement or expression form the kernel language does not have.</summary>
        public static readonly DiagnosticDescriptor UnsupportedSyntax = Create(
            "OPTAD101",
            "Kernel syntax is not supported",
            "Kernel '{0}': {1}");

        /// <summary>A kernel whose signature is not void, static, with double and fixed-length span inputs and one span output.</summary>
        public static readonly DiagnosticDescriptor InvalidSignature = Create(
            "OPTAD102",
            "Kernel signature is not supported",
            "Kernel '{0}': {1}");

        /// <summary>A call that is not a supported Math function, a span method or an inlinable helper.</summary>
        public static readonly DiagnosticDescriptor UnsupportedCall = Create(
            "OPTAD103",
            "Kernel call is not supported",
            "Kernel '{0}': {1}");

        /// <summary>A compile-time index or slice outside its span.</summary>
        public static readonly DiagnosticDescriptor IndexOutOfRange = Create(
            "OPTAD104",
            "Kernel index is out of range",
            "Kernel '{0}': {1}");

        /// <summary>An output element never written, or an element or local read before it is written.</summary>
        public static readonly DiagnosticDescriptor UnassignedValue = Create(
            "OPTAD105",
            "Kernel value is read or returned unassigned",
            "Kernel '{0}': {1}");

        /// <summary>Unrolling that exceeds the iteration, size or call-depth limit.</summary>
        public static readonly DiagnosticDescriptor UnrollLimit = Create(
            "OPTAD106",
            "Kernel exceeds an unrolling limit",
            "Kernel '{0}': {1}");

        /// <summary>A [Jacobian] or [Vjp] attribute that names no differentiable input, repeats another, or sets a bad chunk size.</summary>
        public static readonly DiagnosticDescriptor InvalidDerivativeAttribute = Create(
            "OPTAD107",
            "Kernel derivative attribute is invalid",
            "Kernel '{0}': {1}");

        /// <summary>A Math.Pow whose exponent is not a compile-time constant.</summary>
        public static readonly DiagnosticDescriptor NonConstantExponent = Create(
            "OPTAD108",
            "Kernel Math.Pow exponent must be constant",
            "Kernel '{0}': {1}");

        /// <summary>A write to a static table, which a kernel sees as read-only.</summary>
        public static readonly DiagnosticDescriptor ReadOnlyWrite = Create(
            "OPTAD109",
            "Kernel writes to a read-only table",
            "Kernel '{0}': {1}");

        private static DiagnosticDescriptor Create(string id, string title, string message)
        {
            return new DiagnosticDescriptor(id, title, message, Category, DiagnosticSeverity.Error, isEnabledByDefault: true);
        }
    }
}
