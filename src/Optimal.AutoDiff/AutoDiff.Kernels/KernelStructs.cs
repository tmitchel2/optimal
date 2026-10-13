/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using System.Linq;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The structs a kernel can hold as values, flattened to their fields: a non-generic, non-ref struct with source in
    /// the compilation, whose instance state is fields of type <c>double</c> or of another such struct (a 3 × 3 tensor,
    /// say). Auto-properties and primary-constructor parameters hold hidden state the unroller cannot see, so they are
    /// not allowed; computed properties, methods, constructors and operators are, and are inlined.
    /// </summary>
    public static class KernelStructs
    {
        /// <summary>The deepest structs may nest inside one another.</summary>
        public const int MaxNesting = 16;

        /// <summary>Whether <paramref name="type"/> is a struct (other than a span or a built-in type) a kernel may try to flatten.</summary>
        public static bool IsStruct(ITypeSymbol type)
        {
            return type is INamedTypeSymbol { TypeKind: TypeKind.Struct, SpecialType: SpecialType.None, IsRefLikeType: false } named
                && !KernelASTToIRConverter.IsSpan(named, out _, out _);
        }

        /// <summary>Whether a kernel can hold <paramref name="type"/> as a flattened value.</summary>
        public static bool IsFlattenable(ITypeSymbol type)
        {
            return IsStruct(type) && Problem(type) == null;
        }

        /// <summary>Why a kernel cannot flatten <paramref name="type"/>, or null if it can.</summary>
        public static string? Problem(ITypeSymbol type)
        {
            return Problem(type, 0);
        }

        /// <summary>The instance fields of <paramref name="type"/>, in declaration order: the layout of its <see cref="StructValue"/>.</summary>
        public static ImmutableArray<IFieldSymbol> Fields(INamedTypeSymbol type)
        {
            return type.GetMembers().OfType<IFieldSymbol>().Where(f => !f.IsStatic && !f.IsConst).ToImmutableArray();
        }

        /// <summary>The position of <paramref name="field"/> in its struct's layout.</summary>
        public static int IndexOf(INamedTypeSymbol type, IFieldSymbol field)
        {
            return Fields(type).IndexOf(field, SymbolEqualityComparer.Default);
        }

        /// <summary>A value of <paramref name="type"/> with every field zero: <c>default</c> and <c>new T()</c>.</summary>
        public static StructValue Zero(INamedTypeSymbol type, ScalarGraph g)
        {
            return new StructValue(type, Fields(type)
                .Select(f => f.Type.SpecialType == SpecialType.System_Double ? new ScalarValue(g.Const(0)) : (KernelValue)Zero((INamedTypeSymbol)f.Type, g))
                .ToImmutableArray<KernelValue?>());
        }

        /// <summary>A value of <paramref name="type"/> with every field unassigned, for a local declared without an initialiser.</summary>
        public static StructValue Unassigned(INamedTypeSymbol type)
        {
            return new StructValue(type, Fields(type).Select(_ => (KernelValue?)null).ToImmutableArray());
        }

        // A struct cannot contain itself, but source with that error still reaches the generator, so nesting is bounded.
        private static string? Problem(ITypeSymbol type, int depth)
        {
            if (type is not INamedTypeSymbol named || !IsStruct(type))
            {
                return $"{type.ToDisplayString()} is not a struct";
            }

            if (depth > MaxNesting)
            {
                return $"{named.ToDisplayString()} nests structs more than {MaxNesting} deep";
            }

            if (named.IsGenericType)
            {
                return $"{named.ToDisplayString()} is generic";
            }

            var declarations = named.DeclaringSyntaxReferences.Select(r => r.GetSyntax()).OfType<TypeDeclarationSyntax>().ToList();
            if (declarations.Count == 0)
            {
                return $"{named.ToDisplayString()} has no source in this compilation";
            }

            if (declarations.Any(d => d.ParameterList != null))
            {
                return $"{named.ToDisplayString()} has a primary constructor, whose parameters are hidden state; declare fields";
            }

            var autoProperty = named.GetMembers().OfType<IPropertySymbol>().FirstOrDefault(p => !p.IsStatic && IsAutoProperty(p));
            if (autoProperty != null)
            {
                return $"{named.ToDisplayString()}.{autoProperty.Name} is an auto-property; a kernel struct keeps its state in double fields";
            }

            var fields = Fields(named);
            if (fields.IsEmpty)
            {
                return $"{named.ToDisplayString()} has no fields";
            }

            var bad = fields.FirstOrDefault(f => f.Type.SpecialType != SpecialType.System_Double && Problem(f.Type, depth + 1) != null);
            return bad == null
                ? null
                : $"{named.ToDisplayString()}.{bad.Name} has type {bad.Type.ToDisplayString()}; a kernel struct's fields are double or another such struct";
        }

        private static bool IsAutoProperty(IPropertySymbol property)
        {
            return property.DeclaringSyntaxReferences
                .Select(r => r.GetSyntax())
                .OfType<PropertyDeclarationSyntax>()
                .Any(p => p.ExpressionBody == null && p.AccessorList != null && p.AccessorList.Accessors.All(a => a.Body == null && a.ExpressionBody == null));
        }
    }
}
