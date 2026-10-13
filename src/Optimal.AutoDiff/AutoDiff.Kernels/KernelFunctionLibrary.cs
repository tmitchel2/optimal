/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Generic;
using System.Linq;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The kernels and helpers of one compilation as statement IR, each converted once. It decides what a kernel may
    /// inline: a static, non-generic method with a body in this compilation, declared in an <c>[OptimalCode]</c> class
    /// or in the generator's own <c>Optimal.KernelMath</c>.
    /// </summary>
    public sealed class KernelFunctionLibrary
    {
        /// <summary>The generated class of smooth helpers every compilation receives.</summary>
        public const string KernelMathTypeName = "Optimal.KernelMath";

        private readonly KernelASTToIRConverter _converter;
        private readonly Dictionary<IMethodSymbol, KernelFunctionNode> _functions = new(SymbolEqualityComparer.Default);

        public KernelFunctionLibrary(Compilation compilation)
        {
            _converter = new KernelASTToIRConverter(compilation);
        }

        /// <summary>The IR of a kernel method.</summary>
        public KernelFunctionNode Kernel(IMethodSymbol method, MethodDeclarationSyntax syntax)
        {
            return Function(method, syntax);
        }

        /// <summary>The IR of a helper called at <paramref name="call"/>, if the helper may be inlined.</summary>
        public KernelFunctionNode Helper(IMethodSymbol method, MethodCallNode call)
        {
            var syntax = method.DeclaringSyntaxReferences.Select(r => r.GetSyntax()).OfType<MethodDeclarationSyntax>().FirstOrDefault();
            var inlinable = method.IsStatic
                && method.MethodKind == MethodKind.Ordinary
                && !method.IsGenericMethod
                && syntax != null
                && (IsOptimalCode(method.ContainingType) || method.ContainingType.ToDisplayString() == KernelMathTypeName);
            return inlinable
                ? Function(method, syntax!)
                : throw new KernelException(
                    KernelDiagnostics.UnsupportedCall,
                    call,
                    $"'{method.ContainingType.ToDisplayString()}.{method.Name}' cannot be inlined; a kernel calls System.Math functions, span methods, Optimal.KernelMath, and static non-generic methods with source in an [OptimalCode] class");
        }

        /// <summary>The source location an IR node was converted from, if it was converted by this library.</summary>
        public Location? LocationOf(IRNode node)
        {
            return _converter.LocationOf(node);
        }

        private static bool IsOptimalCode(INamedTypeSymbol type)
        {
            return type.GetAttributes().Any(a => a.AttributeClass?.ToDisplayString() == "Optimal.OptimalCodeAttribute");
        }

        private KernelFunctionNode Function(IMethodSymbol method, MethodDeclarationSyntax syntax)
        {
            if (!_functions.TryGetValue(method, out var function))
            {
                function = _converter.ConvertFunction(method, syntax);
                _functions[method] = function;
            }

            return function;
        }
    }
}
