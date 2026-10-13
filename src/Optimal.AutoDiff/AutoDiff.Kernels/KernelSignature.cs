/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using System.Linq;
using System.Text.RegularExpressions;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Checks a kernel's signature and describes its parameters: a static void method whose inputs are doubles and
    /// <c>[FixedLength(n)] ReadOnlySpan&lt;double&gt;</c>s, with exactly one <c>[FixedLength(n)] Span&lt;double&gt;</c>
    /// output.
    /// </summary>
    public static class KernelSignature
    {
        // Names the emitter gives its locals: v{id}, x{group}_{index} and the spill buffer.
        private static readonly Regex ReservedName = new(@"^([vx]\d+(_\d+)?|spill)$", RegexOptions.CultureInvariant);

        public static ImmutableArray<KernelParameter> Analyze(IMethodSymbol method, MethodDeclarationSyntax syntax)
        {
            if (!method.IsStatic || !method.ReturnsVoid || method.IsGenericMethod)
            {
                throw Invalid(syntax.Identifier.GetLocation(), "a kernel is a static, non-generic method returning void that writes its result to one Span<double> output");
            }

            var parameters = method.Parameters.Select(p => Describe(p, syntax.ParameterList.Parameters[p.Ordinal])).ToImmutableArray();
            var outputs = parameters.Count(p => p.IsOutput);
            return outputs == 1
                ? parameters
                : throw Invalid(syntax.ParameterList.GetLocation(), $"a kernel has exactly one Span<double> output, and this one has {outputs}");
        }

        private static KernelParameter Describe(IParameterSymbol parameter, ParameterSyntax syntax)
        {
            var location = syntax.GetLocation();
            if (ReservedName.IsMatch(parameter.Name))
            {
                throw Invalid(location, $"the parameter name '{parameter.Name}' is reserved for generated locals; rename it");
            }

            if (parameter.RefKind != RefKind.None)
            {
                throw Invalid(location, $"parameter '{parameter.Name}' is passed by {parameter.RefKind.ToString().ToLowerInvariant()}; a kernel takes its inputs by value");
            }

            if (parameter.Type.SpecialType == SpecialType.System_Double)
            {
                return new KernelParameter(parameter.Name, 1, IsArray: false, IsOutput: false, parameter.Ordinal);
            }

            if (!KernelASTToIRConverter.IsSpan(parameter.Type, out var element, out var isWritable) || element.SpecialType != SpecialType.System_Double)
            {
                throw Invalid(location, $"parameter '{parameter.Name}' has type {parameter.Type.ToDisplayString()}; a kernel takes double and ReadOnlySpan<double> inputs and one Span<double> output");
            }

            var length = parameter.GetAttributes()
                .Where(a => a.AttributeClass?.ToDisplayString() == "Optimal.FixedLengthAttribute")
                .Select(a => (int?)a.ConstructorArguments[0].Value)
                .FirstOrDefault();
            return length is > 0
                ? new KernelParameter(parameter.Name, length.Value, IsArray: true, isWritable, parameter.Ordinal)
                : throw Invalid(location, $"span parameter '{parameter.Name}' needs [FixedLength(n)] with n > 0, so its derivatives have a known size");
        }

        private static KernelException Invalid(Location location, string detail)
        {
            return new KernelException(KernelDiagnostics.InvalidSignature, location, detail);
        }
    }
}
