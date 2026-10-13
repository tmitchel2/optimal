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

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// One <c>[Jacobian(nameof(p))]</c> or <c>[Vjp(nameof(p), …)]</c> on a kernel: which inputs to differentiate, and
    /// the chunk size of the generated method.
    /// </summary>
    public sealed record DerivativeRequest(bool IsJacobian, ImmutableArray<KernelParameter> Wrt, int ChunkSize, Location Location)
    {
        public const string JacobianAttributeName = "Optimal.JacobianAttribute";

        public const string VjpAttributeName = "Optimal.VjpAttribute";

        /// <summary>The chunk size when none is given: small enough that the JIT optimises every chunk.</summary>
        public const int DefaultChunkSize = 500;

        public static bool IsDerivativeAttribute(AttributeData attribute)
        {
            var name = attribute.AttributeClass?.ToDisplayString();
            return name is JacobianAttributeName or VjpAttributeName;
        }

        /// <summary>Reads and checks an attribute against the kernel's parameters.</summary>
        public static DerivativeRequest From(AttributeData attribute, ImmutableArray<KernelParameter> parameters, Location fallback)
        {
            var location = attribute.ApplicationSyntaxReference?.GetSyntax().GetLocation() ?? fallback;
            var isJacobian = attribute.AttributeClass?.ToDisplayString() == JacobianAttributeName;
            var argument = attribute.ConstructorArguments.FirstOrDefault();
            var names = argument.Kind == TypedConstantKind.Array
                ? argument.Values.Select(v => v.Value as string).ToImmutableArray()
                : ImmutableArray.Create(argument.Value as string);
            if (names.IsEmpty || names.Distinct().Count() != names.Length)
            {
                throw Invalid(location, "name each input to differentiate once");
            }

            var wrt = names.Select(n => parameters.FirstOrDefault(p => p.Name == n && !p.IsOutput)
                    ?? throw Invalid(location, $"'{n}' is not an input of the kernel; differentiate with respect to a double or ReadOnlySpan<double> parameter"))
                .ToImmutableArray();
            var chunkSize = attribute.NamedArguments
                .Where(a => a.Key == "ChunkSize")
                .Select(a => (int)a.Value.Value!)
                .DefaultIfEmpty(DefaultChunkSize)
                .First();
            return chunkSize > 0
                ? new DerivativeRequest(isJacobian, wrt, chunkSize, location)
                : throw Invalid(location, $"ChunkSize is {chunkSize}; a chunk holds at least one operation");
        }

        /// <summary>The generated method's name, such as <c>ResidualJacobian_u</c> or <c>ResidualVjp_X_E</c>.</summary>
        public string MethodName(string kernelName)
        {
            return $"{kernelName}{(IsJacobian ? "Jacobian" : "Vjp")}_{string.Join("_", Wrt.Select(w => w.Name))}";
        }

        private static KernelException Invalid(Location location, string detail)
        {
            return new KernelException(KernelDiagnostics.InvalidDerivativeAttribute, location, detail);
        }
    }
}
