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
    /// One <c>[Jacobian(nameof(p))]</c>, <c>[Vjp(nameof(p), …)]</c>, <c>[Hessian(nameof(p))]</c> or
    /// <c>[Hvp(nameof(p))]</c> on a kernel: which inputs to differentiate, and the chunk size of the generated method.
    /// </summary>
    public sealed record DerivativeRequest(DerivativeKind Kind, ImmutableArray<KernelParameter> Wrt, int ChunkSize, Location Location)
    {
        public const string JacobianAttributeName = "Optimal.JacobianAttribute";

        public const string VjpAttributeName = "Optimal.VjpAttribute";

        public const string HessianAttributeName = "Optimal.HessianAttribute";

        public const string HvpAttributeName = "Optimal.HvpAttribute";

        /// <summary>The chunk size when none is given: small enough that the JIT optimises every chunk.</summary>
        public const int DefaultChunkSize = 500;

        /// <summary>Whether the request takes second derivatives.</summary>
        public bool IsSecondOrder => Kind is DerivativeKind.Hessian or DerivativeKind.Hvp;

        public static bool IsDerivativeAttribute(AttributeData attribute)
        {
            return KindOf(attribute) != null;
        }

        /// <summary>Reads and checks an attribute against the kernel's parameters.</summary>
        public static DerivativeRequest From(AttributeData attribute, ImmutableArray<KernelParameter> parameters, Location fallback)
        {
            var location = attribute.ApplicationSyntaxReference?.GetSyntax().GetLocation() ?? fallback;
            var kind = KindOf(attribute)!.Value;
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
            var output = parameters.Single(p => p.IsOutput);
            if (kind is DerivativeKind.Hessian or DerivativeKind.Hvp && output.Length != 1)
            {
                throw Invalid(location, $"a [{kind}] differentiates a kernel with one output (an energy), and '{output.Name}' has {output.Length}");
            }

            var chunkSize = attribute.NamedArguments
                .Where(a => a.Key == "ChunkSize")
                .Select(a => (int)a.Value.Value!)
                .DefaultIfEmpty(DefaultChunkSize)
                .First();
            return chunkSize > 0
                ? new DerivativeRequest(kind, wrt, chunkSize, location)
                : throw Invalid(location, $"ChunkSize is {chunkSize}; a chunk holds at least one operation");
        }

        /// <summary>The generated method's name, such as <c>ResidualJacobian_u</c>, <c>ResidualVjp_X_E</c> or <c>EnergyHessian_F</c>.</summary>
        public string MethodName(string kernelName)
        {
            return $"{kernelName}{Kind}_{string.Join("_", Wrt.Select(w => w.Name))}";
        }

        private static DerivativeKind? KindOf(AttributeData attribute)
        {
            return attribute.AttributeClass?.ToDisplayString() switch
            {
                JacobianAttributeName => DerivativeKind.Jacobian,
                VjpAttributeName => DerivativeKind.Vjp,
                HessianAttributeName => DerivativeKind.Hessian,
                HvpAttributeName => DerivativeKind.Hvp,
                _ => null
            };
        }

        private static KernelException Invalid(Location location, string detail)
        {
            return new KernelException(KernelDiagnostics.InvalidDerivativeAttribute, location, detail);
        }
    }
}
