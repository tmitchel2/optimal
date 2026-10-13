/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Reverse mode as a graph transform, with no tape at run time. Adjoints are graph nodes accumulated in reverse id
    /// order (a reverse topological order), seeded from λ, and flow only into active nodes: those that depend on a
    /// differentiated input. The adjoint sweep is shared by every parameter differentiated in one call.
    /// </summary>
    public static class VjpTransform
    {
        /// <summary>
        /// λᵀ ∂output/∂p for each p in <paramref name="wrts"/>, with λ read from input group
        /// <paramref name="lambdaGroup"/>. Gradient k of parameter p is element k of the p-th result.
        /// </summary>
        public static ImmutableArray<ImmutableArray<int>> Apply(UnrolledKernel kernel, ImmutableArray<KernelParameter> wrts, int lambdaGroup)
        {
            var g = kernel.Graph;
            var zero = g.Const(0);
            var groups = wrts.Select(w => w.Group).ToImmutableHashSet();
            var active = DerivativeFlow.Active(kernel, groups.Contains);
            var adjoints = new int[kernel.PrimalCount];
            for (var id = 0; id < adjoints.Length; id++)
            {
                adjoints[id] = -1;
            }

            void Accumulate(int target, int contribution)
            {
                if (active[target])
                {
                    adjoints[target] = adjoints[target] < 0 ? contribution : g.Add(adjoints[target], contribution);
                }
            }

            for (var i = 0; i < kernel.OutputIds.Length; i++)
            {
                Accumulate(kernel.OutputIds[i], g.Input(lambdaGroup, i));
            }

            for (var id = adjoints.Length - 1; id >= 0; id--)
            {
                var bar = adjoints[id];
                var node = g[id];
                if (bar < 0 || node.Op == ScalarOp.Input)
                {
                    continue;
                }

                if (ScalarOps.IsPiecewise(node.Op))
                {
                    var choice = LocalDerivatives.Choice(g, id);
                    Accumulate(choice.WhenTrue, g.Select(choice.Op, choice.Left, choice.Right, bar, zero));
                    Accumulate(choice.WhenFalse, g.Select(choice.Op, choice.Left, choice.Right, zero, bar));
                    continue;
                }

                var operands = node.Operands.ToArray();
                for (var slot = 0; slot < operands.Length; slot++)
                {
                    if (active[operands[slot]])
                    {
                        Accumulate(operands[slot], g.Mul(bar, LocalDerivatives.Partial(g, id, slot)));
                    }
                }
            }

            return wrts
                .Select(w => Enumerable.Range(0, w.Length)
                    .Select(k => g.TryGetInput(w.Group, k, out var input) && input < adjoints.Length && adjoints[input] >= 0 ? adjoints[input] : zero)
                    .ToImmutableArray())
                .ToImmutableArray();
        }
    }
}
