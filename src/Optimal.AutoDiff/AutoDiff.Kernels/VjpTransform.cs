/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Immutable;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Reverse mode as a graph transform, with no tape at run time. Adjoints are graph nodes accumulated in reverse id
    /// order (a reverse topological order), seeded from λ, and flow only into active nodes: those that depend on a
    /// differentiated input. The adjoint sweep is shared by every parameter differentiated in one call.
    /// </summary>
    /// <remarks>
    /// A piecewise operation (min, max, a select) sends its adjoint to the operand it chose and an exact zero to the
    /// other, as a select. The branch not taken may hold a NaN or an infinity (the plastic branch of a return mapping at
    /// zero deviatoric stress, say), and 0 × NaN is NaN, so the zero must never be multiplied by that branch's partials.
    /// An adjoint that can be such a zero is <see cref="AdjointZeros.CanBeZero">zeroable</see>, and its products are
    /// guarded: a select with a zero arm is distributed over (select(c, a, 0) × p = select(c, a × p, 0)), and any other
    /// zeroable adjoint is multiplied only where it is non-zero. Adjoints of one condition are summed arm by arm
    /// (select(c, a, b) + select(c, d, e) = select(c, a + d, b + e)), which keeps them in that form. Every rewrite gives
    /// the same bits as the product or sum it replaces wherever that is finite.
    /// </remarks>
    public static class VjpTransform
    {
        // How deep a guarded product distributes over nested selects, and a sum merges them, before it falls back to a
        // compare (a product) or a plain sum. It bounds the code a long chain of conditions can generate.
        private const int MaxDistributionDepth = 8;

        /// <summary>
        /// λᵀ ∂output/∂p for each p in <paramref name="wrts"/>, with λ read from input group
        /// <paramref name="lambdaGroup"/>. Gradient k of parameter p is element k of the p-th result.
        /// </summary>
        public static ImmutableArray<ImmutableArray<int>> Apply(UnrolledKernel kernel, ImmutableArray<KernelParameter> wrts, int lambdaGroup)
        {
            return Apply(kernel, wrts, i => kernel.Graph.Input(lambdaGroup, i));
        }

        /// <summary>
        /// λᵀ ∂output/∂p for each p in <paramref name="wrts"/>, with λ_i the node <paramref name="seed"/>(i): an input for
        /// a VJP, or the constant 1 for the gradient of a kernel with one output.
        /// </summary>
        public static ImmutableArray<ImmutableArray<int>> Apply(UnrolledKernel kernel, ImmutableArray<KernelParameter> wrts, Func<int, int> seed)
        {
            var g = kernel.Graph;
            var zero = g.Const(0);
            var groups = wrts.Select(w => w.Group).ToImmutableHashSet();
            var active = DerivativeFlow.Active(kernel, groups.Contains);
            var zeros = new AdjointZeros(g);
            var adjoints = new int[kernel.PrimalCount];
            for (var id = 0; id < adjoints.Length; id++)
            {
                adjoints[id] = -1;
            }

            void Accumulate(int target, int contribution)
            {
                if (active[target])
                {
                    adjoints[target] = adjoints[target] < 0 ? contribution : Sum(g, adjoints[target], contribution, 0);
                }
            }

            for (var i = 0; i < kernel.OutputIds.Length; i++)
            {
                Accumulate(kernel.OutputIds[i], seed(i));
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
                        Accumulate(operands[slot], Product(g, zeros, bar, LocalDerivatives.Partial(g, id, slot), 0));
                    }
                }
            }

            return wrts
                .Select(w => Enumerable.Range(0, w.Length)
                    .Select(k => g.TryGetInput(w.Group, k, out var input) && input < adjoints.Length && adjoints[input] >= 0 ? adjoints[input] : zero)
                    .ToImmutableArray())
                .ToImmutableArray();
        }

        // bar × partial, never multiplying an adjoint that is zero because a branch was not taken.
        private static int Product(ScalarGraph g, AdjointZeros zeros, int bar, int partial, int depth)
        {
            if (g.IsConst(bar, 0))
            {
                return bar;
            }

            if (g.IsConst(partial, out _) || !zeros.CanBeZero(bar))
            {
                return g.Mul(bar, partial);
            }

            var node = g[bar];
            if (ScalarOps.IsSelect(node.Op) && (g.IsConst(node.C, 0) || g.IsConst(node.D, 0)) && depth < MaxDistributionDepth)
            {
                return g.Select(node.Op, node.A, node.B, Product(g, zeros, node.C, partial, depth + 1), Product(g, zeros, node.D, partial, depth + 1));
            }

            var zero = g.Const(0);
            return g.Select(ScalarOp.SelectEqual, bar, zero, zero, g.Mul(bar, partial));
        }

        // a + b, summing two selects of one condition arm by arm.
        private static int Sum(ScalarGraph g, int a, int b, int depth)
        {
            var left = g[a];
            var right = g[b];
            var sameCondition = ScalarOps.IsSelect(left.Op) && left.Op == right.Op && left.A == right.A && left.B == right.B;
            return sameCondition && depth < MaxDistributionDepth
                ? g.Select(left.Op, left.A, left.B, Sum(g, left.C, right.C, depth + 1), Sum(g, left.D, right.D, depth + 1))
                : g.Add(a, b);
        }
    }
}
