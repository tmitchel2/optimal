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
    /// Vector forward mode as a graph transform. Every primal node that is both active and live gets a tangent: one
    /// graph node per element of the differentiated parameter, carried at once. Tangents are built through the
    /// graph's builders, so the zeros of a sparse tangent fold away and shared terms are built once.
    /// </summary>
    public static class JacobianTransform
    {
        /// <summary>
        /// The Jacobian ∂output/∂<paramref name="wrt"/> as graph nodes, row-major: element (i, j) is at
        /// <c>i × wrt.Length + j</c>.
        /// </summary>
        public static ImmutableArray<int> Apply(UnrolledKernel kernel, KernelParameter wrt)
        {
            var g = kernel.Graph;
            var n = wrt.Length;
            var zero = g.Const(0);
            var active = DerivativeFlow.Active(kernel, group => group == wrt.Group);
            var live = DerivativeFlow.Live(kernel);
            var tangents = new int[]?[kernel.PrimalCount];
            for (var id = 0; id < tangents.Length; id++)
            {
                tangents[id] = active[id] && live[id] ? Tangent(g, id, tangents, n, zero) : null;
            }

            return kernel.OutputIds
                .SelectMany(id => tangents[id] ?? Enumerable.Repeat(zero, n).ToArray())
                .ToImmutableArray();
        }

        private static int[]? Tangent(ScalarGraph g, int id, int[]?[] tangents, int n, int zero)
        {
            var node = g[id];
            if (node.Op == ScalarOp.Input)
            {
                return Enumerable.Range(0, n).Select(j => j == node.B ? g.Const(1) : zero).ToArray();
            }

            return ScalarOps.IsPiecewise(node.Op)
                ? PiecewiseTangent(g, LocalDerivatives.Choice(g, id), tangents, n, zero)
                : SmoothTangent(g, id, tangents);
        }

        // ṫ = Σ_k (∂node/∂operand_k) ṫ_k over the operands that carry a tangent.
        private static int[]? SmoothTangent(ScalarGraph g, int id, int[]?[] tangents)
        {
            var operands = g[id].Operands.ToArray();
            int[]? sum = null;
            for (var slot = 0; slot < operands.Length; slot++)
            {
                var operandTangent = tangents[operands[slot]];
                if (operandTangent == null)
                {
                    continue;
                }

                var partial = LocalDerivatives.Partial(g, id, slot);
                var term = operandTangent.Select(t => g.Mul(partial, t)).ToArray();
                sum = sum == null ? term : sum.Zip(term, g.Add).ToArray();
            }

            return sum;
        }

        // A select of the operands' tangents, so the branch not taken contributes nothing, not even a NaN.
        private static int[]? PiecewiseTangent(ScalarGraph g, PiecewiseChoice choice, int[]?[] tangents, int n, int zero)
        {
            var whenTrue = tangents[choice.WhenTrue];
            var whenFalse = tangents[choice.WhenFalse];
            if (whenTrue == null && whenFalse == null)
            {
                return null;
            }

            return Enumerable.Range(0, n)
                .Select(j => g.Select(choice.Op, choice.Left, choice.Right, whenTrue?[j] ?? zero, whenFalse?[j] ?? zero))
                .ToArray();
        }
    }
}
