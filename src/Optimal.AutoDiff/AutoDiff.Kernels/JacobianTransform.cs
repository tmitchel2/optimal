/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Vector forward mode as a graph transform. Every node that is both active and live gets a tangent: one graph node
    /// per direction, carried at once. Tangents are built through the graph's builders, so the zeros of a sparse tangent
    /// fold away and shared terms are built once. The transform applies to any nodes of the graph, not only the
    /// kernel's own outputs: applied to a VJP's gradient it gives a Hessian (forward over reverse).
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
            return Apply(g, kernel.PrimalCount, kernel.OutputIds, wrt.Group, wrt.Length, index => Unit(g, index, wrt.Length));
        }

        /// <summary>
        /// The tangents of <paramref name="outputIds"/>, row-major (output i, direction j at <c>i × n + j</c>), over the
        /// first <paramref name="count"/> nodes, when element k of input group <paramref name="wrtGroup"/> has the
        /// <paramref name="n"/>-direction tangent <paramref name="seed"/>(k). Unit seeds give a Jacobian; one direction
        /// read from another input gives a Jacobian–vector product.
        /// </summary>
        public static ImmutableArray<int> Apply(ScalarGraph g, int count, IReadOnlyList<int> outputIds, int wrtGroup, int n, Func<int, int[]> seed)
        {
            var zero = g.Const(0);
            var active = DerivativeFlow.Active(g, count, group => group == wrtGroup);
            var live = DerivativeFlow.Live(g, count, outputIds);
            var tangents = Sweep(g, 0, count, id => active[id] && live[id], id => g[id].Op == ScalarOp.Input ? seed(g[id].B) : null, n, zero);
            return outputIds
                .SelectMany(id => tangents[id] ?? Enumerable.Repeat(zero, n).ToArray())
                .ToImmutableArray();
        }

        /// <summary>
        /// Tangents of the nodes from <paramref name="from"/> up to <paramref name="count"/> that <paramref name="include"/>
        /// admits: a node <paramref name="seedOf"/> gives a tangent for takes it, and any other is the chain rule over its
        /// operands. Nodes before <paramref name="from"/>, and nodes no seed reaches, have none (null).
        /// </summary>
        public static int[]?[] Sweep(ScalarGraph g, int from, int count, Func<int, bool> include, Func<int, int[]?> seedOf, int n, int zero)
        {
            var tangents = new int[]?[count];
            for (var id = from; id < count; id++)
            {
                if (include(id))
                {
                    tangents[id] = seedOf(id) ?? Tangent(g, id, tangents, n, zero);
                }
            }

            return tangents;
        }

        private static int[] Unit(ScalarGraph g, int index, int n)
        {
            var zero = g.Const(0);
            return Enumerable.Range(0, n).Select(j => j == index ? g.Const(1) : zero).ToArray();
        }

        private static int[]? Tangent(ScalarGraph g, int id, int[]?[] tangents, int n, int zero)
        {
            var node = g[id];
            if (node.Op is ScalarOp.Input or ScalarOp.Const or ScalarOp.Detach)
            {
                return null;
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
