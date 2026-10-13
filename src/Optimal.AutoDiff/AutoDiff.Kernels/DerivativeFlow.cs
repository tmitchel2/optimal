/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The analyses both derivative transforms share: which operands a node's derivative flows through, which primal
    /// nodes depend on the differentiated inputs (activity), and which nodes an output reaches (liveness).
    /// </summary>
    public static class DerivativeFlow
    {
        /// <summary>
        /// The operands a derivative flows through. A select's comparison operands are excluded, because the select is
        /// piecewise constant in them, and a detached node has none.
        /// </summary>
        public static IEnumerable<int> DerivativeOperands(ScalarNode node)
        {
            return node.Op switch
            {
                ScalarOp.Detach => Array.Empty<int>(),
                _ when ScalarOps.IsSelect(node.Op) => new[] { node.C, node.D },
                _ => node.Operands
            };
        }

        /// <summary>For each primal node, whether its derivative can be non-zero: it depends on an input in <paramref name="isActiveGroup"/>.</summary>
        public static bool[] Active(UnrolledKernel kernel, Func<int, bool> isActiveGroup)
        {
            return Active(kernel.Graph, kernel.PrimalCount, isActiveGroup);
        }

        /// <summary>For each of the first <paramref name="count"/> nodes, whether it depends on an input in <paramref name="isActiveGroup"/>.</summary>
        public static bool[] Active(ScalarGraph g, int count, Func<int, bool> isActiveGroup)
        {
            var active = new bool[count];
            for (var id = 0; id < active.Length; id++)
            {
                var node = g[id];
                active[id] = node.Op == ScalarOp.Input
                    ? isActiveGroup(node.A)
                    : DerivativeOperands(node).Any(operand => active[operand]);
            }

            return active;
        }

        /// <summary>For each primal node, whether an output's derivative can flow back to it.</summary>
        public static bool[] Live(UnrolledKernel kernel)
        {
            return Live(kernel.Graph, kernel.PrimalCount, kernel.OutputIds);
        }

        /// <summary>For each of the first <paramref name="count"/> nodes, whether the derivative of one of <paramref name="outputs"/> can flow back to it.</summary>
        public static bool[] Live(ScalarGraph g, int count, IEnumerable<int> outputs)
        {
            var live = new bool[count];
            foreach (var id in outputs)
            {
                live[id] = true;
            }

            for (var id = live.Length - 1; id >= 0; id--)
            {
                if (live[id])
                {
                    foreach (var operand in DerivativeOperands(g[id]))
                    {
                        live[operand] = true;
                    }
                }
            }

            return live;
        }
    }
}
