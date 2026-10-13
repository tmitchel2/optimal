/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Generic;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Which nodes of a graph can be zero at run time because a select chose a constant zero: a select with such an arm,
    /// a sum or difference of such values, and a product, quotient or negation of one. The <see cref="VjpTransform"/>
    /// guards the products of these adjoints, so the zero of a branch not taken is not multiplied by a NaN. The facts
    /// are computed in id order as the graph grows, so no node is visited twice and nothing recurses.
    /// </summary>
    public sealed class AdjointZeros
    {
        private readonly ScalarGraph _graph;
        private readonly List<bool> _canBeZero = new();

        public AdjointZeros(ScalarGraph graph)
        {
            _graph = graph;
        }

        /// <summary>Whether node <paramref name="id"/> can be an exact zero chosen by a select.</summary>
        public bool CanBeZero(int id)
        {
            while (_canBeZero.Count <= id)
            {
                _canBeZero.Add(Compute(_graph[_canBeZero.Count]));
            }

            return _canBeZero[id];
        }

        private bool Compute(ScalarNode node)
        {
            return node.Op switch
            {
                ScalarOp.Const => node.Value == 0,
                ScalarOp.SelectLess or ScalarOp.SelectLessEqual or ScalarOp.SelectEqual => _canBeZero[node.C] || _canBeZero[node.D],
                ScalarOp.Add or ScalarOp.Sub => _canBeZero[node.A] && _canBeZero[node.B],
                ScalarOp.Mul => _canBeZero[node.A] || _canBeZero[node.B],
                ScalarOp.Div or ScalarOp.Neg => _canBeZero[node.A],
                _ => false
            };
        }
    }
}
