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
    /// One SSA value of a <see cref="ScalarGraph"/>. Operands always have lower ids than the node that uses them, so
    /// id order is a topological order. Unused operand slots are −1.
    /// </summary>
    public readonly struct ScalarNode
    {
        public ScalarNode(ScalarOp op, int a, int b, int c, int d, double value)
        {
            Op = op;
            A = a;
            B = b;
            C = c;
            D = d;
            Value = value;
        }

        public ScalarOp Op { get; }

        /// <summary>The first operand, or the parameter group of an <see cref="ScalarOp.Input"/>.</summary>
        public int A { get; }

        /// <summary>The second operand, or the element index of an <see cref="ScalarOp.Input"/>.</summary>
        public int B { get; }

        /// <summary>The value selected when a select's comparison holds.</summary>
        public int C { get; }

        /// <summary>The value selected when a select's comparison fails.</summary>
        public int D { get; }

        /// <summary>The value of a <see cref="ScalarOp.Const"/>, or the exponent of a <see cref="ScalarOp.PowConst"/>.</summary>
        public double Value { get; }

        /// <summary>The ids this node reads, in slot order. Constants and inputs read none.</summary>
        public IEnumerable<int> Operands
        {
            get
            {
                switch (ScalarOps.Arity(Op))
                {
                    case 4:
                        yield return A;
                        yield return B;
                        yield return C;
                        yield return D;
                        break;
                    case 2:
                        yield return A;
                        yield return B;
                        break;
                    case 1:
                        yield return A;
                        break;
                    default:
                        break;
                }
            }
        }
    }
}
