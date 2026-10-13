/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The derivative rule of every <see cref="ScalarOp"/>, written once and used by both
    /// <see cref="JacobianTransform"/> (forward) and <see cref="VjpTransform"/> (reverse). A smooth operation has a
    /// partial derivative per operand slot, built as graph nodes; a piecewise one (min, max, select) instead names the
    /// comparison that chooses which operand's derivative it follows.
    /// </summary>
    public static class LocalDerivatives
    {
        /// <summary>
        /// ∂(node <paramref name="id"/>)/∂(operand in <paramref name="slot"/>) as a graph node, for an operation that is not
        /// piecewise. A partial is built only when asked for, so an inactive operand costs nothing.
        /// </summary>
        public static int Partial(ScalarGraph g, int id, int slot)
        {
            var node = g[id];
            if (slot < 0 || slot >= ScalarOps.Arity(node.Op) || ScalarOps.IsPiecewise(node.Op))
            {
                throw new ArgumentOutOfRangeException(nameof(slot), slot, $"{node.Op} has no smooth partial in slot {slot}.");
            }

            return node.Op switch
            {
                ScalarOp.Add => g.Const(1),
                ScalarOp.Sub => g.Const(slot == 0 ? 1 : -1),
                ScalarOp.Neg => g.Const(-1),
                ScalarOp.Mul => slot == 0 ? node.B : node.A,

                // q = a/b: ∂q/∂a = 1/b and ∂q/∂b = −q/b, sharing the reciprocal.
                ScalarOp.Div => slot == 0 ? Reciprocal(g, node.B) : g.Neg(g.Mul(id, Reciprocal(g, node.B))),
                ScalarOp.Sqrt => g.Div(g.Const(0.5), id),
                ScalarOp.Cbrt => g.Div(g.Const(1.0 / 3.0), g.Mul(id, id)),

                // The minimum-norm subgradient: 0 at the kink.
                ScalarOp.Abs => g.Unary(ScalarOp.Sign, node.A),
                ScalarOp.Sign or ScalarOp.Detach => g.Const(0),
                ScalarOp.PowConst => g.Mul(g.Const(node.Value), g.PowConst(node.A, node.Value - 1)),
                ScalarOp.Exp => id,
                ScalarOp.Log => Reciprocal(g, node.A),
                ScalarOp.Sin => g.Unary(ScalarOp.Cos, node.A),
                ScalarOp.Cos => g.Neg(g.Unary(ScalarOp.Sin, node.A)),
                ScalarOp.Tan => g.Add(g.Const(1), g.Mul(id, id)),
                ScalarOp.Atan => Reciprocal(g, g.Add(g.Const(1), g.Mul(node.A, node.A))),
                ScalarOp.Tanh => g.Sub(g.Const(1), g.Mul(id, id)),

                // θ = atan2(y, x): ∂θ/∂y = x/(x² + y²) and ∂θ/∂x = −y/(x² + y²).
                ScalarOp.Atan2 => slot == 0
                    ? g.Div(node.B, SquaredNorm(g, node.A, node.B))
                    : g.Neg(g.Div(node.A, SquaredNorm(g, node.A, node.B))),
                _ => throw new ArgumentOutOfRangeException(nameof(id), node.Op, "No derivative rule for this operation.")
            };
        }

        /// <summary>
        /// For a piecewise operation, the select that picks its active operand: the result's derivative is the derivative
        /// of <see cref="PiecewiseChoice.WhenTrue"/> where the comparison holds and of
        /// <see cref="PiecewiseChoice.WhenFalse"/> elsewhere. Min and max follow their first operand at a tie.
        /// </summary>
        public static PiecewiseChoice Choice(ScalarGraph g, int id)
        {
            var node = g[id];
            return node.Op switch
            {
                ScalarOp.Min => new PiecewiseChoice(ScalarOp.SelectLessEqual, node.A, node.B, node.A, node.B),
                ScalarOp.Max => new PiecewiseChoice(ScalarOp.SelectLessEqual, node.B, node.A, node.A, node.B),
                ScalarOp.SelectLess or ScalarOp.SelectLessEqual or ScalarOp.SelectEqual
                    => new PiecewiseChoice(node.Op, node.A, node.B, node.C, node.D),
                _ => throw new ArgumentOutOfRangeException(nameof(id), node.Op, "Not a piecewise operation.")
            };
        }

        private static int Reciprocal(ScalarGraph g, int a)
        {
            return g.Div(g.Const(1), a);
        }

        private static int SquaredNorm(ScalarGraph g, int a, int b)
        {
            return g.Add(g.Mul(a, a), g.Mul(b, b));
        }
    }
}
