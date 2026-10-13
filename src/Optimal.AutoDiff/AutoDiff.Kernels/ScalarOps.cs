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
    /// <summary>Static facts about each <see cref="ScalarOp"/>.</summary>
    public static class ScalarOps
    {
        /// <summary>The number of operand slots an operation reads.</summary>
        public static int Arity(ScalarOp op)
        {
            return op switch
            {
                ScalarOp.Const or ScalarOp.Input => 0,
                ScalarOp.Neg or ScalarOp.Sqrt or ScalarOp.Cbrt or ScalarOp.Abs or ScalarOp.Sign or ScalarOp.PowConst
                    or ScalarOp.Exp or ScalarOp.Log or ScalarOp.Sin or ScalarOp.Cos or ScalarOp.Tan or ScalarOp.Atan
                    or ScalarOp.Tanh => 1,
                ScalarOp.Add or ScalarOp.Sub or ScalarOp.Mul or ScalarOp.Div or ScalarOp.Atan2 or ScalarOp.Min
                    or ScalarOp.Max => 2,
                ScalarOp.SelectLess or ScalarOp.SelectLessEqual or ScalarOp.SelectEqual => 4,
                _ => throw new ArgumentOutOfRangeException(nameof(op), op, "Unknown scalar operation.")
            };
        }

        /// <summary>Whether the operation is a select, A ⋚ B ? C : D.</summary>
        public static bool IsSelect(ScalarOp op)
        {
            return op is ScalarOp.SelectLess or ScalarOp.SelectLessEqual or ScalarOp.SelectEqual;
        }

        /// <summary>
        /// Whether the operation's derivative follows one operand chosen by a comparison (min, max, select) rather than a
        /// sum of partial derivatives. Its tangent is a select of operand tangents, so an infinite or NaN tangent on the
        /// branch not taken never reaches the result.
        /// </summary>
        public static bool IsPiecewise(ScalarOp op)
        {
            return op is ScalarOp.Min or ScalarOp.Max || IsSelect(op);
        }

        /// <summary>Whether the operands of a binary operation may be swapped without changing any result bit.</summary>
        public static bool IsCommutative(ScalarOp op)
        {
            return op is ScalarOp.Add or ScalarOp.Mul or ScalarOp.Min or ScalarOp.Max;
        }
    }
}
