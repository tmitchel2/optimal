/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>The operation of a <see cref="ScalarNode"/>. The operand slots each one uses are listed.</summary>
    public enum ScalarOp
    {
        /// <summary>A constant, held in <see cref="ScalarNode.Value"/>.</summary>
        Const,

        /// <summary>An input element: <see cref="ScalarNode.A"/> is the parameter group, <see cref="ScalarNode.B"/> the index.</summary>
        Input,

        /// <summary>A + B.</summary>
        Add,

        /// <summary>A − B.</summary>
        Sub,

        /// <summary>A × B.</summary>
        Mul,

        /// <summary>A ÷ B.</summary>
        Div,

        /// <summary>−A.</summary>
        Neg,

        /// <summary>√A.</summary>
        Sqrt,

        /// <summary>∛A.</summary>
        Cbrt,

        /// <summary>|A|.</summary>
        Abs,

        /// <summary>The sign of A as −1, 0 or 1 (0 for NaN). Piecewise constant, so its derivative is zero.</summary>
        Sign,

        /// <summary>A raised to the constant exponent held in <see cref="ScalarNode.Value"/>.</summary>
        PowConst,

        /// <summary>e^A.</summary>
        Exp,

        /// <summary>ln A.</summary>
        Log,

        /// <summary>sin A.</summary>
        Sin,

        /// <summary>cos A.</summary>
        Cos,

        /// <summary>tan A.</summary>
        Tan,

        /// <summary>atan A.</summary>
        Atan,

        /// <summary>tanh A.</summary>
        Tanh,

        /// <summary>atan2(A, B), with A the y coordinate.</summary>
        Atan2,

        /// <summary>min(A, B); its derivative follows the selected operand (a subgradient at a tie).</summary>
        Min,

        /// <summary>max(A, B); its derivative follows the selected operand (a subgradient at a tie).</summary>
        Max,

        /// <summary>A &lt; B ? C : D.</summary>
        SelectLess,

        /// <summary>A ≤ B ? C : D.</summary>
        SelectLessEqual,

        /// <summary>A = B ? C : D.</summary>
        SelectEqual,

        /// <summary>
        /// A's value with a zero derivative. An implicit solve detaches its Newton iterate, so no derivative flows
        /// through the iterations; the implicit-function step that follows carries the whole derivative.
        /// </summary>
        Detach
    }
}
