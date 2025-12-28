/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear
{
    /// <summary>
    /// Reason why the optimization process terminated.
    /// </summary>
    public enum StoppingReason
    {
        /// <summary>
        /// Gradient norm fell below the specified tolerance.
        /// </summary>
        GradientTolerance,

        /// <summary>
        /// Change in function value fell below the specified tolerance.
        /// </summary>
        FunctionTolerance,

        /// <summary>
        /// Change in parameter values fell below the specified tolerance.
        /// </summary>
        ParameterTolerance,

        /// <summary>
        /// Maximum number of iterations was reached.
        /// </summary>
        MaxIterations,

        /// <summary>
        /// Maximum number of function evaluations was reached.
        /// </summary>
        MaxFunctionEvaluations,

        /// <summary>
        /// User requested termination via callback.
        /// </summary>
        UserRequested,

        /// <summary>
        /// Numerical error occurred during optimization.
        /// </summary>
        NumericalError
    }
}
