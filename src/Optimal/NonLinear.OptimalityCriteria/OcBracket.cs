/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear.OptimalityCriteria
{
    /// <summary>
    /// How an optimality-criteria update ended.
    /// </summary>
    internal enum OcBracket
    {
        /// <summary>The constraint does not bind within the move limits: every variable goes to its upper limit.</summary>
        Inactive,

        /// <summary>The constraint cannot be met within the move limits: every variable goes to its lower limit.</summary>
        Infeasible,

        /// <summary>λ was found by bisection.</summary>
        Bisected
    }
}
