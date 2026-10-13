/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// How the MMA subproblem's reduced Newton system is solved. The system is m×m when m &lt; n and n×n otherwise
    /// (Svanberg 2007), so it has min(m, n) unknowns either way.
    /// </summary>
    public enum MmaLinearSolver
    {
        /// <summary>
        /// Direct when min(m, n) is at most <see cref="MovingAsymptotesOptions.DirectSolveLimit"/>, conjugate gradients
        /// otherwise.
        /// </summary>
        Automatic,

        /// <summary>
        /// Form the reduced matrix densely and factorise it by Cholesky (Svanberg's choice). Its cost grows as
        /// min(m, n)³, and forming it costs O(min(m, n) · stored Jacobian values).
        /// </summary>
        Direct,

        /// <summary>
        /// Jacobi-preconditioned conjugate gradients on the reduced operator, applied matrix-free through the sparse
        /// constraint Jacobian: O(stored Jacobian values) per iteration, for many sparse constraints.
        /// </summary>
        ConjugateGradient
    }
}
