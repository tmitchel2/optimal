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
    /// Interface for preconditioning strategies in L-BFGS optimization.
    /// Preconditioners improve convergence on ill-conditioned problems by
    /// approximating the inverse Hessian scaling.
    /// </summary>
    public interface IPreconditioner
    {
        /// <summary>
        /// Applies the preconditioner to a gradient vector.
        /// Returns M^{-1} * gradient where M approximates the Hessian.
        /// </summary>
        /// <param name="gradient">The gradient vector to precondition.</param>
        /// <returns>The preconditioned gradient.</returns>
        double[] ApplyToGradient(double[] gradient);

        /// <summary>
        /// Applies the inverse of the preconditioner to a direction vector.
        /// Returns M * direction.
        /// </summary>
        /// <param name="direction">The direction vector.</param>
        /// <returns>The transformed direction.</returns>
        double[] ApplyInverseToDirection(double[] direction);

        /// <summary>
        /// Updates the preconditioner with new curvature information from L-BFGS.
        /// </summary>
        /// <param name="s">Position difference: x_{k+1} - x_k.</param>
        /// <param name="y">Gradient difference: grad_{k+1} - grad_k.</param>
        void Update(double[] s, double[] y);

        /// <summary>
        /// Gets the diagonal scaling factors used by this preconditioner.
        /// For non-diagonal preconditioners, returns an approximation.
        /// </summary>
        /// <returns>Array of diagonal scaling factors.</returns>
        double[] GetDiagonalScaling();

        /// <summary>
        /// Resets the preconditioner to its initial state.
        /// </summary>
        void Reset();
    }
}
