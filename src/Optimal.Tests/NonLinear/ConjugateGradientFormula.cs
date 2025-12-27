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
    /// Formulas for computing the beta parameter in conjugate gradient methods.
    /// </summary>
    public enum ConjugateGradientFormula
    {
        /// <summary>
        /// Fletcher-Reeves formula: beta = ||grad_new||^2 / ||grad_old||^2
        /// Most robust, guarantees descent with exact line search.
        /// </summary>
        FletcherReeves,

        /// <summary>
        /// Polak-Ribiere formula: beta = grad_new^T * (grad_new - grad_old) / ||grad_old||^2
        /// Often faster than FR, can automatically restart if beta becomes negative.
        /// </summary>
        PolakRibiere,

        /// <summary>
        /// Hestenes-Stiefel formula: beta = grad_new^T * (grad_new - grad_old) / (d_old^T * (grad_new - grad_old))
        /// Theoretically elegant, but can be numerically unstable.
        /// </summary>
        HestenesStiefel
    }
}
