/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>The derivative a <see cref="DerivativeRequest"/> asks of a kernel; each names its generated method.</summary>
    public enum DerivativeKind
    {
        /// <summary><c>[Jacobian]</c>: the dense ∂output/∂p, by vector forward mode.</summary>
        Jacobian,

        /// <summary><c>[Vjp]</c>: λᵀ ∂output/∂p for one or more p, by reverse mode.</summary>
        Vjp,

        /// <summary><c>[Hessian]</c>: the dense ∂²output/∂p² of a kernel with one output, by forward over reverse.</summary>
        Hessian,

        /// <summary><c>[Hvp]</c>: the Hessian–vector product (∂²output/∂p²) v of a kernel with one output.</summary>
        Hvp
    }
}
