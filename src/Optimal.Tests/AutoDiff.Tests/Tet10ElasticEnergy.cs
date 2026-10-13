/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// The linear-elastic Tet10's strain energy W(u) = ½ uᵀ R(u), with R the residual of <see cref="Tet10Elastic"/>
    /// inlined as a helper. R = K u, so the generated Hessian of W is the element stiffness K.
    /// </summary>
    [OptimalCode]
    public static class Tet10ElasticEnergy
    {
        [Vjp(nameof(u))]
        [Hessian(nameof(u))]
        public static void Energy(
            [FixedLength(30)] ReadOnlySpan<double> u,
            [FixedLength(30)] ReadOnlySpan<double> X,
            double E,
            double nu,
            [FixedLength(1)] Span<double> w)
        {
            Span<double> r = stackalloc double[30];
            Tet10Elastic.Residual(u, X, E, nu, r);
            var sum = 0.0;
            for (var i = 0; i < 30; i++)
            {
                sum += u[i] * r[i];
            }

            w[0] = 0.5 * sum;
        }
    }
}
