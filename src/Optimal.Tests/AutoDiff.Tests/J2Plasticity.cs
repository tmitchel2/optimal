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
    /// Small-strain J2 plasticity with linear isotropic hardening: the radial return of one step, written with an
    /// <c>if</c> on the trial yield function, which the generator if-converts. From the trial deviatoric stress
    /// s = 2μ dev(ε − εᵖ) and f = ‖s‖ − √(2/3)(σ_y + H α): elastic when f ≤ 0; otherwise Δγ = f / (2μ + ⅔H) and
    /// s ← (1 − 2μΔγ/‖s‖) s. σ = s + κ tr(ε − εᵖ) I. The generated ∂σ/∂ε is the algorithmic (consistent) tangent.
    /// </summary>
    [OptimalCode]
    public static class J2Plasticity
    {
        [Jacobian(nameof(strain))]
        [Vjp(nameof(strain))]
        public static void Stress(
            [FixedLength(9)] ReadOnlySpan<double> strain,
            [FixedLength(9)] ReadOnlySpan<double> plasticStrain,
            double alpha,
            double E,
            double nu,
            double yieldStress,
            double hardening,
            [FixedLength(9)] Span<double> stress)
        {
            var mu = E / (2 * (1 + nu));
            var kappa = E / (3 * (1 - (2 * nu)));
            var elastic = Tensor3.From(strain) - Tensor3.From(plasticStrain);
            var s = 2 * mu * elastic.Deviator();
            var norm = s.Norm();
            var f = norm - (Math.Sqrt(2.0 / 3.0) * (yieldStress + (hardening * alpha)));
            if (f > 0)
            {
                var dGamma = f / ((2 * mu) + (2.0 / 3.0 * hardening));
                s = (1 - (2 * mu * dGamma / norm)) * s;
            }

            var sigma = s + (kappa * elastic.Trace * Tensor3.Identity);
            sigma.CopyTo(stress);
        }
    }
}
