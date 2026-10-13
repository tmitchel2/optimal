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
    /// Small-strain J2 plasticity with Voce hardening, σ_y(α) = σ₀ + (σ∞ − σ₀)(1 − e^(−δα)) + Hα: the return mapping's
    /// plastic multiplier has no closed form, so it is the root of φ(Δγ) = ‖s‖ − 2μΔγ − √(2/3) σ_y(α + √(2/3)Δγ),
    /// solved by <see cref="KernelMath.ImplicitSolve"/> inside the if-converted plastic branch. The generator
    /// differentiates Δγ by the implicit-function theorem.
    /// </summary>
    [OptimalCode]
    public static class VocePlasticity
    {
        [Jacobian(nameof(strain))]
        [Vjp(nameof(strain))]
        public static void Stress(
            [FixedLength(9)] ReadOnlySpan<double> strain,
            [FixedLength(9)] ReadOnlySpan<double> plasticStrain,
            double alpha,
            double E,
            double nu,
            [FixedLength(4)] ReadOnlySpan<double> hardening,
            [FixedLength(9)] Span<double> stress)
        {
            var mu = E / (2 * (1 + nu));
            var kappa = E / (3 * (1 - (2 * nu)));
            var elastic = Tensor3.From(strain) - Tensor3.From(plasticStrain);
            var s = 2 * mu * elastic.Deviator();
            var norm = s.Norm();
            var f = norm - (Math.Sqrt(2.0 / 3.0) * Yield(alpha, hardening));
            if (f > 0)
            {
                Span<double> p = stackalloc double[7];
                p[0] = norm;
                p[1] = mu;
                p[2] = alpha;
                p[3] = hardening[0];
                p[4] = hardening[1];
                p[5] = hardening[2];
                p[6] = hardening[3];
                Span<double> dGamma = stackalloc double[1];
                KernelMath.ImplicitSolve(Residual, p, dGamma, 12);
                s = (1 - (2 * mu * dGamma[0] / norm)) * s;
            }

            var sigma = s + (kappa * elastic.Trace * Tensor3.Identity);
            sigma.CopyTo(stress);
        }

        // φ(Δγ) with p = (‖s‖, μ, α, σ₀, σ∞, δ, H).
        private static void Residual(ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)
        {
            Span<double> hardening = stackalloc double[4];
            p.Slice(3, 4).CopyTo(hardening);
            var a = p[2] + (Math.Sqrt(2.0 / 3.0) * y[0]);
            g[0] = p[0] - (2 * p[1] * y[0]) - (Math.Sqrt(2.0 / 3.0) * Yield(a, hardening));
        }

        // σ_y(α) with hardening = (σ₀, σ∞, δ, H).
        private static double Yield(double a, ReadOnlySpan<double> hardening)
        {
            return hardening[0] + ((hardening[1] - hardening[0]) * (1 - Math.Exp(-hardening[2] * a))) + (hardening[3] * a);
        }
    }
}
