/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.OptimalityCriteria
{
    /// <summary>
    /// One optimality-criteria update for min f(x) subject to vᵀx &lt;= T and lo &lt;= x &lt;= hi:
    /// x_j(λ) = clamp(x_j·(max(0, -∂f/∂x_j)/(λ v_j))^η, lo_j, hi_j), with λ found by bisection on l = λ^η so that
    /// vᵀx(λ) = T. The initial bracket is explicit (top99neo, Ferrari and Sigmund 2020): [0, Σ_j v_j P_j / T], with
    /// P_j = x_j·(max(0, -∂f/∂x_j)/v_j)^η, the root when no clamp binds; it is doubled while it does not bracket.
    /// </summary>
    internal static class OcUpdate
    {
        private const int MaxBracketExpansions = 64;

        /// <summary>
        /// Writes the new point into <paramref name="xNew"/> and returns λ and the bisections taken. <paramref name="p"/>
        /// receives P_j. Every v_j must be positive.
        /// </summary>
        public static (double Lambda, int Bisections, OcBracket Bracket) Apply(
            ReadOnlySpan<double> x,
            ReadOnlySpan<double> gradient,
            ReadOnlySpan<double> weights,
            double target,
            ReadOnlySpan<double> lo,
            ReadOnlySpan<double> hi,
            double eta,
            double tolerance,
            int maxBisections,
            Span<double> p,
            Span<double> xNew)
        {
            var weighted = 0.0;
            var volumeAtZero = 0.0;
            var volumeAtInfinity = 0.0;
            for (var j = 0; j < x.Length; j++)
            {
                p[j] = x[j] * Math.Pow(Math.Max(0.0, -gradient[j]) / weights[j], eta);
                weighted += weights[j] * p[j];
                volumeAtZero += weights[j] * (p[j] > 0.0 ? hi[j] : lo[j]);
                volumeAtInfinity += weights[j] * lo[j];
            }

            if (volumeAtZero <= target)
            {
                Clamp(p, 0.0, lo, hi, xNew);
                return (0.0, 0, OcBracket.Inactive);
            }

            if (volumeAtInfinity > target || target <= 0.0)
            {
                lo.CopyTo(xNew);
                return (double.PositiveInfinity, 0, OcBracket.Infeasible);
            }

            var upper = weighted / target;
            for (var expansion = 0; expansion < MaxBracketExpansions && Volume(p, upper, lo, hi, weights) > target; expansion++)
            {
                upper *= 2.0;
            }

            var lower = 0.0;
            var middle = upper;
            var bisections = 0;
            while ((upper - lower) / (lower + upper) > tolerance && bisections < maxBisections)
            {
                middle = 0.5 * (lower + upper);
                if (Volume(p, middle, lo, hi, weights) > target)
                {
                    lower = middle;
                }
                else
                {
                    upper = middle;
                }

                bisections++;
            }

            Clamp(p, middle, lo, hi, xNew);
            return (Math.Pow(middle, 1.0 / eta), bisections, OcBracket.Bisected);
        }

        private static double Volume(ReadOnlySpan<double> p, double l, ReadOnlySpan<double> lo, ReadOnlySpan<double> hi, ReadOnlySpan<double> weights)
        {
            var volume = 0.0;
            for (var j = 0; j < p.Length; j++)
            {
                volume += weights[j] * Math.Clamp(p[j] / l, lo[j], hi[j]);
            }

            return volume;
        }

        private static void Clamp(ReadOnlySpan<double> p, double l, ReadOnlySpan<double> lo, ReadOnlySpan<double> hi, Span<double> xNew)
        {
            for (var j = 0; j < p.Length; j++)
            {
                xNew[j] = l == 0.0 ? (p[j] > 0.0 ? hi[j] : lo[j]) : Math.Clamp(p[j] / l, lo[j], hi[j]);
            }
        }
    }
}
