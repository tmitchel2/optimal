/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// Svanberg's rules for the moving asymptotes L and U and for the subproblem's box [α, β] (mmasub, 2007).
    /// </summary>
    internal static class Asymptotes
    {
        /// <summary>
        /// Iterations 1 and 2 place the asymptotes at x ∓ initial·range. Later, a variable that moved the same way
        /// twice running has its asymptotes widened by the increase factor, one that oscillated narrowed by the decrease
        /// factor; then each is kept between minimum·range and maximum·range from x.
        /// </summary>
        public static void Update(
            int iteration,
            ReadOnlySpan<double> x,
            ReadOnlySpan<double> xOld1,
            ReadOnlySpan<double> xOld2,
            ReadOnlySpan<double> range,
            Span<double> low,
            Span<double> upp,
            MovingAsymptotesOptions options)
        {
            if (iteration <= 2)
            {
                for (var j = 0; j < x.Length; j++)
                {
                    low[j] = x[j] - (options.AsymptoteInitial * range[j]);
                    upp[j] = x[j] + (options.AsymptoteInitial * range[j]);
                }

                return;
            }

            for (var j = 0; j < x.Length; j++)
            {
                var trend = (x[j] - xOld1[j]) * (xOld1[j] - xOld2[j]);
                var factor = trend > 0.0 ? options.AsymptoteIncrease : trend < 0.0 ? options.AsymptoteDecrease : 1.0;
                var lower = x[j] - (factor * (xOld1[j] - low[j]));
                var upper = x[j] + (factor * (upp[j] - xOld1[j]));
                low[j] = Math.Min(Math.Max(lower, x[j] - (options.AsymptoteMaximumDistance * range[j])), x[j] - (options.AsymptoteMinimumDistance * range[j]));
                upp[j] = Math.Max(Math.Min(upper, x[j] + (options.AsymptoteMaximumDistance * range[j])), x[j] + (options.AsymptoteMinimumDistance * range[j]));
            }
        }

        /// <summary>
        /// α_j = max(L_j + albefa·(x_j - L_j), x_j - move·range_j, lower_j) and
        /// β_j = min(U_j - albefa·(U_j - x_j), x_j + move·range_j, upper_j).
        /// </summary>
        public static void Box(
            ReadOnlySpan<double> x,
            ReadOnlySpan<double> low,
            ReadOnlySpan<double> upp,
            ReadOnlySpan<double> lower,
            ReadOnlySpan<double> upper,
            ReadOnlySpan<double> range,
            Span<double> alpha,
            Span<double> beta,
            MovingAsymptotesOptions options)
        {
            for (var j = 0; j < x.Length; j++)
            {
                var move = options.MoveLimit * range[j];
                alpha[j] = Math.Max(Math.Max(low[j] + (options.AsymptoteBoundFactor * (x[j] - low[j])), x[j] - move), lower[j]);
                beta[j] = Math.Min(Math.Min(upp[j] - (options.AsymptoteBoundFactor * (upp[j] - x[j])), x[j] + move), upper[j]);
            }
        }

        /// <summary>
        /// Throws if the asymptote and move settings are outside the ranges the method needs.
        /// </summary>
        public static void Validate(MovingAsymptotesOptions options)
        {
            ArgumentNullException.ThrowIfNull(options);
            RequireInRange(options.MoveLimit, 0.0, double.PositiveInfinity, nameof(options.MoveLimit));
            RequireInRange(options.AsymptoteInitial, 0.0, double.PositiveInfinity, nameof(options.AsymptoteInitial));
            RequireInRange(options.AsymptoteIncrease, 1.0, double.PositiveInfinity, nameof(options.AsymptoteIncrease), inclusiveLow: true);
            RequireInRange(options.AsymptoteDecrease, 0.0, 1.0, nameof(options.AsymptoteDecrease), inclusiveHigh: true);
            RequireInRange(options.AsymptoteMinimumDistance, 0.0, options.AsymptoteMaximumDistance, nameof(options.AsymptoteMinimumDistance));
            RequireInRange(options.AsymptoteBoundFactor, 0.0, 1.0, nameof(options.AsymptoteBoundFactor));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(options.MaxIterations);
            if (!(options.Tolerance >= 0.0) || !(options.ChangeTolerance >= 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(options), "Tolerance and ChangeTolerance must be non-negative.");
            }
        }

        private static void RequireInRange(
            double value,
            double low,
            double high,
            string name,
            bool inclusiveLow = false,
            bool inclusiveHigh = false)
        {
            var aboveLow = inclusiveLow ? value >= low : value > low;
            var belowHigh = inclusiveHigh ? value <= high : value < high;
            if (!(aboveLow && belowHigh))
            {
                throw new ArgumentOutOfRangeException(name, value, $"Must be in {(inclusiveLow ? "[" : "(")}{low}, {high}{(inclusiveHigh ? "]" : ")")}.");
            }
        }
    }
}
