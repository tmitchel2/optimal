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
    /// Every Math function, smooth helper and run-time select an array kernel supports, one per output, so each
    /// derivative rule is checked in both modes; and compile-time int arithmetic driving a loop.
    /// </summary>
    [OptimalCode]
    public static class MathKernels
    {
        public const int Outputs = 23;

        [Jacobian(nameof(x))]
        [Vjp(nameof(x))]
        public static void Functions([FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(Outputs)] Span<double> y)
        {
            y[0] = Math.Sqrt(x[0]);
            y[1] = Math.Cbrt(x[1]);
            y[2] = Math.Abs(x[1]) * x[2];
            y[3] = Math.Pow(x[0], 2.5);
            y[4] = Math.Pow(x[0], -1.5) + Math.Pow(x[2], 2) + Math.Pow(x[2], 0.5);
            y[5] = Math.Exp(x[2]);
            y[6] = Math.Log(x[0]);
            y[7] = Math.Sin(x[1] * x[2]);
            y[8] = Math.Cos(x[0] + x[1]);
            y[9] = Math.Tan(x[2]);
            y[10] = Math.Atan(x[1]);
            y[11] = Math.Tanh(x[0] - x[2]);
            y[12] = Math.Atan2(x[1], x[0]);
            y[13] = Math.Min(x[0], x[1]) * x[2];
            y[14] = Math.Max(x[0], x[1]) * x[2];
            y[15] = Math.Clamp(x[2], -0.25, 0.25) + Math.Clamp(x[0], -1.0, 1.0);
            y[16] = Math.Sign(x[1]) * x[0];
            y[17] = KernelMath.SmoothMin(x[0], x[1], 0.1);
            y[18] = KernelMath.SmoothMax(x[0], x[1], 0.1);
            y[19] = KernelMath.SmoothAbs(x[1], 0.1);
            y[20] = x[0] < x[1] ? x[0] * x[2] : x[1] * x[2];
            y[21] = x[0] != 0 ? x[1] / x[0] : 0;

            // Compile-time ints: %, && and Math on ints choose which terms are summed.
            var sum = 0.0;
            for (var k = 0; k < 6; k++)
            {
                if (k % 2 == 0 && k != 4)
                {
                    sum += x[Math.Min(k, 2)] * (k + 1);
                }
            }

            y[22] = sum;
        }

        /// <summary>The derivatives at a kink: |x|, min and max where their operands tie.</summary>
        [Jacobian(nameof(x))]
        public static void Kinks([FixedLength(2)] ReadOnlySpan<double> x, [FixedLength(3)] Span<double> y)
        {
            y[0] = Math.Abs(x[0]);
            y[1] = Math.Min(x[0], x[1]);
            y[2] = Math.Max(x[0], x[1]);
        }
    }
}
