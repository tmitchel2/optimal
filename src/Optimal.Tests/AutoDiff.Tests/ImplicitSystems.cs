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
    /// A coupled nonlinear system of three unknowns g(y, p) = 0 solved by <see cref="KernelMath.ImplicitSolve"/>. With
    /// p₃ = 0 its Jacobian has a zero diagonal at the start, so the elimination must pivot. <see cref="Energy"/> is a
    /// scalar of the solution, whose Hessian needs the solve's second derivatives.
    /// </summary>
    [OptimalCode]
    public static class ImplicitSystems
    {
        public const int Iterations = 20;

        [Jacobian(nameof(p))]
        [Vjp(nameof(p))]
        public static void Solve([FixedLength(4)] ReadOnlySpan<double> p, [FixedLength(3)] Span<double> y)
        {
            Span<double> unknowns = stackalloc double[3];
            KernelMath.ImplicitSolve(Residual, p, unknowns, Iterations);
            unknowns.CopyTo(y);
        }

        [Vjp(nameof(p))]
        [Hessian(nameof(p))]
        public static void Energy([FixedLength(4)] ReadOnlySpan<double> p, [FixedLength(1)] Span<double> w)
        {
            Span<double> y = stackalloc double[3];
            Solve(p, y);
            w[0] = (y[0] * y[0]) + (y[1] * y[2]) + (p[3] * y[0]);
        }

        /// <summary>The residual: g₀ = p₃y₀ + y₁ + 0.1y₁³ − p₀, g₁ = y₀ + p₃y₁ + 0.2y₂ + 0.1y₀² − p₁, g₂ = 0.5y₀y₁ + y₂ + 0.1y₂³ − p₂.</summary>
        public static void Residual(ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)
        {
            g[0] = (p[3] * y[0]) + y[1] + (0.1 * y[1] * y[1] * y[1]) - p[0];
            g[1] = y[0] + (p[3] * y[1]) + (0.2 * y[2]) + (0.1 * y[0] * y[0]) - p[1];
            g[2] = (0.5 * y[0] * y[1]) + y[2] + (0.1 * y[2] * y[2] * y[2]) - p[2];
        }
    }
}
