/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Computes the generalized Cauchy point for L-BFGS-B.
    /// The Cauchy point is obtained by following the piecewise linear path
    /// formed when the projected gradient direction hits variable bounds.
    /// </summary>
    public static class CauchyPointCalculator
    {
        /// <summary>
        /// Result of the Cauchy point computation.
        /// </summary>
        public sealed class CauchyResult
        {
            /// <summary>
            /// Gets or sets the generalized Cauchy point.
            /// </summary>
            public required double[] CauchyPoint { get; init; }

            /// <summary>
            /// Gets or sets which variables are free (true) vs at bounds (false) at the Cauchy point.
            /// </summary>
            public required bool[] FreeVariables { get; init; }

            /// <summary>
            /// Gets or sets the descent direction at the Cauchy point (for free variables only).
            /// </summary>
            public required double[] Direction { get; init; }

            /// <summary>
            /// Gets or sets the total step taken to reach the Cauchy point.
            /// </summary>
            public double TotalStep { get; init; }
        }

        /// <summary>
        /// Computes the generalized Cauchy point starting from x along the projected gradient direction.
        /// </summary>
        /// <param name="x">Current point (must be feasible).</param>
        /// <param name="gradient">Gradient at current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <param name="memory">L-BFGS memory for Hessian approximation (can be null for steepest descent).</param>
        /// <returns>Cauchy point result containing the point, free variables, and direction.</returns>
        public static CauchyResult Compute(
            double[] x,
            double[] gradient,
            double[] lower,
            double[] upper,
            LBFGSMemory? memory = null)
        {
            var n = x.Length;

            // Compute initial descent direction (negative gradient projected onto bounds)
            var d = new double[n];
            var isActive = new bool[n]; // Initially all free

            for (var i = 0; i < n; i++)
            {
                // At lower bound and gradient is positive -> can't move
                if (x[i] <= lower[i] + 1e-10 && gradient[i] > 0)
                {
                    d[i] = 0;
                    isActive[i] = true;
                }
                // At upper bound and gradient is negative -> can't move
                else if (x[i] >= upper[i] - 1e-10 && gradient[i] < 0)
                {
                    d[i] = 0;
                    isActive[i] = true;
                }
                else
                {
                    d[i] = -gradient[i];
                    isActive[i] = false;
                }
            }

            // Check if all variables are active (no movement possible)
            if (AllZero(d))
            {
                // No movement possible - already at constrained optimum
                var freeVars = new bool[n];
                for (var i = 0; i < n; i++)
                {
                    freeVars[i] = !isActive[i];
                }

                return new CauchyResult
                {
                    CauchyPoint = (double[])x.Clone(),
                    FreeVariables = freeVars,
                    Direction = new double[n],
                    TotalStep = 0.0
                };
            }

            // Compute breakpoints: times when each variable hits a bound
            var breakpoints = ComputeBreakpoints(x, d, lower, upper, isActive);

            // Sort breakpoints by time
            breakpoints.Sort((a, b) => a.Time.CompareTo(b.Time));

            // Initialize Cauchy point at x
            var xCauchy = (double[])x.Clone();
            var currentT = 0.0;

            // Compute initial quadratic model terms
            // f(x + t*d) ≈ f(x) + t*g'*d + 0.5*t^2*d'*B*d
            // where B is the Hessian approximation
            var gTd = DotProduct(gradient, d);

            // Compute d'*B*d using the L-BFGS approximation or identity
            var dTBd = ComputeQuadraticTerm(d, memory);

            // Follow the piecewise linear path
            foreach (var bp in breakpoints)
            {
                if (bp.Time <= currentT + 1e-16)
                {
                    continue; // Skip breakpoints at or before current position
                }

                // Check if we should stop before this breakpoint
                // Optimal step along current segment: t* = -g'*d / (d'*B*d)
                var optimalT = double.PositiveInfinity;
                if (dTBd > 1e-16)
                {
                    // g'*d should be negative for descent, so optimal t* = -(g'*d + currentT*d'*B*d) / d'*B*d
                    // Simplify: find t where derivative of quadratic is zero
                    // dQ/dt = g'*d + t*d'*B*d = 0  =>  t* = -g'*d / d'*B*d
                    optimalT = -gTd / dTBd;
                }

                if (optimalT > currentT && optimalT < bp.Time)
                {
                    // Optimal point is before the next breakpoint
                    var dt = optimalT - currentT;
                    for (var i = 0; i < n; i++)
                    {
                        xCauchy[i] += dt * d[i];
                    }

                    // Identify free variables at Cauchy point
                    var freeVars = IdentifyFreeVariables(xCauchy, lower, upper);

                    return new CauchyResult
                    {
                        CauchyPoint = xCauchy,
                        FreeVariables = freeVars,
                        Direction = (double[])d.Clone(),
                        TotalStep = optimalT
                    };
                }

                // Move to this breakpoint
                var deltaT = bp.Time - currentT;
                for (var i = 0; i < n; i++)
                {
                    xCauchy[i] += deltaT * d[i];
                }
                currentT = bp.Time;

                // Project to ensure we're exactly at the bound
                xCauchy[bp.Index] = bp.IsLower ? lower[bp.Index] : upper[bp.Index];

                // Update direction: this variable becomes active
                var oldDi = d[bp.Index];
                d[bp.Index] = 0;
                isActive[bp.Index] = true;

                // Update quadratic terms for reduced direction
                // g'*d decreases by gradient[i] * d[i]
                gTd -= gradient[bp.Index] * oldDi;

                // d'*B*d needs to be recomputed (approximation: subtract contribution)
                // This is an approximation - for exact computation, would need full Hessian
                dTBd = ComputeQuadraticTerm(d, memory);

                // Check if direction is now zero (all variables active)
                if (AllZero(d))
                {
                    break;
                }
            }

            // At end of path (all breakpoints processed or direction is zero)
            var finalFreeVars = IdentifyFreeVariables(xCauchy, lower, upper);

            return new CauchyResult
            {
                CauchyPoint = xCauchy,
                FreeVariables = finalFreeVars,
                Direction = d,
                TotalStep = currentT
            };
        }

        /// <summary>
        /// Simplified Cauchy point computation for the case without L-BFGS memory.
        /// Uses a simple backtracking approach along the projected gradient.
        /// </summary>
        public static CauchyResult ComputeSimple(
            double[] x,
            double[] gradient,
            double[] lower,
            double[] upper,
            double initialStepSize = 1.0)
        {
            var n = x.Length;

            // Compute projected descent direction
            var d = new double[n];
            for (var i = 0; i < n; i++)
            {
                if (x[i] <= lower[i] + 1e-10 && gradient[i] > 0)
                {
                    d[i] = 0;
                }
                else if (x[i] >= upper[i] - 1e-10 && gradient[i] < 0)
                {
                    d[i] = 0;
                }
                else
                {
                    d[i] = -gradient[i];
                }
            }

            // Find maximum feasible step
            var maxStep = ProjectedGradient.MaxFeasibleStep(x, d, lower, upper);
            var step = Math.Min(initialStepSize, maxStep);

            // Compute Cauchy point
            var xCauchy = new double[n];
            for (var i = 0; i < n; i++)
            {
                xCauchy[i] = x[i] + step * d[i];
            }

            // Project to ensure feasibility
            ProjectedGradient.ProjectInPlace(xCauchy, lower, upper);

            // Identify free variables
            var freeVars = IdentifyFreeVariables(xCauchy, lower, upper);

            return new CauchyResult
            {
                CauchyPoint = xCauchy,
                FreeVariables = freeVars,
                Direction = d,
                TotalStep = step
            };
        }

        private sealed class Breakpoint
        {
            public int Index { get; init; }
            public double Time { get; init; }
            public bool IsLower { get; init; }
        }

        private static List<Breakpoint> ComputeBreakpoints(
            double[] x,
            double[] d,
            double[] lower,
            double[] upper,
            bool[] isActive)
        {
            var breakpoints = new List<Breakpoint>();

            for (var i = 0; i < x.Length; i++)
            {
                if (isActive[i] || Math.Abs(d[i]) < 1e-16)
                {
                    continue; // Already active or not moving
                }

                double time;
                bool isLower;

                if (d[i] < 0)
                {
                    // Moving toward lower bound
                    time = (lower[i] - x[i]) / d[i];
                    isLower = true;
                }
                else
                {
                    // Moving toward upper bound
                    time = (upper[i] - x[i]) / d[i];
                    isLower = false;
                }

                if (time > 1e-16 && !double.IsInfinity(time))
                {
                    breakpoints.Add(new Breakpoint
                    {
                        Index = i,
                        Time = time,
                        IsLower = isLower
                    });
                }
            }

            return breakpoints;
        }

        private static bool[] IdentifyFreeVariables(double[] x, double[] lower, double[] upper, double tolerance = 1e-10)
        {
            var n = x.Length;
            var freeVars = new bool[n];

            for (var i = 0; i < n; i++)
            {
                freeVars[i] = x[i] > lower[i] + tolerance && x[i] < upper[i] - tolerance;
            }

            return freeVars;
        }

        private static double ComputeQuadraticTerm(double[] d, LBFGSMemory? memory)
        {
            // Compute d'*B*d where B is the Hessian approximation
            // If no memory, use identity (d'*I*d = d'*d)
            if (memory == null || memory.Count == 0)
            {
                return DotProduct(d, d);
            }

            // Use L-BFGS implicit Hessian approximation
            // B*d can be computed via the two-loop recursion, but we need d'*B*d
            // For simplicity, use the approximation B ≈ gamma*I where gamma is the scaling
            // gamma = y'*s / (y'*y) from the most recent pair
            var (s, y, _) = memory.GetPair(memory.Count - 1);
            var yTy = DotProduct(y, y);
            var yTs = DotProduct(y, s);

            if (yTy < 1e-16 || Math.Abs(yTs) < 1e-16)
            {
                return DotProduct(d, d);
            }

            var gamma = yTs / yTy;
            return DotProduct(d, d) / gamma;
        }

        private static double DotProduct(double[] a, double[] b)
        {
            var sum = 0.0;
            for (var i = 0; i < a.Length; i++)
            {
                sum += a[i] * b[i];
            }
            return sum;
        }

        private static bool AllZero(double[] d)
        {
            for (var i = 0; i < d.Length; i++)
            {
                if (Math.Abs(d[i]) > 1e-16)
                {
                    return false;
                }
            }
            return true;
        }
    }
}
