/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.Control
{
    /// <summary>
    /// Provides Legendre-Gauss-Lobatto (LGL) quadrature points, weights, and differentiation matrices.
    /// LGL collocation uses the endpoints and roots of the derivative of Legendre polynomials
    /// as collocation points, providing high-order accurate integration and differentiation.
    /// </summary>
    public static class LegendreGaussLobatto
    {
        private static readonly Dictionary<int, CachedLGLData> s_cache = new();
        private static readonly object s_lock = new();

        private sealed class CachedLGLData
        {
            public double[] Points { get; init; } = Array.Empty<double>();
            public double[] Weights { get; init; } = Array.Empty<double>();
            public double[,] DifferentiationMatrix { get; init; } = new double[0, 0];
        }

        /// <summary>
        /// Gets the LGL points and quadrature weights for a given order.
        /// Results are cached for performance.
        /// </summary>
        /// <param name="order">Number of LGL points (must be >= 2).</param>
        /// <returns>Tuple of (points, weights) on the interval [-1, 1].</returns>
        public static (double[] points, double[] weights) GetPointsAndWeights(int order)
        {
            if (order < 2)
            {
                throw new ArgumentException("Order must be at least 2.", nameof(order));
            }

            lock (s_lock)
            {
                if (!s_cache.TryGetValue(order, out var cached))
                {
                    var points = ComputeLGLPoints(order);
                    var weights = ComputeLGLWeights(points, order);
                    var diffMatrix = ComputeDifferentiationMatrix(points, order);

                    cached = new CachedLGLData
                    {
                        Points = points,
                        Weights = weights,
                        DifferentiationMatrix = diffMatrix
                    };

                    s_cache[order] = cached;
                }

                return (cached.Points, cached.Weights);
            }
        }

        /// <summary>
        /// Gets the LGL differentiation matrix for a given order.
        /// The matrix D satisfies: (df/dτ)|_{τ_i} ≈ Σ_j D_{ij} f(τ_j)
        /// </summary>
        /// <param name="order">Number of LGL points (must be >= 2).</param>
        /// <returns>Differentiation matrix of size order × order.</returns>
        public static double[,] GetDifferentiationMatrix(int order)
        {
            if (order < 2)
            {
                throw new ArgumentException("Order must be at least 2.", nameof(order));
            }

            lock (s_lock)
            {
                if (!s_cache.TryGetValue(order, out var cached))
                {
                    // Trigger computation and caching
                    GetPointsAndWeights(order);
                    cached = s_cache[order];
                }

                return cached.DifferentiationMatrix;
            }
        }

        /// <summary>
        /// Computes the LGL collocation points on [-1, 1].
        /// Uses analytically known values for common orders and Newton-Raphson for higher orders.
        /// </summary>
        private static double[] ComputeLGLPoints(int order)
        {
            var points = new double[order];

            // Endpoints are always -1 and +1
            points[0] = -1.0;
            points[order - 1] = 1.0;

            if (order == 2)
            {
                return points;
            }

            // Use analytically known values for common orders
            if (order == 3)
            {
                points[1] = 0.0;
                return points;
            }

            if (order == 4)
            {
                // Interior points are ±√(1/5)
                points[1] = -Math.Sqrt(0.2);
                points[2] = Math.Sqrt(0.2);
                return points;
            }

            if (order == 5)
            {
                // Interior points are 0 and ±√(3/7)
                points[1] = -Math.Sqrt(3.0 / 7.0);
                points[2] = 0.0;
                points[3] = Math.Sqrt(3.0 / 7.0);
                return points;
            }

            // For order > 5, use Newton's method with improved algorithm
            // Based on Trefethen "Spectral Methods in MATLAB"
            var n = order - 1;

            for (var i = 1; i < order - 1; i++)
            {
                // Initial guess using improved Chebyshev nodes
                var theta = Math.PI * i / n;
                var x = -Math.Cos(theta);

                // Newton iteration
                for (var iter = 0; iter < 100; iter++)
                {
                    var (p, dp) = EvaluateLegendreAndFirstDerivative(n, x);

                    if (Math.Abs(dp) < 1e-15)
                    {
                        break;
                    }

                    // Second derivative using three-term recurrence
                    // P''_n = [n*xP'_n - n*P_{n-1}]/(x²-1) but we use a more stable form
                    var oneMinusXSq = 1.0 - x * x;
                    if (Math.Abs(oneMinusXSq) < 1e-14)
                    {
                        break;
                    }

                    var ddp = (2.0 * x * dp - n * (n + 1) * p) / oneMinusXSq;

                    if (Math.Abs(ddp) < 1e-14)
                    {
                        break;
                    }

                    x -= dp / ddp;

                    if (Math.Abs(dp) < 1e-15)
                    {
                        break;
                    }
                }

                points[i] = x;
            }

            return points;
        }

        /// <summary>
        /// Evaluates P_n(x) and P'_n(x) using standard three-term recurrence.
        /// More stable than computing second derivatives directly.
        /// </summary>
        private static (double p, double dp) EvaluateLegendreAndFirstDerivative(int n, double x)
        {
            if (n == 0)
            {
                return (1.0, 0.0);
            }
            if (n == 1)
            {
                return (x, 1.0);
            }

            var p0 = 1.0;
            var p1 = x;
            var p2 = 0.0;

            for (var k = 1; k < n; k++)
            {
                p2 = ((2 * k + 1) * x * p1 - k * p0) / (k + 1);
                p0 = p1;
                p1 = p2;
            }

            // p2 is now P_n(x), p0 is P_{n-1}(x)
            // Use relation: (1-x²)P'_n = n[P_{n-1} - xP_n]
            var dp = n * (p0 - x * p2) / (1.0 - x * x);

            return (p2, dp);
        }

        /// <summary>
        /// Computes the LGL quadrature weights.
        /// Weight formula: w_i = 2 / [order * (order - 1) * P_{order-1}(x_i)^2]
        /// </summary>
        private static double[] ComputeLGLWeights(double[] points, int order)
        {
            var weights = new double[order];
            var n = order - 1;

            for (var i = 0; i < order; i++)
            {
                var pn = EvaluateLegendrePolynomial(n, points[i]);
                weights[i] = 2.0 / (order * n * pn * pn);
            }

            return weights;
        }

        /// <summary>
        /// Computes the LGL differentiation matrix.
        /// For i ≠ j: D_{ij} = P_{n}(x_i) / [P_{n}(x_j) * (x_i - x_j)]
        /// For i = j = 0: D_{00} = -n(n+1)/4
        /// For i = j = n: D_{nn} = n(n+1)/4
        /// For other i = j: D_{ii} = 0
        /// </summary>
        private static double[,] ComputeDifferentiationMatrix(double[] points, int order)
        {
            var D = new double[order, order];
            var n = order - 1;

            // Evaluate Legendre polynomial at all points
            var pValues = new double[order];
            for (var i = 0; i < order; i++)
            {
                pValues[i] = EvaluateLegendrePolynomial(n, points[i]);
            }

            // Compute matrix entries
            for (var i = 0; i < order; i++)
            {
                for (var j = 0; j < order; j++)
                {
                    if (i == j)
                    {
                        // Diagonal entries
                        if (i == 0)
                        {
                            D[i, j] = -n * (n + 1) / 4.0;
                        }
                        else if (i == order - 1)
                        {
                            D[i, j] = n * (n + 1) / 4.0;
                        }
                        else
                        {
                            D[i, j] = 0.0;
                        }
                    }
                    else
                    {
                        // Off-diagonal entries
                        D[i, j] = pValues[i] / (pValues[j] * (points[i] - points[j]));
                    }
                }
            }

            return D;
        }

        /// <summary>
        /// Evaluates the Legendre polynomial P_n(x) using recurrence relations.
        /// P_0(x) = 1, P_1(x) = x
        /// (n+1) P_{n+1}(x) = (2n+1) x P_n(x) - n P_{n-1}(x)
        /// </summary>
        private static double EvaluateLegendrePolynomial(int n, double x)
        {
            if (n == 0)
            {
                return 1.0;
            }
            if (n == 1)
            {
                return x;
            }

            var p0 = 1.0;
            var p1 = x;
            var p2 = 0.0;

            for (var k = 1; k < n; k++)
            {
                p2 = ((2 * k + 1) * x * p1 - k * p0) / (k + 1);
                p0 = p1;
                p1 = p2;
            }

            return p2;
        }

        /// <summary>
        /// Evaluates P_n(x), P'_n(x), and P''_n(x) using stable recurrence relations.
        /// Returns (P_n, P'_n, P''_n).
        /// Uses recurrences that avoid division by (x²-1) for numerical stability.
        /// </summary>
        private static (double p, double dp, double ddp) EvaluateLegendreAndDerivatives(int n, double x)
        {
            if (n == 0)
            {
                return (1.0, 0.0, 0.0);
            }
            if (n == 1)
            {
                return (x, 1.0, 0.0);
            }

            // Build up P_n, P'_n using stable recurrence
            // P-recurrence: (k+1)P_{k+1} = (2k+1)xP_k - kP_{k-1}
            // P'-recurrence: (k+1)P'_{k+1} = (2k+1)[P_k + xP'_k] - kP'_{k-1}
            var pkm2 = 1.0;    // P_0
            var pkm1 = x;      // P_1
            var dpkm2 = 0.0;   // P'_0
            var dpkm1 = 1.0;   // P'_1

            var pk = 0.0;
            var dpk = 0.0;

            for (var k = 1; k < n; k++)
            {
                // Compute P_{k+1}
                pk = ((2 * k + 1) * x * pkm1 - k * pkm2) / (k + 1);

                // Compute P'_{k+1} using derivative recurrence
                dpk = ((2 * k + 1) * (pkm1 + x * dpkm1) - k * dpkm2) / (k + 1);

                // Shift for next iteration
                pkm2 = pkm1;
                pkm1 = pk;
                dpkm2 = dpkm1;
                dpkm1 = dpk;
            }

            // Now pk = P_n(x), dpk = P'_n(x)
            // For P''_n, use: P''_n = [2xP'_n - n(n+1)P_n]/(x²-1)
            // Near ±1, use L'Hospital's rule or explicit formulas

            var xSquared = x * x;
            double ddpk;

            if (Math.Abs(xSquared - 1.0) < 1e-10)
            {
                // Near ±1, use explicit formulas
                // P''_n(1) = n(n+1)(n-1)(n+2)/12
                // P''_n(-1) = (-1)^n * n(n+1)(n-1)(n+2)/12
                var baseValue = n * (n + 1) * (n - 1) * (n + 2) / 12.0;
                if (x > 0)
                {
                    ddpk = baseValue;
                }
                else
                {
                    ddpk = (n % 2 == 0) ? baseValue : -baseValue;
                }
            }
            else
            {
                ddpk = (2.0 * x * dpk - n * (n + 1) * pk) / (xSquared - 1.0);
            }

            return (pk, dpk, ddpk);
        }
    }
}
