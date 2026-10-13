/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Mma.Tests
{
    /// <summary>
    /// Published test problems for MMA, GCMMA and the sparse constraint path.
    /// </summary>
    internal static class MmaTestProblems
    {
        private static readonly double[] s_cantileverCoefficients = [61.0, 37.0, 19.0, 7.0, 1.0];

        /// <summary>
        /// Svanberg's cantilever beam of five hollow square segments (K. Svanberg, "The method of moving asymptotes —
        /// a new method for structural optimization", Int. J. Numer. Meth. Engng 24 (1987) 359–373; repeated in
        /// Svanberg's 2007 notes "MMA and GCMMA – two methods for nonlinear optimization"):
        /// minimize 0.0624 Σ x_j subject to 61/x1³ + 37/x2³ + 19/x3³ + 7/x4³ + 1/x5³ &lt;= 1, 1 &lt;= x_j &lt;= 10, from
        /// x = 5. Published optimum: f* = 1.340 at x* = (6.016, 5.309, 4.494, 3.502, 2.153).
        /// </summary>
        public static InequalityProblem Cantilever()
        {
            var block = new ConstraintBlock(ConstraintSparsity.Dense(1, 5), (x, values, jacobian) =>
            {
                var sum = 0.0;
                for (var j = 0; j < 5; j++)
                {
                    sum += s_cantileverCoefficients[j] / (x[j] * x[j] * x[j]);
                    if (!jacobian.IsEmpty)
                    {
                        jacobian[j] = -3.0 * s_cantileverCoefficients[j] / (x[j] * x[j] * x[j] * x[j]);
                    }
                }

                values[0] = sum - 1.0;
            });
            return new InequalityProblem
            {
                Objective = (x, gradient) =>
                {
                    gradient.Fill(0.0624);
                    var sum = 0.0;
                    foreach (var value in x)
                    {
                        sum += value;
                    }

                    return 0.0624 * sum;
                },
                Constraints = [block],
                LowerBounds = [1.0, 1.0, 1.0, 1.0, 1.0],
                UpperBounds = [10.0, 10.0, 10.0, 10.0, 10.0],
            };
        }

        /// <summary>
        /// The cantilever's analytic optimum: x_j = a_j^(1/4) (Σ_k a_k^(1/4))^(1/3), from stationarity of
        /// 0.0624 Σ x_j + λ Σ a_j/x_j³ with the constraint active.
        /// </summary>
        public static double[] CantileverAnalyticOptimum()
        {
            var sum = 0.0;
            foreach (var a in s_cantileverCoefficients)
            {
                sum += Math.Pow(a, 0.25);
            }

            var scale = Math.Cbrt(sum);
            return Array.ConvertAll(s_cantileverCoefficients, a => Math.Pow(a, 0.25) * scale);
        }

        /// <summary>
        /// Svanberg's two-bar truss (Svanberg 1987, ibid.): x1 the bars' cross-section, x2 half the span over the
        /// height. minimize x1 √(1 + x2²) subject to 0.124 √(1 + x2²) (8/x1 ± 1/(x1 x2)) &lt;= 1, 0.2 &lt;= x1 &lt;= 4,
        /// 0.1 &lt;= x2 &lt;= 1.6, from (1.5, 0.5). Published optimum: x* = (1.41, 0.38), f* = 1.51.
        /// </summary>
        public static InequalityProblem TwoBarTruss()
        {
            var block = new ConstraintBlock(ConstraintSparsity.Dense(2, 2), (x, values, jacobian) =>
            {
                var s = Math.Sqrt(1.0 + (x[1] * x[1]));
                for (var i = 0; i < 2; i++)
                {
                    var sign = i == 0 ? 1.0 : -1.0;
                    var h = s * (8.0 + (sign / x[1]));
                    values[i] = (0.124 * h / x[0]) - 1.0;
                    if (!jacobian.IsEmpty)
                    {
                        var dh = (x[1] / s * (8.0 + (sign / x[1]))) - (sign * s / (x[1] * x[1]));
                        jacobian[(2 * i) + 0] = -0.124 * h / (x[0] * x[0]);
                        jacobian[(2 * i) + 1] = 0.124 * dh / x[0];
                    }
                }
            });
            return new InequalityProblem
            {
                Objective = (x, gradient) =>
                {
                    var s = Math.Sqrt(1.0 + (x[1] * x[1]));
                    if (!gradient.IsEmpty)
                    {
                        gradient[0] = s;
                        gradient[1] = x[0] * x[1] / s;
                    }

                    return x[0] * s;
                },
                Constraints = [block],
                LowerBounds = [0.2, 0.1],
                UpperBounds = [4.0, 1.6],
            };
        }

        /// <summary>
        /// The truss's optimum with the first constraint active: x2 solves 16 x2³ + x2² - 1 = 0 (stationarity of
        /// (1 + x2²)(8 + 1/x2)), and x1 = 0.124 √(1 + x2²) (8 + 1/x2).
        /// </summary>
        public static (double X1, double X2, double F) TwoBarTrussAnalyticOptimum()
        {
            var t = 0.4;
            for (var k = 0; k < 50; k++)
            {
                t -= ((16.0 * t * t * t) + (t * t) - 1.0) / ((48.0 * t * t) + (2.0 * t));
            }

            var s = Math.Sqrt(1.0 + (t * t));
            var x1 = 0.124 * s * (8.0 + (1.0 / t));
            return (x1, t, x1 * s);
        }

        /// <summary>
        /// Rosenbrock's function constrained by a cubic and a line (a standard nonconvex test, e.g. Simionescu's
        /// collection): minimize (1 - x)² + 100 (y - x²)² subject to (x - 1)³ - y + 1 &lt;= 0 and x + y - 2 &lt;= 0,
        /// -1.5 &lt;= x &lt;= 1.5, -0.5 &lt;= y &lt;= 2.5. The global optimum is f(1, 1) = 0, where both constraints are active.
        /// </summary>
        public static InequalityProblem ConstrainedRosenbrock()
        {
            var block = new ConstraintBlock(ConstraintSparsity.Dense(2, 2), (x, values, jacobian) =>
            {
                var u = x[0] - 1.0;
                values[0] = (u * u * u) - x[1] + 1.0;
                values[1] = x[0] + x[1] - 2.0;
                if (!jacobian.IsEmpty)
                {
                    jacobian[0] = 3.0 * u * u;
                    jacobian[1] = -1.0;
                    jacobian[2] = 1.0;
                    jacobian[3] = 1.0;
                }
            });
            return new InequalityProblem
            {
                Objective = (x, gradient) =>
                {
                    var a = 1.0 - x[0];
                    var b = x[1] - (x[0] * x[0]);
                    if (!gradient.IsEmpty)
                    {
                        gradient[0] = (-2.0 * a) - (400.0 * x[0] * b);
                        gradient[1] = 200.0 * b;
                    }

                    return (a * a) + (100.0 * b * b);
                },
                Constraints = [block],
                LowerBounds = [-1.5, -0.5],
                UpperBounds = [1.5, 2.5],
            };
        }

        /// <summary>
        /// A separable problem with local constraints: minimize Σ_j c_j/x_j + x_j subject to, for each window
        /// W_i = {i, .., i + width - 1} (wrapping), Σ_{j∈W_i} w_ij x_j - b_i &lt;= 0, 0.1 &lt;= x_j &lt;= 10. As one sparse
        /// block when <paramref name="sparse"/>, otherwise as dense rows holding the same values with explicit zeros.
        /// </summary>
        public static InequalityProblem LocalWindows(int n, int m, int width, bool sparse)
        {
            var columns = new int[m * width];
            var pointers = new int[m + 1];
            var weights = new double[m * width];
            var limits = new double[m];
            for (var i = 0; i < m; i++)
            {
                var start = i * n / m;
                var window = new int[width];
                for (var t = 0; t < width; t++)
                {
                    window[t] = (start + t) % n;
                }

                Array.Sort(window);
                for (var t = 0; t < width; t++)
                {
                    columns[(i * width) + t] = window[t];
                    weights[(i * width) + t] = 1.0 + (0.1 * ((window[t] * 7 + i) % 5));
                }

                pointers[i + 1] = (i + 1) * width;
                limits[i] = 0.6 * width * (1.0 + (0.05 * (i % 3)));
            }

            var structure = sparse ? ConstraintSparsity.Csr(n, pointers, columns) : ConstraintSparsity.Dense(m, n);
            var block = new ConstraintBlock(structure, (x, values, jacobian) =>
            {
                if (!jacobian.IsEmpty && !sparse)
                {
                    jacobian.Clear();
                }

                for (var i = 0; i < m; i++)
                {
                    var sum = 0.0;
                    for (var k = pointers[i]; k < pointers[i + 1]; k++)
                    {
                        sum += weights[k] * x[columns[k]];
                        if (!jacobian.IsEmpty)
                        {
                            jacobian[sparse ? k : (i * n) + columns[k]] = weights[k];
                        }
                    }

                    values[i] = sum - limits[i];
                }
            });
            var lower = new double[n];
            var upper = new double[n];
            Array.Fill(lower, 0.1);
            Array.Fill(upper, 10.0);
            return new InequalityProblem
            {
                Objective = (x, gradient) =>
                {
                    var value = 0.0;
                    for (var j = 0; j < x.Length; j++)
                    {
                        var c = 1.0 + (0.5 * (j % 4));
                        value += (c / x[j]) + x[j];
                        if (!gradient.IsEmpty)
                        {
                            gradient[j] = (-c / (x[j] * x[j])) + 1.0;
                        }
                    }

                    return value;
                },
                Constraints = [block],
                LowerBounds = lower,
                UpperBounds = upper,
            };
        }
    }
}
