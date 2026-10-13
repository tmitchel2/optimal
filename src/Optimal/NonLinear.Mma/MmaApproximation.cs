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
    /// The separable convex approximations of the objective and the constraints at x_k (Svanberg 2007, §2 and §5):
    /// f̃_i(x) = r_i + Σ_j p_ij/(U_j - x_j) + q_ij/(x_j - L_j), with
    /// p_ij = (U_j - x_j)²·(1.001 (∂f_i/∂x_j)⁺ + 0.001 (∂f_i/∂x_j)⁻ + ρ_i/(x_max,j - x_min,j)),
    /// q_ij = (x_j - L_j)²·(0.001 (∂f_i/∂x_j)⁺ + 1.001 (∂f_i/∂x_j)⁻ + ρ_i/(x_max,j - x_min,j)).
    /// MMA uses ρ_i = raa0 = 1e-5; GCMMA raises ρ_i until the approximations are conservative. A sparse row's terms are
    /// formed on its pattern only.
    /// </summary>
    internal sealed class MmaApproximation
    {
        private readonly ConstraintMatrix _matrix;

        public MmaApproximation(ConstraintMatrix matrix)
        {
            _matrix = matrix;
            var n = matrix.ColumnCount;
            P0 = new double[n];
            Q0 = new double[n];
            P = new double[matrix.StoredCount];
            Q = new double[matrix.StoredCount];
            B = new double[matrix.RowCount];
            Scratch = new double[2 * n];
        }

        /// <summary>Gets p_0j.</summary>
        public double[] P0 { get; }

        /// <summary>Gets q_0j.</summary>
        public double[] Q0 { get; }

        /// <summary>Gets p_ij, in the constraint matrix's layout.</summary>
        public double[] P { get; }

        /// <summary>Gets q_ij, in the constraint matrix's layout.</summary>
        public double[] Q { get; }

        /// <summary>Gets b_i = -r_i, the right-hand sides of the subproblem's constraints.</summary>
        public double[] B { get; }

        /// <summary>Gets r_0, the objective approximation's constant.</summary>
        public double R0 { get; private set; }

        private double[] Scratch { get; }

        /// <summary>
        /// Builds the approximations at x from the objective and constraint values and gradients there.
        /// </summary>
        public void Build(
            ReadOnlySpan<double> x,
            ReadOnlySpan<double> rangeInverse,
            ReadOnlySpan<double> low,
            ReadOnlySpan<double> upp,
            double f0,
            ReadOnlySpan<double> df0,
            ReadOnlySpan<double> g,
            ReadOnlySpan<double> dg,
            double rho0,
            ReadOnlySpan<double> rho)
        {
            var n = x.Length;
            var uxInverse = Scratch.AsSpan(0, n);
            var xlInverse = Scratch.AsSpan(n, n);
            var r0 = f0;
            for (var j = 0; j < n; j++)
            {
                var ux = upp[j] - x[j];
                var xl = x[j] - low[j];
                var (p, q) = Terms(df0[j], rho0 * rangeInverse[j]);
                P0[j] = p * ux * ux;
                Q0[j] = q * xl * xl;
                uxInverse[j] = 1.0 / ux;
                xlInverse[j] = 1.0 / xl;
                r0 -= P0[j] * uxInverse[j] + Q0[j] * xlInverse[j];
            }

            R0 = r0;
            for (var i = 0; i < _matrix.RowCount; i++)
            {
                BuildRow(i, x, rangeInverse, low, upp, dg, rho[i]);
            }

            _matrix.MultiplyPair(P, uxInverse, Q, xlInverse, B);
            for (var i = 0; i < B.Length; i++)
            {
                B[i] -= g[i];
            }
        }

        /// <summary>
        /// f̃_0 at a point of the subproblem's box.
        /// </summary>
        public double Objective(ReadOnlySpan<double> x, ReadOnlySpan<double> low, ReadOnlySpan<double> upp)
        {
            var value = R0;
            for (var j = 0; j < x.Length; j++)
            {
                value += P0[j] / (upp[j] - x[j]) + Q0[j] / (x[j] - low[j]);
            }

            return value;
        }

        /// <summary>
        /// f̃_i at a point of the subproblem's box, for every constraint.
        /// </summary>
        public void Constraints(ReadOnlySpan<double> x, ReadOnlySpan<double> low, ReadOnlySpan<double> upp, Span<double> result)
        {
            var n = x.Length;
            var uxInverse = Scratch.AsSpan(0, n);
            var xlInverse = Scratch.AsSpan(n, n);
            for (var j = 0; j < n; j++)
            {
                uxInverse[j] = 1.0 / (upp[j] - x[j]);
                xlInverse[j] = 1.0 / (x[j] - low[j]);
            }

            _matrix.MultiplyPair(P, uxInverse, Q, xlInverse, result);
            for (var i = 0; i < result.Length; i++)
            {
                result[i] -= B[i];
            }
        }

        private void BuildRow(
            int row,
            ReadOnlySpan<double> x,
            ReadOnlySpan<double> rangeInverse,
            ReadOnlySpan<double> low,
            ReadOnlySpan<double> upp,
            ReadOnlySpan<double> dg,
            double rho)
        {
            var offset = _matrix.RowOffset(row);
            var length = _matrix.RowLength(row);
            var dense = _matrix.IsDense(row);
            var columns = _matrix.RowColumns(row);
            var gradient = dg.Slice(offset, length);
            var p = P.AsSpan(offset, length);
            var q = Q.AsSpan(offset, length);
            for (var t = 0; t < length; t++)
            {
                var j = dense ? t : columns[t];
                var ux = upp[j] - x[j];
                var xl = x[j] - low[j];
                var (pt, qt) = Terms(gradient[t], rho * rangeInverse[j]);
                p[t] = pt * ux * ux;
                q[t] = qt * xl * xl;
            }
        }

        private static (double P, double Q) Terms(double derivative, double regularization)
        {
            var positive = Math.Max(derivative, 0.0);
            var negative = Math.Max(-derivative, 0.0);
            var shared = 0.001 * (positive + negative) + regularization;
            return (positive + shared, negative + shared);
        }
    }
}
