/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Mma.Tests
{
    /// <summary>
    /// A synthetic MMA subproblem shaped like local volume constraints in topology optimisation: the variables are the
    /// cells of an nx × ny × nz grid with a compliance-like objective Σ c_j/x_j; each constraint bounds the mean density
    /// of a 3 × 3 × 3 window, the windows centred every 2 cells in x and y and every 2.5 in z so that neighbours share
    /// cells; optionally one dense volume row (Σ x_j - 0.32 n)/27, scaled like the local rows. The approximation is
    /// built at a point 0.3–0.4 with the first iteration's asymptotes and a 0.2 move limit.
    /// </summary>
    internal sealed class SyntheticSubproblem
    {
        private readonly MmaApproximation _approximation;
        private readonly MmaSubproblemSolver _solver;
        private readonly double[] _low;
        private readonly double[] _upp;
        private readonly double[] _alpha;
        private readonly double[] _beta;

        private SyntheticSubproblem(ConstraintMatrix matrix, MmaApproximation approximation, MmaSubproblemSolver solver, double[] low, double[] upp, double[] alpha, double[] beta)
        {
            Matrix = matrix;
            _approximation = approximation;
            _solver = solver;
            _low = low;
            _upp = upp;
            _alpha = alpha;
            _beta = beta;
        }

        public ConstraintMatrix Matrix { get; }

        public MmaSubproblemSolver Solver => _solver;

        public int N => Matrix.ColumnCount;

        public int M => Matrix.RowCount;

        public int Stored => Matrix.StoredCount;

        public static SyntheticSubproblem Create(int nx, int ny, int nz, bool includeVolume, MmaLinearSolver linearSolver)
        {
            var n = nx * ny * nz;
            var (pointers, columns) = Windows(nx, ny, nz);
            var blocks = new List<ConstraintSparsity> { ConstraintSparsity.Csr(n, pointers, columns) };
            if (includeVolume)
            {
                blocks.Add(ConstraintSparsity.Dense(1, n));
            }

            var matrix = ConstraintMatrix.Create(n, blocks);
            var m = matrix.RowCount;
            var x = new double[n];
            var df0 = new double[n];
            var range = new double[n];
            var rangeInverse = new double[n];
            var low = new double[n];
            var upp = new double[n];
            var alpha = new double[n];
            var beta = new double[n];
            var lower = new double[n];
            var upper = new double[n];
            var f0 = 0.0;
            for (var j = 0; j < n; j++)
            {
                x[j] = 0.3 + (0.1 * Hash(j));
                var c = 0.5 + Hash(j + 7919);
                f0 += c / x[j];
                df0[j] = -c / (x[j] * x[j]);
                lower[j] = 1e-3;
                upper[j] = 1.0;
                range[j] = upper[j] - lower[j];
                rangeInverse[j] = 1.0 / range[j];
            }

            var g = new double[m];
            var dg = new double[matrix.StoredCount];
            for (var i = 0; i < m; i++)
            {
                var offset = matrix.RowOffset(i);
                var length = matrix.RowLength(i);
                var dense = matrix.IsDense(i);
                var limit = dense ? 0.32 : 0.3 + (0.05 * (i % 3));
                var scale = dense ? 1.0 / 27.0 : 1.0 / length;
                var sum = 0.0;
                for (var t = 0; t < length; t++)
                {
                    var j = dense ? t : matrix.RowColumns(i)[t];
                    dg[offset + t] = scale;
                    sum += x[j];
                }

                g[i] = scale * (sum - (limit * length));
            }

            var options = new MmaOptions { MoveLimit = 0.2, LinearSolver = linearSolver };
            Asymptotes.Update(1, x, x, x, range, low, upp, options);
            Asymptotes.Box(x, low, upp, lower, upper, range, alpha, beta, options);
            var rho = new double[m];
            Array.Fill(rho, options.Regularization);
            var approximation = new MmaApproximation(matrix);
            approximation.Build(x, rangeInverse, low, upp, f0, df0, g, dg, options.Regularization, rho);
            var solver = new MmaSubproblemSolver(matrix, SubproblemSettings.From(options, m));
            return new SyntheticSubproblem(matrix, approximation, solver, low, upp, alpha, beta);
        }

        public bool Solve()
        {
            return _solver.Solve(_approximation, _low, _upp, _alpha, _beta);
        }

        public string Describe()
        {
            var approximated = new double[M];
            _approximation.Constraints(_solver.X, _low, _upp, approximated);
            var violation = 0.0;
            var active = 0;
            for (var i = 0; i < M; i++)
            {
                violation = Math.Max(violation, approximated[i]);
                active += _solver.Lambda[i] > 1e-6 ? 1 : 0;
            }

            return $"Newton {_solver.NewtonIterations}, residuals {_solver.ResidualEvaluations}, CG {_solver.LinearIterations} (unconverged solves {_solver.UnconvergedLinearSolves}), m×m {_solver.ConstraintSpace}, direct {_solver.DirectLinearSolve}; largest max(0, f̃_i) {violation:E3}, {active} active";
        }

        /// <summary>
        /// Asserts that the solution is feasible for the approximated constraints with y = 0, inside the box.
        /// </summary>
        public void AssertSolutionIsFeasible()
        {
            var approximated = new double[M];
            _approximation.Constraints(_solver.X, _low, _upp, approximated);
            for (var i = 0; i < M; i++)
            {
                Assert.IsLessThanOrEqualTo(1e-6, approximated[i], $"constraint {i}");
                Assert.IsLessThanOrEqualTo(1e-6, _solver.Y[i], $"artificial y of constraint {i}");
            }

            for (var j = 0; j < N; j++)
            {
                Assert.IsTrue(_solver.X[j] > _alpha[j] && _solver.X[j] < _beta[j], $"x{j} outside (α, β)");
            }
        }

        private static (int[] Pointers, int[] Columns) Windows(int nx, int ny, int nz)
        {
            var pointers = new List<int> { 0 };
            var columns = new List<int>();
            for (var a = 0; a < nx / 2; a++)
            {
                for (var b = 0; b < ny / 2; b++)
                {
                    for (var c = 0; c < (int)(nz / 2.5); c++)
                    {
                        AddWindow((2 * a) + 1, (2 * b) + 1, (int)(2.5 * c) + 1, nx, ny, nz, columns);
                        pointers.Add(columns.Count);
                    }
                }
            }

            return (pointers.ToArray(), columns.ToArray());
        }

        private static void AddWindow(int cx, int cy, int cz, int nx, int ny, int nz, List<int> columns)
        {
            var window = new List<int>();
            for (var dz = -1; dz <= 1; dz++)
            {
                for (var dy = -1; dy <= 1; dy++)
                {
                    for (var dx = -1; dx <= 1; dx++)
                    {
                        var (x, y, z) = (cx + dx, cy + dy, cz + dz);
                        if (x >= 0 && x < nx && y >= 0 && y < ny && z >= 0 && z < nz)
                        {
                            window.Add(x + (nx * (y + (ny * z))));
                        }
                    }
                }
            }

            window.Sort();
            columns.AddRange(window);
        }

        private static double Hash(int j)
        {
            var h = (uint)j * 2654435761u;
            h ^= h >> 15;
            h *= 2246822519u;
            h ^= h >> 13;
            return h / (double)uint.MaxValue;
        }
    }
}
