/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>Small dense linear algebra built as graph nodes, unrolled at generation time.</summary>
    public static class GraphLinearAlgebra
    {
        /// <summary>
        /// x with A x = b, for the n × n matrix <paramref name="a"/> (row-major) and <paramref name="b"/>, by Gaussian
        /// elimination with partial pivoting. Each pivot choice is a run-time comparison: row i replaces the pivot row when
        /// |a_ik| &gt; |a_kk|, as selects of whole rows, so the choice costs no branch and its derivative is the derivative of
        /// the elimination it chose. A structurally zero entry is never a candidate, and constants fold.
        /// </summary>
        public static int[] Solve(ScalarGraph g, IReadOnlyList<int> a, IReadOnlyList<int> b)
        {
            var n = b.Count;
            if (a.Count != n * n)
            {
                throw new ArgumentException($"A {n}-unknown system needs {n * n} matrix entries, not {a.Count}.", nameof(a));
            }

            var m = Enumerable.Range(0, n).Select(i => a.Skip(i * n).Take(n).ToArray()).ToArray();
            var rhs = b.ToArray();
            for (var k = 0; k < n; k++)
            {
                Pivot(g, m, rhs, k);
                Eliminate(g, m, rhs, k);
            }

            var x = new int[n];
            for (var k = n - 1; k >= 0; k--)
            {
                var sum = rhs[k];
                for (var j = k + 1; j < n; j++)
                {
                    sum = g.Sub(sum, g.Mul(m[k][j], x[j]));
                }

                x[k] = g.Div(sum, m[k][k]);
            }

            return x;
        }

        // Brings the largest |a_ik|, i ≥ k, to row k by compare-and-swap down the column.
        private static void Pivot(ScalarGraph g, int[][] m, int[] rhs, int k)
        {
            var n = rhs.Length;
            for (var i = k + 1; i < n; i++)
            {
                if (g.IsConst(m[i][k], 0))
                {
                    continue;
                }

                var current = g.Unary(ScalarOp.Abs, m[k][k]);
                var candidate = g.Unary(ScalarOp.Abs, m[i][k]);
                for (var j = k; j < n; j++)
                {
                    (m[k][j], m[i][j]) = Swap(g, current, candidate, m[k][j], m[i][j]);
                }

                (rhs[k], rhs[i]) = Swap(g, current, candidate, rhs[k], rhs[i]);
            }
        }

        private static (int Pivot, int Other) Swap(ScalarGraph g, int current, int candidate, int pivotRow, int otherRow)
        {
            return (g.Select(ScalarOp.SelectLess, current, candidate, otherRow, pivotRow), g.Select(ScalarOp.SelectLess, current, candidate, pivotRow, otherRow));
        }

        private static void Eliminate(ScalarGraph g, int[][] m, int[] rhs, int k)
        {
            var n = rhs.Length;
            for (var i = k + 1; i < n; i++)
            {
                if (g.IsConst(m[i][k], 0))
                {
                    continue;
                }

                var factor = g.Div(m[i][k], m[k][k]);
                for (var j = k + 1; j < n; j++)
                {
                    m[i][j] = g.Sub(m[i][j], g.Mul(factor, m[k][j]));
                }

                rhs[i] = g.Sub(rhs[i], g.Mul(factor, rhs[k]));
            }
        }
    }
}
