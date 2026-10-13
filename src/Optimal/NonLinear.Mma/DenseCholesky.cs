/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Numerics.Tensors;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// In-place Cholesky factorisation A = LLᵀ of a dense symmetric positive definite matrix stored row-major, using
    /// only the lower triangle, and the triangular solves that follow it.
    /// </summary>
    internal static class DenseCholesky
    {
        /// <summary>
        /// Overwrites the lower triangle of the k×k matrix with L. Returns false if a pivot is not positive.
        /// </summary>
        public static bool Factor(Span<double> a, int k)
        {
            for (var i = 0; i < k; i++)
            {
                var rowI = a.Slice(i * k, k);
                for (var j = 0; j < i; j++)
                {
                    var rowJ = a.Slice(j * k, k);
                    rowI[j] = (rowI[j] - TensorPrimitives.Dot(rowI[..j], rowJ[..j])) / rowJ[j];
                }

                var pivot = rowI[i] - TensorPrimitives.Dot(rowI[..i], rowI[..i]);
                if (!(pivot > 0.0) || !double.IsFinite(pivot))
                {
                    return false;
                }

                rowI[i] = Math.Sqrt(pivot);
            }

            return true;
        }

        /// <summary>
        /// Solves LLᵀ x = b in place (b becomes x).
        /// </summary>
        public static void Solve(ReadOnlySpan<double> l, int k, Span<double> b)
        {
            for (var i = 0; i < k; i++)
            {
                var row = l.Slice(i * k, k);
                b[i] = (b[i] - TensorPrimitives.Dot(row[..i], b[..i])) / row[i];
            }

            for (var i = k - 1; i >= 0; i--)
            {
                var sum = b[i];
                for (var j = i + 1; j < k; j++)
                {
                    sum -= l[(j * k) + i] * b[j];
                }

                b[i] = sum / l[(i * k) + i];
            }
        }
    }
}
