/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Buffers;
using System.Numerics.Tensors;

namespace Optimal.NonLinear
{
    /// <summary>
    /// SIMD-accelerated vector operations using TensorPrimitives.
    /// Provides high-performance implementations for common numerical operations.
    /// </summary>
    public static class VectorOps
    {
        /// <summary>
        /// Computes the dot product of two vectors using SIMD acceleration.
        /// </summary>
        /// <param name="a">First vector.</param>
        /// <param name="b">Second vector.</param>
        /// <returns>The dot product a · b.</returns>
        public static double Dot(ReadOnlySpan<double> a, ReadOnlySpan<double> b)
            => TensorPrimitives.Dot(a, b);

        /// <summary>
        /// Computes result = a + scale * b using SIMD acceleration.
        /// </summary>
        /// <param name="a">First vector.</param>
        /// <param name="scale">Scalar multiplier for b.</param>
        /// <param name="b">Second vector.</param>
        /// <param name="result">Output vector (can be same as a or b).</param>
        public static void AddScaled(ReadOnlySpan<double> a, double scale,
            ReadOnlySpan<double> b, Span<double> result)
        {
            // TensorPrimitives.MultiplyAdd computes: result[i] = b[i] * scale + a[i]
            TensorPrimitives.MultiplyAdd(b, scale, a, result);
        }

        /// <summary>
        /// Computes result = scale * a using SIMD acceleration.
        /// </summary>
        /// <param name="a">Input vector.</param>
        /// <param name="scale">Scalar multiplier.</param>
        /// <param name="result">Output vector (can be same as a).</param>
        public static void Scale(ReadOnlySpan<double> a, double scale, Span<double> result)
            => TensorPrimitives.Multiply(a, scale, result);

        /// <summary>
        /// Computes result = -a using SIMD acceleration.
        /// </summary>
        /// <param name="a">Input vector.</param>
        /// <param name="result">Output vector (can be same as a).</param>
        public static void Negate(ReadOnlySpan<double> a, Span<double> result)
            => TensorPrimitives.Negate(a, result);

        /// <summary>
        /// Copies source to destination using optimized memory operations.
        /// </summary>
        /// <param name="source">Source span.</param>
        /// <param name="dest">Destination span.</param>
        public static void Copy(ReadOnlySpan<double> source, Span<double> dest)
            => source.CopyTo(dest);

        /// <summary>
        /// Computes result = a + b using SIMD acceleration.
        /// </summary>
        /// <param name="a">First vector.</param>
        /// <param name="b">Second vector.</param>
        /// <param name="result">Output vector (can be same as a or b).</param>
        public static void Add(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> result)
            => TensorPrimitives.Add(a, b, result);

        /// <summary>
        /// Computes result = a - b using SIMD acceleration.
        /// </summary>
        /// <param name="a">First vector.</param>
        /// <param name="b">Second vector.</param>
        /// <param name="result">Output vector (can be same as a or b).</param>
        public static void Subtract(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> result)
            => TensorPrimitives.Subtract(a, b, result);

        /// <summary>
        /// Computes the sum of squared elements (squared L2 norm) using SIMD acceleration.
        /// </summary>
        /// <param name="a">Input vector.</param>
        /// <returns>Sum of a[i]^2.</returns>
        public static double SumOfSquares(ReadOnlySpan<double> a)
            => TensorPrimitives.Dot(a, a);

        /// <summary>
        /// Computes the L2 norm (Euclidean length) using SIMD acceleration.
        /// </summary>
        /// <param name="a">Input vector.</param>
        /// <returns>||a||_2 = sqrt(sum(a[i]^2)).</returns>
        public static double Norm2(ReadOnlySpan<double> a)
            => Math.Sqrt(TensorPrimitives.Dot(a, a));

        /// <summary>
        /// Rents an array from the shared pool.
        /// The returned array may be larger than the requested size.
        /// </summary>
        /// <param name="minimumLength">Minimum required length.</param>
        /// <returns>A pooled array of at least the requested length.</returns>
        public static double[] RentArray(int minimumLength)
            => ArrayPool<double>.Shared.Rent(minimumLength);

        /// <summary>
        /// Returns a rented array to the shared pool.
        /// </summary>
        /// <param name="array">The array to return.</param>
        /// <param name="clearArray">If true, clears the array before returning (default false).</param>
        public static void ReturnArray(double[] array, bool clearArray = false)
            => ArrayPool<double>.Shared.Return(array, clearArray);
    }
}
