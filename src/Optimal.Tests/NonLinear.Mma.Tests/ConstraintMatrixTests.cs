/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Mma.Tests
{
    [TestClass]
    public sealed class ConstraintMatrixTests
    {
        private const int N = 7;

        private static readonly int[] s_secondSparseRow = [1, 5];

        [TestMethod]
        public void ConcatenatesBlocksInOrder()
        {
            var matrix = Mixed();

            Assert.AreEqual(4, matrix.RowCount);
            Assert.AreEqual(N, matrix.ColumnCount);
            Assert.AreEqual(3 + N + 2 + N, matrix.StoredCount);
            Assert.AreEqual(4, matrix.BlockCount);
            Assert.IsFalse(matrix.IsDense(0));
            Assert.IsTrue(matrix.IsDense(1));
            Assert.IsFalse(matrix.IsDense(2));
            Assert.IsTrue(matrix.IsDense(3));
            Assert.AreEqual(3, matrix.BlockValueOffset(1));
            Assert.AreEqual(N, matrix.BlockValueCount(1));
            Assert.AreEqual(2, matrix.BlockRowOffset(2));
            Assert.AreEqual(1, matrix.BlockRowCount(2));
            CollectionAssert.AreEqual(s_secondSparseRow, matrix.RowColumns(2).ToArray());
        }

        [TestMethod]
        public void KernelsMatchTheDenseMatrix()
        {
            var matrix = Mixed();
            var a = Values(matrix, 1.0);
            var b = Values(matrix, -0.5);
            var dense = Dense(matrix, a);
            var denseB = Dense(matrix, b);
            var v = Vector(N, 0.3);
            var w = Vector(N, -0.7);
            var u = Vector(matrix.RowCount, 1.1);

            var product = new double[matrix.RowCount];
            matrix.Multiply(a, v, product);
            var pair = new double[matrix.RowCount];
            matrix.MultiplyPair(a, v, b, w, pair);
            var transpose = Vector(N, 0.2);
            var expectedTranspose = (double[])transpose.Clone();
            matrix.MultiplyTransposeAdd(a, u, transpose);
            for (var i = 0; i < matrix.RowCount; i++)
            {
                var expected = 0.0;
                var expectedPair = 0.0;
                for (var j = 0; j < N; j++)
                {
                    expected += dense[i, j] * v[j];
                    expectedPair += (dense[i, j] * v[j]) + (denseB[i, j] * w[j]);
                    expectedTranspose[j] += dense[i, j] * u[i];
                }

                Assert.AreEqual(expected, product[i], 1e-14, $"row {i}");
                Assert.AreEqual(expectedPair, pair[i], 1e-14, $"row {i}");
            }

            for (var j = 0; j < N; j++)
            {
                Assert.AreEqual(expectedTranspose[j], transpose[j], 1e-14, $"column {j}");
            }
        }

        [TestMethod]
        public void FusedKernelMatchesItsParts()
        {
            var matrix = Mixed();
            var p = Values(matrix, 1.0);
            var q = Values(matrix, 2.0);
            var u = Vector(matrix.RowCount, 0.4);
            var v = Vector(N, 0.3);
            var w = Vector(N, 0.9);
            var pTarget = Vector(N, 1.0);
            var qTarget = Vector(N, 2.0);
            var result = new double[matrix.RowCount];
            matrix.TransposeAddAndMultiplyPair(p, q, u, pTarget, qTarget, v, w, result);

            var pExpected = Vector(N, 1.0);
            var qExpected = Vector(N, 2.0);
            var expected = new double[matrix.RowCount];
            matrix.MultiplyTransposeAdd(p, u, pExpected);
            matrix.MultiplyTransposeAdd(q, u, qExpected);
            matrix.MultiplyPair(p, v, q, w, expected);
            for (var j = 0; j < N; j++)
            {
                Assert.AreEqual(pExpected[j], pTarget[j], 1e-14);
                Assert.AreEqual(qExpected[j], qTarget[j], 1e-14);
            }

            for (var i = 0; i < matrix.RowCount; i++)
            {
                Assert.AreEqual(expected[i], result[i], 1e-14);
            }
        }

        [TestMethod]
        public void WeightedSquaresAreTheDiagonals()
        {
            var matrix = Mixed();
            var a = Values(matrix, 1.0);
            var dense = Dense(matrix, a);
            var w = Vector(N, 0.5, positive: true);
            var u = Vector(matrix.RowCount, 0.5, positive: true);
            var rows = new double[matrix.RowCount];
            matrix.RowWeightedSquares(a, w, rows);
            var columns = new double[N];
            matrix.ColumnWeightedSquaresAdd(a, u, columns);
            var absolute = new double[matrix.RowCount];
            matrix.RowWeightedAbsoluteSums(a, w, absolute);
            for (var i = 0; i < matrix.RowCount; i++)
            {
                var expected = 0.0;
                var expectedAbsolute = 0.0;
                for (var j = 0; j < N; j++)
                {
                    expected += dense[i, j] * dense[i, j] * w[j];
                    expectedAbsolute += Math.Abs(dense[i, j]) * w[j];
                }

                Assert.AreEqual(expected, rows[i], 1e-14);
                Assert.AreEqual(expectedAbsolute, absolute[i], 1e-14);
            }

            for (var j = 0; j < N; j++)
            {
                var expected = 0.0;
                for (var i = 0; i < matrix.RowCount; i++)
                {
                    expected += u[i] * dense[i, j] * dense[i, j];
                }

                Assert.AreEqual(expected, columns[j], 1e-14);
            }
        }

        [TestMethod]
        public void RejectsVectorsOfTheWrongLength()
        {
            var matrix = Mixed();
            var a = Values(matrix, 1.0);

            Assert.ThrowsExactly<ArgumentException>(() => matrix.Multiply(a, new double[N - 1], new double[matrix.RowCount]));
            Assert.ThrowsExactly<ArgumentException>(() => matrix.MultiplyTransposeAdd(a, new double[matrix.RowCount + 1], new double[N]));
            Assert.ThrowsExactly<ArgumentException>(() => matrix.Multiply(new double[3], new double[N], new double[matrix.RowCount]));
        }

        /// <summary>
        /// Rows: sparse {0, 3, 6}; dense; sparse {1, 5}; dense.
        /// </summary>
        internal static ConstraintMatrix Mixed()
        {
            return ConstraintMatrix.Create(N, [
                ConstraintSparsity.Csr(N, [0, 3], [0, 3, 6]),
                ConstraintSparsity.Dense(1, N),
                ConstraintSparsity.Csr(N, [0, 2], [1, 5]),
                ConstraintSparsity.Dense(1, N),
            ]);
        }

        internal static double[] Values(ConstraintMatrix matrix, double seed)
        {
            var values = new double[matrix.StoredCount];
            for (var k = 0; k < values.Length; k++)
            {
                values[k] = Math.Sin((k + 1) * seed * 1.7) + (0.1 * seed);
            }

            return values;
        }

        internal static double[,] Dense(ConstraintMatrix matrix, double[] values)
        {
            var dense = new double[matrix.RowCount, matrix.ColumnCount];
            for (var i = 0; i < matrix.RowCount; i++)
            {
                var offset = matrix.RowOffset(i);
                var columns = matrix.RowColumns(i);
                for (var t = 0; t < matrix.RowLength(i); t++)
                {
                    dense[i, matrix.IsDense(i) ? t : columns[t]] = values[offset + t];
                }
            }

            return dense;
        }

        internal static double[] Vector(int length, double seed, bool positive = false)
        {
            var vector = new double[length];
            for (var k = 0; k < length; k++)
            {
                var value = Math.Cos((k + 1) * seed * 2.3);
                vector[k] = positive ? 0.5 + Math.Abs(value) : value;
            }

            return vector;
        }
    }
}
