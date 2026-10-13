/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Numerics.Tensors;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// The structure of the m×n constraint Jacobian, concatenated from the blocks in order: each row is either dense
    /// (n values) or sparse (CSR). Values live in caller-owned arrays of <see cref="StoredCount"/> entries laid out
    /// row by row, so the Jacobian, P, Q and the subproblem's G share one structure and a block's evaluation writes
    /// straight into its slice.
    /// </summary>
    internal sealed class ConstraintMatrix
    {
        private const int DenseRow = -1;

        private readonly int[] _rowStart;
        private readonly int[] _columnStart;
        private readonly int[] _columnIndices;
        private readonly int[] _blockRowStart;
        private readonly int[] _blockValueStart;

        private ConstraintMatrix(
            int rowCount,
            int columnCount,
            int[] rowStart,
            int[] columnStart,
            int[] columnIndices,
            int[] blockRowStart,
            int[] blockValueStart)
        {
            RowCount = rowCount;
            ColumnCount = columnCount;
            _rowStart = rowStart;
            _columnStart = columnStart;
            _columnIndices = columnIndices;
            _blockRowStart = blockRowStart;
            _blockValueStart = blockValueStart;
        }

        /// <summary>Gets m.</summary>
        public int RowCount { get; }

        /// <summary>Gets n.</summary>
        public int ColumnCount { get; }

        /// <summary>Gets the number of stored values.</summary>
        public int StoredCount => _rowStart[RowCount];

        /// <summary>Gets the number of blocks.</summary>
        public int BlockCount => _blockRowStart.Length - 1;

        /// <summary>
        /// Concatenates the blocks' structures.
        /// </summary>
        public static ConstraintMatrix Create(int columnCount, IReadOnlyList<ConstraintSparsity> blocks)
        {
            var rowCount = 0;
            var stored = 0L;
            var sparseStored = 0;
            foreach (var block in blocks)
            {
                rowCount += block.RowCount;
                stored += block.StoredCount;
                sparseStored += block.IsDense ? 0 : block.StoredCount;
            }

            if (stored > Array.MaxLength)
            {
                throw new ArgumentException($"The constraint Jacobian stores {stored} values, more than an array can hold.");
            }

            var rowStart = new int[rowCount + 1];
            var columnStart = new int[rowCount];
            var columnIndices = new int[sparseStored];
            var blockRowStart = new int[blocks.Count + 1];
            var blockValueStart = new int[blocks.Count + 1];
            var row = 0;
            var value = 0;
            var column = 0;
            for (var b = 0; b < blocks.Count; b++)
            {
                blockRowStart[b] = row;
                blockValueStart[b] = value;
                var block = blocks[b];
                for (var i = 0; i < block.RowCount; i++, row++)
                {
                    rowStart[row] = value;
                    if (block.IsDense)
                    {
                        columnStart[row] = DenseRow;
                        value += columnCount;
                        continue;
                    }

                    var start = block.RowPointers[i];
                    var length = block.RowPointers[i + 1] - start;
                    columnStart[row] = column;
                    block.ColumnIndices.Slice(start, length).CopyTo(columnIndices.AsSpan(column, length));
                    column += length;
                    value += length;
                }
            }

            rowStart[rowCount] = value;
            blockRowStart[blocks.Count] = row;
            blockValueStart[blocks.Count] = value;
            return new ConstraintMatrix(rowCount, columnCount, rowStart, columnStart, columnIndices, blockRowStart, blockValueStart);
        }

        /// <summary>Gets whether row i is dense.</summary>
        public bool IsDense(int row) => _columnStart[row] == DenseRow;

        /// <summary>Gets the offset of row i's values.</summary>
        public int RowOffset(int row) => _rowStart[row];

        /// <summary>Gets the number of values stored for row i.</summary>
        public int RowLength(int row) => _rowStart[row + 1] - _rowStart[row];

        /// <summary>Gets row i's column indices; empty for a dense row (whose columns are 0..n-1).</summary>
        public ReadOnlySpan<int> RowColumns(int row) =>
            IsDense(row) ? ReadOnlySpan<int>.Empty : _columnIndices.AsSpan(_columnStart[row], RowLength(row));

        /// <summary>Gets the first row of block b.</summary>
        public int BlockRowOffset(int block) => _blockRowStart[block];

        /// <summary>Gets the number of rows of block b.</summary>
        public int BlockRowCount(int block) => _blockRowStart[block + 1] - _blockRowStart[block];

        /// <summary>Gets the offset of block b's values.</summary>
        public int BlockValueOffset(int block) => _blockValueStart[block];

        /// <summary>Gets the number of values block b stores.</summary>
        public int BlockValueCount(int block) => _blockValueStart[block + 1] - _blockValueStart[block];

        /// <summary>
        /// result_i = Σ_j A_ij v_j + Σ_j B_ij w_j, for A and B in this structure.
        /// </summary>
        public void MultiplyPair(
            ReadOnlySpan<double> a,
            ReadOnlySpan<double> v,
            ReadOnlySpan<double> b,
            ReadOnlySpan<double> w,
            Span<double> result)
        {
            RequireLengths(a, v, result);
            RequireLengths(b, w, result);
            ref var columns = ref MemoryMarshal.GetArrayDataReference(_columnIndices);
            ref var aValues = ref MemoryMarshal.GetReference(a);
            ref var bValues = ref MemoryMarshal.GetReference(b);
            ref var vValues = ref MemoryMarshal.GetReference(v);
            ref var wValues = ref MemoryMarshal.GetReference(w);
            for (var i = 0; i < RowCount; i++)
            {
                var offset = _rowStart[i];
                var length = _rowStart[i + 1] - offset;
                var columnOffset = _columnStart[i];
                if (columnOffset == DenseRow)
                {
                    result[i] = TensorPrimitives.Dot(a.Slice(offset, length), v) + TensorPrimitives.Dot(b.Slice(offset, length), w);
                    continue;
                }

                var sum = 0.0;
                for (var t = 0; t < length; t++)
                {
                    var j = Unsafe.Add(ref columns, columnOffset + t);
                    sum += (Unsafe.Add(ref aValues, offset + t) * Unsafe.Add(ref vValues, j))
                        + (Unsafe.Add(ref bValues, offset + t) * Unsafe.Add(ref wValues, j));
                }

                result[i] = sum;
            }
        }

        /// <summary>
        /// result_i = Σ_j A_ij v_j.
        /// </summary>
        public void Multiply(ReadOnlySpan<double> a, ReadOnlySpan<double> v, Span<double> result)
        {
            RequireLengths(a, v, result);
            ref var columns = ref MemoryMarshal.GetArrayDataReference(_columnIndices);
            ref var aValues = ref MemoryMarshal.GetReference(a);
            ref var vValues = ref MemoryMarshal.GetReference(v);
            for (var i = 0; i < RowCount; i++)
            {
                var offset = _rowStart[i];
                var length = _rowStart[i + 1] - offset;
                var columnOffset = _columnStart[i];
                if (columnOffset == DenseRow)
                {
                    result[i] = TensorPrimitives.Dot(a.Slice(offset, length), v);
                    continue;
                }

                var sum = 0.0;
                for (var t = 0; t < length; t++)
                {
                    sum += Unsafe.Add(ref aValues, offset + t) * Unsafe.Add(ref vValues, Unsafe.Add(ref columns, columnOffset + t));
                }

                result[i] = sum;
            }
        }

        /// <summary>
        /// result_j += Σ_i A_ij u_i.
        /// </summary>
        public void MultiplyTransposeAdd(ReadOnlySpan<double> a, ReadOnlySpan<double> u, Span<double> result)
        {
            RequireLengths(a, result, u);
            ref var columns = ref MemoryMarshal.GetArrayDataReference(_columnIndices);
            ref var aValues = ref MemoryMarshal.GetReference(a);
            ref var target = ref MemoryMarshal.GetReference(result);
            for (var i = 0; i < RowCount; i++)
            {
                var ui = u[i];
                if (ui == 0.0)
                {
                    continue;
                }

                var offset = _rowStart[i];
                var length = _rowStart[i + 1] - offset;
                var columnOffset = _columnStart[i];
                if (columnOffset == DenseRow)
                {
                    TensorPrimitives.MultiplyAdd(a.Slice(offset, length), ui, result, result);
                    continue;
                }

                for (var t = 0; t < length; t++)
                {
                    Unsafe.Add(ref target, Unsafe.Add(ref columns, columnOffset + t)) += ui * Unsafe.Add(ref aValues, offset + t);
                }
            }
        }

        /// <summary>
        /// One pass over P and Q: pTarget_j += Σ_i P_ij u_i, qTarget_j += Σ_i Q_ij u_i, and
        /// result_i = Σ_j P_ij v_j + Q_ij w_j. The subproblem's p_λ, q_λ and g(x) together.
        /// </summary>
        public void TransposeAddAndMultiplyPair(
            ReadOnlySpan<double> p,
            ReadOnlySpan<double> q,
            ReadOnlySpan<double> u,
            Span<double> pTarget,
            Span<double> qTarget,
            ReadOnlySpan<double> v,
            ReadOnlySpan<double> w,
            Span<double> result)
        {
            RequireLengths(p, v, result);
            RequireLengths(q, w, u);
            RequireLengths(p, pTarget, result);
            RequireLengths(q, qTarget, result);
            ref var columns = ref MemoryMarshal.GetArrayDataReference(_columnIndices);
            ref var pValues = ref MemoryMarshal.GetReference(p);
            ref var qValues = ref MemoryMarshal.GetReference(q);
            ref var vValues = ref MemoryMarshal.GetReference(v);
            ref var wValues = ref MemoryMarshal.GetReference(w);
            ref var pOut = ref MemoryMarshal.GetReference(pTarget);
            ref var qOut = ref MemoryMarshal.GetReference(qTarget);
            for (var i = 0; i < RowCount; i++)
            {
                var ui = u[i];
                var offset = _rowStart[i];
                var length = _rowStart[i + 1] - offset;
                var columnOffset = _columnStart[i];
                if (columnOffset == DenseRow)
                {
                    var pRow = p.Slice(offset, length);
                    var qRow = q.Slice(offset, length);
                    result[i] = TensorPrimitives.Dot(pRow, v) + TensorPrimitives.Dot(qRow, w);
                    TensorPrimitives.MultiplyAdd(pRow, ui, pTarget, pTarget);
                    TensorPrimitives.MultiplyAdd(qRow, ui, qTarget, qTarget);
                    continue;
                }

                var sum = 0.0;
                for (var t = 0; t < length; t++)
                {
                    var j = Unsafe.Add(ref columns, columnOffset + t);
                    var pij = Unsafe.Add(ref pValues, offset + t);
                    var qij = Unsafe.Add(ref qValues, offset + t);
                    sum += (pij * Unsafe.Add(ref vValues, j)) + (qij * Unsafe.Add(ref wValues, j));
                    Unsafe.Add(ref pOut, j) += ui * pij;
                    Unsafe.Add(ref qOut, j) += ui * qij;
                }

                result[i] = sum;
            }
        }

        /// <summary>
        /// result_i = Σ_j A_ij² w_j: the diagonal of A diag(w) Aᵀ.
        /// </summary>
        public void RowWeightedSquares(ReadOnlySpan<double> a, ReadOnlySpan<double> w, Span<double> result)
        {
            RequireLengths(a, w, result);
            for (var i = 0; i < RowCount; i++)
            {
                var offset = _rowStart[i];
                var values = a.Slice(offset, _rowStart[i + 1] - offset);
                var columns = RowColumns(i);
                var dense = columns.IsEmpty;
                var sum = 0.0;
                for (var t = 0; t < values.Length; t++)
                {
                    sum += values[t] * values[t] * w[dense ? t : columns[t]];
                }

                result[i] = sum;
            }
        }

        /// <summary>
        /// result_i = Σ_j |A_ij| w_j.
        /// </summary>
        public void RowWeightedAbsoluteSums(ReadOnlySpan<double> a, ReadOnlySpan<double> w, Span<double> result)
        {
            RequireLengths(a, w, result);
            for (var i = 0; i < RowCount; i++)
            {
                var offset = _rowStart[i];
                var values = a.Slice(offset, _rowStart[i + 1] - offset);
                var columns = RowColumns(i);
                var dense = columns.IsEmpty;
                var sum = 0.0;
                for (var t = 0; t < values.Length; t++)
                {
                    sum += Math.Abs(values[t]) * w[dense ? t : columns[t]];
                }

                result[i] = sum;
            }
        }

        /// <summary>
        /// result_j += Σ_i u_i A_ij²: the diagonal of Aᵀ diag(u) A.
        /// </summary>
        public void ColumnWeightedSquaresAdd(ReadOnlySpan<double> a, ReadOnlySpan<double> u, Span<double> result)
        {
            RequireLengths(a, result, u);
            for (var i = 0; i < RowCount; i++)
            {
                var offset = _rowStart[i];
                var values = a.Slice(offset, _rowStart[i + 1] - offset);
                var columns = RowColumns(i);
                var dense = columns.IsEmpty;
                var ui = u[i];
                for (var t = 0; t < values.Length; t++)
                {
                    result[dense ? t : columns[t]] += ui * values[t] * values[t];
                }
            }
        }

        /// <summary>
        /// The unchecked kernels index stored values, n-vectors and m-vectors through these lengths.
        /// </summary>
        private void RequireLengths(ReadOnlySpan<double> stored, ReadOnlySpan<double> columnVector, ReadOnlySpan<double> rowVector)
        {
            if (stored.Length != StoredCount || columnVector.Length != ColumnCount || rowVector.Length != RowCount)
            {
                throw new ArgumentException(
                    $"Expected {StoredCount} stored values, {ColumnCount}-vectors and {RowCount}-vectors; got {stored.Length}, {columnVector.Length} and {rowVector.Length}.");
            }
        }
    }
}
