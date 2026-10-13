/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Constraints
{
    /// <summary>
    /// The structure of a constraint block's Jacobian: either dense (every row holds a value for every variable, stored
    /// row-major) or compressed sparse row (CSR), whose pattern is fixed for the whole run.
    /// </summary>
    /// <remarks>
    /// A sparse pattern is structural: constraint i must not depend on a variable outside row i's pattern anywhere in
    /// the design space, not just at the current point. The optimisers regularise and approximate a sparse row on its
    /// pattern only, so that m local constraints cost O(nnz), not O(m·n).
    /// </remarks>
    public sealed class ConstraintSparsity
    {
        private readonly int[] _rowPointers;
        private readonly int[] _columnIndices;

        private ConstraintSparsity(int rowCount, int columnCount, bool isDense, int[] rowPointers, int[] columnIndices)
        {
            RowCount = rowCount;
            ColumnCount = columnCount;
            IsDense = isDense;
            _rowPointers = rowPointers;
            _columnIndices = columnIndices;
        }

        /// <summary>
        /// Gets the number of constraints (rows) in the block.
        /// </summary>
        public int RowCount { get; }

        /// <summary>
        /// Gets the number of design variables (columns).
        /// </summary>
        public int ColumnCount { get; }

        /// <summary>
        /// Gets a value indicating whether the Jacobian is dense (row-major, RowCount × ColumnCount values).
        /// </summary>
        public bool IsDense { get; }

        /// <summary>
        /// Gets the CSR row pointers (RowCount + 1 entries). Empty for a dense block.
        /// </summary>
        public ReadOnlySpan<int> RowPointers => _rowPointers;

        /// <summary>
        /// Gets the CSR column indices, strictly increasing within each row. Empty for a dense block.
        /// </summary>
        public ReadOnlySpan<int> ColumnIndices => _columnIndices;

        /// <summary>
        /// Gets the number of Jacobian values the block writes: RowCount × ColumnCount when dense, the number of
        /// structural nonzeros when sparse.
        /// </summary>
        public int StoredCount => IsDense ? checked(RowCount * ColumnCount) : _columnIndices.Length;

        /// <summary>
        /// Creates a dense structure.
        /// </summary>
        /// <param name="rowCount">The number of constraints.</param>
        /// <param name="columnCount">The number of design variables.</param>
        /// <returns>The dense structure.</returns>
        public static ConstraintSparsity Dense(int rowCount, int columnCount)
        {
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(rowCount);
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(columnCount);
            _ = checked(rowCount * columnCount);
            return new ConstraintSparsity(rowCount, columnCount, true, [], []);
        }

        /// <summary>
        /// Creates a compressed sparse row structure. The arrays are copied.
        /// </summary>
        /// <param name="columnCount">The number of design variables.</param>
        /// <param name="rowPointers">Row pointers: row i's entries are [rowPointers[i], rowPointers[i + 1]).</param>
        /// <param name="columnIndices">Column indices, strictly increasing within each row.</param>
        /// <returns>The sparse structure.</returns>
        public static ConstraintSparsity Csr(int columnCount, ReadOnlySpan<int> rowPointers, ReadOnlySpan<int> columnIndices)
        {
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(columnCount);
            if (rowPointers.Length < 2)
            {
                throw new ArgumentException("A CSR structure needs at least one row (two row pointers).", nameof(rowPointers));
            }

            if (rowPointers[0] != 0 || rowPointers[^1] != columnIndices.Length)
            {
                throw new ArgumentException(
                    $"Row pointers must start at 0 and end at the number of column indices ({columnIndices.Length}).",
                    nameof(rowPointers));
            }

            for (var i = 0; i + 1 < rowPointers.Length; i++)
            {
                ValidateRow(columnCount, i, columnIndices[rowPointers[i]..rowPointers[i + 1]]);
            }

            return new ConstraintSparsity(rowPointers.Length - 1, columnCount, false, rowPointers.ToArray(), columnIndices.ToArray());
        }

        private static void ValidateRow(int columnCount, int row, ReadOnlySpan<int> columns)
        {
            var previous = -1;
            foreach (var column in columns)
            {
                if (column <= previous || column >= columnCount)
                {
                    throw new ArgumentException(
                        $"Row {row}: column indices must be strictly increasing and in [0, {columnCount}); found {column} after {previous}.",
                        nameof(columns));
                }

                previous = column;
            }
        }
    }
}
