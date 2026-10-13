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
    /// Solves the symmetric positive definite reduced system of the subproblem's Newton step. When m &lt; n it lives in
    /// constraint space, A = diag(D) + G diag(W) Gᵀ (m×m); otherwise in variable space, A = diag(D) + Gᵀ diag(W) G
    /// (n×n). G is the m×n matrix in the constraint layout. The system is either formed densely and factorised, or
    /// solved by Jacobi-preconditioned conjugate gradients with A applied through G.
    /// </summary>
    internal sealed class ReducedSystemSolver
    {
        private readonly ConstraintMatrix _matrix;
        private readonly double _tolerance;
        private readonly int _maxIterations;
        private readonly double[] _dense;
        private readonly double[] _scatter;
        private readonly double[] _jacobi;
        private readonly double[] _residual;
        private readonly double[] _preconditioned;
        private readonly double[] _direction;
        private readonly double[] _product;
        private readonly double[] _inner;
        private double[] _g = [];
        private double[] _diagonal = [];
        private double[] _weights = [];

        public ReducedSystemSolver(ConstraintMatrix matrix, MmaLinearSolver kind, int directLimit, double tolerance, int maxIterations)
        {
            _matrix = matrix;
            _tolerance = tolerance;
            _maxIterations = maxIterations;
            ConstraintSpace = matrix.RowCount < matrix.ColumnCount;
            Size = ConstraintSpace ? matrix.RowCount : matrix.ColumnCount;
            IsDirect = kind switch
            {
                MmaLinearSolver.Direct => true,
                MmaLinearSolver.ConjugateGradient => false,
                MmaLinearSolver.Automatic => Size <= directLimit,
                _ => throw new ArgumentOutOfRangeException(nameof(kind), kind, "Unknown linear solver."),
            };
            var otherSize = ConstraintSpace ? matrix.ColumnCount : matrix.RowCount;
            _dense = IsDirect ? new double[checked(Size * Size)] : [];
            _scatter = IsDirect && ConstraintSpace ? new double[matrix.ColumnCount] : [];
            _jacobi = IsDirect ? [] : new double[Size];
            _residual = IsDirect ? [] : new double[Size];
            _preconditioned = IsDirect ? [] : new double[Size];
            _direction = IsDirect ? [] : new double[Size];
            _product = IsDirect ? [] : new double[Size];
            _inner = IsDirect ? [] : new double[otherSize];
        }

        /// <summary>Gets whether the system is m×m (m &lt; n) rather than n×n.</summary>
        public bool ConstraintSpace { get; }

        /// <summary>Gets the number of unknowns, min(m, n) (n when m = n).</summary>
        public int Size { get; }

        /// <summary>Gets whether the system is factorised directly.</summary>
        public bool IsDirect { get; }

        /// <summary>Gets the conjugate-gradient iterations since the last reset.</summary>
        public int LinearIterations { get; private set; }

        /// <summary>Gets the conjugate-gradient solves that hit the iteration cap since the last reset.</summary>
        public int UnconvergedSolves { get; private set; }

        public void ResetStatistics()
        {
            LinearIterations = 0;
            UnconvergedSolves = 0;
        }

        /// <summary>
        /// Sets the system's G (constraint layout), D (Size entries) and W (the other space's size). Returns false if
        /// a direct factorisation finds the matrix not positive definite.
        /// </summary>
        public bool Prepare(double[] g, double[] diagonal, double[] weights)
        {
            _g = g;
            _diagonal = diagonal;
            _weights = weights;
            if (!IsDirect)
            {
                PrepareJacobi();
                return true;
            }

            if (ConstraintSpace)
            {
                FormConstraintSpace();
            }
            else
            {
                FormVariableSpace();
            }

            return DenseCholesky.Factor(_dense, Size);
        }

        /// <summary>
        /// Solves A x = rhs.
        /// </summary>
        public void Solve(ReadOnlySpan<double> rhs, Span<double> solution)
        {
            if (IsDirect)
            {
                rhs.CopyTo(solution);
                DenseCholesky.Solve(_dense, Size, solution);
                return;
            }

            ConjugateGradient(rhs, solution);
        }

        private void FormConstraintSpace()
        {
            var m = Size;
            var dense = _dense.AsSpan();
            var scatter = _scatter.AsSpan();
            for (var i = 0; i < m; i++)
            {
                ScatterWeightedRow(i, scatter);
                for (var k = 0; k <= i; k++)
                {
                    dense[(i * m) + k] = RowDot(k, scatter);
                }

                dense[(i * m) + i] += _diagonal[i];
                ClearRow(i, scatter);
            }
        }

        private void FormVariableSpace()
        {
            var n = Size;
            var dense = _dense.AsSpan();
            dense.Clear();
            for (var i = 0; i < _matrix.RowCount; i++)
            {
                var offset = _matrix.RowOffset(i);
                var values = _g.AsSpan(offset, _matrix.RowLength(i));
                var weight = _weights[i];
                if (_matrix.IsDense(i))
                {
                    for (var s = 0; s < n; s++)
                    {
                        var target = dense.Slice(s * n, s + 1);
                        TensorPrimitives.MultiplyAdd(values[..(s + 1)], weight * values[s], target, target);
                    }

                    continue;
                }

                var columns = _matrix.RowColumns(i);
                for (var s = 0; s < values.Length; s++)
                {
                    var scaled = weight * values[s];
                    var rowOffset = columns[s] * n;
                    for (var t = 0; t <= s; t++)
                    {
                        dense[rowOffset + columns[t]] += scaled * values[t];
                    }
                }
            }

            for (var j = 0; j < n; j++)
            {
                dense[(j * n) + j] += _diagonal[j];
            }
        }

        private void ScatterWeightedRow(int row, Span<double> scatter)
        {
            var offset = _matrix.RowOffset(row);
            var values = _g.AsSpan(offset, _matrix.RowLength(row));
            if (_matrix.IsDense(row))
            {
                TensorPrimitives.Multiply(values, _weights, scatter);
                return;
            }

            var columns = _matrix.RowColumns(row);
            for (var t = 0; t < values.Length; t++)
            {
                scatter[columns[t]] = values[t] * _weights[columns[t]];
            }
        }

        private double RowDot(int row, ReadOnlySpan<double> scatter)
        {
            var offset = _matrix.RowOffset(row);
            var values = _g.AsSpan(offset, _matrix.RowLength(row));
            if (_matrix.IsDense(row))
            {
                return TensorPrimitives.Dot(values, scatter);
            }

            var columns = _matrix.RowColumns(row);
            var sum = 0.0;
            for (var t = 0; t < values.Length; t++)
            {
                sum += values[t] * scatter[columns[t]];
            }

            return sum;
        }

        private void ClearRow(int row, Span<double> scatter)
        {
            if (_matrix.IsDense(row))
            {
                scatter.Clear();
                return;
            }

            foreach (var column in _matrix.RowColumns(row))
            {
                scatter[column] = 0.0;
            }
        }

        private void PrepareJacobi()
        {
            if (ConstraintSpace)
            {
                _matrix.RowWeightedSquares(_g, _weights, _jacobi);
                TensorPrimitives.Add(_jacobi, _diagonal, _jacobi);
                return;
            }

            _diagonal.CopyTo(_jacobi.AsSpan());
            _matrix.ColumnWeightedSquaresAdd(_g, _weights, _jacobi);
        }

        private void Apply(ReadOnlySpan<double> v, Span<double> result)
        {
            var inner = _inner.AsSpan();
            if (ConstraintSpace)
            {
                inner.Clear();
                _matrix.MultiplyTransposeAdd(_g, v, inner);
                TensorPrimitives.Multiply(inner, _weights, inner);
                _matrix.Multiply(_g, inner, result);
                TensorPrimitives.MultiplyAdd(v, _diagonal, result, result);
                return;
            }

            _matrix.Multiply(_g, v, inner);
            TensorPrimitives.Multiply(inner, _weights, inner);
            TensorPrimitives.Multiply(v, _diagonal, result);
            _matrix.MultiplyTransposeAdd(_g, inner, result);
        }

        private void ConjugateGradient(ReadOnlySpan<double> rhs, Span<double> solution)
        {
            var r = _residual.AsSpan();
            var z = _preconditioned.AsSpan();
            var p = _direction.AsSpan();
            var ap = _product.AsSpan();
            solution.Clear();
            rhs.CopyTo(r);
            var target = _tolerance * TensorPrimitives.Norm(rhs);
            if (target == 0.0)
            {
                return;
            }

            TensorPrimitives.Divide(r, _jacobi, z);
            z.CopyTo(p);
            var rz = TensorPrimitives.Dot(r, z);
            for (var iteration = 1; iteration <= _maxIterations; iteration++)
            {
                Apply(p, ap);
                var alpha = rz / TensorPrimitives.Dot(p, ap);
                TensorPrimitives.MultiplyAdd(p, alpha, solution, solution);
                TensorPrimitives.MultiplyAdd(ap, -alpha, r, r);
                LinearIterations++;
                if (TensorPrimitives.Norm(r) <= target)
                {
                    return;
                }

                TensorPrimitives.Divide(r, _jacobi, z);
                var rzNext = TensorPrimitives.Dot(r, z);
                TensorPrimitives.MultiplyAdd(p, rzNext / rz, z, p);
                rz = rzNext;
            }

            UnconvergedSolves++;
        }
    }
}
