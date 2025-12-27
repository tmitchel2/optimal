/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Circular buffer for storing L-BFGS correction pairs (s, y).
    /// Maintains the last m vector pairs for quasi-Newton approximation.
    /// </summary>
    public sealed class LBFGSMemory
    {
        private readonly int _m; // Memory size
        private readonly int _n; // Dimension
        private readonly double[][] _s; // Position differences: s_k = x_{k+1} - x_k
        private readonly double[][] _y; // Gradient differences: y_k = grad_{k+1} - grad_k
        private readonly double[] _rho; // Precomputed: rho_k = 1 / (y_k^T * s_k)
        private int _count; // Number of stored pairs (0 to m)
        private int _startIndex; // Index of oldest pair in circular buffer

        /// <summary>
        /// Initializes a new instance of the <see cref="LBFGSMemory"/> class.
        /// </summary>
        /// <param name="m">Number of correction pairs to store (typically 3-20).</param>
        /// <param name="n">Dimension of the optimization problem.</param>
        public LBFGSMemory(int m, int n)
        {
            if (m <= 0)
            {
                throw new ArgumentException("Memory size must be positive", nameof(m));
            }

            if (n <= 0)
            {
                throw new ArgumentException("Dimension must be positive", nameof(n));
            }

            _m = m;
            _n = n;
            _s = new double[m][];
            _y = new double[m][];
            _rho = new double[m];

            for (var i = 0; i < m; i++)
            {
                _s[i] = new double[n];
                _y[i] = new double[n];
            }

            _count = 0;
            _startIndex = 0;
        }

        /// <summary>
        /// Gets the number of stored correction pairs.
        /// </summary>
        public int Count => _count;

        /// <summary>
        /// Gets the memory size (maximum number of pairs).
        /// </summary>
        public int Capacity => _m;

        /// <summary>
        /// Adds a new correction pair (s, y) to the memory.
        /// If memory is full, the oldest pair is discarded.
        /// </summary>
        /// <param name="s">Position difference: x_{k+1} - x_k.</param>
        /// <param name="y">Gradient difference: grad_{k+1} - grad_k.</param>
        public void Push(double[] s, double[] y)
        {
            if (s.Length != _n || y.Length != _n)
            {
                throw new ArgumentException("Vector dimension mismatch");
            }

            // Compute rho = 1 / (y^T * s)
            var yTs = DotProduct(y, s);
            if (Math.Abs(yTs) < 1e-16)
            {
                // Skip this update if y^T * s is too small (curvature condition violated)
                return;
            }

            var rho = 1.0 / yTs;

            // Determine index for new pair
            int index;
            if (_count < _m)
            {
                // Memory not full yet, add to end
                index = _count;
                _count++;
            }
            else
            {
                // Memory full, overwrite oldest
                index = _startIndex;
                _startIndex = (_startIndex + 1) % _m;
            }

            // Store the pair
            Array.Copy(s, _s[index], _n);
            Array.Copy(y, _y[index], _n);
            _rho[index] = rho;
        }

        /// <summary>
        /// Gets the correction pair at the specified position (0 = oldest, Count-1 = newest).
        /// </summary>
        /// <param name="position">Position in the memory (0 to Count-1).</param>
        /// <returns>Tuple of (s, y, rho) for the specified position.</returns>
        public (double[] s, double[] y, double rho) GetPair(int position)
        {
            if (position < 0 || position >= _count)
            {
                throw new ArgumentOutOfRangeException(nameof(position));
            }

            var index = (_startIndex + position) % _m;
            return (_s[index], _y[index], _rho[index]);
        }

        /// <summary>
        /// Clears all stored correction pairs.
        /// </summary>
        public void Clear()
        {
            _count = 0;
            _startIndex = 0;
        }

        private static double DotProduct(double[] a, double[] b)
        {
            var result = 0.0;
            for (var i = 0; i < a.Length; i++)
            {
                result += a[i] * b[i];
            }
            return result;
        }
    }
}
