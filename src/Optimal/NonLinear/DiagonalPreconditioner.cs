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
    /// Strategy for updating diagonal scaling estimates.
    /// </summary>
    public enum DiagonalUpdateStrategy
    {
        /// <summary>
        /// Fixed initial estimate, no updates.
        /// </summary>
        Fixed,

        /// <summary>
        /// Accumulated curvature: D[i] = sum(y[i]^2 / (y^T * s)).
        /// </summary>
        AccumulatedCurvature,

        /// <summary>
        /// Exponential moving average with decay factor.
        /// </summary>
        ExponentialMovingAverage
    }

    /// <summary>
    /// Diagonal preconditioner for L-BFGS optimization.
    /// Approximates the inverse Hessian diagonal using accumulated curvature estimates.
    /// </summary>
    public sealed class DiagonalPreconditioner : IPreconditioner
    {
        private readonly int _dimension;
        private readonly DiagonalUpdateStrategy _updateStrategy;
        private readonly double _decayFactor;
        private readonly double _minScaling;
        private readonly double _maxScaling;
        private readonly double[] _diagonalEstimate;
        private readonly double[] _updateCount;
        private bool _initialized;

        /// <summary>
        /// Creates a new diagonal preconditioner.
        /// </summary>
        /// <param name="dimension">Problem dimension.</param>
        /// <param name="updateStrategy">Strategy for updating diagonal estimates.</param>
        /// <param name="decayFactor">Decay factor for exponential moving average (0.9 typical).</param>
        /// <param name="minScaling">Minimum diagonal scaling to prevent numerical issues.</param>
        /// <param name="maxScaling">Maximum diagonal scaling to prevent numerical issues.</param>
        public DiagonalPreconditioner(
            int dimension,
            DiagonalUpdateStrategy updateStrategy = DiagonalUpdateStrategy.AccumulatedCurvature,
            double decayFactor = 0.9,
            double minScaling = 1e-8,
            double maxScaling = 1e8)
        {
            if (dimension <= 0)
            {
                throw new ArgumentException("Dimension must be positive", nameof(dimension));
            }

            if (decayFactor <= 0 || decayFactor >= 1)
            {
                throw new ArgumentException("Decay factor must be in (0, 1)", nameof(decayFactor));
            }

            if (minScaling <= 0 || maxScaling <= 0 || minScaling >= maxScaling)
            {
                throw new ArgumentException("Scaling bounds must be positive with min < max");
            }

            _dimension = dimension;
            _updateStrategy = updateStrategy;
            _decayFactor = decayFactor;
            _minScaling = minScaling;
            _maxScaling = maxScaling;
            _diagonalEstimate = new double[dimension];
            _updateCount = new double[dimension];
            _initialized = false;

            // Initialize to identity scaling
            for (var i = 0; i < dimension; i++)
            {
                _diagonalEstimate[i] = 1.0;
            }
        }

        /// <summary>
        /// Gets the problem dimension.
        /// </summary>
        public int Dimension => _dimension;

        /// <summary>
        /// Gets the update strategy.
        /// </summary>
        public DiagonalUpdateStrategy UpdateStrategy => _updateStrategy;

        /// <inheritdoc/>
        public double[] ApplyToGradient(double[] gradient)
        {
            if (gradient.Length != _dimension)
            {
                throw new ArgumentException($"Gradient dimension ({gradient.Length}) does not match preconditioner ({_dimension})");
            }

            var result = new double[_dimension];
            for (var i = 0; i < _dimension; i++)
            {
                // Apply inverse of diagonal: g_precond[i] = g[i] / D[i]
                result[i] = gradient[i] / _diagonalEstimate[i];
            }
            return result;
        }

        /// <inheritdoc/>
        public double[] ApplyInverseToDirection(double[] direction)
        {
            if (direction.Length != _dimension)
            {
                throw new ArgumentException($"Direction dimension ({direction.Length}) does not match preconditioner ({_dimension})");
            }

            var result = new double[_dimension];
            for (var i = 0; i < _dimension; i++)
            {
                // Apply diagonal: d_transformed[i] = D[i] * d[i]
                result[i] = _diagonalEstimate[i] * direction[i];
            }
            return result;
        }

        /// <inheritdoc/>
        public void Update(double[] s, double[] y)
        {
            if (s.Length != _dimension || y.Length != _dimension)
            {
                throw new ArgumentException("Vector dimensions do not match preconditioner dimension");
            }

            // Compute y^T * s
            var yTs = 0.0;
            for (var i = 0; i < _dimension; i++)
            {
                yTs += y[i] * s[i];
            }

            // Skip update if curvature condition is violated
            if (yTs <= 1e-16)
            {
                return;
            }

            switch (_updateStrategy)
            {
                case DiagonalUpdateStrategy.Fixed:
                    // No update for fixed strategy
                    if (!_initialized)
                    {
                        InitializeFromFirstPair(s, y, yTs);
                        _initialized = true;
                    }
                    break;

                case DiagonalUpdateStrategy.AccumulatedCurvature:
                    UpdateAccumulated(y, yTs);
                    break;

                case DiagonalUpdateStrategy.ExponentialMovingAverage:
                    UpdateExponentialMovingAverage(y, yTs);
                    break;
            }
        }

        /// <inheritdoc/>
        public double[] GetDiagonalScaling()
        {
            return (double[])_diagonalEstimate.Clone();
        }

        /// <inheritdoc/>
        public void Reset()
        {
            for (var i = 0; i < _dimension; i++)
            {
                _diagonalEstimate[i] = 1.0;
                _updateCount[i] = 0.0;
            }
            _initialized = false;
        }

        private void InitializeFromFirstPair(double[] _, double[] y, double yTs)
        {
            // Initialize diagonal estimate from first curvature pair
            // D[i] = y[i]^2 / (y^T * s), clamped to bounds
            for (var i = 0; i < _dimension; i++)
            {
                if (Math.Abs(y[i]) > 1e-16)
                {
                    var estimate = (y[i] * y[i]) / yTs;
                    _diagonalEstimate[i] = Math.Max(_minScaling, Math.Min(_maxScaling, estimate));
                }
            }
        }

        private void UpdateAccumulated(double[] y, double yTs)
        {
            // Accumulate curvature: D[i] = sqrt(sum(y[i]^2 / (y^T * s)))
            // We accumulate the sum and take sqrt when applying
            for (var i = 0; i < _dimension; i++)
            {
                var contribution = (y[i] * y[i]) / yTs;
                _updateCount[i] += 1.0;

                // Running average for numerical stability
                var newEstimate = _diagonalEstimate[i] + (contribution - _diagonalEstimate[i]) / _updateCount[i];
                _diagonalEstimate[i] = Math.Max(_minScaling, Math.Min(_maxScaling, newEstimate));
            }
            _initialized = true;
        }

        private void UpdateExponentialMovingAverage(double[] y, double yTs)
        {
            for (var i = 0; i < _dimension; i++)
            {
                var contribution = (y[i] * y[i]) / yTs;

                if (!_initialized)
                {
                    // First update: use contribution directly
                    _diagonalEstimate[i] = Math.Max(_minScaling, Math.Min(_maxScaling, contribution));
                }
                else
                {
                    // Exponential moving average: D_new = decay * D_old + (1 - decay) * contribution
                    var newEstimate = _decayFactor * _diagonalEstimate[i] + (1.0 - _decayFactor) * contribution;
                    _diagonalEstimate[i] = Math.Max(_minScaling, Math.Min(_maxScaling, newEstimate));
                }
            }
            _initialized = true;
        }
    }
}
