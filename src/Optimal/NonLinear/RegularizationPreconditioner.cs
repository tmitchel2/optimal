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
    /// Tikhonov regularization preconditioner for L-BFGS optimization.
    /// Adds a regularization term λ*I to the effective Hessian approximation,
    /// which helps with ill-conditioned problems by bounding the condition number.
    /// </summary>
    public sealed class RegularizationPreconditioner : IPreconditioner
    {
        private readonly int _dimension;
        private readonly double _regularizationParameter;
        private readonly double[] _diagonalScaling;
        private double _gamma = 1.0;

        /// <summary>
        /// Creates a new regularization preconditioner.
        /// </summary>
        /// <param name="dimension">Problem dimension.</param>
        /// <param name="regularizationParameter">Regularization parameter λ (default 1e-6).</param>
        public RegularizationPreconditioner(int dimension, double regularizationParameter = 1e-6)
        {
            if (dimension <= 0)
            {
                throw new ArgumentException("Dimension must be positive", nameof(dimension));
            }

            if (regularizationParameter < 0)
            {
                throw new ArgumentException("Regularization parameter must be non-negative", nameof(regularizationParameter));
            }

            _dimension = dimension;
            _regularizationParameter = regularizationParameter;
            _diagonalScaling = new double[dimension];

            // Initialize to (1 + λ) for initial gamma of 1
            for (var i = 0; i < dimension; i++)
            {
                _diagonalScaling[i] = 1.0 + regularizationParameter;
            }
        }

        /// <summary>
        /// Gets the problem dimension.
        /// </summary>
        public int Dimension => _dimension;

        /// <summary>
        /// Gets the regularization parameter λ.
        /// </summary>
        public double RegularizationParameter => _regularizationParameter;

        /// <summary>
        /// Gets the current gamma value used in L-BFGS H_0 initialization.
        /// </summary>
        public double Gamma => _gamma;

        /// <summary>
        /// Gets the regularized gamma value: gamma + λ.
        /// </summary>
        public double RegularizedGamma => _gamma + _regularizationParameter;

        /// <inheritdoc/>
        public double[] ApplyToGradient(double[] gradient)
        {
            if (gradient.Length != _dimension)
            {
                throw new ArgumentException($"Gradient dimension ({gradient.Length}) does not match preconditioner ({_dimension})");
            }

            // Apply regularized inverse Hessian: (gamma + λ) * I
            // This corresponds to H_0 = (gamma + λ) * I in two-loop recursion
            var result = new double[_dimension];
            var regGamma = RegularizedGamma;

            for (var i = 0; i < _dimension; i++)
            {
                result[i] = regGamma * gradient[i];
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

            // Apply inverse: 1 / (gamma + λ) * I
            var result = new double[_dimension];
            var invRegGamma = 1.0 / RegularizedGamma;

            for (var i = 0; i < _dimension; i++)
            {
                result[i] = invRegGamma * direction[i];
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

            // Update gamma based on new curvature information
            // gamma = (s^T * y) / (y^T * y), same as L-BFGS
            var sTs = 0.0;
            var yTy = 0.0;
            var sTy = 0.0;

            for (var i = 0; i < _dimension; i++)
            {
                sTy += s[i] * y[i];
                yTy += y[i] * y[i];
                sTs += s[i] * s[i];
            }

            if (yTy > 1e-16 && sTy > 1e-16)
            {
                _gamma = sTy / yTy;

                // Update diagonal scaling to reflect regularized gamma
                var regGamma = _gamma + _regularizationParameter;
                for (var i = 0; i < _dimension; i++)
                {
                    _diagonalScaling[i] = regGamma;
                }
            }
        }

        /// <inheritdoc/>
        public double[] GetDiagonalScaling()
        {
            return (double[])_diagonalScaling.Clone();
        }

        /// <inheritdoc/>
        public void Reset()
        {
            _gamma = 1.0;
            for (var i = 0; i < _dimension; i++)
            {
                _diagonalScaling[i] = 1.0 + _regularizationParameter;
            }
        }
    }
}
