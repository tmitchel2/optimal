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
    /// Combined preconditioner that applies both diagonal scaling and Tikhonov regularization.
    /// The combined preconditioner uses: M = D + λI, where D is the diagonal estimate.
    /// </summary>
    public sealed class CombinedPreconditioner : IPreconditioner
    {
        private readonly DiagonalPreconditioner _diagonalPreconditioner;
        private readonly double _regularizationParameter;

        /// <summary>
        /// Creates a new combined preconditioner.
        /// </summary>
        /// <param name="diagonalPreconditioner">The diagonal preconditioner to use.</param>
        /// <param name="regularizationParameter">Regularization parameter λ.</param>
        public CombinedPreconditioner(
            DiagonalPreconditioner diagonalPreconditioner,
            double regularizationParameter = 1e-6)
        {
            _diagonalPreconditioner = diagonalPreconditioner ?? throw new ArgumentNullException(nameof(diagonalPreconditioner));

            if (regularizationParameter < 0)
            {
                throw new ArgumentException("Regularization parameter must be non-negative", nameof(regularizationParameter));
            }

            _regularizationParameter = regularizationParameter;
        }

        /// <summary>
        /// Gets the problem dimension.
        /// </summary>
        public int Dimension => _diagonalPreconditioner.Dimension;

        /// <summary>
        /// Gets the regularization parameter λ.
        /// </summary>
        public double RegularizationParameter => _regularizationParameter;

        /// <inheritdoc/>
        public double[] ApplyToGradient(double[] gradient)
        {
            var diagonal = _diagonalPreconditioner.GetDiagonalScaling();
            var result = new double[gradient.Length];

            for (var i = 0; i < gradient.Length; i++)
            {
                // Apply inverse of (D[i] + λ): g_precond[i] = g[i] / (D[i] + λ)
                result[i] = gradient[i] / (diagonal[i] + _regularizationParameter);
            }

            return result;
        }

        /// <inheritdoc/>
        public double[] ApplyInverseToDirection(double[] direction)
        {
            var diagonal = _diagonalPreconditioner.GetDiagonalScaling();
            var result = new double[direction.Length];

            for (var i = 0; i < direction.Length; i++)
            {
                // Apply (D[i] + λ): d_transformed[i] = (D[i] + λ) * d[i]
                result[i] = (diagonal[i] + _regularizationParameter) * direction[i];
            }

            return result;
        }

        /// <inheritdoc/>
        public void Update(double[] s, double[] y)
        {
            _diagonalPreconditioner.Update(s, y);
        }

        /// <inheritdoc/>
        public double[] GetDiagonalScaling()
        {
            var diagonal = _diagonalPreconditioner.GetDiagonalScaling();
            var result = new double[diagonal.Length];

            for (var i = 0; i < diagonal.Length; i++)
            {
                result[i] = diagonal[i] + _regularizationParameter;
            }

            return result;
        }

        /// <inheritdoc/>
        public void Reset()
        {
            _diagonalPreconditioner.Reset();
        }
    }
}
