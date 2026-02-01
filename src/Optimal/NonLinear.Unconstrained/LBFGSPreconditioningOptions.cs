/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.NonLinear.Unconstrained
{
    /// <summary>
    /// Type of preconditioning to apply.
    /// </summary>
    public enum PreconditioningType
    {
        /// <summary>
        /// No preconditioning.
        /// </summary>
        None,

        /// <summary>
        /// Diagonal scaling based on accumulated curvature estimates.
        /// </summary>
        Diagonal,

        /// <summary>
        /// Tikhonov regularization (adds λ*I to effective Hessian).
        /// </summary>
        Regularization,

        /// <summary>
        /// Combined diagonal and regularization preconditioning.
        /// </summary>
        Combined
    }

    /// <summary>
    /// Configuration options for L-BFGS preconditioning.
    /// </summary>
    public sealed record LBFGSPreconditioningOptions
    {
        /// <summary>
        /// Gets or sets whether to automatically enable preconditioning based on detected conditioning.
        /// </summary>
        public bool EnableAutomaticPreconditioning { get; init; }

        /// <summary>
        /// Gets or sets the condition number threshold to trigger automatic preconditioning.
        /// Default is 1e4.
        /// </summary>
        public double PreconditioningThreshold { get; init; } = 1e4;

        /// <summary>
        /// Gets or sets the type of preconditioning to apply.
        /// </summary>
        public PreconditioningType Type { get; init; } = PreconditioningType.None;

        /// <summary>
        /// Gets or sets the regularization parameter λ for regularization preconditioning.
        /// Default is 1e-6.
        /// </summary>
        public double RegularizationParameter { get; init; } = 1e-6;

        /// <summary>
        /// Gets or sets the diagonal update strategy for diagonal preconditioning.
        /// </summary>
        public DiagonalUpdateStrategy DiagonalUpdateStrategy { get; init; } = DiagonalUpdateStrategy.AccumulatedCurvature;

        /// <summary>
        /// Gets or sets the decay factor for exponential moving average updates.
        /// Default is 0.9.
        /// </summary>
        public double DecayFactor { get; init; } = 0.9;

        /// <summary>
        /// Gets or sets the minimum diagonal scaling to prevent numerical issues.
        /// Default is 1e-8.
        /// </summary>
        public double MinScaling { get; init; } = 1e-8;

        /// <summary>
        /// Gets or sets the maximum diagonal scaling to prevent numerical issues.
        /// Default is 1e8.
        /// </summary>
        public double MaxScaling { get; init; } = 1e8;

        /// <summary>
        /// Creates default preconditioning options (no preconditioning).
        /// </summary>
        public static LBFGSPreconditioningOptions Default => new();

        /// <summary>
        /// Creates options for automatic preconditioning based on detected conditioning.
        /// </summary>
        /// <param name="threshold">Condition number threshold to trigger preconditioning.</param>
        /// <returns>Preconditioning options with automatic triggering enabled.</returns>
        public static LBFGSPreconditioningOptions Automatic(double threshold = 1e4) => new()
        {
            EnableAutomaticPreconditioning = true,
            PreconditioningThreshold = threshold,
            Type = PreconditioningType.Diagonal
        };

        /// <summary>
        /// Creates options for diagonal preconditioning.
        /// </summary>
        /// <param name="strategy">Diagonal update strategy.</param>
        /// <returns>Preconditioning options with diagonal scaling.</returns>
        public static LBFGSPreconditioningOptions DiagonalScaling(
            DiagonalUpdateStrategy strategy = DiagonalUpdateStrategy.AccumulatedCurvature) => new()
        {
            Type = PreconditioningType.Diagonal,
            DiagonalUpdateStrategy = strategy
        };

        /// <summary>
        /// Creates options for Tikhonov regularization preconditioning.
        /// </summary>
        /// <param name="lambda">Regularization parameter.</param>
        /// <returns>Preconditioning options with regularization.</returns>
        public static LBFGSPreconditioningOptions Regularized(double lambda = 1e-6) => new()
        {
            Type = PreconditioningType.Regularization,
            RegularizationParameter = lambda
        };
    }
}
