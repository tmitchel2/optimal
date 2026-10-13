/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// Options shared by MMA and GCMMA. The defaults are Svanberg's (2007, "MMA and GCMMA – two methods for nonlinear
    /// optimization", the mmasub/gcmmasub/subsolv codes).
    /// </summary>
    /// <remarks>
    /// <see cref="OptimizerOptions.Tolerance"/> is the tolerance on the largest component of the KKT residual, in
    /// projected-gradient form, so that it does not grow with n; the residual cannot fall much below
    /// <see cref="SubproblemTolerance"/>, whose barrier perturbs the multipliers. <see cref="OptimizerOptions.MaxIterations"/>
    /// caps the outer iterations.
    /// Svanberg's artificial-variable form is minimize f0 + a0 z + Σ (c_i y_i + ½ d_i y_i²) subject to
    /// g_i - a_i z - y_i &lt;= 0, with y, z &gt;= 0: with a_i = 0 and c_i large, y = 0 at a feasible optimum and the problem
    /// is the original one; a_i = 1 on some rows with f0 = 0 makes it a min-max problem over those rows.
    /// </remarks>
    public record MovingAsymptotesOptions : OptimizerOptions
    {
        /// <summary>
        /// Gets the move limit, as a fraction of each variable's range: |x_j - x_j,k| &lt;= MoveLimit·(upper_j - lower_j).
        /// </summary>
        public double MoveLimit { get; init; } = 0.5;

        /// <summary>
        /// Gets the initial distance of the asymptotes from x, as a fraction of the range (iterations 1 and 2).
        /// </summary>
        public double AsymptoteInitial { get; init; } = 0.5;

        /// <summary>
        /// Gets the factor that widens the asymptotes when a variable moves the same way twice running.
        /// </summary>
        public double AsymptoteIncrease { get; init; } = 1.2;

        /// <summary>
        /// Gets the factor that narrows the asymptotes when a variable oscillates.
        /// </summary>
        public double AsymptoteDecrease { get; init; } = 0.7;

        /// <summary>
        /// Gets the smallest distance of an asymptote from x, as a fraction of the range.
        /// </summary>
        public double AsymptoteMinimumDistance { get; init; } = 0.01;

        /// <summary>
        /// Gets the largest distance of an asymptote from x, as a fraction of the range.
        /// </summary>
        public double AsymptoteMaximumDistance { get; init; } = 10.0;

        /// <summary>
        /// Gets Svanberg's albefa: the subproblem keeps x a fraction of the way from each asymptote,
        /// α_j = max(L_j + albefa·(x_j - L_j), ...), β_j = min(U_j - albefa·(U_j - x_j), ...).
        /// </summary>
        public double AsymptoteBoundFactor { get; init; } = 0.1;

        /// <summary>
        /// Gets a0, the objective's coefficient of z.
        /// </summary>
        public double A0 { get; init; } = 1.0;

        /// <summary>
        /// Gets a_i per constraint (m entries), or null for a_i = 0 on every constraint.
        /// </summary>
        public double[]? ConstraintA { get; init; }

        /// <summary>
        /// Gets c_i per constraint (m entries), or null for c_i = 1000 on every constraint.
        /// </summary>
        public double[]? ConstraintC { get; init; }

        /// <summary>
        /// Gets d_i per constraint (m entries), or null for d_i = 1 on every constraint.
        /// </summary>
        public double[]? ConstraintD { get; init; }

        /// <summary>
        /// Gets the subproblem's final barrier parameter (Svanberg's epsimin): the interior-point method drives ε from
        /// 1 down to this value by factors of ten.
        /// </summary>
        public double SubproblemTolerance { get; init; } = 1e-7;

        /// <summary>
        /// Gets the cap on Newton steps per barrier level.
        /// </summary>
        public int MaxNewtonIterations { get; init; } = 200;

        /// <summary>
        /// Gets how the reduced Newton system is solved.
        /// </summary>
        public MmaLinearSolver LinearSolver { get; init; } = MmaLinearSolver.Automatic;

        /// <summary>
        /// Gets the largest reduced system (min(m, n) unknowns) that <see cref="MmaLinearSolver.Automatic"/> solves directly.
        /// </summary>
        public int DirectSolveLimit { get; init; } = 500;

        /// <summary>
        /// Gets the conjugate-gradient tolerance on the preconditioned residual, relative to the right-hand side.
        /// </summary>
        public double ConjugateGradientTolerance { get; init; } = 1e-9;

        /// <summary>
        /// Gets the cap on conjugate-gradient iterations per solve.
        /// </summary>
        public int MaxConjugateGradientIterations { get; init; } = 2000;

        /// <summary>
        /// Gets the tolerance on the scaled change max_j |Δx_j| / (upper_j - lower_j) that also ends the run; 0 turns
        /// it off, leaving the KKT residual as the only test.
        /// </summary>
        public double ChangeTolerance { get; init; }

        /// <summary>
        /// Gets an optional callback invoked after each outer iteration.
        /// </summary>
        public Action<InequalityIteration>? ProgressCallback { get; init; }
    }
}
