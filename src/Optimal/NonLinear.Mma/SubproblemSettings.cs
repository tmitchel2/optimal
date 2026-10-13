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
    /// The subproblem's fixed data: Svanberg's a0, a, c and d, and how its Newton systems are solved.
    /// </summary>
    internal sealed record SubproblemSettings
    {
        public required double A0 { get; init; }

        public required double[] A { get; init; }

        public required double[] C { get; init; }

        public required double[] D { get; init; }

        public required double Tolerance { get; init; }

        public required int MaxNewtonIterations { get; init; }

        public required MmaLinearSolver LinearSolver { get; init; }

        public required int DirectSolveLimit { get; init; }

        public required double ConjugateGradientTolerance { get; init; }

        public required int MaxConjugateGradientIterations { get; init; }

        /// <summary>
        /// Resolves the options for m constraints, validating them.
        /// </summary>
        public static SubproblemSettings From(MovingAsymptotesOptions options, int constraintCount)
        {
            ArgumentNullException.ThrowIfNull(options);
            var settings = new SubproblemSettings
            {
                A0 = options.A0,
                A = PerConstraint(options.ConstraintA, 0.0, constraintCount, nameof(options.ConstraintA)),
                C = PerConstraint(options.ConstraintC, 1000.0, constraintCount, nameof(options.ConstraintC)),
                D = PerConstraint(options.ConstraintD, 1.0, constraintCount, nameof(options.ConstraintD)),
                Tolerance = options.SubproblemTolerance,
                MaxNewtonIterations = options.MaxNewtonIterations,
                LinearSolver = options.LinearSolver,
                DirectSolveLimit = options.DirectSolveLimit,
                ConjugateGradientTolerance = options.ConjugateGradientTolerance,
                MaxConjugateGradientIterations = options.MaxConjugateGradientIterations,
            };
            settings.Validate();
            return settings;
        }

        private void Validate()
        {
            if (!(A0 > 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(A0), A0, "a0 must be positive.");
            }

            for (var i = 0; i < C.Length; i++)
            {
                if (!(A[i] >= 0.0) || !(C[i] >= 0.0) || !(D[i] >= 0.0) || !(C[i] + D[i] > 0.0))
                {
                    throw new ArgumentException($"Constraint {i}: a, c and d must be non-negative with c + d > 0 (a {A[i]}, c {C[i]}, d {D[i]}).");
                }
            }

            if (!(Tolerance > 0.0 && Tolerance < 1.0))
            {
                throw new ArgumentOutOfRangeException(nameof(Tolerance), Tolerance, "The subproblem tolerance must be in (0, 1).");
            }

            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(MaxNewtonIterations);
            ArgumentOutOfRangeException.ThrowIfNegative(DirectSolveLimit);
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(MaxConjugateGradientIterations);
            if (!(ConjugateGradientTolerance > 0.0))
            {
                throw new ArgumentOutOfRangeException(nameof(ConjugateGradientTolerance), ConjugateGradientTolerance, "Must be positive.");
            }
        }

        private static double[] PerConstraint(double[]? values, double standard, int count, string name)
        {
            if (values is null)
            {
                var filled = new double[count];
                Array.Fill(filled, standard);
                return filled;
            }

            if (values.Length != count)
            {
                throw new ArgumentException($"{name} has {values.Length} entries; the problem has {count} constraints.", name);
            }

            return (double[])values.Clone();
        }
    }
}
