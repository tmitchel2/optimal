/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Constrained
{
    /// <summary>
    /// Options for the Augmented Lagrangian optimizer.
    /// </summary>
    public record AugmentedLagrangianOptions : OptimizerOptions
    {
        /// <summary>
        /// Initial penalty parameter.
        /// </summary>
        public double PenaltyParameter { get; init; } = 1.0;

        /// <summary>
        /// Constraint violation tolerance.
        /// </summary>
        public double ConstraintTolerance { get; init; } = 1e-6;

        /// <summary>
        /// Equality constraints h(x) = 0.
        /// </summary>
        public IReadOnlyList<Func<double[], (double value, double[] gradient)>> EqualityConstraints { get; init; }
            = Array.Empty<Func<double[], (double value, double[] gradient)>>();

        /// <summary>
        /// Inequality constraints g(x) &lt;= 0.
        /// </summary>
        public IReadOnlyList<Func<double[], (double value, double[] gradient)>> InequalityConstraints { get; init; }
            = Array.Empty<Func<double[], (double value, double[] gradient)>>();

        /// <summary>
        /// Box constraints (optional).
        /// </summary>
        public BoxConstraints? BoxConstraints { get; init; }

        /// <summary>
        /// Optional callback invoked after each outer Augmented Lagrangian iteration.
        /// Parameters: (outerIteration, objectiveValue, maxViolation, penaltyParameter, innerIterations, currentPoint).
        /// </summary>
        public Action<int, double, double, double, int, double[]>? ProgressCallback { get; init; }
    }
}
