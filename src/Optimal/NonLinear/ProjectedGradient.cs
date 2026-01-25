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
    /// Utilities for projected gradient computation in bound-constrained optimization.
    /// These methods are essential for the L-BFGS-B algorithm.
    /// </summary>
    public static class ProjectedGradient
    {
        /// <summary>
        /// Projects a point onto the feasible box defined by lower and upper bounds.
        /// </summary>
        /// <param name="x">Point to project.</param>
        /// <param name="lower">Lower bounds (use double.NegativeInfinity for unbounded).</param>
        /// <param name="upper">Upper bounds (use double.PositiveInfinity for unbounded).</param>
        /// <returns>Projected point satisfying lower ≤ x ≤ upper.</returns>
        public static double[] Project(double[] x, double[] lower, double[] upper)
        {
            var n = x.Length;
            var projected = new double[n];
            for (var i = 0; i < n; i++)
            {
                projected[i] = Math.Max(lower[i], Math.Min(upper[i], x[i]));
            }
            return projected;
        }

        /// <summary>
        /// Projects a point onto the feasible box in-place.
        /// </summary>
        /// <param name="x">Point to project (modified in-place).</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        public static void ProjectInPlace(double[] x, double[] lower, double[] upper)
        {
            for (var i = 0; i < x.Length; i++)
            {
                x[i] = Math.Max(lower[i], Math.Min(upper[i], x[i]));
            }
        }

        /// <summary>
        /// Computes the projected gradient for convergence testing in bound-constrained optimization.
        /// The projected gradient measures how far the point would move if we took a unit
        /// gradient step and projected back onto the feasible region.
        /// pg_i = clamp(x_i - g_i, lower_i, upper_i) - x_i
        /// </summary>
        /// <param name="x">Current point (must be feasible).</param>
        /// <param name="gradient">Gradient at current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>Projected gradient vector.</returns>
        public static double[] ComputeProjectedGradient(double[] x, double[] gradient, double[] lower, double[] upper)
        {
            var n = x.Length;
            var pg = new double[n];
            for (var i = 0; i < n; i++)
            {
                // Take a unit step in negative gradient direction, then project
                var xMinusG = x[i] - gradient[i];
                var projected = Math.Max(lower[i], Math.Min(upper[i], xMinusG));
                pg[i] = projected - x[i];
            }
            return pg;
        }

        /// <summary>
        /// Computes the infinity norm of the projected gradient.
        /// This is the standard convergence measure for L-BFGS-B.
        /// The optimization is considered converged when this value is below tolerance.
        /// </summary>
        /// <param name="x">Current point (must be feasible).</param>
        /// <param name="gradient">Gradient at current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>||pg||_∞ = max_i |pg_i|</returns>
        public static double ProjectedGradientNormInf(double[] x, double[] gradient, double[] lower, double[] upper)
        {
            var maxNorm = 0.0;
            for (var i = 0; i < x.Length; i++)
            {
                var xMinusG = x[i] - gradient[i];
                var projected = Math.Max(lower[i], Math.Min(upper[i], xMinusG));
                var pg = Math.Abs(projected - x[i]);
                if (pg > maxNorm)
                {
                    maxNorm = pg;
                }
            }
            return maxNorm;
        }

        /// <summary>
        /// Computes the 2-norm of the projected gradient.
        /// Alternative convergence measure for L-BFGS-B.
        /// </summary>
        /// <param name="x">Current point (must be feasible).</param>
        /// <param name="gradient">Gradient at current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>||pg||_2</returns>
        public static double ProjectedGradientNorm2(double[] x, double[] gradient, double[] lower, double[] upper)
        {
            var sumSq = 0.0;
            for (var i = 0; i < x.Length; i++)
            {
                var xMinusG = x[i] - gradient[i];
                var projected = Math.Max(lower[i], Math.Min(upper[i], xMinusG));
                var pg = projected - x[i];
                sumSq += pg * pg;
            }
            return Math.Sqrt(sumSq);
        }

        /// <summary>
        /// Computes the maximum feasible step size along a direction while respecting bounds.
        /// Returns the largest alpha such that lower ≤ x + alpha * direction ≤ upper.
        /// </summary>
        /// <param name="x">Current point (must be feasible).</param>
        /// <param name="direction">Search direction.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>Maximum feasible step size (positive infinity if unbounded).</returns>
        public static double MaxFeasibleStep(double[] x, double[] direction, double[] lower, double[] upper)
        {
            var maxStep = double.PositiveInfinity;

            for (var i = 0; i < x.Length; i++)
            {
                if (direction[i] > 1e-16)
                {
                    // Moving toward upper bound
                    var stepToUpper = (upper[i] - x[i]) / direction[i];
                    if (stepToUpper < maxStep)
                    {
                        maxStep = stepToUpper;
                    }
                }
                else if (direction[i] < -1e-16)
                {
                    // Moving toward lower bound
                    var stepToLower = (lower[i] - x[i]) / direction[i];
                    if (stepToLower < maxStep)
                    {
                        maxStep = stepToLower;
                    }
                }
            }

            return Math.Max(0.0, maxStep);
        }

        /// <summary>
        /// Identifies which variables are at their bounds.
        /// A variable is considered at a bound if it is within tolerance of the bound
        /// and the gradient would push it further into the bound.
        /// </summary>
        /// <param name="x">Current point.</param>
        /// <param name="gradient">Gradient at current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <param name="tolerance">Tolerance for bound detection (default 1e-10).</param>
        /// <returns>
        /// Array indicating variable status:
        /// -1 = at lower bound (active)
        ///  0 = free (not at bound)
        /// +1 = at upper bound (active)
        /// </returns>
        public static int[] IdentifyActiveSet(double[] x, double[] gradient, double[] lower, double[] upper, double tolerance = 1e-10)
        {
            var n = x.Length;
            var activeSet = new int[n];

            for (var i = 0; i < n; i++)
            {
                if (x[i] <= lower[i] + tolerance && gradient[i] > 0)
                {
                    // At lower bound and gradient points outward (would decrease x further)
                    activeSet[i] = -1;
                }
                else if (x[i] >= upper[i] - tolerance && gradient[i] < 0)
                {
                    // At upper bound and gradient points outward (would increase x further)
                    activeSet[i] = 1;
                }
                else
                {
                    // Free variable
                    activeSet[i] = 0;
                }
            }

            return activeSet;
        }

        /// <summary>
        /// Computes the search direction with active variables zeroed out.
        /// For variables at bounds where the direction would push them further into
        /// the bound, the direction component is set to zero.
        /// </summary>
        /// <param name="direction">Unconstrained search direction.</param>
        /// <param name="x">Current point.</param>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <param name="tolerance">Tolerance for bound detection.</param>
        /// <returns>Direction with active components zeroed.</returns>
        public static double[] ZeroActiveComponents(double[] direction, double[] x, double[] lower, double[] upper, double tolerance = 1e-10)
        {
            var n = direction.Length;
            var constrainedDirection = new double[n];

            for (var i = 0; i < n; i++)
            {
                var atLower = x[i] <= lower[i] + tolerance;
                var atUpper = x[i] >= upper[i] - tolerance;

                if (atLower && direction[i] < 0)
                {
                    // At lower bound, direction would push further down - zero it
                    constrainedDirection[i] = 0;
                }
                else if (atUpper && direction[i] > 0)
                {
                    // At upper bound, direction would push further up - zero it
                    constrainedDirection[i] = 0;
                }
                else
                {
                    constrainedDirection[i] = direction[i];
                }
            }

            return constrainedDirection;
        }
    }
}
