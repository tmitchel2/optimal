/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control
{
    /// <summary>
    /// Provides numerical gradient computation utilities for optimal control problems.
    /// </summary>
    public static class NumericalGradients
    {
        /// <summary>
        /// Computes the gradient of a scalar function using central finite differences.
        /// </summary>
        /// <param name="f">The function to differentiate.</param>
        /// <param name="x">The point at which to compute the gradient.</param>
        /// <param name="epsilon">The finite difference step size (default: 1e-8).</param>
        /// <returns>The gradient vector.</returns>
        public static double[] ComputeGradient(Func<double[], double> f, double[] x, double epsilon = 1e-8)
        {
            var n = x.Length;
            var gradient = new double[n];
            var xPerturbed = new double[n];
            Array.Copy(x, xPerturbed, n);

            for (var i = 0; i < n; i++)
            {
                // Forward perturbation
                var original = xPerturbed[i];
                xPerturbed[i] = original + epsilon;
                var fPlus = f(xPerturbed);

                // Backward perturbation
                xPerturbed[i] = original - epsilon;
                var fMinus = f(xPerturbed);

                // Central difference
                gradient[i] = (fPlus - fMinus) / (2.0 * epsilon);

                // Restore original value
                xPerturbed[i] = original;
            }

            return gradient;
        }

        /// <summary>
        /// Computes the gradient of a vector-valued function (Jacobian row) using finite differences.
        /// </summary>
        /// <param name="f">The function to differentiate (returns a single scalar).</param>
        /// <param name="x">The point at which to compute the gradient.</param>
        /// <param name="epsilon">The finite difference step size.</param>
        /// <returns>The gradient of the scalar output.</returns>
        public static double[] ComputeConstraintGradient(Func<double[], double> f, double[] x, double epsilon = 1e-8)
        {
            return ComputeGradient(f, x, epsilon);
        }

        /// <summary>
        /// Chooses an appropriate epsilon based on the magnitude of x.
        /// </summary>
        /// <param name="x">The point at which to compute derivatives.</param>
        /// <returns>Adaptive epsilon value.</returns>
        public static double ChooseEpsilon(double[] x)
        {
            var norm = 0.0;
            foreach (var xi in x)
            {
                norm += xi * xi;
            }
            norm = Math.Sqrt(norm);

            // Use relative epsilon for large values, absolute for small
            var baseEpsilon = 1e-8;
            if (norm > 1.0)
            {
                return baseEpsilon * Math.Sqrt(norm);
            }

            return baseEpsilon;
        }
    }
}
