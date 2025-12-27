/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear.Tests
{
    /// <summary>
    /// High-dimensional test functions with analytical gradients.
    /// Used for testing optimizers on large-scale problems where AutoDiff code generation is impractical.
    /// </summary>
    public static class HighDimensionalFunctions
    {
        /// <summary>
        /// Extended Rosenbrock function in arbitrary dimensions with analytical gradient.
        /// f(x) = sum_{i=1}^{n-1} [100(x_{i+1} - x_i^2)^2 + (1 - x_i)^2]
        /// Minimum: x* = (1, 1, ..., 1), f(x*) = 0
        /// </summary>
        /// <param name="x">Parameter vector of dimension n.</param>
        /// <returns>Tuple of (function value, gradient vector).</returns>
        public static (double value, double[] gradient) ExtendedRosenbrock(double[] x)
        {
            var n = x.Length;
            if (n < 2)
            {
                throw new ArgumentException("Rosenbrock requires at least 2 dimensions", nameof(x));
            }

            var value = 0.0;
            var gradient = new double[n];

            // Compute function value and gradient contributions
            for (var i = 0; i < n - 1; i++)
            {
                var xi = x[i];
                var xip1 = x[i + 1];

                // Terms for this pair
                var term1 = xip1 - xi * xi;
                var term2 = 1.0 - xi;

                // Add to function value
                value += 100.0 * term1 * term1 + term2 * term2;

                // Gradient contributions
                // ∂f/∂x_i from term involving (x_{i+1} - x_i^2)
                gradient[i] += -400.0 * xi * term1;

                // ∂f/∂x_i from term involving (1 - x_i)
                gradient[i] += -2.0 * term2;

                // ∂f/∂x_{i+1} from term involving (x_{i+1} - x_i^2)
                gradient[i + 1] += 200.0 * term1;
            }

            return (value, gradient);
        }

        /// <summary>
        /// Creates an initial point for extended Rosenbrock that alternates between -1.2 and 1.0.
        /// This is a standard starting point that is far from the optimum.
        /// </summary>
        /// <param name="n">Dimension of the problem.</param>
        /// <returns>Initial point vector.</returns>
        public static double[] RosenbrockStartingPoint(int n)
        {
            var x0 = new double[n];
            for (var i = 0; i < n; i++)
            {
                x0[i] = (i % 2 == 0) ? -1.2 : 1.0;
            }
            return x0;
        }
    }
}
