/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Threading;

namespace Optimal.NonLinear
{
    /// <summary>
    /// An optimizer for problems with simple bounds and inequality constraints (<see cref="InequalityProblem"/>).
    /// </summary>
    public interface IInequalityOptimizer
    {
        /// <summary>
        /// Minimizes the problem's objective from the given point, which is first projected onto the bounds.
        /// </summary>
        /// <param name="problem">The problem.</param>
        /// <param name="initialPoint">The starting point (n entries).</param>
        /// <param name="cancellationToken">Stops the run between iterations; the result is then UserRequested.</param>
        /// <returns>The result.</returns>
        InequalityOptimizerResult Minimize(InequalityProblem problem, double[] initialPoint, CancellationToken cancellationToken);
    }
}
