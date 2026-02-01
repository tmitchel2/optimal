/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Threading;
using Optimal.Control.Collocation;
using Optimal.NonLinear.Monitoring;

namespace Optimal.Control.Core
{
    /// <summary>
    /// Interface for optimal control problem solvers.
    /// </summary>
    public interface ISolver
    {
        /// <summary>
        /// Solves the optimal control problem.
        /// </summary>
        /// <param name="problem">The control problem to solve.</param>
        /// <param name="initialGuess">Initial guess for state and control trajectories.</param>
        /// <param name="cancellationToken">Optional cancellation token to stop optimization early.</param>
        /// <returns>The optimal control solution.</returns>
        CollocationResult Solve(ControlProblem problem, InitialGuess initialGuess, CancellationToken cancellationToken);

        /// <summary>
        /// Gets the optimization monitor report after solving.
        /// Returns null if no monitor was configured.
        /// </summary>
        /// <returns>The monitoring report, or null if monitoring was not enabled.</returns>
        OptimisationMonitorReport? GetMonitorReport();
    }
}
