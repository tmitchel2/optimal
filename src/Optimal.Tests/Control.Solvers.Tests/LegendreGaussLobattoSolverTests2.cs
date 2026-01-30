/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public sealed class LegendreGaussLobattoSolverTests2 : SolverTests
    {
        protected override ISolver CreateSolver(
            int segments = 20,
            double tolerance = 1e-6,
            int maxIterations = 100,
            bool verbose = false)
        {
            return new LegendreGaussLobattoSolver(
                new LegendreGaussLobattoSolverOptions
                {
                    Segments = segments,
                    Tolerance = tolerance,
                    MaxIterations = maxIterations,
                    Verbose = verbose,
                    Order = 4
                },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));
        }

        protected override InitialGuess CreateInitialGuess(ControlProblem problem, int segments)
        {
            // LGL solver uses default order of 4
            return InitialGuessFactory.CreateForLGL(problem, segments, 4);
        }
    }
}
