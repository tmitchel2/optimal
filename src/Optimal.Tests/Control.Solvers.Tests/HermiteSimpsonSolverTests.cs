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
    public sealed class HermiteSimpsonSolverTests : SolverTests
    {
        protected override ISolver CreateSolver(
            int segments = 20,
            double tolerance = 1e-6,
            int maxIterations = 100,
            bool verbose = false)
        {
            return new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions
                {
                    Segments = segments,
                    Tolerance = tolerance,
                    MaxIterations = maxIterations,
                    Verbose = verbose
                },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()));
        }

        protected override InitialGuess CreateInitialGuess(ControlProblem problem, int segments)
        {
            return InitialGuessFactory.CreateWithControlHeuristics(problem, segments);
        }
    }
}
