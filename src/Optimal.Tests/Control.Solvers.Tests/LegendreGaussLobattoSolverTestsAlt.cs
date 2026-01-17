/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public sealed class LegendreGaussLobattoSolverTestsAlt : SolverTestsAlt
    {
        [TestMethod]
        public void ThrowsExceptionForInvalidOrder()
        {
            var solver = CreateSolver();

            Assert.ThrowsException<ArgumentException>(() => solver.WithOrder(1));
            Assert.ThrowsException<ArgumentException>(() => solver.WithOrder(0));
            Assert.ThrowsException<ArgumentException>(() => solver.WithOrder(-1));
        }

        protected override ISolver CreateSolver()
        {
            return new LegendreGaussLobattoSolver();
        }

        protected override InitialGuess CreateInitialGuess(ControlProblem problem, int segments, int order)
        {
            // LGL uses the order parameter
            return InitialGuessFactory.CreateForLGL(problem, segments, order);
        }
    }
}
