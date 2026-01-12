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

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    public sealed class HermiteSimpsonSolverTestsAlt : SolverTestsAlt
    {
        protected override ISolver CreateSolver()
        {
            return new HermiteSimpsonSolver();
        }
    }
}
