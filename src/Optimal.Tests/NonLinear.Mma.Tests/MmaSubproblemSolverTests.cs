/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Mma.Tests
{
    [TestClass]
    public sealed class MmaSubproblemSolverTests
    {
        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        [DataRow(MmaLinearSolver.Direct)]
        [DataRow(MmaLinearSolver.ConjugateGradient)]
        public void SolveAllocatesNothingAfterTheFirst(MmaLinearSolver solver)
        {
            var synthetic = SyntheticSubproblem.Create(10, 10, 8, includeVolume: true, solver);
            Assert.IsTrue(synthetic.Solve());
            var before = GC.GetAllocatedBytesForCurrentThread();
            Assert.IsTrue(synthetic.Solve());
            var allocated = GC.GetAllocatedBytesForCurrentThread() - before;

            TestContext.WriteLine($"{solver}: n = {synthetic.N}, m = {synthetic.M}, {allocated} bytes; {synthetic.Describe()}");
            Assert.AreEqual(0L, allocated);
        }

        [TestMethod]
        public void DirectAndConjugateGradientGiveTheSameSolution()
        {
            var direct = SyntheticSubproblem.Create(10, 10, 8, includeVolume: true, MmaLinearSolver.Direct);
            var iterative = SyntheticSubproblem.Create(10, 10, 8, includeVolume: true, MmaLinearSolver.ConjugateGradient);
            Assert.IsTrue(direct.Solve());
            Assert.IsTrue(iterative.Solve());

            Assert.IsTrue(direct.Solver.DirectLinearSolve);
            Assert.IsFalse(iterative.Solver.DirectLinearSolve);
            direct.AssertSolutionIsFeasible();
            iterative.AssertSolutionIsFeasible();
            for (var j = 0; j < direct.N; j++)
            {
                Assert.AreEqual(direct.Solver.X[j], iterative.Solver.X[j], 1e-9, $"x{j}");
            }

            for (var i = 0; i < direct.M; i++)
            {
                Assert.AreEqual(direct.Solver.Lambda[i], iterative.Solver.Lambda[i], 1e-7 * Math.Max(1.0, Math.Abs(direct.Solver.Lambda[i])), $"λ{i}");
            }
        }
    }
}
