/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Mma.Tests
{
    [TestClass]
    public sealed class ReducedSystemSolverTests
    {
        [TestMethod]
        [DataRow(MmaLinearSolver.Direct)]
        [DataRow(MmaLinearSolver.ConjugateGradient)]
        public void ConstraintSpaceSolvesDiagPlusGWGt(MmaLinearSolver kind)
        {
            // m = 4 < n = 7: A = diag(D) + G diag(W) Gᵀ, 4×4.
            var matrix = ConstraintMatrixTests.Mixed();
            AssertSolves(matrix, kind, constraintSpace: true);
        }

        [TestMethod]
        [DataRow(MmaLinearSolver.Direct)]
        [DataRow(MmaLinearSolver.ConjugateGradient)]
        public void VariableSpaceSolvesDiagPlusGtWG(MmaLinearSolver kind)
        {
            // m = 5 >= n = 3: A = diag(D) + Gᵀ diag(W) G, 3×3.
            var matrix = ConstraintMatrix.Create(3, [
                ConstraintSparsity.Dense(2, 3),
                ConstraintSparsity.Csr(3, [0, 1, 3, 4], [2, 0, 1, 1]),
            ]);
            AssertSolves(matrix, kind, constraintSpace: false);
        }

        [TestMethod]
        public void AutomaticIsDirectUpToTheLimit()
        {
            var matrix = ConstraintMatrixTests.Mixed();

            Assert.IsTrue(new ReducedSystemSolver(matrix, MmaLinearSolver.Automatic, 4, 1e-12, 100).IsDirect);
            Assert.IsFalse(new ReducedSystemSolver(matrix, MmaLinearSolver.Automatic, 3, 1e-12, 100).IsDirect);
        }

        [TestMethod]
        public void DirectReportsAMatrixThatIsNotPositiveDefinite()
        {
            var matrix = ConstraintMatrixTests.Mixed();
            var solver = new ReducedSystemSolver(matrix, MmaLinearSolver.Direct, 100, 1e-12, 100);
            var g = new double[matrix.StoredCount];
            var diagonal = new double[matrix.RowCount];
            Array.Fill(diagonal, -1.0);

            Assert.IsFalse(solver.Prepare(g, diagonal, new double[matrix.ColumnCount]));
        }

        private static void AssertSolves(ConstraintMatrix matrix, MmaLinearSolver kind, bool constraintSpace)
        {
            var solver = new ReducedSystemSolver(matrix, kind, 100, 1e-13, 1000);
            Assert.AreEqual(constraintSpace, solver.ConstraintSpace);
            var size = solver.Size;
            var other = constraintSpace ? matrix.ColumnCount : matrix.RowCount;
            var g = ConstraintMatrixTests.Values(matrix, 0.8);
            var diagonal = ConstraintMatrixTests.Vector(size, 0.6, positive: true);
            var weights = ConstraintMatrixTests.Vector(other, 1.3, positive: true);
            var rhs = ConstraintMatrixTests.Vector(size, 0.9);
            Assert.IsTrue(solver.Prepare(g, diagonal, weights));
            var solution = new double[size];
            solver.Solve(rhs, solution);

            var dense = ConstraintMatrixTests.Dense(matrix, g);
            for (var r = 0; r < size; r++)
            {
                var applied = diagonal[r] * solution[r];
                for (var c = 0; c < size; c++)
                {
                    var entry = 0.0;
                    for (var k = 0; k < other; k++)
                    {
                        entry += constraintSpace
                            ? dense[r, k] * weights[k] * dense[c, k]
                            : dense[k, r] * weights[k] * dense[k, c];
                    }

                    applied += entry * solution[c];
                }

                Assert.AreEqual(rhs[r], applied, 1e-11, $"row {r}");
            }
        }
    }
}
