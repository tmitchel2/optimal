/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Constraints.Tests
{
    [TestClass]
    public sealed class ConstraintSparsityTests
    {
        [TestMethod]
        public void DenseStoresEveryEntry()
        {
            var sparsity = ConstraintSparsity.Dense(3, 4);

            Assert.IsTrue(sparsity.IsDense);
            Assert.AreEqual(3, sparsity.RowCount);
            Assert.AreEqual(12, sparsity.StoredCount);
            Assert.IsTrue(sparsity.RowPointers.IsEmpty);
        }

        [TestMethod]
        public void CsrCopiesAValidPattern()
        {
            int[] pointers = [0, 2, 2, 3];
            int[] columns = [1, 3, 0];
            var sparsity = ConstraintSparsity.Csr(4, pointers, columns);
            pointers[1] = 99;

            Assert.IsFalse(sparsity.IsDense);
            Assert.AreEqual(3, sparsity.RowCount);
            Assert.AreEqual(3, sparsity.StoredCount);
            Assert.AreEqual(2, sparsity.RowPointers[1]);
        }

        [TestMethod]
        public void CsrRejectsUnsortedDuplicateOrOutOfRangeColumns()
        {
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [0, 2], [3, 1]));
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [0, 2], [1, 1]));
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [0, 1], [4]));
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [0, 1], [-1]));
        }

        [TestMethod]
        public void CsrRejectsInconsistentRowPointers()
        {
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [0], []));
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [1, 2], [0, 1]));
            Assert.ThrowsExactly<ArgumentException>(() => ConstraintSparsity.Csr(4, [0, 1], [0, 1]));
        }

        [TestMethod]
        public void DenseRejectsEmptyShapes()
        {
            Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstraintSparsity.Dense(0, 4));
            Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => ConstraintSparsity.Dense(2, 0));
        }
    }
}
