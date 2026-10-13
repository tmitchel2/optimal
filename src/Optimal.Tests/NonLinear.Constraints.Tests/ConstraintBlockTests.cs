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
    public sealed class ConstraintBlockTests
    {
        private static readonly double[] s_values = [5.0, -1.0];
        private static readonly double[] s_jacobian = [3.0, 2.0, 1.0, -1.0];
        private static readonly double[] s_weights = [1.0, 2.0, 3.0];

        [TestMethod]
        public void FromFunctionsWritesRowsOfTheDenseJacobian()
        {
            var block = ConstraintBlock.FromFunctions(2, [
                x => ((x[0] * x[1]) - 1.0, [x[1], x[0]]),
                x => (x[0] - x[1], [1.0, -1.0]),
            ]);
            var values = new double[2];
            var jacobian = new double[4];
            block.Evaluate([2.0, 3.0], values, jacobian);

            Assert.IsTrue(block.Sparsity.IsDense);
            CollectionAssert.AreEqual(s_values, values);
            CollectionAssert.AreEqual(s_jacobian, jacobian);
        }

        [TestMethod]
        public void FromFunctionsSkipsTheJacobianWhenItIsNotWanted()
        {
            var block = ConstraintBlock.FromFunctions(1, [x => (x[0], [1.0])]);
            var values = new double[1];
            block.Evaluate([4.0], values, Span<double>.Empty);

            Assert.AreEqual(4.0, values[0]);
        }

        [TestMethod]
        public void FromFunctionsRejectsAGradientOfTheWrongLength()
        {
            var block = ConstraintBlock.FromFunctions(2, [x => (x[0], [1.0])]);

            Assert.ThrowsExactly<InvalidOperationException>(() => block.Evaluate([1.0, 2.0], new double[1], new double[2]));
        }

        [TestMethod]
        public void LinearBlockIsWeightsDotXLessTheLimit()
        {
            var block = new LinearConstraintBlock([1.0, 2.0, 3.0], 4.0);
            var values = new double[1];
            var jacobian = new double[3];
            block.Evaluate([1.0, 1.0, 1.0], values, jacobian);

            Assert.AreEqual(2.0, values[0]);
            CollectionAssert.AreEqual(s_weights, jacobian);
            Assert.AreEqual(4.0, block.Limit);
        }

        [TestMethod]
        public void LinearBlockRejectsANonFiniteLimit()
        {
            Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new LinearConstraintBlock([1.0], double.NaN));
        }
    }
}
