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
    public sealed class DenseCholeskyTests
    {
        [TestMethod]
        public void SolvesASymmetricPositiveDefiniteSystem()
        {
            const int k = 6;
            var a = new double[k * k];
            for (var i = 0; i < k; i++)
            {
                for (var j = 0; j < k; j++)
                {
                    a[(i * k) + j] = (i == j ? k + 1.0 : 0.0) + (1.0 / (1.0 + i + j));
                }
            }

            var original = (double[])a.Clone();
            var b = new double[k];
            for (var i = 0; i < k; i++)
            {
                b[i] = Math.Sin(i + 1.0);
            }

            var x = (double[])b.Clone();
            Assert.IsTrue(DenseCholesky.Factor(a, k));
            DenseCholesky.Solve(a, k, x);

            for (var i = 0; i < k; i++)
            {
                var sum = 0.0;
                for (var j = 0; j < k; j++)
                {
                    sum += original[(i * k) + j] * x[j];
                }

                Assert.AreEqual(b[i], sum, 1e-13, $"row {i}");
            }
        }

        [TestMethod]
        public void RefusesAnIndefiniteMatrix()
        {
            double[] a = [1.0, 2.0, 2.0, 1.0];

            Assert.IsFalse(DenseCholesky.Factor(a, 2));
        }
    }
}
