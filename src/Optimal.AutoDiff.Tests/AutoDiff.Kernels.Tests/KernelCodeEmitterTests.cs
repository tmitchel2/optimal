/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Immutable;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    [TestClass]
    public class KernelCodeEmitterTests
    {
        [TestMethod]
        [DataRow(1.5, "1.5d")]
        [DataRow(-2.0, "(-2d)")]
        [DataRow(0.1, "0.1d")]
        [DataRow(1e300, "1E+300d")]
        [DataRow(double.NaN, "double.NaN")]
        [DataRow(double.PositiveInfinity, "double.PositiveInfinity")]
        [DataRow(double.NegativeInfinity, "double.NegativeInfinity")]
        public void LiteralRoundTrips(double value, string expected)
        {
            Assert.AreEqual(expected, KernelCodeEmitter.Literal(value));
        }

        [TestMethod]
        public void NegativeZeroKeepsItsSign()
        {
            Assert.AreEqual("(-0d)", KernelCodeEmitter.Literal(-0.0));
            Assert.AreEqual("0d", KernelCodeEmitter.Literal(0.0));
        }

        [TestMethod]
        public void DeadNodesAreNotEmitted()
        {
            var g = new ScalarGraph();
            var x = g.Input(0, 0);
            var live = g.Mul(x, x);
            _ = g.Unary(ScalarOp.Sin, x);
            var input = new KernelParameter("x", 1, IsArray: false, IsOutput: false, Group: 0);
            var code = new KernelCodeEmitter(500).Emit("M", "Doc.", g, [input], [new KernelOutput("y", ImmutableArray.Create(live))]);
            Assert.Contains($"var v{live} = x * x;", code);
            Assert.DoesNotContain("Math.Sin", code);
        }

        [TestMethod]
        public void ChunkSizeMustBePositive()
        {
            _ = Assert.ThrowsExactly<ArgumentOutOfRangeException>(() => new KernelCodeEmitter(0));
        }
    }
}
