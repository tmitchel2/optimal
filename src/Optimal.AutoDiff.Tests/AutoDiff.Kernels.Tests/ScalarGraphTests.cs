/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    [TestClass]
    public class ScalarGraphTests
    {
        [TestMethod]
        public void EqualExpressionsAreOneNode()
        {
            var g = new ScalarGraph();
            var (a, b) = (g.Input(0, 0), g.Input(0, 1));
            Assert.AreEqual(g.Mul(a, b), g.Mul(a, b));
            Assert.AreEqual(g.Mul(a, b), g.Mul(b, a));
            Assert.AreEqual(g.Add(a, b), g.Add(b, a));
            Assert.AreNotEqual(g.Sub(a, b), g.Sub(b, a));
        }

        [TestMethod]
        public void IdentitiesFold()
        {
            var g = new ScalarGraph();
            var x = g.Input(0, 0);
            var y = g.Input(0, 1);
            Assert.AreEqual(x, g.Add(x, g.Const(0)));
            Assert.AreEqual(x, g.Mul(g.Const(1), x));
            Assert.IsTrue(g.IsConst(g.Mul(x, g.Const(0)), 0));
            Assert.IsTrue(g.IsConst(g.Sub(x, x), 0));
            Assert.AreEqual(x, g.Neg(g.Neg(x)));
            Assert.AreEqual(g.Sub(x, y), g.Add(x, g.Neg(y)));
            Assert.AreEqual(g.Neg(g.Mul(x, y)), g.Mul(g.Neg(x), y));
            Assert.AreEqual(g.Mul(x, y), g.Mul(g.Neg(x), g.Neg(y)));
            Assert.AreEqual(g.Neg(x), g.Div(x, g.Const(-1)));
        }

        [TestMethod]
        public void ConstantsFold()
        {
            var g = new ScalarGraph();
            Assert.IsTrue(g.IsConst(g.Add(g.Const(2), g.Const(3)), 5));
            Assert.IsTrue(g.IsConst(g.Unary(ScalarOp.Sqrt, g.Const(4)), 2));
            Assert.IsTrue(g.IsConst(g.PowConst(g.Const(2), 10), 1024));
            Assert.IsTrue(g.IsConst(g.Min(g.Const(2), g.Const(-1)), -1));
            Assert.IsTrue(g.IsConst(g.Unary(ScalarOp.Sign, g.Const(-0.5)), -1));
        }

        [TestMethod]
        public void ConstantCubeRootIsLeftToTheRunTime()
        {
            var g = new ScalarGraph();
            Assert.AreEqual(ScalarOp.Cbrt, g[g.Unary(ScalarOp.Cbrt, g.Const(8))].Op);
        }

        [TestMethod]
        public void SmallConstantPowersAreRewritten()
        {
            var g = new ScalarGraph();
            var x = g.Input(0, 0);
            Assert.IsTrue(g.IsConst(g.PowConst(x, 0), 1));
            Assert.AreEqual(x, g.PowConst(x, 1));
            Assert.AreEqual(g.Mul(x, x), g.PowConst(x, 2));
            Assert.AreEqual(g.Div(g.Const(1), x), g.PowConst(x, -1));
            Assert.AreEqual(ScalarOp.PowConst, g[g.PowConst(x, 2.5)].Op);
        }

        [TestMethod]
        public void SelectsFold()
        {
            var g = new ScalarGraph();
            var (x, y) = (g.Input(0, 0), g.Input(0, 1));
            Assert.AreEqual(x, g.Select(ScalarOp.SelectLess, g.Const(1), g.Const(2), x, y));
            Assert.AreEqual(y, g.Select(ScalarOp.SelectLessEqual, g.Const(3), g.Const(2), x, y));
            Assert.AreEqual(x, g.Select(ScalarOp.SelectEqual, x, y, x, x));
            Assert.AreEqual(ScalarOp.SelectLess, g[g.Select(ScalarOp.SelectLess, x, y, x, y)].Op);
        }

        [TestMethod]
        public void AbsOfAbsAndOfANegationFold()
        {
            var g = new ScalarGraph();
            var x = g.Input(0, 0);
            var abs = g.Unary(ScalarOp.Abs, x);
            Assert.AreEqual(abs, g.Unary(ScalarOp.Abs, abs));
            Assert.AreEqual(abs, g.Unary(ScalarOp.Abs, g.Neg(x)));
        }

        [TestMethod]
        public void NegativeZeroIsNotZeroToIsConst()
        {
            var g = new ScalarGraph();
            Assert.IsFalse(g.IsConst(g.Const(-0.0), 0.0));
            Assert.AreNotEqual(g.Const(0.0), g.Const(-0.0));
        }
    }
}
