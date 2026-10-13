/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Linq;
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
        public void DetachFoldsConstantsAndRepeats()
        {
            var g = new ScalarGraph();
            var x = g.Input(0, 0);
            var detached = g.Detach(x);
            Assert.AreEqual(ScalarOp.Detach, g[detached].Op);
            Assert.AreEqual(detached, g.Detach(detached));
            Assert.IsTrue(g.IsConst(g.Detach(g.Const(2)), 2));
        }

        [TestMethod]
        public void DetachedNodesCarryNoDerivative()
        {
            var g = new ScalarGraph();
            var x = g.Input(0, 0);
            var detached = g.Detach(x);
            var y = g.Mul(detached, x);
            var active = DerivativeFlow.Active(g, g.Count, group => group == 0);
            Assert.IsFalse(DerivativeFlow.DerivativeOperands(g[detached]).Any());
            Assert.IsFalse(active[detached]);
            Assert.IsTrue(active[y]);
        }

        [TestMethod]
        public void RebuildReplacesOperandsThroughTheBuilders()
        {
            var g = new ScalarGraph();
            var (x, y) = (g.Input(0, 0), g.Input(0, 1));
            var product = g[g.Mul(x, y)];
            Assert.IsTrue(g.IsConst(g.Rebuild(product, g.Const(3), g.Const(4), -1, -1), 12));
            Assert.AreEqual(x, g.Rebuild(product, x, g.Const(1), -1, -1));
            var select = g[g.Select(ScalarOp.SelectLess, x, y, x, y)];
            Assert.AreEqual(y, g.Rebuild(select, g.Const(2), g.Const(1), x, y));
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
