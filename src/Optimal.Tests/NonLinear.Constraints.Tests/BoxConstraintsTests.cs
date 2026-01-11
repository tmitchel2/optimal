/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - test code using inline arrays is clearer

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Constraints.Tests
{
    [TestClass]
    public sealed class BoxConstraintsTests
    {
        [TestMethod]
        public void ConstructorThrowsWhenBoundsHaveDifferentLengths()
        {
            var lower = new[] { 0.0, 0.0 };
            var upper = new[] { 1.0, 1.0, 1.0 };

            Assert.ThrowsException<ArgumentException>(() => new BoxConstraints(lower, upper));
        }

        [TestMethod]
        public void ConstructorClonesArrays()
        {
            var lower = new[] { 0.0, 0.0 };
            var upper = new[] { 1.0, 1.0 };
            var constraints = new BoxConstraints(lower, upper);

            // Modify original arrays
            lower[0] = 999.0;
            upper[0] = 999.0;

            // Constraints should still have original values
            Assert.AreEqual(0.0, constraints.Lower[0]);
            Assert.AreEqual(1.0, constraints.Upper[0]);
        }

        [TestMethod]
        public void LowerPropertyReturnsClonedArray()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 1.0 }, new[] { 2.0, 3.0 });
            var lower = constraints.Lower;

            lower[0] = 999.0;

            Assert.AreEqual(0.0, constraints.Lower[0]);
        }

        [TestMethod]
        public void UpperPropertyReturnsClonedArray()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 1.0 }, new[] { 2.0, 3.0 });
            var upper = constraints.Upper;

            upper[0] = 999.0;

            Assert.AreEqual(2.0, constraints.Upper[0]);
        }

        [TestMethod]
        public void ProjectClampsToLowerBound()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { -1.0, -2.0 };

            var projected = constraints.Project(x);

            Assert.AreEqual(0.0, projected[0]);
            Assert.AreEqual(0.0, projected[1]);
        }

        [TestMethod]
        public void ProjectClampsToUpperBound()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 2.0, 3.0 };

            var projected = constraints.Project(x);

            Assert.AreEqual(1.0, projected[0]);
            Assert.AreEqual(1.0, projected[1]);
        }

        [TestMethod]
        public void ProjectLeavesPointWithinBoundsUnchanged()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 0.5, 0.5 };

            var projected = constraints.Project(x);

            Assert.AreEqual(0.5, projected[0]);
            Assert.AreEqual(0.5, projected[1]);
        }

        [TestMethod]
        public void ProjectHandlesMixedViolations()
        {
            var constraints = new BoxConstraints(new[] { 0.0, -1.0 }, new[] { 1.0, 2.0 });
            var x = new[] { -0.5, 5.0 };

            var projected = constraints.Project(x);

            Assert.AreEqual(0.0, projected[0]); // Clamped to lower
            Assert.AreEqual(2.0, projected[1]); // Clamped to upper
        }

        [TestMethod]
        public void IsFeasibleReturnsTrueForPointWithinBounds()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 0.5, 0.5 };

            Assert.IsTrue(constraints.IsFeasible(x));
        }

        [TestMethod]
        public void IsFeasibleReturnsFalseForPointBelowLowerBound()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { -0.1, 0.5 };

            Assert.IsFalse(constraints.IsFeasible(x));
        }

        [TestMethod]
        public void IsFeasibleReturnsFalseForPointAboveUpperBound()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 0.5, 1.1 };

            Assert.IsFalse(constraints.IsFeasible(x));
        }

        [TestMethod]
        public void IsFeasibleReturnsTrueForPointOnBoundary()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 0.0, 1.0 };

            Assert.IsTrue(constraints.IsFeasible(x));
        }

        [TestMethod]
        public void IsFeasibleUsesTolerance()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { -1e-12, 1.0 + 1e-12 };

            Assert.IsTrue(constraints.IsFeasible(x, tolerance: 1e-10));
        }

        [TestMethod]
        public void MaxViolationReturnsZeroForFeasiblePoint()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 0.5, 0.5 };

            Assert.AreEqual(0.0, constraints.MaxViolation(x));
        }

        [TestMethod]
        public void MaxViolationReturnsCorrectValueForLowerViolation()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { -0.3, 0.5 };

            Assert.AreEqual(0.3, constraints.MaxViolation(x), 1e-10);
        }

        [TestMethod]
        public void MaxViolationReturnsCorrectValueForUpperViolation()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { 0.5, 1.4 };

            Assert.AreEqual(0.4, constraints.MaxViolation(x), 1e-10);
        }

        [TestMethod]
        public void MaxViolationReturnsLargestViolation()
        {
            var constraints = new BoxConstraints(new[] { 0.0, 0.0 }, new[] { 1.0, 1.0 });
            var x = new[] { -0.2, 1.5 }; // Lower violation 0.2, upper violation 0.5

            Assert.AreEqual(0.5, constraints.MaxViolation(x), 1e-10);
        }

        [TestMethod]
        public void ProjectReturnsNewArray()
        {
            var constraints = new BoxConstraints(new[] { 0.0 }, new[] { 1.0 });
            var x = new[] { 0.5 };

            var projected = constraints.Project(x);

            Assert.AreNotSame(x, projected);
        }

        [TestMethod]
        public void WorksWithInfinityBounds()
        {
            var constraints = new BoxConstraints(
                new[] { double.NegativeInfinity, 0.0 },
                new[] { double.PositiveInfinity, 1.0 });
            var x = new[] { -1000.0, 0.5 };

            Assert.IsTrue(constraints.IsFeasible(x));
            Assert.AreEqual(0.0, constraints.MaxViolation(x));

            var projected = constraints.Project(x);
            Assert.AreEqual(-1000.0, projected[0]);
        }
    }
}
