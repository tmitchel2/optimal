/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Control.Core.Tests
{
    [TestClass]
    public sealed class ControlProblemTests
    {
        private static readonly double[] s_initialState = new[] { 0.0, 0.0 };
        private static readonly double[] s_finalState = new[] { 1.0, 0.0 };
        private static readonly double[] s_controlLower = new[] { -1.0 };
        private static readonly double[] s_controlUpper = new[] { 1.0 };
        private static readonly double[] s_stateLower = new[] { -10.0, -5.0 };
        private static readonly double[] s_stateUpper = new[] { 10.0, 5.0 };

        [TestMethod]
        public void CanBuildSimpleProblem()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 10.0)
                .WithInitialCondition(s_initialState)
                .WithFinalCondition(s_finalState);

            Assert.AreEqual(2, problem.StateDim);
            Assert.AreEqual(1, problem.ControlDim);
            Assert.AreEqual(0.0, problem.InitialTime);
            Assert.AreEqual(10.0, problem.FinalTime);
            Assert.IsNotNull(problem.InitialState);
            Assert.IsNotNull(problem.FinalState);
        }

        [TestMethod]
        public void CanSetBounds()
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithControlBounds(s_controlLower, s_controlUpper)
                .WithStateBounds(s_stateLower, s_stateUpper);

            Assert.IsNotNull(problem.ControlLowerBounds);
            Assert.IsNotNull(problem.ControlUpperBounds);
            Assert.IsNotNull(problem.StateLowerBounds);
            Assert.IsNotNull(problem.StateUpperBounds);
        }

        [TestMethod]
        public void ThrowsOnInvalidStateDimension()
        {
            var problem = new ControlProblem();
            Assert.ThrowsException<ArgumentException>(() => problem.WithStateSize(0));
            Assert.ThrowsException<ArgumentException>(() => problem.WithStateSize(-1));
        }

        [TestMethod]
        public void ThrowsOnInvalidControlDimension()
        {
            var problem = new ControlProblem();
            Assert.ThrowsException<ArgumentException>(() => problem.WithControlSize(0));
            Assert.ThrowsException<ArgumentException>(() => problem.WithControlSize(-1));
        }

        [TestMethod]
        public void ThrowsOnInvalidTimeHorizon()
        {
            var problem = new ControlProblem();
            Assert.ThrowsException<ArgumentException>(() => problem.WithTimeHorizon(10.0, 10.0));
            Assert.ThrowsException<ArgumentException>(() => problem.WithTimeHorizon(10.0, 5.0));
        }
    }
}
