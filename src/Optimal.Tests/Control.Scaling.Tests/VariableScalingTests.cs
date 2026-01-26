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

namespace Optimal.Control.Scaling.Tests
{
    [TestClass]
    public sealed class VariableScalingTests
    {
        [TestMethod]
        public void FromBoundsWithSymmetricRangeComputesCorrectScaling()
        {
            // [-10, 10] should have scale=10, center=0
            var scaling = VariableScaling.FromBounds(
                new[] { -10.0 }, new[] { 10.0 },
                new[] { -1.0 }, new[] { 1.0 });

            Assert.AreEqual(10.0, scaling.StateScales[0], 1e-10);
            Assert.AreEqual(0.0, scaling.StateCenters[0], 1e-10);
            Assert.AreEqual(1.0, scaling.ControlScales[0], 1e-10);
            Assert.AreEqual(0.0, scaling.ControlCenters[0], 1e-10);
        }

        [TestMethod]
        public void FromBoundsWithAsymmetricRangeComputesCorrectCenter()
        {
            // [0, 100] should have scale=50, center=50
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 100.0 },
                new[] { -2.0 }, new[] { 2.0 });

            Assert.AreEqual(50.0, scaling.StateScales[0], 1e-10);
            Assert.AreEqual(50.0, scaling.StateCenters[0], 1e-10);
            Assert.AreEqual(2.0, scaling.ControlScales[0], 1e-10);
            Assert.AreEqual(0.0, scaling.ControlCenters[0], 1e-10);
        }

        [TestMethod]
        public void FromBoundsWithMultipleVariablesComputesCorrectlyForEach()
        {
            // V: [1, 70], lambda: [-0.3, 0.3]
            var scaling = VariableScaling.FromBounds(
                new[] { 1.0, -0.3 }, new[] { 70.0, 0.3 },
                new[] { -0.5 }, new[] { 0.5 });

            // V: scale = (70-1)/2 = 34.5, center = (70+1)/2 = 35.5
            Assert.AreEqual(34.5, scaling.StateScales[0], 1e-10);
            Assert.AreEqual(35.5, scaling.StateCenters[0], 1e-10);

            // lambda: scale = (0.3-(-0.3))/2 = 0.3, center = 0
            Assert.AreEqual(0.3, scaling.StateScales[1], 1e-10);
            Assert.AreEqual(0.0, scaling.StateCenters[1], 1e-10);

            // delta: scale = 0.5, center = 0
            Assert.AreEqual(0.5, scaling.ControlScales[0], 1e-10);
            Assert.AreEqual(0.0, scaling.ControlCenters[0], 1e-10);
        }

        [TestMethod]
        public void ScaleUnscaleRoundTripPreservesValues()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 1.0, -15.0 }, new[] { 70.0, 15.0 },
                new[] { -0.5, -1.0 }, new[] { 0.5, 1.0 });

            var originalState = new[] { 35.5, 7.5 };
            var originalControl = new[] { 0.25, -0.5 };

            var scaledState = scaling.ScaleState(originalState);
            var scaledControl = scaling.ScaleControl(originalControl);

            var recoveredState = scaling.UnscaleState(scaledState);
            var recoveredControl = scaling.UnscaleControl(scaledControl);

            Assert.AreEqual(originalState[0], recoveredState[0], 1e-12);
            Assert.AreEqual(originalState[1], recoveredState[1], 1e-12);
            Assert.AreEqual(originalControl[0], recoveredControl[0], 1e-12);
            Assert.AreEqual(originalControl[1], recoveredControl[1], 1e-12);
        }

        [TestMethod]
        public void ScaleStateMapsLowerBoundToMinusOne()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 1.0 }, new[] { 70.0 },
                new[] { -1.0 }, new[] { 1.0 });

            var scaled = scaling.ScaleState(new[] { 1.0 });

            Assert.AreEqual(-1.0, scaled[0], 1e-10);
        }

        [TestMethod]
        public void ScaleStateMapsUpperBoundToPlusOne()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 1.0 }, new[] { 70.0 },
                new[] { -1.0 }, new[] { 1.0 });

            var scaled = scaling.ScaleState(new[] { 70.0 });

            Assert.AreEqual(1.0, scaled[0], 1e-10);
        }

        [TestMethod]
        public void ScaleStateMapsCenterToZero()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 1.0 }, new[] { 70.0 },
                new[] { -1.0 }, new[] { 1.0 });

            var center = (1.0 + 70.0) / 2.0; // 35.5
            var scaled = scaling.ScaleState(new[] { center });

            Assert.AreEqual(0.0, scaled[0], 1e-10);
        }

        [TestMethod]
        public void HandleUnboundedVariableUsesDefaultScale()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { double.NegativeInfinity }, new[] { double.PositiveInfinity },
                new[] { -1.0 }, new[] { 1.0 });

            Assert.AreEqual(1.0, scaling.StateScales[0], 1e-10);
            Assert.AreEqual(0.0, scaling.StateCenters[0], 1e-10);
        }

        [TestMethod]
        public void HandleZeroRangeVariableTreatsAsFixed()
        {
            // Fixed variable at value 5.0
            var scaling = VariableScaling.FromBounds(
                new[] { 5.0 }, new[] { 5.0 },
                new[] { 0.0 }, new[] { 0.0 });

            Assert.AreEqual(1.0, scaling.StateScales[0], 1e-10);
            Assert.AreEqual(5.0, scaling.StateCenters[0], 1e-10);
        }

        [TestMethod]
        public void HandleTinyRangeVariableTreatsAsFixed()
        {
            // Very small range - should be treated as fixed
            var scaling = VariableScaling.FromBounds(
                new[] { 5.0 }, new[] { 5.0 + 1e-15 },
                new[] { 0.0 }, new[] { 1.0 });

            Assert.AreEqual(1.0, scaling.StateScales[0], 1e-10);
        }

        [TestMethod]
        public void ScaleStateBoundsReturnsMinusOneToPlusOne()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 1.0, -15.0 }, new[] { 70.0, 15.0 },
                new[] { -0.5 }, new[] { 0.5 });

            var (scaledLower, scaledUpper) = scaling.ScaleStateBounds(
                new[] { 1.0, -15.0 }, new[] { 70.0, 15.0 });

            Assert.AreEqual(-1.0, scaledLower[0], 1e-10);
            Assert.AreEqual(-1.0, scaledLower[1], 1e-10);
            Assert.AreEqual(1.0, scaledUpper[0], 1e-10);
            Assert.AreEqual(1.0, scaledUpper[1], 1e-10);
        }

        [TestMethod]
        public void ScaleControlBoundsReturnsMinusOneToPlusOne()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 10.0 },
                new[] { -0.5, -1.0 }, new[] { 0.5, 1.0 });

            var (scaledLower, scaledUpper) = scaling.ScaleControlBounds(
                new[] { -0.5, -1.0 }, new[] { 0.5, 1.0 });

            Assert.AreEqual(-1.0, scaledLower[0], 1e-10);
            Assert.AreEqual(-1.0, scaledLower[1], 1e-10);
            Assert.AreEqual(1.0, scaledUpper[0], 1e-10);
            Assert.AreEqual(1.0, scaledUpper[1], 1e-10);
        }

        [TestMethod]
        public void IdentityDoesNotTransform()
        {
            var scaling = VariableScaling.Identity(3, 2);

            var state = new[] { 1.0, 2.0, 3.0 };
            var control = new[] { 4.0, 5.0 };

            var scaledState = scaling.ScaleState(state);
            var scaledControl = scaling.ScaleControl(control);

            Assert.AreEqual(1.0, scaledState[0], 1e-10);
            Assert.AreEqual(2.0, scaledState[1], 1e-10);
            Assert.AreEqual(3.0, scaledState[2], 1e-10);
            Assert.AreEqual(4.0, scaledControl[0], 1e-10);
            Assert.AreEqual(5.0, scaledControl[1], 1e-10);
        }

        [TestMethod]
        public void ConstructorThrowsOnMismatchedStateLengths()
        {
            Assert.Throws<ArgumentException>(() => new VariableScaling(
                new[] { 1.0, 2.0 },
                new[] { 0.0 }, // Wrong length
                new[] { 1.0 },
                new[] { 0.0 }));
        }

        [TestMethod]
        public void ConstructorThrowsOnMismatchedControlLengths()
        {
            Assert.Throws<ArgumentException>(() => new VariableScaling(
                new[] { 1.0 },
                new[] { 0.0 },
                new[] { 1.0, 2.0 },
                new[] { 0.0 })); // Wrong length
        }

        [TestMethod]
        public void FromBoundsThrowsOnMismatchedStateBounds()
        {
            Assert.Throws<ArgumentException>(() => VariableScaling.FromBounds(
                new[] { 0.0, 0.0 },
                new[] { 1.0 }, // Wrong length
                new[] { 0.0 },
                new[] { 1.0 }));
        }

        [TestMethod]
        public void FromBoundsThrowsOnMismatchedControlBounds()
        {
            Assert.Throws<ArgumentException>(() => VariableScaling.FromBounds(
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 0.0, 0.0 },
                new[] { 1.0 })); // Wrong length
        }

        [TestMethod]
        public void ScaleStateThrowsOnWrongDimension()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 1.0 },
                new[] { 0.0 }, new[] { 1.0 });

            Assert.Throws<ArgumentException>(() => scaling.ScaleState(new[] { 0.5, 0.5 }));
        }

        [TestMethod]
        public void UnscaleStateThrowsOnWrongDimension()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 1.0 },
                new[] { 0.0 }, new[] { 1.0 });

            Assert.Throws<ArgumentException>(() => scaling.UnscaleState(new[] { 0.5, 0.5 }));
        }

        [TestMethod]
        public void ScaleControlThrowsOnWrongDimension()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 1.0 },
                new[] { 0.0 }, new[] { 1.0 });

            Assert.Throws<ArgumentException>(() => scaling.ScaleControl(new[] { 0.5, 0.5 }));
        }

        [TestMethod]
        public void UnscaleControlThrowsOnWrongDimension()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 1.0 },
                new[] { 0.0 }, new[] { 1.0 });

            Assert.Throws<ArgumentException>(() => scaling.UnscaleControl(new[] { 0.5, 0.5 }));
        }

        [TestMethod]
        public void StateDimPropertyReturnsCorrectValue()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0, 1.0, 2.0 }, new[] { 1.0, 2.0, 3.0 },
                new[] { 0.0 }, new[] { 1.0 });

            Assert.AreEqual(3, scaling.StateDim);
        }

        [TestMethod]
        public void ControlDimPropertyReturnsCorrectValue()
        {
            var scaling = VariableScaling.FromBounds(
                new[] { 0.0 }, new[] { 1.0 },
                new[] { 0.0, 1.0 }, new[] { 1.0, 2.0 });

            Assert.AreEqual(2, scaling.ControlDim);
        }

        [TestMethod]
        public void CornerProblemLikeScalingWorksCorrectly()
        {
            // Mimics the CornerProblem variable scales
            var scaling = VariableScaling.FromBounds(
                stateLower: new[] { 1.0, -15.0, -15.0, -2.0, -1.047, -0.3, -2.0, 0.0 },
                stateUpper: new[] { 70.0, 15.0, 15.0, 2.0, 1.047, 0.3, 2.0, 60.0 },
                controlLower: new[] { -0.5, -1.0 },
                controlUpper: new[] { 0.5, 1.0 });

            // Test that all variables scale to approximately [-1, 1]
            var lowerScaled = scaling.ScaleState(new[] { 1.0, -15.0, -15.0, -2.0, -1.047, -0.3, -2.0, 0.0 });
            var upperScaled = scaling.ScaleState(new[] { 70.0, 15.0, 15.0, 2.0, 1.047, 0.3, 2.0, 60.0 });

            for (var i = 0; i < 8; i++)
            {
                Assert.AreEqual(-1.0, lowerScaled[i], 1e-10, $"State {i} lower bound should scale to -1");
                Assert.AreEqual(1.0, upperScaled[i], 1e-10, $"State {i} upper bound should scale to 1");
            }

            var controlLowerScaled = scaling.ScaleControl(new[] { -0.5, -1.0 });
            var controlUpperScaled = scaling.ScaleControl(new[] { 0.5, 1.0 });

            Assert.AreEqual(-1.0, controlLowerScaled[0], 1e-10);
            Assert.AreEqual(-1.0, controlLowerScaled[1], 1e-10);
            Assert.AreEqual(1.0, controlUpperScaled[0], 1e-10);
            Assert.AreEqual(1.0, controlUpperScaled[1], 1e-10);
        }
    }
}
