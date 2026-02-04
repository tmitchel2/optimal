/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Control.Collocation.Tests
{
    [TestClass]
    public sealed class TrajectoryInterpolatorTests
    {
        [TestMethod]
        public void InterpolateStateAtNodePointsReturnsExactValues()
        {
            // Arrange: Create a simple trajectory with known values
            var times = new[] { 0.0, 1.0, 2.0 };
            var states = new[]
            {
                new[] { 0.0, 0.0 },
                new[] { 1.0, 2.0 },
                new[] { 3.0, 5.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0, 2.0 },  // f at t=0
                new[] { 2.0, 3.0 },  // f at t=1
                new[] { 3.0, 4.0 }   // f at t=2
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Act & Assert: At node points, should return exact values
            var s0 = interpolator.InterpolateState(0.0);
            var s1 = interpolator.InterpolateState(1.0);
            var s2 = interpolator.InterpolateState(2.0);

            Assert.AreEqual(states[0][0], s0[0], 1e-10);
            Assert.AreEqual(states[0][1], s0[1], 1e-10);
            Assert.AreEqual(states[1][0], s1[0], 1e-10);
            Assert.AreEqual(states[1][1], s1[1], 1e-10);
            Assert.AreEqual(states[2][0], s2[0], 1e-10);
            Assert.AreEqual(states[2][1], s2[1], 1e-10);
        }

        [TestMethod]
        public void InterpolateStateAtMidpointMatchesHermiteFormula()
        {
            // Arrange: Create a trajectory with known values
            var times = new[] { 0.0, 1.0 };
            var states = new[]
            {
                new[] { 0.0 },
                new[] { 1.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0 },  // f at t=0
                new[] { 1.0 }   // f at t=1
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Act
            var sMid = interpolator.InterpolateState(0.5);

            // Assert: For this linear case (constant derivative), midpoint should be 0.5
            // Hermite basis at tau=0.5:
            // H00 = 2*(0.5)^3 - 3*(0.5)^2 + 1 = 0.25 - 0.75 + 1 = 0.5
            // H10 = (0.5)^3 - 2*(0.5)^2 + 0.5 = 0.125 - 0.5 + 0.5 = 0.125
            // H01 = -2*(0.5)^3 + 3*(0.5)^2 = -0.25 + 0.75 = 0.5
            // H11 = (0.5)^3 - (0.5)^2 = 0.125 - 0.25 = -0.125
            // x(0.5) = 0.5*0 + 0.125*1*1 + 0.5*1 + (-0.125)*1*1 = 0.125 + 0.5 - 0.125 = 0.5
            Assert.AreEqual(0.5, sMid[0], 1e-10);
        }

        [TestMethod]
        public void InterpolateStateBelowRangeThrowsException()
        {
            // Arrange
            var times = new[] { 0.0, 1.0, 2.0 };
            var states = new[]
            {
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 2.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0 },
                new[] { 1.0 },
                new[] { 1.0 }
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Act & Assert
            Assert.Throws<ArgumentOutOfRangeException>(() => interpolator.InterpolateState(-0.5));
        }

        [TestMethod]
        public void InterpolateStateAboveRangeThrowsException()
        {
            // Arrange
            var times = new[] { 0.0, 1.0, 2.0 };
            var states = new[]
            {
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 2.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0 },
                new[] { 1.0 },
                new[] { 1.0 }
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Act & Assert
            Assert.Throws<ArgumentOutOfRangeException>(() => interpolator.InterpolateState(2.5));
        }

        [TestMethod]
        public void GetInterpolatedPointsReturnsCorrectCount()
        {
            // Arrange
            var times = new[] { 0.0, 1.0, 2.0 };
            var states = new[]
            {
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 2.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0 },
                new[] { 1.0 },
                new[] { 1.0 }
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Act
            var pointsPerSegment = 5;
            var points = interpolator.GetInterpolatedPoints(pointsPerSegment).ToArray();

            // Assert: 2 segments * 5 points + 1 final point = 11 points
            Assert.HasCount(11, points);
        }

        [TestMethod]
        public void FromResultWithNullDerivativesThrowsException()
        {
            // Arrange
            var result = new CollocationResult
            {
                Success = true,
                Times = new[] { 0.0, 1.0 },
                States = new[] { new[] { 0.0 }, new[] { 1.0 } },
                StateDerivatives = null
            };

            // Act & Assert
            Assert.Throws<ArgumentException>(() => TrajectoryInterpolator.FromResult(result));
        }

        [TestMethod]
        public void SegmentsReturnsCorrectCount()
        {
            // Arrange
            var times = new[] { 0.0, 1.0, 2.0, 3.0 };
            var states = new[]
            {
                new[] { 0.0 },
                new[] { 1.0 },
                new[] { 2.0 },
                new[] { 3.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0 },
                new[] { 1.0 },
                new[] { 1.0 },
                new[] { 1.0 }
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Assert
            Assert.AreEqual(3, interpolator.Segments);
        }

        [TestMethod]
        public void StateDimReturnsCorrectDimension()
        {
            // Arrange
            var times = new[] { 0.0, 1.0 };
            var states = new[]
            {
                new[] { 0.0, 1.0, 2.0 },
                new[] { 1.0, 2.0, 3.0 }
            };
            var derivatives = new[]
            {
                new[] { 1.0, 1.0, 1.0 },
                new[] { 1.0, 1.0, 1.0 }
            };

            var result = new CollocationResult
            {
                Success = true,
                Times = times,
                States = states,
                StateDerivatives = derivatives
            };

            var interpolator = TrajectoryInterpolator.FromResult(result);

            // Assert
            Assert.AreEqual(3, interpolator.StateDim);
        }
    }
}
