/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Tests
{
    [TestClass]
    public class SimpleBrachistochroneTests
    {
        private const double Epsilon = 1e-5;
        private const double Tolerance = 1e-4;

        [TestMethod]
        public void CanCalculateDescentTime()
        {
            var radius = 100.0;
            var startHeight = 100.0;
            var endX = 100.0;
            var segments = 100;

            var time = SimpleBrachistochrone.GetDescentTime(radius, startHeight, endX, segments);

            Assert.IsTrue(time > 0, "Descent time should be positive");
            Assert.IsTrue(time < 10.0, "Descent time should be reasonable (less than 10 seconds for 100m drop)");
        }

        [TestMethod]
        public void LargerRadiusGivesDifferentTime()
        {
            var startHeight = 100.0;
            var endX = 100.0;
            var segments = 50;

            var time1 = SimpleBrachistochrone.GetDescentTime(100.0, startHeight, endX, segments);
            var time2 = SimpleBrachistochrone.GetDescentTime(150.0, startHeight, endX, segments);

            Assert.AreNotEqual(time1, time2, 1e-6, "Different radii should produce different times");
        }

        [TestMethod]
        public void GradientWithRespectToRadius()
        {
            var radius = 120.0;
            var startHeight = 100.0;
            var endX = 100.0;
            var segments = 50;

            var (value, gradient) = SimpleBrachistochroneGradients.GetDescentTimeForward_radius(radius, startHeight, endX, segments);

            // Validate against finite difference
            var fRadius = SimpleBrachistochrone.GetDescentTime(radius, startHeight, endX, segments);
            var fRadiusPlusEps = SimpleBrachistochrone.GetDescentTime(radius + Epsilon, startHeight, endX, segments);

            var numericalGradient = (fRadiusPlusEps - fRadius) / Epsilon;

            Assert.AreEqual(fRadius, value, 1e-10, "Forward mode should compute correct value");
            Assert.AreEqual(numericalGradient, gradient, Tolerance,
                $"Analytical gradient ({gradient}) should match numerical gradient ({numericalGradient})");
        }

        [TestMethod]
        public void GradientWithRespectToStartHeight()
        {
            var radius = 120.0;
            var startHeight = 100.0;
            var endX = 100.0;
            var segments = 50;

            var (value, gradient) = SimpleBrachistochroneGradients.GetDescentTimeForward_startHeight(radius, startHeight, endX, segments);

            // Validate against finite difference
            var fHeight = SimpleBrachistochrone.GetDescentTime(radius, startHeight, endX, segments);
            var fHeightPlusEps = SimpleBrachistochrone.GetDescentTime(radius, startHeight + Epsilon, endX, segments);

            var numericalGradient = (fHeightPlusEps - fHeight) / Epsilon;

            Assert.AreEqual(fHeight, value, 1e-10, "Forward mode should compute correct value");
            Assert.AreEqual(numericalGradient, gradient, Tolerance,
                $"Analytical gradient ({gradient}) should match numerical gradient ({numericalGradient})");
        }

        [TestMethod]
        public void ReverseMode()
        {
            var radius = 120.0;
            var startHeight = 100.0;
            var endX = 100.0;
            var segments = 50;

            var (value, gradients) = SimpleBrachistochroneGradients.GetDescentTimeReverse(radius, startHeight, endX, segments);

            Assert.AreEqual(3, gradients.Length, "Should have 3 gradients (radius, startHeight, endX)");

            // Validate against forward mode
            var (valueForwardR, gradientR) = SimpleBrachistochroneGradients.GetDescentTimeForward_radius(radius, startHeight, endX, segments);
            var (valueForwardH, gradientH) = SimpleBrachistochroneGradients.GetDescentTimeForward_startHeight(radius, startHeight, endX, segments);
            var (valueForwardX, gradientX) = SimpleBrachistochroneGradients.GetDescentTimeForward_endX(radius, startHeight, endX, segments);

            Assert.AreEqual(valueForwardR, value, 1e-10, "Reverse mode value should match forward mode");
            Assert.AreEqual(gradientR, gradients[0], 1e-10, "Gradient w.r.t. radius should match");
            Assert.AreEqual(gradientH, gradients[1], 1e-10, "Gradient w.r.t. startHeight should match");
            Assert.AreEqual(gradientX, gradients[2], 1e-10, "Gradient w.r.t. endX should match");
        }
    }
}
