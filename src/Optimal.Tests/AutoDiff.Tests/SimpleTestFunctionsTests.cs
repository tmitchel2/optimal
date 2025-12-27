/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.AutoDiff.Tests
{
    [TestClass]
    public class SimpleTestFunctionsTests
    {
        [TestMethod]
        public void SquareGradientIsCorrect()
        {
            var x = 3.0;
            var (value, gradient) = SimpleTestFunctionsGradients.SquareForward_x(x);

            Assert.AreEqual(9.0, value, 1e-10, "Value should be x^2 = 9");
            Assert.AreEqual(6.0, gradient, 1e-10, "Gradient should be 2*x = 6");
        }

        [TestMethod]
        public void SquareGradientMatchesFiniteDifference()
        {
            var x = 3.0;
            var epsilon = 1e-5;

            var (value, analyticalGradient) = SimpleTestFunctionsGradients.SquareForward_x(x);

            var fXPlusEps = SimpleTestFunctions.Square(x + epsilon);
            var fX = SimpleTestFunctions.Square(x);
            var numericalGradient = (fXPlusEps - fX) / epsilon;

            Assert.AreEqual(numericalGradient, analyticalGradient, 1e-4,
                "Analytical gradient should match numerical gradient");
        }

        [TestMethod]
        public void AddGradients()
        {
            var x = 2.0;
            var y = 3.0;

            var (value1, gradX) = SimpleTestFunctionsGradients.AddForward_x(x, y);
            Assert.AreEqual(5.0, value1, 1e-10);
            Assert.AreEqual(1.0, gradX, 1e-10, "d(x+y)/dx = 1");

            var (value2, gradY) = SimpleTestFunctionsGradients.AddForward_y(x, y);
            Assert.AreEqual(5.0, value2, 1e-10);
            Assert.AreEqual(1.0, gradY, 1e-10, "d(x+y)/dy = 1");
        }

        [TestMethod]
        public void MultiplyGradients()
        {
            var x = 3.0;
            var y = 4.0;

            var (value1, gradX) = SimpleTestFunctionsGradients.MultiplyForward_x(x, y);
            Assert.AreEqual(12.0, value1, 1e-10);
            Assert.AreEqual(4.0, gradX, 1e-10, "d(x*y)/dx = y = 4");

            var (value2, gradY) = SimpleTestFunctionsGradients.MultiplyForward_y(x, y);
            Assert.AreEqual(12.0, value2, 1e-10);
            Assert.AreEqual(3.0, gradY, 1e-10, "d(x*y)/dy = x = 3");
        }

        [TestMethod]
        public void KineticEnergyGradients()
        {
            var mass = 2.0;
            var velocity = 3.0;

            var (energy1, gradMass) = SimpleTestFunctionsGradients.KineticEnergyForward_mass(mass, velocity);
            Assert.AreEqual(9.0, energy1, 1e-10, "KE = 0.5 * 2 * 3^2 = 9");
            Assert.AreEqual(4.5, gradMass, 1e-10, "dKE/dm = 0.5 * v^2 = 0.5 * 9 = 4.5");

            var (energy2, gradVelocity) = SimpleTestFunctionsGradients.KineticEnergyForward_velocity(mass, velocity);
            Assert.AreEqual(9.0, energy2, 1e-10);
            Assert.AreEqual(6.0, gradVelocity, 1e-10, "dKE/dv = m * v = 2 * 3 = 6");
        }

        [TestMethod]
        public void KineticEnergyGradientsMatchFiniteDifference()
        {
            var mass = 2.0;
            var velocity = 3.0;
            var epsilon = 1e-5;

            var (_, gradMass) = SimpleTestFunctionsGradients.KineticEnergyForward_mass(mass, velocity);

            var keAtMass = SimpleTestFunctions.KineticEnergy(mass, velocity);
            var keAtMassPlusEps = SimpleTestFunctions.KineticEnergy(mass + epsilon, velocity);
            var numericalGradMass = (keAtMassPlusEps - keAtMass) / epsilon;

            Assert.AreEqual(numericalGradMass, gradMass, 1e-4,
                "Analytical gradient w.r.t. mass should match numerical gradient");

            var (_, gradVelocity) = SimpleTestFunctionsGradients.KineticEnergyForward_velocity(mass, velocity);

            var keAtVelocity = SimpleTestFunctions.KineticEnergy(mass, velocity);
            var keAtVelocityPlusEps = SimpleTestFunctions.KineticEnergy(mass, velocity + epsilon);
            var numericalGradVelocity = (keAtVelocityPlusEps - keAtVelocity) / epsilon;

            Assert.AreEqual(numericalGradVelocity, gradVelocity, 1e-4,
                "Analytical gradient w.r.t. velocity should match numerical gradient");
        }
    }
}
