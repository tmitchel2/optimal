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
    public class ConstantFunctionsTests
    {
        [TestMethod]
        public void OneGradientIsZero()
        {
            var x = 5.0;
            var (value, gradient) = ConstantFunctionsGradients.OneForward_x(x);

            Assert.AreEqual(1.0, value, 1e-10);
            Assert.AreEqual(0.0, gradient, 1e-10);
        }

        [TestMethod]
        public void ZeroGradientIsZero()
        {
            var x = 5.0;
            var (value, gradient) = ConstantFunctionsGradients.ZeroForward_x(x);

            Assert.AreEqual(0.0, value, 1e-10);
            Assert.AreEqual(0.0, gradient, 1e-10);
        }

        [TestMethod]
        public void ConstantWithMultipleParamsAllGradientsZero()
        {
            var x = 1.0;
            var y = 2.0;
            var z = 3.0;

            var (value, gradients) = ConstantFunctionsGradients.ConstantWithMultipleParamsReverse(x, y, z);

            Assert.AreEqual(1.0, value, 1e-10);
            Assert.AreEqual(0.0, gradients[0], 1e-10); // ∂/∂x
            Assert.AreEqual(0.0, gradients[1], 1e-10); // ∂/∂y
            Assert.AreEqual(0.0, gradients[2], 1e-10); // ∂/∂z
        }

        [TestMethod]
        public void ComputedConstantGradientIsZero()
        {
            var x = 5.0;
            var (value, gradient) = ConstantFunctionsGradients.ComputedConstantForward_x(x);

            Assert.AreEqual(2.0 * 3.14159, value, 1e-10);
            Assert.AreEqual(0.0, gradient, 1e-10);
        }
    }
}
