/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Tests
{
    [TestClass]
    public sealed class ProjectedGradientTests
    {
        private static readonly double[] s_lower = [-1.0, -2.0, 0.0];
        private static readonly double[] s_upper = [1.0, 2.0, 5.0];

        [TestMethod]
        public void ProjectPointInsideBoundsReturnsUnchanged()
        {
            var x = new[] { 0.0, 0.0, 2.5 };

            var projected = ProjectedGradient.Project(x, s_lower, s_upper);

            Assert.AreEqual(0.0, projected[0], 1e-10);
            Assert.AreEqual(0.0, projected[1], 1e-10);
            Assert.AreEqual(2.5, projected[2], 1e-10);
        }

        [TestMethod]
        public void ProjectPointBelowLowerClampsToLower()
        {
            var x = new[] { -5.0, -10.0, -1.0 };

            var projected = ProjectedGradient.Project(x, s_lower, s_upper);

            Assert.AreEqual(-1.0, projected[0], 1e-10, "Should clamp to lower bound");
            Assert.AreEqual(-2.0, projected[1], 1e-10, "Should clamp to lower bound");
            Assert.AreEqual(0.0, projected[2], 1e-10, "Should clamp to lower bound");
        }

        [TestMethod]
        public void ProjectPointAboveUpperClampsToUpper()
        {
            var x = new[] { 5.0, 10.0, 100.0 };

            var projected = ProjectedGradient.Project(x, s_lower, s_upper);

            Assert.AreEqual(1.0, projected[0], 1e-10, "Should clamp to upper bound");
            Assert.AreEqual(2.0, projected[1], 1e-10, "Should clamp to upper bound");
            Assert.AreEqual(5.0, projected[2], 1e-10, "Should clamp to upper bound");
        }

        [TestMethod]
        public void ProjectInPlaceModifiesArray()
        {
            var x = new[] { -5.0, 10.0, 2.5 };

            ProjectedGradient.ProjectInPlace(x, s_lower, s_upper);

            Assert.AreEqual(-1.0, x[0], 1e-10, "Should clamp to lower bound in place");
            Assert.AreEqual(2.0, x[1], 1e-10, "Should clamp to upper bound in place");
            Assert.AreEqual(2.5, x[2], 1e-10, "Should not change interior point");
        }

        [TestMethod]
        public void ComputeProjectedGradientAtInteriorPointReturnsNegativeGradient()
        {
            var x = new[] { 0.0, 0.0, 2.5 };
            var gradient = new[] { 1.0, -1.0, 0.5 };

            var pg = ProjectedGradient.ComputeProjectedGradient(x, gradient, s_lower, s_upper);

            // At interior point, pg = project(x - g) - x = (x - g) - x = -g
            Assert.AreEqual(-1.0, pg[0], 1e-10);
            Assert.AreEqual(1.0, pg[1], 1e-10);
            Assert.AreEqual(-0.5, pg[2], 1e-10);
        }

        [TestMethod]
        public void ComputeProjectedGradientAtLowerBoundWithPositiveGradientReturnsZero()
        {
            // At lower bound with positive gradient (would push below bound)
            var x = new[] { -1.0, 0.0, 0.0 };
            var gradient = new[] { 1.0, 0.0, 0.0 }; // Positive gradient at lower bound

            var pg = ProjectedGradient.ComputeProjectedGradient(x, gradient, s_lower, s_upper);

            // x - g = -1 - 1 = -2, project to -1, so pg = -1 - (-1) = 0
            Assert.AreEqual(0.0, pg[0], 1e-10, "PG should be zero when at bound and gradient pushes outward");
        }

        [TestMethod]
        public void ComputeProjectedGradientAtUpperBoundWithNegativeGradientReturnsZero()
        {
            // At upper bound with negative gradient (would push above bound)
            var x = new[] { 1.0, 0.0, 5.0 };
            var gradient = new[] { -1.0, 0.0, -1.0 }; // Negative gradient at upper bound

            var pg = ProjectedGradient.ComputeProjectedGradient(x, gradient, s_lower, s_upper);

            // x - g = 1 - (-1) = 2, project to 1, so pg = 1 - 1 = 0
            Assert.AreEqual(0.0, pg[0], 1e-10, "PG should be zero when at bound and gradient pushes outward");
            Assert.AreEqual(0.0, pg[2], 1e-10, "PG should be zero when at bound and gradient pushes outward");
        }

        [TestMethod]
        public void ProjectedGradientNormInfReturnsMaxAbsComponent()
        {
            var x = new[] { 0.0, 0.0, 2.5 };
            var gradient = new[] { 0.1, -0.5, 0.3 };

            var norm = ProjectedGradient.ProjectedGradientNormInf(x, gradient, s_lower, s_upper);

            // PG = -gradient for interior, so max |pg| = max(0.1, 0.5, 0.3) = 0.5
            Assert.AreEqual(0.5, norm, 1e-10);
        }

        [TestMethod]
        public void ProjectedGradientNormInfAtOptimumIsZero()
        {
            // At optimum of bounded problem, projected gradient should be zero
            // x at bound with gradient pointing outward
            var x = new[] { -1.0, 2.0, 0.0 }; // All at bounds
            var gradient = new[] { 1.0, -1.0, 1.0 }; // All pointing outward

            var norm = ProjectedGradient.ProjectedGradientNormInf(x, gradient, s_lower, s_upper);

            Assert.AreEqual(0.0, norm, 1e-10, "Projected gradient norm should be zero at constrained optimum");
        }

        [TestMethod]
        public void ProjectedGradientNorm2ComputesCorrectly()
        {
            var x = new[] { 0.0, 0.0 };
            var gradient = new[] { 3.0, 4.0 };
            var lower = new[] { -10.0, -10.0 };
            var upper = new[] { 10.0, 10.0 };

            var norm = ProjectedGradient.ProjectedGradientNorm2(x, gradient, lower, upper);

            // PG = -gradient = (-3, -4), ||pg|| = 5
            Assert.AreEqual(5.0, norm, 1e-10);
        }

        [TestMethod]
        public void MaxFeasibleStepDirectionTowardsUpperLimitedByUpper()
        {
            var x = new[] { 0.0, 0.0, 2.5 };
            var direction = new[] { 1.0, 1.0, 1.0 }; // Moving in positive direction

            var maxStep = ProjectedGradient.MaxFeasibleStep(x, direction, s_lower, s_upper);

            // Distance to upper bounds: (1-0)/1=1, (2-0)/1=2, (5-2.5)/1=2.5
            // Minimum is 1.0
            Assert.AreEqual(1.0, maxStep, 1e-10);
        }

        [TestMethod]
        public void MaxFeasibleStepDirectionTowardsLowerLimitedByLower()
        {
            var x = new[] { 0.0, 0.0, 2.5 };
            var direction = new[] { -1.0, -1.0, -1.0 }; // Moving in negative direction

            var maxStep = ProjectedGradient.MaxFeasibleStep(x, direction, s_lower, s_upper);

            // Distance to lower bounds: (-1-0)/(-1)=1, (-2-0)/(-1)=2, (0-2.5)/(-1)=2.5
            // Minimum is 1.0
            Assert.AreEqual(1.0, maxStep, 1e-10);
        }

        [TestMethod]
        public void MaxFeasibleStepZeroDirectionReturnsInfinity()
        {
            var x = new[] { 0.0 };
            var direction = new[] { 0.0 };
            var lower = new[] { -1.0 };
            var upper = new[] { 1.0 };

            var maxStep = ProjectedGradient.MaxFeasibleStep(x, direction, lower, upper);

            Assert.IsTrue(double.IsPositiveInfinity(maxStep), "Zero direction should allow infinite step");
        }

        [TestMethod]
        public void MaxFeasibleStepMixedDirectionsReturnsMinimum()
        {
            var x = new[] { 0.0, 0.0 };
            var direction = new[] { 2.0, -0.5 }; // Different scales
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var maxStep = ProjectedGradient.MaxFeasibleStep(x, direction, lower, upper);

            // Toward upper: (1-0)/2 = 0.5
            // Toward lower: (-1-0)/(-0.5) = 2
            // Minimum is 0.5
            Assert.AreEqual(0.5, maxStep, 1e-10);
        }

        [TestMethod]
        public void IdentifyActiveSetAtLowerWithPositiveGradientReturnsMinusOne()
        {
            var x = new[] { -1.0, 0.0, 0.0 };
            var gradient = new[] { 1.0, 0.0, 0.0 }; // Would push x lower

            var activeSet = ProjectedGradient.IdentifyActiveSet(x, gradient, s_lower, s_upper);

            Assert.AreEqual(-1, activeSet[0], "Should be active at lower bound");
            Assert.AreEqual(0, activeSet[1], "Should be free");
            Assert.AreEqual(0, activeSet[2], "Should be free");
        }

        [TestMethod]
        public void IdentifyActiveSetAtUpperWithNegativeGradientReturnsPlusOne()
        {
            var x = new[] { 1.0, 0.0, 5.0 };
            var gradient = new[] { -1.0, 0.0, -1.0 }; // Would push x higher

            var activeSet = ProjectedGradient.IdentifyActiveSet(x, gradient, s_lower, s_upper);

            Assert.AreEqual(1, activeSet[0], "Should be active at upper bound");
            Assert.AreEqual(0, activeSet[1], "Should be free");
            Assert.AreEqual(1, activeSet[2], "Should be active at upper bound");
        }

        [TestMethod]
        public void IdentifyActiveSetAtBoundWithGradientAllowingMovementReturnsFree()
        {
            // At lower bound but gradient would push x UP (into interior)
            var x = new[] { -1.0, 0.0 };
            var gradient = new[] { -1.0, 0.0 }; // Negative gradient = steepest descent goes positive
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var activeSet = ProjectedGradient.IdentifyActiveSet(x, gradient, lower, upper);

            // Gradient is negative, so -gradient direction is positive (into interior)
            Assert.AreEqual(0, activeSet[0], "Should be free since gradient allows movement into interior");
        }

        [TestMethod]
        public void ZeroActiveComponentsZeroesComponentsAtActiveBounds()
        {
            var direction = new[] { -1.0, 0.5, 2.0 };
            var x = new[] { -1.0, 0.0, 5.0 }; // At lower, interior, at upper

            var constrained = ProjectedGradient.ZeroActiveComponents(direction, x, s_lower, s_upper);

            // x[0]=-1 at lower, direction[0]=-1 < 0 would push further down -> zero
            Assert.AreEqual(0.0, constrained[0], 1e-10, "Should zero direction at lower bound when pushing down");
            Assert.AreEqual(0.5, constrained[1], 1e-10, "Should not change interior direction");
            // x[2]=5 at upper, direction[2]=2 > 0 would push further up -> zero
            Assert.AreEqual(0.0, constrained[2], 1e-10, "Should zero direction at upper bound when pushing up");
        }

        [TestMethod]
        public void ZeroActiveComponentsAllowsMovementIntoInterior()
        {
            var direction = new[] { 1.0, -1.0 }; // Moving away from bounds
            var x = new[] { -1.0, 1.0 }; // At lower, at upper
            var lower = new[] { -1.0, -1.0 };
            var upper = new[] { 1.0, 1.0 };

            var constrained = ProjectedGradient.ZeroActiveComponents(direction, x, lower, upper);

            // x[0]=-1 at lower, direction[0]=1 > 0 moves into interior -> keep
            Assert.AreEqual(1.0, constrained[0], 1e-10, "Should allow movement into interior from lower bound");
            // x[1]=1 at upper, direction[1]=-1 < 0 moves into interior -> keep
            Assert.AreEqual(-1.0, constrained[1], 1e-10, "Should allow movement into interior from upper bound");
        }

        [TestMethod]
        public void ProjectHandlesInfinityBounds()
        {
            var x = new[] { 1000.0, -1000.0 };
            var lower = new[] { double.NegativeInfinity, 0.0 };
            var upper = new[] { 0.0, double.PositiveInfinity };

            var projected = ProjectedGradient.Project(x, lower, upper);

            Assert.AreEqual(0.0, projected[0], 1e-10, "Should clamp to upper bound");
            Assert.AreEqual(0.0, projected[1], 1e-10, "Should clamp to lower bound");
        }

        [TestMethod]
        public void MaxFeasibleStepWithInfinityBoundsReturnsFiniteOrInfinity()
        {
            var x = new[] { 0.0, 0.0 };
            var direction = new[] { 1.0, -1.0 };
            var lower = new[] { double.NegativeInfinity, -1.0 };
            var upper = new[] { 1.0, double.PositiveInfinity };

            var maxStep = ProjectedGradient.MaxFeasibleStep(x, direction, lower, upper);

            // Toward upper[0]=1: (1-0)/1 = 1
            // Toward lower[1]=-1: (-1-0)/(-1) = 1
            Assert.AreEqual(1.0, maxStep, 1e-10);
        }
    }
}
