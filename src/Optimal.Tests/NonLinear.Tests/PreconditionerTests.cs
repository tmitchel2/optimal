/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Tests
{
    [TestClass]
    public sealed class PreconditionerTests
    {
        private static readonly double[] s_s1 = [1.0, 1.0];
        private static readonly double[] s_y1 = [2.0, 4.0];
        private static readonly double[] s_s2 = [1.0, 0.0];
        private static readonly double[] s_y2 = [2.0, 0.0];
        private static readonly double[] s_startPoint = [5.0, 5.0];
        private static readonly double[] s_initPoint = [1.0, 1.0];

        #region DiagonalPreconditioner Tests

        [TestMethod]
        public void DiagonalPreconditionerInitializesToIdentity()
        {
            var preconditioner = new DiagonalPreconditioner(3);
            var scaling = preconditioner.GetDiagonalScaling();

            Assert.HasCount(3, scaling);
            Assert.AreEqual(1.0, scaling[0]);
            Assert.AreEqual(1.0, scaling[1]);
            Assert.AreEqual(1.0, scaling[2]);
        }

        [TestMethod]
        public void DiagonalPreconditionerApplyToGradientWorks()
        {
            var preconditioner = new DiagonalPreconditioner(2);

            preconditioner.Update(s_s1, s_y1);

            var gradient = new double[] { 10.0, 20.0 };
            var result = preconditioner.ApplyToGradient(gradient);

            Assert.HasCount(2, result);
        }

        [TestMethod]
        public void DiagonalPreconditionerResetClearsUpdates()
        {
            var preconditioner = new DiagonalPreconditioner(2);

            preconditioner.Update(s_s1, s_y1);

            preconditioner.Reset();
            var scaling = preconditioner.GetDiagonalScaling();

            Assert.AreEqual(1.0, scaling[0]);
            Assert.AreEqual(1.0, scaling[1]);
        }

        [TestMethod]
        public void DiagonalPreconditionerEnforcesMinMaxBounds()
        {
            var preconditioner = new DiagonalPreconditioner(
                dimension: 2,
                minScaling: 0.1,
                maxScaling: 10.0);

            var s = new double[] { 1.0, 0.0001 };
            var y = new double[] { 1.0, 100.0 };
            preconditioner.Update(s, y);

            var scaling = preconditioner.GetDiagonalScaling();

            Assert.IsTrue(scaling[0] >= 0.1 && scaling[0] <= 10.0, $"Scaling[0] should be in bounds: {scaling[0]}");
            Assert.IsTrue(scaling[1] >= 0.1 && scaling[1] <= 10.0, $"Scaling[1] should be in bounds: {scaling[1]}");
        }

        [TestMethod]
        public void DiagonalPreconditionerThrowsOnInvalidDimension()
        {
            var exceptionThrown = false;
            try
            {
                _ = new DiagonalPreconditioner(0);
            }
            catch (ArgumentException)
            {
                exceptionThrown = true;
            }
            Assert.IsTrue(exceptionThrown, "Should throw ArgumentException for invalid dimension");
        }

        private static readonly double[] s_mismatchedGradient = [1.0, 2.0, 3.0];

        [TestMethod]
        public void DiagonalPreconditionerThrowsOnDimensionMismatch()
        {
            var exceptionThrown = false;
            try
            {
                var preconditioner = new DiagonalPreconditioner(2);
                preconditioner.ApplyToGradient(s_mismatchedGradient);
            }
            catch (ArgumentException)
            {
                exceptionThrown = true;
            }
            Assert.IsTrue(exceptionThrown, "Should throw ArgumentException for dimension mismatch");
        }

        #endregion

        #region RegularizationPreconditioner Tests

        [TestMethod]
        public void RegularizationPreconditionerInitializesWithRegularization()
        {
            var preconditioner = new RegularizationPreconditioner(3, 0.01);

            Assert.AreEqual(3, preconditioner.Dimension);
            Assert.AreEqual(0.01, preconditioner.RegularizationParameter);
            Assert.AreEqual(1.0, preconditioner.Gamma);
            Assert.AreEqual(1.01, preconditioner.RegularizedGamma);
        }

        [TestMethod]
        public void RegularizationPreconditionerUpdatesGamma()
        {
            var preconditioner = new RegularizationPreconditioner(2, 0.001);

            preconditioner.Update(s_s2, s_y2);

            // gamma = (s^T * y) / (y^T * y) = 2 / 4 = 0.5
            Assert.AreEqual(0.5, preconditioner.Gamma, 1e-10);
            Assert.AreEqual(0.501, preconditioner.RegularizedGamma, 1e-10);
        }

        [TestMethod]
        public void RegularizationPreconditionerApplyScalesUniformly()
        {
            var preconditioner = new RegularizationPreconditioner(2, 0.1);
            var gradient = new double[] { 1.0, 2.0 };

            var result = preconditioner.ApplyToGradient(gradient);

            // Should scale by (gamma + lambda) = (1.0 + 0.1) = 1.1
            Assert.AreEqual(1.1, result[0], 1e-10);
            Assert.AreEqual(2.2, result[1], 1e-10);
        }

        [TestMethod]
        public void RegularizationPreconditionerResetRestoresDefaults()
        {
            var preconditioner = new RegularizationPreconditioner(2, 0.01);

            preconditioner.Update(s_s2, s_y2);

            preconditioner.Reset();

            Assert.AreEqual(1.0, preconditioner.Gamma);
        }

        [TestMethod]
        public void RegularizationPreconditionerThrowsOnNegativeParameter()
        {
            var exceptionThrown = false;
            try
            {
                _ = new RegularizationPreconditioner(2, -0.1);
            }
            catch (ArgumentException)
            {
                exceptionThrown = true;
            }
            Assert.IsTrue(exceptionThrown, "Should throw ArgumentException for negative parameter");
        }

        #endregion

        #region CombinedPreconditioner Tests

        [TestMethod]
        public void CombinedPreconditionerCombinesDiagonalAndRegularization()
        {
            var diagonal = new DiagonalPreconditioner(2);
            var combined = new CombinedPreconditioner(diagonal, 0.1);

            Assert.AreEqual(2, combined.Dimension);
            Assert.AreEqual(0.1, combined.RegularizationParameter);
        }

        [TestMethod]
        public void CombinedPreconditionerApplyAddsDiagonalAndLambda()
        {
            var diagonal = new DiagonalPreconditioner(2);
            var combined = new CombinedPreconditioner(diagonal, 0.5);

            // Initially diagonal = 1.0, so combined = 1.0 + 0.5 = 1.5
            var gradient = new double[] { 3.0, 6.0 };
            var result = combined.ApplyToGradient(gradient);

            // Result = gradient[i] / (D[i] + lambda) = gradient[i] / 1.5
            Assert.AreEqual(2.0, result[0], 1e-10);
            Assert.AreEqual(4.0, result[1], 1e-10);
        }

        [TestMethod]
        public void CombinedPreconditionerDelegatesUpdate()
        {
            var diagonal = new DiagonalPreconditioner(2);
            var combined = new CombinedPreconditioner(diagonal, 0.1);

            var s = new double[] { 1.0, 1.0 };
            var y = new double[] { 2.0, 2.0 };

            combined.Update(s, y);

            var scaling = combined.GetDiagonalScaling();
            Assert.IsGreaterThan(0.1, scaling[0], "Scaling should be greater than regularization alone");
            Assert.IsGreaterThan(0.1, scaling[1], "Scaling should be greater than regularization alone");
        }

        #endregion

        #region Integration Tests with L-BFGS

        [TestMethod]
        public void LBFGSWithDiagonalPreconditioningConvergesOnWellConditioned()
        {
            static (double value, double[] gradient) Objective(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            var options = new LBFGSOptions
            {
                MaxIterations = 100,
                Tolerance = 1e-8,
                Preconditioning = LBFGSPreconditioningOptions.DiagonalScaling()
            };

            var optimizer = new LBFGSOptimizer(options, new BacktrackingLineSearch());
            var result = optimizer.Minimize(Objective, s_startPoint);

            Assert.IsTrue(result.Success, $"Optimization should succeed: {result.Message}");
            Assert.AreEqual(0.0, result.OptimalPoint[0], 1e-6);
            Assert.AreEqual(0.0, result.OptimalPoint[1], 1e-6);
        }

        [TestMethod]
        public void LBFGSWithRegularizationConvergesOnWellConditioned()
        {
            static (double value, double[] gradient) Objective(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            var options = new LBFGSOptions
            {
                MaxIterations = 100,
                Tolerance = 1e-8,
                Preconditioning = LBFGSPreconditioningOptions.Regularized(1e-4)
            };

            var optimizer = new LBFGSOptimizer(options, new BacktrackingLineSearch());
            var result = optimizer.Minimize(Objective, s_startPoint);

            Assert.IsTrue(result.Success, $"Optimization should succeed: {result.Message}");
            Assert.AreEqual(0.0, result.OptimalPoint[0], 1e-6);
            Assert.AreEqual(0.0, result.OptimalPoint[1], 1e-6);
        }

        [TestMethod]
        public void LBFGSWithPreconditioningHandlesIllConditioned()
        {
            static (double value, double[] gradient) IllConditionedObjective(double[] x)
            {
                var value = x[0] * x[0] + 1000 * x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2000 * x[1] };
                return (value, gradient);
            }

            var optionsWithPreconditioning = new LBFGSOptions
            {
                MaxIterations = 200,
                Tolerance = 1e-6,
                Preconditioning = LBFGSPreconditioningOptions.DiagonalScaling()
            };

            var optionsWithoutPreconditioning = new LBFGSOptions
            {
                MaxIterations = 200,
                Tolerance = 1e-6,
                Preconditioning = null
            };

            var optimizerWith = new LBFGSOptimizer(optionsWithPreconditioning, new BacktrackingLineSearch());
            var optimizerWithout = new LBFGSOptimizer(optionsWithoutPreconditioning, new BacktrackingLineSearch());

            var resultWith = optimizerWith.Minimize(IllConditionedObjective, s_initPoint);
            var resultWithout = optimizerWithout.Minimize(IllConditionedObjective, s_initPoint);

            Assert.IsTrue(resultWith.Success, $"Preconditioned should succeed: {resultWith.Message}");
            Assert.IsTrue(resultWithout.Success, $"Non-preconditioned should succeed: {resultWithout.Message}");

            Assert.AreEqual(0.0, resultWith.OptimalPoint[0], 1e-4);
            Assert.AreEqual(0.0, resultWith.OptimalPoint[1], 1e-4);
        }

        [TestMethod]
        public void LBFGSOptionsPreconditioningOptionFactories()
        {
            var defaultOpts = LBFGSPreconditioningOptions.Default;
            Assert.AreEqual(PreconditioningType.None, defaultOpts.Type);

            var diagonalOpts = LBFGSPreconditioningOptions.DiagonalScaling();
            Assert.AreEqual(PreconditioningType.Diagonal, diagonalOpts.Type);

            var regularizedOpts = LBFGSPreconditioningOptions.Regularized(1e-3);
            Assert.AreEqual(PreconditioningType.Regularization, regularizedOpts.Type);
            Assert.AreEqual(1e-3, regularizedOpts.RegularizationParameter);

            var automaticOpts = LBFGSPreconditioningOptions.Automatic(1e5);
            Assert.IsTrue(automaticOpts.EnableAutomaticPreconditioning);
            Assert.AreEqual(1e5, automaticOpts.PreconditioningThreshold);
        }

        #endregion
    }
}
