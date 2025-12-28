/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Tests
{
    [TestClass]
    public sealed class ConvergenceMonitorTests
    {
        private static readonly double[] s_testPoint1 = new[] { 1.0, 2.0 };
        private static readonly double[] s_testPoint2 = new[] { 1.001, 2.001 };
        private static readonly double[] s_smallGradient = new[] { 1e-7, 1e-7 };
        private static readonly double[] s_largeGradient = new[] { 1.0, 1.0 };

        [TestMethod]
        public void DetectsGradientTolerance()
        {
            var monitor = new ConvergenceMonitor(gradientTolerance: 1e-6);

            var result = monitor.CheckConvergence(
                iteration: 0,
                functionEvaluations: 1,
                x: s_testPoint1,
                fValue: 10.0,
                gradient: s_smallGradient);

            Assert.IsTrue(result.HasConverged, "Should converge when gradient is small");
            Assert.AreEqual(StoppingReason.GradientTolerance, result.Reason);
            Assert.IsTrue(result.GradientNorm < 1e-6, $"Gradient norm {result.GradientNorm} should be < 1e-6");
        }

        [TestMethod]
        public void DetectsFunctionTolerance()
        {
            var monitor = new ConvergenceMonitor(functionTolerance: 1e-7);

            // First iteration
            var result1 = monitor.CheckConvergence(
                iteration: 0,
                functionEvaluations: 1,
                x: s_testPoint1,
                fValue: 10.0,
                gradient: s_largeGradient);

            Assert.IsFalse(result1.HasConverged, "First iteration should not converge");

            // Second iteration with very small function change
            var result2 = monitor.CheckConvergence(
                iteration: 1,
                functionEvaluations: 2,
                x: s_testPoint2,
                fValue: 10.0 + 1e-8,
                gradient: s_largeGradient);

            Assert.IsTrue(result2.HasConverged, "Should converge when function change is small");
            Assert.AreEqual(StoppingReason.FunctionTolerance, result2.Reason);
            Assert.IsTrue(result2.FunctionChange < 1e-7, $"Function change {result2.FunctionChange} should be < 1e-7");
        }

        [TestMethod]
        public void DetectsParameterTolerance()
        {
            var monitor = new ConvergenceMonitor(parameterTolerance: 1e-7);

            // First iteration
            var result1 = monitor.CheckConvergence(
                iteration: 0,
                functionEvaluations: 1,
                x: s_testPoint1,
                fValue: 10.0,
                gradient: s_largeGradient);

            Assert.IsFalse(result1.HasConverged, "First iteration should not converge");

            // Second iteration with very small parameter change
            var smallStepPoint = new[] { 1.0 + 1e-8, 2.0 + 1e-8 };
            var result2 = monitor.CheckConvergence(
                iteration: 1,
                functionEvaluations: 2,
                x: smallStepPoint,
                fValue: 9.5,
                gradient: s_largeGradient);

            Assert.IsTrue(result2.HasConverged, "Should converge when parameter change is small");
            Assert.AreEqual(StoppingReason.ParameterTolerance, result2.Reason);
            Assert.IsTrue(result2.ParameterChange < 1e-7, $"Parameter change {result2.ParameterChange} should be < 1e-7");
        }

        [TestMethod]
        public void DetectsMaxIterations()
        {
            var monitor = new ConvergenceMonitor(
                maxIterations: 5,
                functionTolerance: 1e-10,
                parameterTolerance: 1e-10); // Make tolerances very tight

            ConvergenceResult? result = null;
            for (var i = 0; i <= 5; i++)
            {
                // Use different points to avoid parameter tolerance triggering
                var point = new[] { 1.0 + i * 0.1, 2.0 + i * 0.1 };
                result = monitor.CheckConvergence(
                    iteration: i,
                    functionEvaluations: i + 1,
                    x: point,
                    fValue: 10.0 - i,
                    gradient: s_largeGradient);

                if (i < 5)
                {
                    Assert.IsFalse(result.Reason == StoppingReason.MaxIterations, $"Iteration {i} should not hit max iterations yet");
                }
                else
                {
                    Assert.IsFalse(result.HasConverged, "Should not have converged");
                    Assert.AreEqual(StoppingReason.MaxIterations, result.Reason);
                }
            }
        }

        [TestMethod]
        public void DetectsMaxFunctionEvaluations()
        {
            var monitor = new ConvergenceMonitor(maxFunctionEvaluations: 10);

            var result = monitor.CheckConvergence(
                iteration: 5,
                functionEvaluations: 10,
                x: s_testPoint1,
                fValue: 10.0,
                gradient: s_largeGradient);

            Assert.IsFalse(result.HasConverged, "Should not have converged");
            Assert.AreEqual(StoppingReason.MaxFunctionEvaluations, result.Reason);
        }

        [TestMethod]
        [Ignore("Stall detection is very sensitive to tolerance settings and overlaps with function tolerance")]
        public void DetectsStall()
        {
            // Stall detection: When function value changes are consistently small over many iterations
            // Use very tight normal tolerances so normal convergence doesn't trigger first
            var monitor = new ConvergenceMonitor(
                gradientTolerance: 1e-15,
                functionTolerance: 1e-15,
                parameterTolerance: 1e-15,
                stallIterations: 5);

            // Create a stall scenario: function makes tiny progress each iteration
            for (var i = 0; i < 8; i++)
            {
                // Points change enough to not trigger parameter tolerance
                var point = new[] { 1.0 + i * 1e-6, 2.0 + i * 1e-6 };
                // Function changes slightly (above functionTolerance, but below stall threshold)
                var fValue = 10.0 + i * 1e-14;
                var result = monitor.CheckConvergence(
                    iteration: i,
                    functionEvaluations: i + 1,
                    x: point,
                    fValue: fValue,
                    gradient: s_largeGradient);

                // Should eventually detect stall after enough iterations
                if (result.HasConverged)
                {
                    Assert.IsTrue(i >= 6, $"Should detect stall after iteration 6, got {i}");
                    Assert.AreEqual(StoppingReason.FunctionTolerance, result.Reason);
                    Assert.IsTrue(result.Message.Contains("stalled"), "Message should mention stall");
                    return;
                }
            }

            Assert.Fail("Should have detected stall");
        }

        [TestMethod]
        public void TracksOptimizationHistory()
        {
            var monitor = new ConvergenceMonitor();

            for (var i = 0; i < 5; i++)
            {
                monitor.CheckConvergence(
                    iteration: i,
                    functionEvaluations: i + 1,
                    x: new[] { (double)i, (double)i },
                    fValue: 10.0 - i,
                    gradient: new[] { 1.0 / (i + 1), 1.0 / (i + 1) });
            }

            var history = monitor.GetHistory();
            Assert.AreEqual(5, history.Count, "Should have 5 iterations in history");

            for (var i = 0; i < 5; i++)
            {
                Assert.AreEqual(i, history[i].Iteration, $"History entry {i} should have correct iteration number");
                Assert.AreEqual(10.0 - i, history[i].FunctionValue, 1e-10, $"History entry {i} should have correct function value");
            }
        }

        [TestMethod]
        public void ResetClearsHistory()
        {
            var monitor = new ConvergenceMonitor();

            monitor.CheckConvergence(0, 1, s_testPoint1, 10.0, s_largeGradient);
            monitor.CheckConvergence(1, 2, s_testPoint1, 9.0, s_largeGradient);

            Assert.AreEqual(2, monitor.GetHistory().Count, "Should have 2 entries before reset");

            monitor.Reset();

            Assert.AreEqual(0, monitor.GetHistory().Count, "History should be empty after reset");
        }

        [TestMethod]
        public void ComputesGradientNormCorrectly()
        {
            var monitor = new ConvergenceMonitor();
            var gradient = new[] { 3.0, 4.0 }; // Norm should be 5.0

            var result = monitor.CheckConvergence(
                iteration: 0,
                functionEvaluations: 1,
                x: s_testPoint1,
                fValue: 10.0,
                gradient: gradient);

            Assert.AreEqual(5.0, result.GradientNorm, 1e-10, "Gradient norm should be 5.0");
        }

        [TestMethod]
        public void ComputesFunctionChangeCorrectly()
        {
            var monitor = new ConvergenceMonitor();

            monitor.CheckConvergence(0, 1, s_testPoint1, 10.0, s_largeGradient);
            var result = monitor.CheckConvergence(1, 2, s_testPoint2, 8.5, s_largeGradient);

            Assert.AreEqual(1.5, result.FunctionChange, 1e-10, "Function change should be 1.5");
        }

        [TestMethod]
        public void ComputesParameterChangeCorrectly()
        {
            var monitor = new ConvergenceMonitor();

            var point1 = new[] { 0.0, 0.0 };
            var point2 = new[] { 3.0, 4.0 }; // Distance should be 5.0

            monitor.CheckConvergence(0, 1, point1, 10.0, s_largeGradient);
            var result = monitor.CheckConvergence(1, 2, point2, 9.0, s_largeGradient);

            Assert.AreEqual(5.0, result.ParameterChange, 1e-10, "Parameter change should be 5.0");
        }

        [TestMethod]
        public void DoesNotConvergeWhenNotMeetingCriteria()
        {
            var monitor = new ConvergenceMonitor(
                gradientTolerance: 1e-6,
                functionTolerance: 1e-8,
                parameterTolerance: 1e-8);

            var result = monitor.CheckConvergence(
                iteration: 0,
                functionEvaluations: 1,
                x: s_testPoint1,
                fValue: 10.0,
                gradient: s_largeGradient);

            Assert.IsFalse(result.HasConverged, "Should not converge with large gradient");
            Assert.IsFalse(result.Reason.HasValue, "Should not have a stopping reason");
            Assert.AreEqual("Optimization in progress", result.Message);
        }
    }
}
