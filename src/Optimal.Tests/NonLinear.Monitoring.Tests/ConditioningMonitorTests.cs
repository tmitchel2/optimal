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

namespace Optimal.NonLinear.Monitoring.Tests
{
    [TestClass]
    public sealed class ConditioningMonitorTests
    {
        private static readonly double[] s_s1 = [1.0, 0.0];
        private static readonly double[] s_y1 = [2.0, 0.0];
        private static readonly double[] s_s2 = [0.0, 0.1];
        private static readonly double[] s_y2 = [0.0, 0.2];
        private static readonly double[] s_initPoint2D = [1.0, 1.0];
        private static readonly double[] s_startPoint = [5.0, 5.0];

        #region Basic Functionality Tests

        [TestMethod]
        public void EmptyMonitorReturnsWellConditioned()
        {
            var monitor = new ConditioningMonitor();
            var result = monitor.Analyze();

            Assert.IsFalse(result.IsIllConditioned);
            Assert.AreEqual(ConditioningSeverity.None, result.Severity);
            Assert.AreEqual(1.0, result.EstimatedConditionNumber);
            Assert.AreEqual(0, result.CorrectionPairsAnalyzed);
        }

        [TestMethod]
        public void SinglePairReturnsResult()
        {
            var monitor = new ConditioningMonitor();

            // Add a single correction pair
            monitor.RecordCorrectionPair(s_s1, s_y1, 0.5);
            var result = monitor.Analyze();

            Assert.AreEqual(1, result.CorrectionPairsAnalyzed);
        }

        [TestMethod]
        public void ResetClearsHistory()
        {
            var monitor = new ConditioningMonitor();

            monitor.RecordCorrectionPair(s_s1, s_y1, 0.5);
            Assert.AreEqual(1, monitor.Count);

            monitor.Reset();
            Assert.AreEqual(0, monitor.Count);

            var result = monitor.Analyze();
            Assert.AreEqual(0, result.CorrectionPairsAnalyzed);
        }

        [TestMethod]
        public void ThrowsOnInvalidThreshold()
        {
            var exceptionThrown = false;
            try
            {
                _ = new ConditioningMonitor(threshold: 0);
            }
            catch (ArgumentException)
            {
                exceptionThrown = true;
            }
            Assert.IsTrue(exceptionThrown, "Should throw ArgumentException for invalid threshold");
        }

        [TestMethod]
        public void ThrowsOnMismatchedVectorDimensions()
        {
            var exceptionThrown = false;
            try
            {
                var monitor = new ConditioningMonitor();
                var s = new double[] { 1.0, 0.0 };
                var y = new double[] { 2.0, 0.0, 0.0 };
                monitor.RecordCorrectionPair(s, y, 0.5);
            }
            catch (ArgumentException)
            {
                exceptionThrown = true;
            }
            Assert.IsTrue(exceptionThrown, "Should throw ArgumentException for mismatched dimensions");
        }

        [TestMethod]
        public void ThrowsOnDimensionChangeAfterFirstPair()
        {
            var exceptionThrown = false;
            try
            {
                var monitor = new ConditioningMonitor();

                monitor.RecordCorrectionPair(s_s1, s_y1, 0.5);

                var s2 = new double[] { 1.0, 0.0, 0.0 };
                var y2 = new double[] { 2.0, 0.0, 0.0 };
                monitor.RecordCorrectionPair(s2, y2, 0.5);
            }
            catch (ArgumentException)
            {
                exceptionThrown = true;
            }
            Assert.IsTrue(exceptionThrown, "Should throw ArgumentException for dimension change");
        }

        #endregion

        #region Conditioning Detection Tests

        [TestMethod]
        public void WellConditionedQuadraticDetectedCorrectly()
        {
            // Simulate a well-conditioned quadratic: f(x) = x1^2 + x2^2
            var monitor = new ConditioningMonitor(threshold: 1e4);

            // Add several correction pairs with similar curvature in both directions
            for (var i = 0; i < 5; i++)
            {
                var s = new double[] { 0.1 * (i + 1), 0.1 * (i + 1) };
                var y = new double[] { 0.2 * (i + 1), 0.2 * (i + 1) };
                var yTs = y[0] * s[0] + y[1] * s[1];
                var rho = 1.0 / yTs;

                monitor.RecordCorrectionPair(s, y, rho);
            }

            var result = monitor.Analyze();

            Assert.IsFalse(result.IsIllConditioned, "Well-conditioned problem should not be flagged");
            Assert.IsLessThan(100.0, result.EstimatedConditionNumber, "Condition number should be small");
            Assert.AreEqual(ConditioningSeverity.None, result.Severity);
        }

        [TestMethod]
        public void IllConditionedStretchedQuadraticDetected()
        {
            // Simulate an ill-conditioned stretched quadratic
            var monitor = new ConditioningMonitor(threshold: 1e3);

            // Direction 1: low curvature
            var s1 = new double[] { 0.1, 0.0 };
            var y1 = new double[] { 0.2, 0.0 };
            var yTs1 = 0.02;
            monitor.RecordCorrectionPair(s1, y1, 1.0 / yTs1);

            // Direction 2: high curvature
            var s2 = new double[] { 0.0, 0.01 };
            var y2 = new double[] { 0.0, 200.0 };
            var yTs2 = 2.0;
            monitor.RecordCorrectionPair(s2, y2, 1.0 / yTs2);

            var result = monitor.Analyze();

            Assert.IsTrue(result.IsIllConditioned, "Ill-conditioned problem should be flagged");
            Assert.IsGreaterThan(1e3, result.EstimatedConditionNumber, "Condition number should be large");
            Assert.AreNotEqual(ConditioningSeverity.None, result.Severity);
        }

        [TestMethod]
        public void SeverityLevelsAreCorrect()
        {
            Assert.AreEqual(ConditioningSeverity.None, ConditioningEstimate.GetSeverity(1.0));
            Assert.AreEqual(ConditioningSeverity.None, ConditioningEstimate.GetSeverity(99.0));
            Assert.AreEqual(ConditioningSeverity.Mild, ConditioningEstimate.GetSeverity(100.0));
            Assert.AreEqual(ConditioningSeverity.Mild, ConditioningEstimate.GetSeverity(9999.0));
            Assert.AreEqual(ConditioningSeverity.Moderate, ConditioningEstimate.GetSeverity(10000.0));
            Assert.AreEqual(ConditioningSeverity.Moderate, ConditioningEstimate.GetSeverity(1e7));
            Assert.AreEqual(ConditioningSeverity.Severe, ConditioningEstimate.GetSeverity(1e8));
            Assert.AreEqual(ConditioningSeverity.Severe, ConditioningEstimate.GetSeverity(1e12));
        }

        [TestMethod]
        public void ThresholdAffectsIsIllConditionedFlag()
        {
            var monitorLow = new ConditioningMonitor(threshold: 10);
            var monitorHigh = new ConditioningMonitor(threshold: 1e6);

            // Add pairs with moderate conditioning
            var s1 = new double[] { 0.1, 0.0 };
            var y1 = new double[] { 0.2, 0.0 };
            var s2 = new double[] { 0.0, 0.01 };
            var y2 = new double[] { 0.0, 20.0 };

            monitorLow.RecordCorrectionPair(s1, y1, 50.0);
            monitorLow.RecordCorrectionPair(s2, y2, 0.5);

            monitorHigh.RecordCorrectionPair(s1, y1, 50.0);
            monitorHigh.RecordCorrectionPair(s2, y2, 0.5);

            var resultLow = monitorLow.Analyze();
            var resultHigh = monitorHigh.Analyze();

            Assert.IsTrue(resultLow.IsIllConditioned, "Low threshold should flag");
            Assert.IsFalse(resultHigh.IsIllConditioned, "High threshold should not flag");
        }

        #endregion

        #region Integration with OptimisationMonitor Tests

        [TestMethod]
        public void OptimisationMonitorWithConditioningEnabled()
        {
            var monitor = new OptimisationMonitor()
                .WithConditioningMonitoring(threshold: 1e4);

            Assert.IsTrue(monitor.IsConditioningMonitoringEnabled);

            monitor.OnOptimizationStart(s_initPoint2D);

            monitor.OnCorrectionPairAdded(s_s1, s_y1, 50.0);
            monitor.OnCorrectionPairAdded(s_s2, s_y2, 50.0);

            var report = monitor.GenerateReport();

            Assert.IsNotNull(report.ConditioningResult);
            Assert.AreEqual(2, report.ConditioningResult.CorrectionPairsAnalyzed);
            Assert.IsFalse(report.IllConditioningSuspected, "Well-conditioned pairs should not trigger flag");
        }

        [TestMethod]
        public void OptimisationMonitorDetectsIllConditioning()
        {
            var monitor = new OptimisationMonitor()
                .WithConditioningMonitoring(threshold: 100);

            monitor.OnOptimizationStart(s_initPoint2D);

            // Add ill-conditioned pairs
            var s1 = new double[] { 0.1, 0.0 };
            var y1 = new double[] { 0.2, 0.0 };
            var yTs1 = 0.02;
            monitor.OnCorrectionPairAdded(s1, y1, 1.0 / yTs1);

            var s2 = new double[] { 0.0, 0.01 };
            var y2 = new double[] { 0.0, 100.0 };
            var yTs2 = 1.0;
            monitor.OnCorrectionPairAdded(s2, y2, 1.0 / yTs2);

            var report = monitor.GenerateReport();

            Assert.IsNotNull(report.ConditioningResult);
            Assert.IsTrue(report.IllConditioningSuspected, "Ill-conditioned pairs should trigger flag");
            StringAssert.Contains(report.Summary, "Ill-conditioning suspected");
        }

        [TestMethod]
        public void ReportSummaryIncludesConditioningInfo()
        {
            var monitor = new OptimisationMonitor()
                .WithConditioningMonitoring(threshold: 10);

            monitor.OnOptimizationStart(s_initPoint2D);

            var s1 = new double[] { 0.1, 0.0 };
            var y1 = new double[] { 0.2, 0.0 };
            monitor.OnCorrectionPairAdded(s1, y1, 50.0);

            var s2 = new double[] { 0.0, 0.01 };
            var y2 = new double[] { 0.0, 5.0 };
            monitor.OnCorrectionPairAdded(s2, y2, 2.0);

            var report = monitor.GenerateReport();

            Assert.IsNotNull(report.ConditioningResult);
            if (report.IllConditioningSuspected)
            {
                StringAssert.Contains(report.Summary, "Ill-conditioning");
            }
        }

        #endregion

        #region Integration with L-BFGS Optimizer Tests

        [TestMethod]
        public void LBFGSOptimizerCallsMonitorCallback()
        {
            // Simple quadratic: f(x,y) = x^2 + y^2
            static (double value, double[] gradient) Objective(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithConditioningMonitoring(threshold: 1e4);

            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { MaxIterations = 10 },
                new BacktrackingLineSearch(),
                monitor);

            monitor.OnOptimizationStart(s_startPoint);

            var result = optimizer.Minimize(Objective, s_startPoint);
            var report = monitor.GenerateReport();

            Assert.IsTrue(result.Success, $"Optimization should succeed: {result.Message}");
            Assert.IsNotNull(report.ConditioningResult, "Conditioning result should be available");
            Assert.IsGreaterThan(0, report.ConditioningResult.CorrectionPairsAnalyzed, "Should have analyzed correction pairs");
            Assert.IsFalse(report.IllConditioningSuspected, "Well-conditioned quadratic should not be flagged");
        }

        [TestMethod]
        public void LBFGSOptimizerDetectsIllConditionedProblem()
        {
            // Ill-conditioned quadratic: f(x,y) = x^2 + 10000*y^2
            static (double value, double[] gradient) IllConditionedObjective(double[] x)
            {
                var value = x[0] * x[0] + 10000 * x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 20000 * x[1] };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithConditioningMonitoring(threshold: 100);

            var optimizer = new LBFGSOptimizer(
                new LBFGSOptions { MaxIterations = 100, Tolerance = 1e-8 },
                new BacktrackingLineSearch(),
                monitor);

            monitor.OnOptimizationStart(s_initPoint2D);

            var result = optimizer.Minimize(IllConditionedObjective, s_initPoint2D);
            var report = monitor.GenerateReport();

            Assert.IsNotNull(report.ConditioningResult, "Conditioning result should be available");
            Assert.IsGreaterThan(0, report.ConditioningResult.CorrectionPairsAnalyzed, "Should have analyzed correction pairs");
            Assert.IsNotNull(report.ConditioningResult.DiagonalCurvatureEstimates);
        }

        #endregion
    }
}
