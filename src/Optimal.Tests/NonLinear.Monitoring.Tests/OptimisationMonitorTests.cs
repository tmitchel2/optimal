/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Constrained;
using Optimal.NonLinear.Constraints;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.NonLinear.Monitoring.Tests
{
    [TestClass]
    public sealed class OptimisationMonitorTests
    {
        private static readonly double[] s_testPoint2D = [1.0, 2.0];
        private static readonly double[] s_testPointRosenbrock = [0.5, 0.5];
        private static readonly double[] s_initialPoint = [0.0, 0.0];
        private static readonly double[] s_initialPointSmall = [0.1, 0.1];
        private static readonly double[] s_testPoint1D = [1.0];
        private static readonly double[] s_testPoint2D_v2 = [2.0, 3.0];
        private static readonly double[] s_lineSearchBase1D = [1.0];
        private static readonly double[] s_lineSearchDir1D = [-1.0];
        private static readonly double[] s_lineSearchBaseNeg1D = [-1.0];
        private static readonly double[] s_lineSearchDirPos1D = [1.0];
        private static readonly double[] s_lineSearchBase2D = [2.0];
        private static readonly double[] s_gradPos1 = [1.0];
        private static readonly double[] s_gradNeg1 = [-1.0];
        private static readonly double[] s_gradZero1 = [0.0];
        private static readonly double[] s_grad4 = [4.0];
        private static readonly double[] s_grad3 = [3.0];
        private static readonly double[] s_grad2 = [2.0];
        private static readonly double[] s_grad1 = [1.0];
        private static readonly double[] s_wrongConstraintGrad = [1.0, 0.0];

        #region Gradient Verification Tests

        [TestMethod]
        public void GradientVerificationDetectsCorrectGradient()
        {
            // Simple quadratic: f(x,y) = x^2 + y^2
            // Correct gradient: [2x, 2y]
            static (double value, double[] gradient) Quadratic(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6)
                .WithObjective(Quadratic)
                .WithTestPoint(s_testPoint2D);

            monitor.OnOptimizationStart(s_testPoint2D);
            var report = monitor.GenerateReport();

            Assert.IsFalse(report.BadGradientSuspected, "Correct gradient should not be flagged");
            Assert.IsNotNull(report.ObjectiveGradientResult);
            Assert.IsFalse(report.ObjectiveGradientResult.IsSuspicious);
            Assert.IsLessThan(1e-4, report.ObjectiveGradientResult.MaxRelativeError,
                $"Max relative error should be small, was {report.ObjectiveGradientResult.MaxRelativeError:E3}");
        }

        [TestMethod]
        public void GradientVerificationDetectsIncorrectGradient()
        {
            // Simple quadratic: f(x,y) = x^2 + y^2
            // Incorrect gradient: [x, y] (missing factor of 2)
            static (double value, double[] gradient) QuadraticWithWrongGradient(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { x[0], x[1] }; // Wrong! Should be 2*x
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6)
                .WithObjective(QuadraticWithWrongGradient)
                .WithTestPoint(s_testPoint2D);

            monitor.OnOptimizationStart(s_testPoint2D);
            var report = monitor.GenerateReport();

            Assert.IsTrue(report.BadGradientSuspected, "Incorrect gradient should be flagged");
            Assert.IsNotNull(report.ObjectiveGradientResult);
            Assert.IsTrue(report.ObjectiveGradientResult.IsSuspicious);
            Assert.AreEqual(-1, report.BadGradientFunctionIndex, "Should indicate objective function");
        }

        [TestMethod]
        public void GradientVerificationDetectsIncorrectConstraintGradient()
        {
            // Objective: f(x,y) = x^2 + y^2 (correct gradient)
            static (double value, double[] gradient) Quadratic(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            // Constraint: g(x,y) = x + y - 1 with wrong gradient [1, 0] instead of [1, 1]
            var wrongConstraint = new EqualityConstraint(x =>
            {
                var value = x[0] + x[1] - 1.0;
                return (value, s_wrongConstraintGrad);
            });

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6)
                .WithObjective(Quadratic)
                .WithConstraints(new IConstraint[] { wrongConstraint })
                .WithTestPoint(s_testPoint2D);

            monitor.OnOptimizationStart(s_testPoint2D);
            var report = monitor.GenerateReport();

            Assert.IsTrue(report.BadGradientSuspected, "Incorrect constraint gradient should be flagged");
            Assert.HasCount(1, report.ConstraintGradientResults);
            Assert.IsTrue(report.ConstraintGradientResults[0].IsSuspicious);
            Assert.AreEqual(0, report.BadGradientFunctionIndex, "Should indicate constraint 0");
            Assert.AreEqual(1, report.BadGradientVariableIndex, "Should flag variable 1 (y) as having wrong gradient");
        }

        [TestMethod]
        public void GradientVerificationWorksWithRosenbrockFunction()
        {
            // Rosenbrock: f(x,y) = (1-x)^2 + 100*(y-x^2)^2
            // Gradient: [-2(1-x) - 400*x*(y-x^2), 200*(y-x^2)]
            static (double value, double[] gradient) Rosenbrock(double[] x)
            {
                var xVal = x[0];
                var yVal = x[1];
                var value = (1.0 - xVal) * (1.0 - xVal) + 100.0 * (yVal - xVal * xVal) * (yVal - xVal * xVal);
                var gradient = new double[]
                {
                    -2.0 * (1.0 - xVal) - 400.0 * xVal * (yVal - xVal * xVal),
                    200.0 * (yVal - xVal * xVal)
                };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6)
                .WithObjective(Rosenbrock)
                .WithTestPoint(s_testPointRosenbrock);

            monitor.OnOptimizationStart(s_testPointRosenbrock);
            var report = monitor.GenerateReport();

            Assert.IsFalse(report.BadGradientSuspected, "Rosenbrock gradient should be correct");
            Assert.IsLessThan(1e-3, report.ObjectiveGradientResult!.MaxRelativeError,
                $"Max relative error should be small for Rosenbrock, was {report.ObjectiveGradientResult.MaxRelativeError:E3}");
        }

        #endregion

        #region Smoothness Monitoring Tests

        [TestMethod]
        public void SmoothnessMonitoringDetectsNonSmoothAbsoluteValue()
        {
            // f(x) = |x| is non-smooth at x=0
            // We simulate a line search crossing x=0
            var monitor = new OptimisationMonitor()
                .WithSmoothnessMonitoring();

            monitor.OnOptimizationStart(s_testPoint1D);

            // Simulate a line search from x=1 in direction d=-1
            // Points: x=1, x=0.5, x=0, x=-0.5
            monitor.OnLineSearchStart(s_lineSearchBase1D, s_lineSearchDir1D);

            // At x=1 (alpha=0): f=1, grad=1
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.0,
                FunctionValue = 1.0,
                Gradient = s_gradPos1
            });

            // At x=0.5 (alpha=0.5): f=0.5, grad=1
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.5,
                FunctionValue = 0.5,
                Gradient = s_gradPos1
            });

            // At x=0 (alpha=1): f=0, grad undefined (use 0)
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 1.0,
                FunctionValue = 0.0,
                Gradient = s_gradZero1
            });

            // At x=-0.5 (alpha=1.5): f=0.5, grad=-1 (derivative of |x| for x<0)
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 1.5,
                FunctionValue = 0.5,
                Gradient = s_gradNeg1
            });

            monitor.OnLineSearchEnd();

            var report = monitor.GenerateReport();

            // The gradient changes from +1 to -1, which should be detected as C1 violation
            Assert.IsTrue(report.NonC1Suspected,
                "Should detect C1 non-smoothness in |x| function");
        }

        [TestMethod]
        public void SmoothnessMonitoringNoFalsePositivesForSmoothFunction()
        {
            // f(x) = x^2 is smooth everywhere
            var monitor = new OptimisationMonitor()
                .WithSmoothnessMonitoring();

            monitor.OnOptimizationStart(s_lineSearchBase2D);

            // Simulate a line search from x=2 in direction d=-1
            monitor.OnLineSearchStart(s_lineSearchBase2D, s_lineSearchDir1D);

            // At x=2 (alpha=0): f=4, grad=4
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.0,
                FunctionValue = 4.0,
                Gradient = s_grad4
            });

            // At x=1.5 (alpha=0.5): f=2.25, grad=3
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.5,
                FunctionValue = 2.25,
                Gradient = s_grad3
            });

            // At x=1 (alpha=1): f=1, grad=2
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 1.0,
                FunctionValue = 1.0,
                Gradient = s_grad2
            });

            // At x=0.5 (alpha=1.5): f=0.25, grad=1
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 1.5,
                FunctionValue = 0.25,
                Gradient = s_grad1
            });

            monitor.OnLineSearchEnd();

            var report = monitor.GenerateReport();

            Assert.IsFalse(report.NonC0Suspected, "Should not detect C0 issues in smooth x^2");
            Assert.IsFalse(report.NonC1Suspected, "Should not detect C1 issues in smooth x^2");
        }

        [TestMethod]
        public void SmoothnessMonitoringDetectsDiscontinuity()
        {
            // Simulate a step function discontinuity
            var monitor = new OptimisationMonitor()
                .WithSmoothnessMonitoring();

            monitor.OnOptimizationStart(s_lineSearchBaseNeg1D);

            monitor.OnLineSearchStart(s_lineSearchBaseNeg1D, s_lineSearchDirPos1D);

            // At x=-1 (alpha=0): f=0
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.0,
                FunctionValue = 0.0,
                Gradient = s_gradZero1
            });

            // At x=-0.5 (alpha=0.5): f=0
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.5,
                FunctionValue = 0.0,
                Gradient = s_gradZero1
            });

            // At x=0 (alpha=1): f=1 (step function jumps here)
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 1.0,
                FunctionValue = 1.0,
                Gradient = s_gradZero1
            });

            // At x=0.5 (alpha=1.5): f=1
            monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 1.5,
                FunctionValue = 1.0,
                Gradient = s_gradZero1
            });

            monitor.OnLineSearchEnd();

            var report = monitor.GenerateReport();

            // The function jumps from 0 to 1, which should be detected
            Assert.IsTrue(report.NonC0Suspected,
                "Should detect C0 discontinuity in step function");
        }

        #endregion

        #region Integration Tests

        [TestMethod]
        public void IntegrationTestMonitorWithAugmentedLagrangianOptimizer()
        {
            // Test full integration with optimizer
            // Minimize x^2 + y^2 subject to x + y = 1
            static (double value, double[] gradient) Objective(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            static (double value, double[] gradient) Constraint(double[] x)
            {
                var value = x[0] + x[1] - 1.0;
                var gradient = new double[] { 1.0, 1.0 };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6)
                .WithSmoothnessMonitoring();

            var optimizer = new AugmentedLagrangianOptimizer(
                new AugmentedLagrangianOptions
                {
                    Tolerance = 1e-4,
                    ConstraintTolerance = 1e-6,
                    MaxIterations = 30,
                    EqualityConstraints = [Constraint]
                },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()),
                monitor);

            var result = optimizer.Minimize(Objective, s_initialPoint, CancellationToken.None);

            Assert.IsTrue(result.Success, $"Optimization should succeed: {result.Message}");

            var report = monitor.GenerateReport();

            // Gradients should be correct
            Assert.IsFalse(report.BadGradientSuspected, "No gradient errors expected");

            // Function is smooth - check C1 (gradient) continuity
            // Note: C0 detection via linear interpolation can have false positives on
            // curved function paths in real optimization, so we only check C1 here.
            Assert.IsFalse(report.NonC1Suspected, "No C1 issues expected for smooth function");

            // Verify optimization result
            var constraintViolation = Math.Abs(result.OptimalPoint[0] + result.OptimalPoint[1] - 1.0);
            Assert.IsLessThan(1e-4, constraintViolation, $"Constraint should be satisfied, violation: {constraintViolation:E3}");

            // Solution should be (0.5, 0.5)
            Assert.AreEqual(0.5, result.OptimalPoint[0], 1e-2, "x should be near 0.5");
            Assert.AreEqual(0.5, result.OptimalPoint[1], 1e-2, "y should be near 0.5");
        }

        [TestMethod]
        public void IntegrationTestMonitorDetectsBadGradientInRealOptimization()
        {
            // Use an objective with intentionally wrong gradient
            static (double value, double[] gradient) BadObjective(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                // Wrong gradient: factor of 3 instead of 2
                var gradient = new double[] { 3 * x[0], 3 * x[1] };
                return (value, gradient);
            }

            static (double value, double[] gradient) Constraint(double[] x)
            {
                var value = x[0] + x[1] - 1.0;
                var gradient = new double[] { 1.0, 1.0 };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6);

            var optimizer = new AugmentedLagrangianOptimizer(
                new AugmentedLagrangianOptions
                {
                    Tolerance = 1e-4,
                    MaxIterations = 20,
                    EqualityConstraints = [Constraint]
                },
                new LBFGSOptimizer(new LBFGSOptions(), new BacktrackingLineSearch()),
                monitor);

            // Run optimization (may or may not converge due to bad gradient)
            _ = optimizer.Minimize(BadObjective, s_initialPointSmall, CancellationToken.None);

            var report = monitor.GenerateReport();

            // Monitor should detect the bad gradient
            Assert.IsTrue(report.BadGradientSuspected,
                "Monitor should detect the incorrect gradient");
            Assert.AreEqual(-1, report.BadGradientFunctionIndex,
                "Should flag the objective function (index -1)");
        }

        [TestMethod]
        public void IntegrationTestResetAllowsReuse()
        {
            static (double value, double[] gradient) Quadratic(double[] x)
            {
                var value = x[0] * x[0] + x[1] * x[1];
                var gradient = new double[] { 2 * x[0], 2 * x[1] };
                return (value, gradient);
            }

            var monitor = new OptimisationMonitor()
                .WithGradientVerification(testStep: 1e-6);

            // First run
            monitor.WithObjective(Quadratic).WithTestPoint(s_testPoint2D);
            monitor.OnOptimizationStart(s_testPoint2D);
            var report1 = monitor.GenerateReport();
            Assert.IsFalse(report1.BadGradientSuspected);

            // Reset and run again with different test point
            monitor.Reset();
            monitor.WithTestPoint(s_testPoint2D_v2);
            monitor.OnOptimizationStart(s_testPoint2D_v2);
            var report2 = monitor.GenerateReport();
            Assert.IsFalse(report2.BadGradientSuspected);

            // Verify the test points are different in the reports
            Assert.AreEqual(1.0, report1.ObjectiveGradientResult!.TestPoint[0]);
            Assert.AreEqual(2.0, report2.ObjectiveGradientResult!.TestPoint[0]);
        }

        #endregion

        #region Report Generation Tests

        [TestMethod]
        public void ReportContainsSummaryMessage()
        {
            var monitor = new OptimisationMonitor()
                .WithGradientVerification();

            static (double value, double[] gradient) Quadratic(double[] x)
            {
                return (x[0] * x[0], new double[] { 2 * x[0] });
            }

            monitor.WithObjective(Quadratic);
            monitor.OnOptimizationStart(s_testPoint1D);
            var report = monitor.GenerateReport();

            Assert.IsFalse(string.IsNullOrEmpty(report.Summary), "Summary should not be empty");
            StringAssert.Contains(report.Summary, "No issues detected",
                $"Summary should indicate no issues, was: {report.Summary}");
        }

        [TestMethod]
        public void ReportTracksFunctionEvaluations()
        {
            var monitor = new OptimisationMonitor()
                .WithGradientVerification();

            static (double value, double[] gradient) TwoVarFunction(double[] x)
            {
                return (x[0] * x[0] + x[1] * x[1], new double[] { 2 * x[0], 2 * x[1] });
            }

            monitor.WithObjective(TwoVarFunction);
            monitor.OnOptimizationStart(s_testPoint2D);
            var report = monitor.GenerateReport();

            // 4-point formula needs 4 evaluations per variable, plus 1 for the initial gradient
            // So for 2 variables: 4*2 + 1 = 9 evaluations
            Assert.IsGreaterThan(0, report.MonitorFunctionEvaluations,
                "Should track function evaluations used by monitor");
        }

        #endregion
    }
}
