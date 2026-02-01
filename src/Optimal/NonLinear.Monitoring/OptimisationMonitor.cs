/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Monitoring
{
    /// <summary>
    /// Monitors optimization for gradient correctness and function smoothness.
    /// </summary>
    public sealed class OptimisationMonitor
    {
        // Configuration
        private bool _enableGradientVerification;
        private double _gradientTestStep = 1e-6;
        private bool _enableSmoothnessMonitoring;
        private bool _enableConditioningMonitoring;
        private double _conditioningThreshold = 1e4;
        private int _lineSearchHistorySize = 100;
        private double[]? _testPoint;

        // Function references
        private Func<double[], (double value, double[] gradient)>? _objective;
        private IReadOnlyList<IConstraint>? _constraints;

        // Internal components
        private GradientVerifier? _gradientVerifier;
        private SmoothnessMonitor? _smoothnessMonitor;
        private ConditioningMonitor? _conditioningMonitor;

        // Results
        private GradientVerificationResult? _objectiveGradientResult;
        private IReadOnlyList<GradientVerificationResult>? _constraintGradientResults;
        private OptimizerResult? _optimizationResult;

        /// <summary>
        /// Enables gradient verification against numerical differentiation.
        /// Uses 4-point central difference formula with O(h^4) accuracy.
        /// Cost: ~4*N additional function evaluations per function verified.
        /// </summary>
        /// <param name="testStep">Step size for finite differences (default 1e-6).</param>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithGradientVerification(double testStep = 1e-6)
        {
            if (testStep <= 0)
            {
                throw new ArgumentException("Test step must be positive", nameof(testStep));
            }

            _enableGradientVerification = true;
            _gradientTestStep = testStep;
            return this;
        }

        /// <summary>
        /// Enables smoothness monitoring during line searches.
        /// Detects C0 discontinuities and C1 non-smoothness.
        /// </summary>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithSmoothnessMonitoring()
        {
            _enableSmoothnessMonitoring = true;
            return this;
        }

        /// <summary>
        /// Enables conditioning monitoring using L-BFGS curvature information.
        /// Detects ill-conditioned problems based on the ratio of curvature estimates.
        /// </summary>
        /// <param name="threshold">Condition number threshold above which problem is flagged (default 1e4).</param>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithConditioningMonitoring(double threshold = 1e4)
        {
            if (threshold <= 0)
            {
                throw new ArgumentException("Threshold must be positive", nameof(threshold));
            }

            _enableConditioningMonitoring = true;
            _conditioningThreshold = threshold;
            return this;
        }

        /// <summary>
        /// Sets the objective function for monitoring.
        /// </summary>
        /// <param name="objective">Objective function returning (value, gradient).</param>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithObjective(
            Func<double[], (double value, double[] gradient)> objective)
        {
            _objective = objective ?? throw new ArgumentNullException(nameof(objective));
            return this;
        }

        /// <summary>
        /// Sets the constraints for monitoring.
        /// </summary>
        /// <param name="constraints">List of constraints to verify.</param>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithConstraints(IReadOnlyList<IConstraint> constraints)
        {
            _constraints = constraints ?? throw new ArgumentNullException(nameof(constraints));
            return this;
        }

        /// <summary>
        /// Sets the test point for gradient verification.
        /// If not set, uses the initial point from OnOptimizationStart.
        /// </summary>
        /// <param name="testPoint">Point at which to verify gradients.</param>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithTestPoint(double[] testPoint)
        {
            _testPoint = (double[])testPoint.Clone();
            return this;
        }

        /// <summary>
        /// Sets the number of line search traces to retain for analysis.
        /// </summary>
        /// <param name="size">Maximum number of traces (default 100).</param>
        /// <returns>This monitor instance for method chaining.</returns>
        public OptimisationMonitor WithLineSearchHistorySize(int size)
        {
            if (size <= 0)
            {
                throw new ArgumentException("Size must be positive", nameof(size));
            }

            _lineSearchHistorySize = size;
            return this;
        }

        /// <summary>
        /// Gets whether smoothness monitoring is enabled.
        /// </summary>
        public bool IsSmoothnessMonitoringEnabled => _enableSmoothnessMonitoring;

        /// <summary>
        /// Gets whether conditioning monitoring is enabled.
        /// </summary>
        public bool IsConditioningMonitoringEnabled => _enableConditioningMonitoring;

        /// <summary>
        /// Called when optimization starts.
        /// </summary>
        /// <param name="x0">Initial point.</param>
        public void OnOptimizationStart(double[] x0)
        {
            // Use initial point as test point if not set
            _testPoint ??= (double[])x0.Clone();

            // Initialize gradient verifier if enabled
            if (_enableGradientVerification)
            {
                _gradientVerifier = new GradientVerifier(_gradientTestStep);
                RunGradientVerification();
            }

            // Initialize smoothness monitor if enabled
            if (_enableSmoothnessMonitoring)
            {
                _smoothnessMonitor = new SmoothnessMonitor(_lineSearchHistorySize);
            }

            // Initialize conditioning monitor if enabled
            if (_enableConditioningMonitoring)
            {
                _conditioningMonitor = new ConditioningMonitor(_conditioningThreshold);
            }
        }

        /// <summary>
        /// Called when a new line search begins.
        /// </summary>
        /// <param name="basePoint">Starting point x.</param>
        /// <param name="direction">Search direction d.</param>
        public void OnLineSearchStart(double[] basePoint, double[] direction)
        {
            _smoothnessMonitor?.StartLineSearch(basePoint, direction);
        }

        /// <summary>
        /// Called for each function evaluation during line search.
        /// </summary>
        /// <param name="step">Line search step information.</param>
        public void OnLineSearchStep(LineSearchStepInfo step)
        {
            _smoothnessMonitor?.RecordLineSearchStep(step);
        }

        /// <summary>
        /// Called when a line search completes.
        /// </summary>
        public void OnLineSearchEnd()
        {
            _smoothnessMonitor?.CompleteLineSearch();
        }

        /// <summary>
        /// Called when a correction pair is added to L-BFGS memory.
        /// Used for conditioning analysis.
        /// </summary>
        /// <param name="s">Position difference: x_{k+1} - x_k.</param>
        /// <param name="y">Gradient difference: grad_{k+1} - grad_k.</param>
        /// <param name="rho">Precomputed: 1 / (y^T * s).</param>
        public void OnCorrectionPairAdded(double[] s, double[] y, double rho)
        {
            _conditioningMonitor?.RecordCorrectionPair(s, y, rho);
        }

        /// <summary>
        /// Called when optimization completes.
        /// </summary>
        /// <param name="result">Optimization result.</param>
        public void OnOptimizationEnd(OptimizerResult result)
        {
            // Store result for potential use in report generation
            _optimizationResult = result;
        }

        /// <summary>
        /// Generates the monitoring report.
        /// </summary>
        /// <returns>Complete monitoring report.</returns>
        public OptimisationMonitorReport GenerateReport()
        {
            var summary = new StringBuilder();
            var badGradientSuspected = false;
            var badGradientFunctionIndex = -1;
            var badGradientVariableIndex = -1;

            // Check gradient verification results
            if (_objectiveGradientResult != null && _objectiveGradientResult.IsSuspicious)
            {
                badGradientSuspected = true;
                badGradientFunctionIndex = -1;
                badGradientVariableIndex = _objectiveGradientResult.WorstVariableIndex;
                summary.AppendLine(string.Format(
                    CultureInfo.InvariantCulture,
                    "Objective gradient error: max relative error {0:E3} at variable {1}",
                    _objectiveGradientResult.MaxRelativeError,
                    badGradientVariableIndex));
            }

            if (_constraintGradientResults != null)
            {
                for (var i = 0; i < _constraintGradientResults.Count; i++)
                {
                    var result = _constraintGradientResults[i];
                    if (result.IsSuspicious)
                    {
                        if (!badGradientSuspected)
                        {
                            badGradientSuspected = true;
                            badGradientFunctionIndex = i;
                            badGradientVariableIndex = result.WorstVariableIndex;
                        }
                        summary.AppendLine(string.Format(
                            CultureInfo.InvariantCulture,
                            "Constraint {0} gradient error: max relative error {1:E3} at variable {2}",
                            i,
                            result.MaxRelativeError,
                            result.WorstVariableIndex));
                    }
                }
            }

            // Analyze smoothness
            SmoothnessTestResult? smoothnessResult = null;
            var nonC0Suspected = false;
            var nonC1Suspected = false;

            if (_smoothnessMonitor != null)
            {
                smoothnessResult = _smoothnessMonitor.Analyze();
                nonC0Suspected = smoothnessResult.IsC0Suspected;
                nonC1Suspected = smoothnessResult.IsC1Suspected;

                if (nonC0Suspected)
                {
                    summary.AppendLine(string.Format(
                        CultureInfo.InvariantCulture,
                        "C0 discontinuity suspected: {0} violation(s) detected",
                        smoothnessResult.C0Violations.Count));
                }
                if (nonC1Suspected)
                {
                    summary.AppendLine(string.Format(
                        CultureInfo.InvariantCulture,
                        "C1 non-smoothness suspected: {0} violation(s) detected",
                        smoothnessResult.C1Violations.Count));
                }
            }

            // Analyze conditioning
            ConditioningEstimate? conditioningResult = null;
            var illConditioningSuspected = false;

            if (_conditioningMonitor != null)
            {
                conditioningResult = _conditioningMonitor.Analyze();
                illConditioningSuspected = conditioningResult.IsIllConditioned;

                if (illConditioningSuspected)
                {
                    summary.AppendLine(string.Format(
                        CultureInfo.InvariantCulture,
                        "Ill-conditioning suspected: condition number ~{0:E2} ({1})",
                        conditioningResult.EstimatedConditionNumber,
                        conditioningResult.Severity));

                    if (conditioningResult.ProblematicVariableIndices.Count > 0)
                    {
                        summary.AppendLine(string.Format(
                            CultureInfo.InvariantCulture,
                            "  Problematic variables: {0}",
                            string.Join(", ", conditioningResult.ProblematicVariableIndices)));
                    }
                }
            }

            // Build summary
            if (summary.Length == 0)
            {
                if (_enableGradientVerification || _enableSmoothnessMonitoring || _enableConditioningMonitoring)
                {
                    if (_optimizationResult?.Success == true)
                    {
                        summary.Append("No issues detected - optimization succeeded");
                    }
                    else
                    {
                        summary.Append("No issues detected");
                    }
                }
                else
                {
                    summary.Append("No monitoring enabled");
                }
            }

            var monitorEvaluations = _gradientVerifier?.FunctionEvaluations ?? 0;

            return new OptimisationMonitorReport
            {
                BadGradientSuspected = badGradientSuspected,
                BadGradientFunctionIndex = badGradientFunctionIndex,
                BadGradientVariableIndex = badGradientVariableIndex,
                ObjectiveGradientResult = _objectiveGradientResult,
                ConstraintGradientResults = _constraintGradientResults ?? Array.Empty<GradientVerificationResult>(),
                NonC0Suspected = nonC0Suspected,
                NonC1Suspected = nonC1Suspected,
                SmoothnessResult = smoothnessResult,
                IllConditioningSuspected = illConditioningSuspected,
                ConditioningResult = conditioningResult,
                MonitorFunctionEvaluations = monitorEvaluations,
                Summary = summary.ToString().TrimEnd()
            };
        }

        /// <summary>
        /// Resets the monitor for a new optimization run.
        /// </summary>
        public void Reset()
        {
            _objectiveGradientResult = null;
            _constraintGradientResults = null;
            _optimizationResult = null;
            _smoothnessMonitor?.Reset();
            _conditioningMonitor?.Reset();
        }

        private void RunGradientVerification()
        {
            if (_gradientVerifier == null || _testPoint == null)
            {
                return;
            }

            // Verify objective gradient
            if (_objective != null)
            {
                _objectiveGradientResult = _gradientVerifier.VerifyObjectiveGradient(_objective, _testPoint);
            }

            // Verify constraint gradients
            if (_constraints != null && _constraints.Count > 0)
            {
                _constraintGradientResults = _gradientVerifier.VerifyConstraintGradients(_constraints, _testPoint);
            }
        }
    }
}
