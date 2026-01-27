/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;

namespace Optimal.NonLinear.Monitoring
{
    /// <summary>
    /// Monitors line searches for smoothness violations (C0 discontinuities and C1 non-smoothness).
    /// </summary>
    internal sealed class SmoothnessMonitor
    {
        private readonly List<LineSearchTrace> _traces = new();
        private readonly int _maxTraces;
        private LineSearchTrace? _currentTrace;

        // Detection thresholds
        // C0 threshold set to catch step function discontinuities (error ~0.5) while
        // avoiding false positives on smooth quadratics in optimizer line searches.
        private const double C0DetectionThreshold = 0.4;
        // C1 threshold set to catch gradient discontinuities in |x| (error ~0.5) while
        // allowing smooth functions with linear gradients (error ~0).
        private const double C1DetectionThreshold = 0.3;

        /// <summary>
        /// Initializes a new instance of the <see cref="SmoothnessMonitor"/> class.
        /// </summary>
        /// <param name="maxTraces">Maximum number of line search traces to retain.</param>
        public SmoothnessMonitor(int maxTraces = 100)
        {
            _maxTraces = maxTraces;
        }

        /// <summary>
        /// Starts recording a new line search.
        /// </summary>
        /// <param name="basePoint">Starting point x.</param>
        /// <param name="direction">Search direction d.</param>
        public void StartLineSearch(double[] basePoint, double[] direction)
        {
            // Complete any previous trace
            if (_currentTrace != null)
            {
                _currentTrace.MarkComplete();
            }

            _currentTrace = new LineSearchTrace();
            _currentTrace.SetBase(basePoint, direction);
            _traces.Add(_currentTrace);

            // Limit memory usage
            while (_traces.Count > _maxTraces)
            {
                _traces.RemoveAt(0);
            }
        }

        /// <summary>
        /// Records a line search step for smoothness analysis.
        /// </summary>
        /// <param name="step">Information about the line search step.</param>
        public void RecordLineSearchStep(LineSearchStepInfo step)
        {
            _currentTrace?.AddStep(step);
        }

        /// <summary>
        /// Marks the current line search as complete.
        /// </summary>
        public void CompleteLineSearch()
        {
            if (_currentTrace != null)
            {
                _currentTrace.MarkComplete();
                _currentTrace = null;
            }
        }

        /// <summary>
        /// Analyzes all recorded traces for smoothness violations.
        /// </summary>
        /// <returns>Smoothness test result.</returns>
        public SmoothnessTestResult Analyze()
        {
            var c0Violations = new List<C0ViolationInfo>();
            var c1Violations = new List<C1ViolationInfo>();

            foreach (var trace in _traces.Where(t => t.IsComplete && t.Steps.Count >= 3))
            {
                // Test #0: Check function values for C0 continuity
                var c0Result = AnalyzeC0Continuity(trace);
                if (c0Result != null)
                {
                    c0Violations.Add(c0Result);
                }

                // Test #1: Check gradients for C1 continuity
                var c1Result = AnalyzeC1Continuity(trace);
                if (c1Result != null)
                {
                    c1Violations.Add(c1Result);
                }
            }

            return new SmoothnessTestResult
            {
                IsC0Suspected = c0Violations.Count > 0,
                IsC1Suspected = c1Violations.Count > 0,
                C0Violations = c0Violations.AsReadOnly(),
                C1Violations = c1Violations.AsReadOnly(),
                TotalLineSearchesAnalyzed = _traces.Count(t => t.IsComplete)
            };
        }

        /// <summary>
        /// Clears all recorded traces.
        /// </summary>
        public void Reset()
        {
            _traces.Clear();
            _currentTrace = null;
        }

        /// <summary>
        /// Analyzes a line search trace for C0 (continuity) violations.
        /// Uses linear interpolation to detect function value jumps.
        /// </summary>
        private static C0ViolationInfo? AnalyzeC0Continuity(LineSearchTrace trace)
        {
            var steps = trace.Steps.OrderBy(s => s.Alpha).ToList();
            if (steps.Count < 3)
            {
                return null;
            }

            C0ViolationInfo? worstViolation = null;
            var worstError = 0.0;

            for (var i = 1; i < steps.Count - 1; i++)
            {
                var prev = steps[i - 1];
                var curr = steps[i];
                var next = steps[i + 1];

                // Linear interpolation between prev and next
                var interpolated = LinearInterpolate(
                    prev.Alpha, prev.FunctionValue,
                    next.Alpha, next.FunctionValue,
                    curr.Alpha);

                var error = Math.Abs(curr.FunctionValue - interpolated);
                var scale = Math.Max(1.0, Math.Abs(interpolated));
                var relError = error / scale;

                if (relError > C0DetectionThreshold && relError > worstError)
                {
                    worstError = relError;
                    worstViolation = new C0ViolationInfo
                    {
                        Alpha = curr.Alpha,
                        ExpectedValue = interpolated,
                        ActualValue = curr.FunctionValue,
                        RelativeError = relError,
                        BasePoint = trace.BasePoint,
                        Direction = trace.Direction
                    };
                }
            }

            return worstViolation;
        }

        /// <summary>
        /// Analyzes a line search trace for C1 (smoothness) violations.
        /// Examines directional derivatives for discontinuities.
        /// </summary>
        private static C1ViolationInfo? AnalyzeC1Continuity(LineSearchTrace trace)
        {
            var steps = trace.Steps.OrderBy(s => s.Alpha).ToList();
            if (steps.Count < 3)
            {
                return null;
            }

            var direction = trace.Direction;
            if (direction.Length == 0)
            {
                return null;
            }

            C1ViolationInfo? worstViolation = null;
            var worstError = 0.0;

            for (var i = 1; i < steps.Count - 1; i++)
            {
                var prev = steps[i - 1];
                var curr = steps[i];
                var next = steps[i + 1];

                // Skip if gradients are missing
                if (prev.Gradient.Length == 0 || curr.Gradient.Length == 0 || next.Gradient.Length == 0)
                {
                    continue;
                }

                // Compute directional derivatives: g^T * d
                var dPrev = DotProduct(prev.Gradient, direction);
                var dCurr = DotProduct(curr.Gradient, direction);
                var dNext = DotProduct(next.Gradient, direction);

                // Linear interpolation of directional derivative
                var interpolated = LinearInterpolate(
                    prev.Alpha, dPrev,
                    next.Alpha, dNext,
                    curr.Alpha);

                var error = Math.Abs(dCurr - interpolated);
                var scale = Math.Max(1.0, Math.Abs(interpolated));
                var relError = error / scale;

                if (relError > C1DetectionThreshold && relError > worstError)
                {
                    worstError = relError;
                    worstViolation = new C1ViolationInfo
                    {
                        Alpha = curr.Alpha,
                        ExpectedDirectionalDerivative = interpolated,
                        ActualDirectionalDerivative = dCurr,
                        RelativeError = relError,
                        BasePoint = trace.BasePoint,
                        Direction = trace.Direction,
                        Gradient = (double[])curr.Gradient.Clone()
                    };
                }
            }

            return worstViolation;
        }

        private static double LinearInterpolate(double x0, double y0, double x1, double y1, double x)
        {
            if (Math.Abs(x1 - x0) < 1e-15)
            {
                return y0;
            }
            var t = (x - x0) / (x1 - x0);
            return y0 + t * (y1 - y0);
        }

        private static double DotProduct(double[] a, double[] b)
        {
            var sum = 0.0;
            var n = Math.Min(a.Length, b.Length);
            for (var i = 0; i < n; i++)
            {
                sum += a[i] * b[i];
            }
            return sum;
        }

        /// <summary>
        /// Records all steps in a single line search for smoothness analysis.
        /// </summary>
        private sealed class LineSearchTrace
        {
            public double[] BasePoint { get; private set; } = Array.Empty<double>();
            public double[] Direction { get; private set; } = Array.Empty<double>();
            public List<LineSearchStepInfo> Steps { get; } = new();
            public bool IsComplete { get; private set; }

            public void SetBase(double[] basePoint, double[] direction)
            {
                BasePoint = (double[])basePoint.Clone();
                Direction = (double[])direction.Clone();
            }

            public void AddStep(LineSearchStepInfo step)
            {
                Steps.Add(step);
            }

            public void MarkComplete()
            {
                IsComplete = true;
            }
        }
    }
}
