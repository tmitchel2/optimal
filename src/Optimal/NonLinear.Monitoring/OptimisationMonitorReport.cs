/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.NonLinear.Monitoring
{
    /// <summary>
    /// Complete report from optimization monitoring.
    /// </summary>
    public sealed record OptimisationMonitorReport
    {
        /// <summary>
        /// True if any gradient appears incorrect.
        /// </summary>
        public bool BadGradientSuspected { get; init; }

        /// <summary>
        /// Index of the function with bad gradient: -1 = objective, 0+ = constraint index.
        /// </summary>
        public int BadGradientFunctionIndex { get; init; } = -1;

        /// <summary>
        /// Index of the variable with the worst gradient error.
        /// </summary>
        public int BadGradientVariableIndex { get; init; } = -1;

        /// <summary>
        /// Gradient verification result for the objective function.
        /// Null if gradient verification was not enabled.
        /// </summary>
        public GradientVerificationResult? ObjectiveGradientResult { get; init; }

        /// <summary>
        /// Gradient verification results for each constraint.
        /// Empty if gradient verification was not enabled or no constraints.
        /// </summary>
        public IReadOnlyList<GradientVerificationResult> ConstraintGradientResults { get; init; }
            = Array.Empty<GradientVerificationResult>();

        /// <summary>
        /// True if C0 discontinuity is suspected.
        /// </summary>
        public bool NonC0Suspected { get; init; }

        /// <summary>
        /// True if C1 non-smoothness is suspected.
        /// </summary>
        public bool NonC1Suspected { get; init; }

        /// <summary>
        /// Detailed smoothness test results.
        /// Null if smoothness monitoring was not enabled.
        /// </summary>
        public SmoothnessTestResult? SmoothnessResult { get; init; }

        /// <summary>
        /// Number of function evaluations used by the monitor.
        /// </summary>
        public int MonitorFunctionEvaluations { get; init; }

        /// <summary>
        /// Summary message describing findings.
        /// </summary>
        public string Summary { get; init; } = string.Empty;
    }
}
