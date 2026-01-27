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
    /// Result of smoothness monitoring analysis.
    /// </summary>
    public sealed record SmoothnessTestResult
    {
        /// <summary>
        /// True if C0 discontinuity (function value jump) is suspected.
        /// </summary>
        public bool IsC0Suspected { get; init; }

        /// <summary>
        /// True if C1 non-smoothness (gradient discontinuity) is suspected.
        /// </summary>
        public bool IsC1Suspected { get; init; }

        /// <summary>
        /// Details of detected C0 violations.
        /// </summary>
        public IReadOnlyList<C0ViolationInfo> C0Violations { get; init; }
            = Array.Empty<C0ViolationInfo>();

        /// <summary>
        /// Details of detected C1 violations.
        /// </summary>
        public IReadOnlyList<C1ViolationInfo> C1Violations { get; init; }
            = Array.Empty<C1ViolationInfo>();

        /// <summary>
        /// Total number of line searches analyzed.
        /// </summary>
        public int TotalLineSearchesAnalyzed { get; init; }
    }
}
