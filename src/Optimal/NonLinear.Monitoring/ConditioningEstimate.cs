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
    /// Severity level of ill-conditioning.
    /// </summary>
    public enum ConditioningSeverity
    {
        /// <summary>
        /// Well-conditioned problem (condition number less than 100).
        /// </summary>
        None,

        /// <summary>
        /// Mildly ill-conditioned (condition number between 10^2 and 10^4).
        /// </summary>
        Mild,

        /// <summary>
        /// Moderately ill-conditioned (condition number between 10^4 and 10^8).
        /// </summary>
        Moderate,

        /// <summary>
        /// Severely ill-conditioned (condition number greater than 10^8).
        /// </summary>
        Severe
    }

    /// <summary>
    /// Result of problem conditioning analysis based on L-BFGS curvature information.
    /// </summary>
    public sealed record ConditioningEstimate
    {
        /// <summary>
        /// Estimated condition number based on the ratio of max/min curvature estimates.
        /// </summary>
        public double EstimatedConditionNumber { get; init; }

        /// <summary>
        /// True if the problem is suspected to be ill-conditioned based on the threshold.
        /// </summary>
        public bool IsIllConditioned { get; init; }

        /// <summary>
        /// Severity level of the ill-conditioning.
        /// </summary>
        public ConditioningSeverity Severity { get; init; }

        /// <summary>
        /// Raw curvature ratio from y^T*s analysis.
        /// </summary>
        public double CurvatureRatio { get; init; }

        /// <summary>
        /// Per-variable diagonal curvature estimates approximating Hessian diagonal.
        /// </summary>
        public IReadOnlyList<double> DiagonalCurvatureEstimates { get; init; }
            = Array.Empty<double>();

        /// <summary>
        /// Indices of variables with extreme curvature (potential problem areas).
        /// </summary>
        public IReadOnlyList<int> ProblematicVariableIndices { get; init; }
            = Array.Empty<int>();

        /// <summary>
        /// Number of correction pairs analyzed.
        /// </summary>
        public int CorrectionPairsAnalyzed { get; init; }

        /// <summary>
        /// Gets the severity level for a given condition number.
        /// </summary>
        /// <param name="conditionNumber">The estimated condition number.</param>
        /// <returns>The severity level.</returns>
        public static ConditioningSeverity GetSeverity(double conditionNumber)
        {
            if (conditionNumber < 1e2)
            {
                return ConditioningSeverity.None;
            }
            else if (conditionNumber < 1e4)
            {
                return ConditioningSeverity.Mild;
            }
            else if (conditionNumber < 1e8)
            {
                return ConditioningSeverity.Moderate;
            }
            else
            {
                return ConditioningSeverity.Severe;
            }
        }
    }
}
