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
    /// Monitors L-BFGS correction pairs to estimate problem conditioning.
    /// Analyzes curvature information from (s, y, rho) pairs to detect ill-conditioned problems.
    /// </summary>
    public sealed class ConditioningMonitor
    {
        private readonly double _threshold;
        private readonly List<CorrectionPairInfo> _pairs = new();
        private int _dimension = -1;

        /// <summary>
        /// Creates a new conditioning monitor with the specified ill-conditioning threshold.
        /// </summary>
        /// <param name="threshold">Condition number threshold above which problem is considered ill-conditioned (default 1e4).</param>
        public ConditioningMonitor(double threshold = 1e4)
        {
            if (threshold <= 0)
            {
                throw new ArgumentException("Threshold must be positive", nameof(threshold));
            }

            _threshold = threshold;
        }

        /// <summary>
        /// Gets the threshold for ill-conditioning detection.
        /// </summary>
        public double Threshold => _threshold;

        /// <summary>
        /// Gets the number of correction pairs recorded.
        /// </summary>
        public int Count => _pairs.Count;

        /// <summary>
        /// Records a correction pair from L-BFGS memory update.
        /// </summary>
        /// <param name="s">Position difference: x_{k+1} - x_k.</param>
        /// <param name="y">Gradient difference: grad_{k+1} - grad_k.</param>
        /// <param name="rho">Precomputed: 1 / (y^T * s).</param>
        public void RecordCorrectionPair(double[] s, double[] y, double rho)
        {
            if (s.Length != y.Length)
            {
                throw new ArgumentException("Vectors s and y must have the same length");
            }

            if (_dimension < 0)
            {
                _dimension = s.Length;
            }
            else if (s.Length != _dimension)
            {
                throw new ArgumentException($"Vector dimension ({s.Length}) does not match expected ({_dimension})");
            }

            _pairs.Add(new CorrectionPairInfo
            {
                S = (double[])s.Clone(),
                Y = (double[])y.Clone(),
                Rho = rho
            });
        }

        /// <summary>
        /// Analyzes recorded correction pairs to estimate problem conditioning.
        /// </summary>
        /// <returns>Conditioning estimate with analysis results.</returns>
        public ConditioningEstimate Analyze()
        {
            if (_pairs.Count == 0 || _dimension <= 0)
            {
                return new ConditioningEstimate
                {
                    EstimatedConditionNumber = 1.0,
                    IsIllConditioned = false,
                    Severity = ConditioningSeverity.None,
                    CurvatureRatio = 1.0,
                    CorrectionPairsAnalyzed = 0
                };
            }

            // Estimate diagonal Hessian from accumulated curvature information
            // D[j] += y[j]² / (y^T * s) approximates the diagonal of the Hessian
            var diagonalEstimate = new double[_dimension];
            var curvatures = new List<double>();

            foreach (var pair in _pairs)
            {
                var yTs = 1.0 / pair.Rho; // y^T * s

                // Compute directional curvature: κ = (y^T * s) / ||s||²
                var sSq = DotProduct(pair.S, pair.S);
                if (sSq > 1e-16)
                {
                    var directionalCurvature = yTs / sSq;
                    if (directionalCurvature > 0)
                    {
                        curvatures.Add(directionalCurvature);
                    }
                }

                // Accumulate diagonal Hessian estimate: D[j] += y[j]² / (y^T * s)
                if (Math.Abs(yTs) > 1e-16)
                {
                    for (var j = 0; j < _dimension; j++)
                    {
                        diagonalEstimate[j] += (pair.Y[j] * pair.Y[j]) / yTs;
                    }
                }
            }

            // Compute condition number estimate from diagonal
            var (minDiag, maxDiag, problematicIndices) = AnalyzeDiagonal(diagonalEstimate);

            double conditionNumber;
            double curvatureRatio;

            if (minDiag > 1e-16 && maxDiag > 1e-16)
            {
                conditionNumber = maxDiag / minDiag;
            }
            else if (curvatures.Count >= 2)
            {
                // Fall back to directional curvature ratio
                curvatures.Sort();
                var minCurv = curvatures[0];
                var maxCurv = curvatures[curvatures.Count - 1];
                conditionNumber = minCurv > 1e-16 ? maxCurv / minCurv : 1.0;
            }
            else
            {
                conditionNumber = 1.0;
            }

            // Compute curvature ratio from directional curvatures
            if (curvatures.Count >= 2)
            {
                curvatures.Sort();
                var minCurv = curvatures[0];
                var maxCurv = curvatures[curvatures.Count - 1];
                curvatureRatio = minCurv > 1e-16 ? maxCurv / minCurv : 1.0;
            }
            else
            {
                curvatureRatio = 1.0;
            }

            var severity = ConditioningEstimate.GetSeverity(conditionNumber);

            return new ConditioningEstimate
            {
                EstimatedConditionNumber = conditionNumber,
                IsIllConditioned = conditionNumber >= _threshold,
                Severity = severity,
                CurvatureRatio = curvatureRatio,
                DiagonalCurvatureEstimates = diagonalEstimate,
                ProblematicVariableIndices = problematicIndices,
                CorrectionPairsAnalyzed = _pairs.Count
            };
        }

        /// <summary>
        /// Clears all recorded correction pairs.
        /// </summary>
        public void Reset()
        {
            _pairs.Clear();
            _dimension = -1;
        }

        private static (double min, double max, List<int> problematicIndices) AnalyzeDiagonal(double[] diagonal)
        {
            var positiveValues = new List<(double value, int index)>();

            for (var i = 0; i < diagonal.Length; i++)
            {
                if (diagonal[i] > 1e-16)
                {
                    positiveValues.Add((diagonal[i], i));
                }
            }

            if (positiveValues.Count == 0)
            {
                return (1.0, 1.0, new List<int>());
            }

            positiveValues.Sort((a, b) => a.value.CompareTo(b.value));

            var min = positiveValues[0].value;
            var max = positiveValues[positiveValues.Count - 1].value;

            // Find problematic variables: those with extreme curvature (ratio > sqrt of threshold)
            var extremeThreshold = Math.Sqrt(max / min);
            var problematicIndices = new List<int>();

            if (extremeThreshold > 10)
            {
                var median = positiveValues[positiveValues.Count / 2].value;

                foreach (var (value, index) in positiveValues)
                {
                    var ratio = value > median ? value / median : median / value;
                    if (ratio > extremeThreshold)
                    {
                        problematicIndices.Add(index);
                    }
                }
            }

            return (min, max, problematicIndices);
        }

        private static double DotProduct(double[] a, double[] b)
        {
            var result = 0.0;
            for (var i = 0; i < a.Length; i++)
            {
                result += a[i] * b[i];
            }
            return result;
        }

        private sealed class CorrectionPairInfo
        {
            public required double[] S { get; init; }
            public required double[] Y { get; init; }
            public required double Rho { get; init; }
        }
    }
}
