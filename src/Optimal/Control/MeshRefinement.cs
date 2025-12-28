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

namespace Optimal.Control
{
    /// <summary>
    /// Provides adaptive mesh refinement for Hermite-Simpson collocation.
    /// Refines the grid where defects are large to improve accuracy.
    /// </summary>
    public sealed class MeshRefinement
    {
        private readonly double _defectThreshold;
        private readonly int _maxSegments;
#pragma warning disable IDE0052 // Remove unread private members
        private readonly int _minSegments;
#pragma warning restore IDE0052 // Remove unread private members

        /// <summary>
        /// Creates a mesh refinement strategy.
        /// </summary>
        /// <param name="defectThreshold">Threshold for defect magnitude to trigger refinement.</param>
        /// <param name="maxSegments">Maximum number of segments allowed.</param>
        /// <param name="minSegments">Minimum number of segments.</param>
        public MeshRefinement(double defectThreshold = 1e-4, int maxSegments = 200, int minSegments = 5)
        {
            _defectThreshold = defectThreshold;
            _maxSegments = maxSegments;
            _minSegments = minSegments;
        }

        /// <summary>
        /// Analyzes defects and determines which segments need refinement.
        /// </summary>
        /// <param name="defects">Defect values from collocation.</param>
        /// <param name="stateDim">State dimension.</param>
        /// <returns>Array indicating whether each segment should be refined.</returns>
        public bool[] IdentifySegmentsForRefinement(double[] defects, int stateDim)
        {
            var numSegments = defects.Length / stateDim;
            var shouldRefine = new bool[numSegments];

            for (var k = 0; k < numSegments; k++)
            {
                // Compute max defect magnitude for this segment
                var maxDefect = 0.0;
                for (var i = 0; i < stateDim; i++)
                {
                    var defectIdx = k * stateDim + i;
                    var absDefect = Math.Abs(defects[defectIdx]);
                    if (absDefect > maxDefect)
                    {
                        maxDefect = absDefect;
                    }
                }

                // Refine if defect exceeds threshold
                shouldRefine[k] = maxDefect > _defectThreshold;
            }

            return shouldRefine;
        }

        /// <summary>
        /// Creates a refined grid by subdividing segments with large defects.
        /// </summary>
        /// <param name="currentGrid">Current collocation grid.</param>
        /// <param name="shouldRefine">Boolean array indicating which segments to refine.</param>
        /// <returns>New refined grid.</returns>
        public CollocationGrid RefineGrid(CollocationGrid currentGrid, bool[] shouldRefine)
        {
            var timePoints = new List<double> { currentGrid.InitialTime };

            for (var k = 0; k < currentGrid.Segments; k++)
            {
                var t0 = currentGrid.TimePoints[k];
                var t1 = currentGrid.TimePoints[k + 1];

                if (shouldRefine[k])
                {
                    // Subdivide this segment into two
                    var tMid = 0.5 * (t0 + t1);
                    timePoints.Add(tMid);
                }

                timePoints.Add(t1);
            }

            // Remove duplicate final point if added
            if (timePoints.Count > 1 && Math.Abs(timePoints[timePoints.Count - 1] - currentGrid.FinalTime) < 1e-12)
            {
                timePoints.RemoveAt(timePoints.Count - 1);
            }

            // Ensure we don't exceed max segments
            var newSegments = timePoints.Count - 1;
            if (newSegments > _maxSegments)
            {
                // Keep the grid as is if we'd exceed limit
                return currentGrid;
            }

            return new CollocationGrid(currentGrid.InitialTime, currentGrid.FinalTime, newSegments);
        }

        /// <summary>
        /// Interpolates a solution from a coarse grid to a fine grid.
        /// </summary>
        /// <param name="coarseTranscription">Transcription on coarse grid.</param>
        /// <param name="fineTranscription">Transcription on fine grid.</param>
        /// <param name="coarseSolution">Solution on coarse grid.</param>
        /// <param name="coarseGrid">Coarse collocation grid.</param>
        /// <param name="fineGrid">Fine collocation grid.</param>
        /// <returns>Initial guess on fine grid.</returns>
        public static double[] InterpolateSolution(
            HermiteSimpsonTranscription coarseTranscription,
            HermiteSimpsonTranscription fineTranscription,
            double[] coarseSolution,
            CollocationGrid coarseGrid,
            CollocationGrid fineGrid)
        {
            var fineSolution = new double[fineTranscription.DecisionVectorSize];

            // Interpolate each fine grid point
            for (var k = 0; k <= fineGrid.Segments; k++)
            {
                var t = fineGrid.TimePoints[k];

                // Find bracketing points in coarse grid
                var coarseIdx = 0;
                for (var i = 0; i < coarseGrid.Segments; i++)
                {
                    if (t >= coarseGrid.TimePoints[i] && t <= coarseGrid.TimePoints[i + 1])
                    {
                        coarseIdx = i;
                        break;
                    }
                }

                var t0 = coarseGrid.TimePoints[coarseIdx];
                var t1 = coarseGrid.TimePoints[coarseIdx + 1];
                var alpha = (t - t0) / (t1 - t0);

                // Linear interpolation of state and control
                var x0 = coarseTranscription.GetState(coarseSolution, coarseIdx);
                var x1 = coarseTranscription.GetState(coarseSolution, coarseIdx + 1);
                var u0 = coarseTranscription.GetControl(coarseSolution, coarseIdx);
                var u1 = coarseTranscription.GetControl(coarseSolution, coarseIdx + 1);

                var x = new double[x0.Length];
                var u = new double[u0.Length];

                for (var i = 0; i < x0.Length; i++)
                {
                    x[i] = (1 - alpha) * x0[i] + alpha * x1[i];
                }

                for (var i = 0; i < u0.Length; i++)
                {
                    u[i] = (1 - alpha) * u0[i] + alpha * u1[i];
                }

                fineTranscription.SetState(fineSolution, k, x);
                fineTranscription.SetControl(fineSolution, k, u);
            }

            return fineSolution;
        }

        /// <summary>
        /// Determines if the mesh refinement should terminate.
        /// </summary>
        /// <param name="maxDefect">Maximum defect in current solution.</param>
        /// <param name="currentSegments">Current number of segments.</param>
        /// <param name="iteration">Current refinement iteration.</param>
        /// <param name="maxIterations">Maximum refinement iterations.</param>
        /// <returns>True if refinement should stop.</returns>
        public bool ShouldTerminate(double maxDefect, int currentSegments, int iteration, int maxIterations)
        {
            // Stop if defects are small enough
            if (maxDefect < _defectThreshold)
            {
                return true;
            }

            // Stop if we've hit the segment limit
            if (currentSegments >= _maxSegments)
            {
                return true;
            }

            // Stop if we've done enough iterations
            if (iteration >= maxIterations)
            {
                return true;
            }

            return false;
        }

        /// <summary>
        /// Computes statistics about mesh refinement.
        /// </summary>
        /// <param name="shouldRefine">Array of refinement indicators.</param>
        /// <returns>Percentage of segments to be refined.</returns>
        public static double ComputeRefinementPercentage(bool[] shouldRefine)
        {
            var count = shouldRefine.Count(x => x);
            return 100.0 * count / shouldRefine.Length;
        }
    }
}
