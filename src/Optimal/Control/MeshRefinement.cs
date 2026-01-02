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
        /// Identifies which segments should be refined based on defect distribution for LGL collocation.
        /// For LGL methods, each segment has multiple interior points with defects.
        /// </summary>
        /// <param name="defects">Flat array of all defects.</param>
        /// <param name="stateDim">State dimension.</param>
        /// <param name="numInteriorPointsPerSegment">Number of interior collocation points per segment.</param>
        /// <returns>Boolean array indicating which segments should be refined.</returns>
        public bool[] IdentifySegmentsForRefinement(double[] defects, int stateDim, int numInteriorPointsPerSegment)
        {
            var defectsPerSegment = numInteriorPointsPerSegment * stateDim;
            var numSegments = defects.Length / defectsPerSegment;
            var shouldRefine = new bool[numSegments];

            for (var k = 0; k < numSegments; k++)
            {
                // Compute max defect magnitude across all interior points in this segment
                var maxDefect = 0.0;
                for (var i = 0; i < defectsPerSegment; i++)
                {
                    var defectIdx = k * defectsPerSegment + i;
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

        /// <summary>
        /// Interpolates a solution from a coarse LGL grid to a fine LGL grid using Lagrange interpolation.
        /// </summary>
        /// <param name="coarseGrid">Coarse collocation grid.</param>
        /// <param name="fineGrid">Fine collocation grid.</param>
        /// <param name="coarseSolution">Solution vector on coarse grid.</param>
        /// <param name="order">LGL collocation order.</param>
        /// <param name="stateDim">State dimension.</param>
        /// <param name="controlDim">Control dimension.</param>
        /// <returns>Interpolated solution vector on fine grid.</returns>
        public static double[] InterpolateLGLSolution(
            CollocationGrid coarseGrid,
            CollocationGrid fineGrid,
            double[] coarseSolution,
            int order,
            int stateDim,
            int controlDim)
        {
            // Get LGL points in reference coordinates [-1, 1]
            var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(order);

            // Calculate total points for shared-endpoint layout
            var coarseTotalPoints = coarseGrid.Segments * (order - 1) + 1;
            var fineTotalPoints = fineGrid.Segments * (order - 1) + 1;

            var fineSolution = new double[fineTotalPoints * (stateDim + controlDim)];

            // Helper to get global point index
            int GetGlobalPointIndex(int segmentIndex, int localPointIndex)
            {
                return segmentIndex * (order - 1) + localPointIndex;
            }

            // Helper to extract state from solution vector
            double[] GetState(double[] solution, int globalPointIndex)
            {
                var state = new double[stateDim];
                var offset = globalPointIndex * (stateDim + controlDim);
                Array.Copy(solution, offset, state, 0, stateDim);
                return state;
            }

            // Helper to extract control from solution vector
            double[] GetControl(double[] solution, int globalPointIndex)
            {
                var control = new double[controlDim];
                var offset = globalPointIndex * (stateDim + controlDim) + stateDim;
                Array.Copy(solution, offset, control, 0, controlDim);
                return control;
            }

            // Helper to set state in solution vector
            void SetState(double[] solution, int globalPointIndex, double[] state)
            {
                var offset = globalPointIndex * (stateDim + controlDim);
                Array.Copy(state, 0, solution, offset, stateDim);
            }

            // Helper to set control in solution vector
            void SetControl(double[] solution, int globalPointIndex, double[] control)
            {
                var offset = globalPointIndex * (stateDim + controlDim) + stateDim;
                Array.Copy(control, 0, solution, offset, controlDim);
            }

            // Interpolate each point in the fine grid
            for (var kFine = 0; kFine < fineGrid.Segments; kFine++)
            {
                for (var iFine = 0; iFine < order; iFine++)
                {
                    var globalIdxFine = GetGlobalPointIndex(kFine, iFine);

                    // Skip if already set by previous segment (shared endpoint)
                    if (kFine > 0 && iFine == 0)
                    {
                        continue;
                    }

                    // Get physical time for this fine grid point
                    var tauFine = lglPoints[iFine];
                    var hFine = fineGrid.GetTimeStep(kFine);
                    var tFine = fineGrid.TimePoints[kFine] + (tauFine + 1.0) * hFine / 2.0;

                    // Find coarse segment containing this time
                    var kCoarse = 0;
                    for (var k = 0; k < coarseGrid.Segments; k++)
                    {
                        if (tFine >= coarseGrid.TimePoints[k] && tFine <= coarseGrid.TimePoints[k + 1])
                        {
                            kCoarse = k;
                            break;
                        }
                    }

                    // Convert physical time to reference coordinate in coarse segment
                    var hCoarse = coarseGrid.GetTimeStep(kCoarse);
                    var tauCoarse = 2.0 * (tFine - coarseGrid.TimePoints[kCoarse]) / hCoarse - 1.0;

                    // Lagrange interpolation from coarse LGL points to this fine point
                    var state = new double[stateDim];
                    var control = new double[controlDim];

                    for (var jCoarse = 0; jCoarse < order; jCoarse++)
                    {
                        var globalIdxCoarse = GetGlobalPointIndex(kCoarse, jCoarse);

                        // Compute Lagrange basis function value at tauCoarse
                        var basis = 1.0;
                        for (var mCoarse = 0; mCoarse < order; mCoarse++)
                        {
                            if (mCoarse != jCoarse)
                            {
                                basis *= (tauCoarse - lglPoints[mCoarse]) / (lglPoints[jCoarse] - lglPoints[mCoarse]);
                            }
                        }

                        // Accumulate contributions
                        var stateCoarse = GetState(coarseSolution, globalIdxCoarse);
                        var controlCoarse = GetControl(coarseSolution, globalIdxCoarse);

                        for (var s = 0; s < stateDim; s++)
                        {
                            state[s] += basis * stateCoarse[s];
                        }

                        for (var c = 0; c < controlDim; c++)
                        {
                            control[c] += basis * controlCoarse[c];
                        }
                    }

                    SetState(fineSolution, globalIdxFine, state);
                    SetControl(fineSolution, globalIdxFine, control);
                }
            }

            return fineSolution;
        }
    }
}
