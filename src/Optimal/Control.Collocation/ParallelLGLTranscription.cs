/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Core;
using System.Threading.Tasks;

namespace Optimal.Control.Collocation
{
    /// <summary>
    /// Parallel version of Legendre-Gauss-Lobatto (LGL) collocation transcription.
    /// Uses TPL parallelization for constraint and cost evaluation on larger problems.
    /// </summary>
    public sealed class ParallelLGLTranscription : ICollocationTranscription
    {
#pragma warning disable IDE0052 // Remove unread private members
        private readonly ControlProblem _problem;
#pragma warning restore IDE0052 // Remove unread private members
        private readonly CollocationGrid _grid;
        private readonly int _stateDim;
        private readonly int _controlDim;
        private readonly int _order;
        private readonly int _totalPoints;
        private readonly int _decisionVectorSize;
        private readonly double[] _lglPoints;
        private readonly double[] _lglWeights;
        private readonly double[,] _lglDiffMatrix;
        private readonly bool _enableParallelization;

        /// <summary>
        /// Gets the total size of the decision vector.
        /// Layout: [x_0, u_0, x_1, u_1, ..., x_M, u_M] where M = TotalPoints - 1
        /// </summary>
        public int DecisionVectorSize => _decisionVectorSize;

        /// <summary>
        /// Gets the number of collocation segments.
        /// </summary>
        public int Segments => _grid.Segments;

        /// <summary>
        /// Gets the LGL order (number of collocation points per segment).
        /// </summary>
        public int Order => _order;

        /// <summary>
        /// Gets the total number of unique collocation points.
        /// For N segments with order p: TotalPoints = N*(p-1) + 1
        /// </summary>
        public int TotalPoints => _totalPoints;

        /// <summary>
        /// Creates a parallel Legendre-Gauss-Lobatto transcription.
        /// </summary>
        /// <param name="problem">The control problem to transcribe.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="order">Number of LGL points per segment (must be >= 2).</param>
        /// <param name="enableParallelization">If true, use parallel computation for larger problems.</param>
        public ParallelLGLTranscription(ControlProblem problem, CollocationGrid grid, int order, bool enableParallelization)
        {
            _problem = problem ?? throw new ArgumentNullException(nameof(problem));
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));

            if (order < 2)
            {
                throw new ArgumentException("LGL order must be at least 2.", nameof(order));
            }

            _order = order;
            _stateDim = problem.StateDim;
            _controlDim = problem.ControlDim;
            _enableParallelization = enableParallelization;

            // Shared-endpoint layout: N segments with order p → N*(p-1) + 1 total points
            _totalPoints = grid.Segments * (order - 1) + 1;
            _decisionVectorSize = _totalPoints * (_stateDim + _controlDim);

            // Get LGL points, weights, and differentiation matrix
            (_lglPoints, _lglWeights) = LegendreGaussLobatto.GetPointsAndWeights(order);
            _lglDiffMatrix = LegendreGaussLobatto.GetDifferentiationMatrix(order);
        }

        /// <summary>
        /// Extracts state vector at a given global point index from the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="globalPointIndex">Global point index (0 to TotalPoints-1).</param>
        /// <returns>State vector at the point.</returns>
        public double[] GetState(double[] z, int globalPointIndex)
        {
            if (globalPointIndex < 0 || globalPointIndex >= _totalPoints)
            {
                throw new ArgumentOutOfRangeException(nameof(globalPointIndex));
            }

            var state = new double[_stateDim];
            var offset = globalPointIndex * (_stateDim + _controlDim);
            Array.Copy(z, offset, state, 0, _stateDim);
            return state;
        }

        /// <summary>
        /// Extracts control vector at a given global point index from the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="globalPointIndex">Global point index (0 to TotalPoints-1).</param>
        /// <returns>Control vector at the point.</returns>
        public double[] GetControl(double[] z, int globalPointIndex)
        {
            if (globalPointIndex < 0 || globalPointIndex >= _totalPoints)
            {
                throw new ArgumentOutOfRangeException(nameof(globalPointIndex));
            }

            var control = new double[_controlDim];
            var offset = globalPointIndex * (_stateDim + _controlDim) + _stateDim;
            Array.Copy(z, offset, control, 0, _controlDim);
            return control;
        }

        /// <summary>
        /// Sets state vector at a given global point index in the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="globalPointIndex">Global point index (0 to TotalPoints-1).</param>
        /// <param name="state">State vector to set.</param>
        public void SetState(double[] z, int globalPointIndex, double[] state)
        {
            if (globalPointIndex < 0 || globalPointIndex >= _totalPoints)
            {
                throw new ArgumentOutOfRangeException(nameof(globalPointIndex));
            }

            if (state.Length != _stateDim)
            {
                throw new ArgumentException($"State must have {_stateDim} elements.", nameof(state));
            }

            var offset = globalPointIndex * (_stateDim + _controlDim);
            Array.Copy(state, 0, z, offset, _stateDim);
        }

        /// <summary>
        /// Sets control vector at a given global point index in the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="globalPointIndex">Global point index (0 to TotalPoints-1).</param>
        /// <param name="control">Control vector to set.</param>
        public void SetControl(double[] z, int globalPointIndex, double[] control)
        {
            if (globalPointIndex < 0 || globalPointIndex >= _totalPoints)
            {
                throw new ArgumentOutOfRangeException(nameof(globalPointIndex));
            }

            if (control.Length != _controlDim)
            {
                throw new ArgumentException($"Control must have {_controlDim} elements.", nameof(control));
            }

            var offset = globalPointIndex * (_stateDim + _controlDim) + _stateDim;
            Array.Copy(control, 0, z, offset, _controlDim);
        }

        /// <summary>
        /// Converts a reference coordinate τ ∈ [-1, 1] to physical time in segment k.
        /// Formula: t = t_k + (τ + 1) * h / 2
        /// </summary>
        /// <param name="tau">Reference coordinate in [-1, 1].</param>
        /// <param name="segmentIndex">Segment index.</param>
        /// <returns>Physical time.</returns>
        private double TauToPhysicalTime(double tau, int segmentIndex)
        {
            var tk = _grid.TimePoints[segmentIndex];
            var h = _grid.GetTimeStep(segmentIndex);
            return tk + (tau + 1.0) * h / 2.0;
        }

        /// <summary>
        /// Gets the global point index for a specific LGL point within a segment.
        /// </summary>
        /// <param name="segmentIndex">Segment index (0 to Segments-1).</param>
        /// <param name="localPointIndex">Local LGL point index within segment (0 to Order-1).</param>
        /// <returns>Global point index in decision vector.</returns>
        private int GetGlobalPointIndex(int segmentIndex, int localPointIndex)
        {
            // Shared-endpoint layout: segment k uses global points from k*(order-1) to k*(order-1) + order
            return segmentIndex * (_order - 1) + localPointIndex;
        }

        /// <summary>
        /// Computes the defect constraint for a single segment at all interior LGL points.
        /// Defect at interior point i: (2/h) * Σ_j D_ij * x_j - f(x_i, u_i, t_i)
        /// Returns defects only at interior points (excluding endpoints).
        /// </summary>
        /// <param name="segmentIndex">Segment index.</param>
        /// <param name="z">Decision vector.</param>
        /// <param name="dynamicsEvaluator">Function to evaluate dynamics: f(x, u, t).</param>
        /// <returns>Defect vector for all interior points in the segment.</returns>
        private double[] ComputeSegmentDefects(
            int segmentIndex,
            double[] z,
            Func<double[], double[], double, double[]> dynamicsEvaluator)
        {
            var h = _grid.GetTimeStep(segmentIndex);
            var alpha = 2.0 / h; // Scaling factor for differentiation matrix

            // Number of interior points (excluding endpoints)
            var numInteriorPoints = _order - 2;
            var defects = new double[numInteriorPoints * _stateDim];

            // Extract states at all LGL points in this segment
            var states = new double[_order][];
            for (var j = 0; j < _order; j++)
            {
                var globalIdx = GetGlobalPointIndex(segmentIndex, j);
                states[j] = GetState(z, globalIdx);
            }

            // Compute defects at each interior point
            for (var i = 1; i < _order - 1; i++)
            {
                var globalIdx = GetGlobalPointIndex(segmentIndex, i);
                var xi = states[i];
                var ui = GetControl(z, globalIdx);
                var ti = TauToPhysicalTime(_lglPoints[i], segmentIndex);

                // Evaluate dynamics at this point
                var fi = dynamicsEvaluator(xi, ui, ti);

                // Compute derivative using differentiation matrix: dx/dt = α * Σ_j D_ij * x_j
                var dxdt = new double[_stateDim];
                for (var j = 0; j < _order; j++)
                {
                    var Dij = _lglDiffMatrix[i, j];
                    for (var s = 0; s < _stateDim; s++)
                    {
                        dxdt[s] += alpha * Dij * states[j][s];
                    }
                }

                // Defect: dx/dt - f(x, u, t)
                var defectStartIdx = (i - 1) * _stateDim; // i-1 because we skip first endpoint
                for (var s = 0; s < _stateDim; s++)
                {
                    defects[defectStartIdx + s] = dxdt[s] - fi[s];
                }
            }

            return defects;
        }

        /// <summary>
        /// Computes all defect constraints for the entire trajectory.
        /// Uses parallel computation for problems with 4 or more segments.
        /// Returns a flat array of all defects at all interior LGL points.
        /// Total defects: Segments * (Order - 2) * StateDim
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="dynamicsEvaluator">Function to evaluate dynamics: f(x, u, t).</param>
        /// <returns>Flat array of all defect constraints.</returns>
        public double[] ComputeAllDefects(double[] z, Func<double[], double[], double, double[]> dynamicsEvaluator)
        {
            var numInteriorPointsPerSegment = _order - 2;
            var totalDefects = _grid.Segments * numInteriorPointsPerSegment * _stateDim;
            var defects = new double[totalDefects];

            if (_enableParallelization && _grid.Segments >= 4)
            {
                // Parallel computation for larger problems
                Parallel.For(0, _grid.Segments, k =>
                {
                    var segmentDefects = ComputeSegmentDefects(k, z, dynamicsEvaluator);
                    var defectOffset = k * numInteriorPointsPerSegment * _stateDim;
                    Array.Copy(segmentDefects, 0, defects, defectOffset, segmentDefects.Length);
                });
            }
            else
            {
                // Sequential computation for small problems or when disabled
                for (var k = 0; k < _grid.Segments; k++)
                {
                    var segmentDefects = ComputeSegmentDefects(k, z, dynamicsEvaluator);
                    var defectOffset = k * numInteriorPointsPerSegment * _stateDim;
                    Array.Copy(segmentDefects, 0, defects, defectOffset, segmentDefects.Length);
                }
            }

            return defects;
        }

        /// <summary>
        /// Computes the maximum absolute defect value.
        /// </summary>
        /// <param name="defects">Defect vector.</param>
        /// <returns>Maximum absolute defect.</returns>
        public static double MaxDefect(double[] defects)
        {
            var max = 0.0;
            foreach (var d in defects)
            {
                var abs = Math.Abs(d);
                if (abs > max)
                {
                    max = abs;
                }
            }
            return max;
        }

        /// <summary>
        /// Creates an initial guess for the decision vector using linear interpolation with consistent controls.
        /// Controls are set to match the derivative of the state trajectory.
        /// </summary>
        /// <param name="initialState">Initial state.</param>
        /// <param name="finalState">Final state.</param>
        /// <param name="constantControl">Constant control value (used as fallback).</param>
        /// <returns>Initial decision vector.</returns>
        public double[] CreateInitialGuess(double[] initialState, double[] finalState, double[] constantControl)
        {
            var z = new double[_decisionVectorSize];
            var t0 = _grid.InitialTime;
            var tf = _grid.FinalTime;
            var duration = tf - t0;

            for (var k = 0; k < _grid.Segments; k++)
            {
                for (var i = 0; i < _order; i++)
                {
                    var globalIdx = GetGlobalPointIndex(k, i);

                    // Skip if this point was already set by previous segment (shared endpoint)
                    if (k > 0 && i == 0)
                    {
                        continue;
                    }

                    var t = TauToPhysicalTime(_lglPoints[i], k);
                    var alpha = (t - t0) / duration;
                    var state = new double[_stateDim];
                    var control = new double[_controlDim];

                    for (var s = 0; s < _stateDim; s++)
                    {
                        // Handle NaN in finalState (free/unspecified terminal condition)
                        if (double.IsNaN(finalState[s]))
                        {
                            state[s] = initialState[s];
                        }
                        else
                        {
                            // Linear interpolation
                            state[s] = (1.0 - alpha) * initialState[s] + alpha * finalState[s];
                        }
                    }

                    // Use constant control for all control dimensions
                    for (var c = 0; c < _controlDim; c++)
                    {
                        control[c] = constantControl[c];
                    }

                    SetState(z, globalIdx, state);
                    SetControl(z, globalIdx, control);
                }
            }

            return z;
        }

        /// <summary>
        /// Converts an InitialGuess to a decision vector.
        /// The InitialGuess must have exactly TotalPoints points.
        /// </summary>
        /// <param name="initialGuess">The initial guess containing state and control trajectories.</param>
        /// <returns>A decision vector suitable for the optimizer.</returns>
        public double[] ToDecisionVector(InitialGuess initialGuess)
        {
            ArgumentNullException.ThrowIfNull(initialGuess);

            var z = new double[_decisionVectorSize];

            for (var k = 0; k < _totalPoints; k++)
            {
                SetState(z, k, initialGuess.StateTrajectory[k]);
                SetControl(z, k, initialGuess.ControlTrajectory[k]);
            }

            return z;
        }

        /// <summary>
        /// Computes the running cost contribution for a single segment.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="segmentIndex">Segment index.</param>
        /// <param name="runningCostEvaluator">Running cost function: L(x, u, t).</param>
        /// <returns>Segment running cost contribution.</returns>
        private double ComputeSegmentRunningCost(
            double[] z,
            int segmentIndex,
            Func<double[], double[], double, double> runningCostEvaluator)
        {
            var h = _grid.GetTimeStep(segmentIndex);
            var segmentCost = 0.0;

            for (var i = 0; i < _order; i++)
            {
                var globalIdx = GetGlobalPointIndex(segmentIndex, i);
                var xi = GetState(z, globalIdx);
                var ui = GetControl(z, globalIdx);
                var ti = TauToPhysicalTime(_lglPoints[i], segmentIndex);

                var Li = runningCostEvaluator(xi, ui, ti);
                segmentCost += _lglWeights[i] * Li;
            }

            // LGL quadrature on [-1, 1] → scale by h/2 for physical interval
            return (h / 2.0) * segmentCost;
        }

        /// <summary>
        /// Computes the running cost (Lagrange term) integrated over the trajectory using LGL quadrature.
        /// Uses parallel computation for problems with 10 or more segments.
        /// J_running = ∫[t0, tf] L(x(t), u(t), t) dt
        /// Approximated as: Σ_k (h/2) * Σ_i w_i * L(x_i^k, u_i^k, t_i^k)
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="runningCostEvaluator">Running cost function: L(x, u, t).</param>
        /// <returns>Integrated running cost.</returns>
        public double ComputeRunningCost(double[] z, Func<double[], double[], double, double> runningCostEvaluator)
        {
            if (_enableParallelization && _grid.Segments >= 10)
            {
                // Parallel cost computation for larger problems
                var segmentCosts = new double[_grid.Segments];

                Parallel.For(0, _grid.Segments, k =>
                {
                    segmentCosts[k] = ComputeSegmentRunningCost(z, k, runningCostEvaluator);
                });

                var totalCost = 0.0;
                foreach (var cost in segmentCosts)
                {
                    totalCost += cost;
                }
                return totalCost;
            }
            else
            {
                // Sequential computation for small problems or when disabled
                var totalCost = 0.0;
                for (var k = 0; k < _grid.Segments; k++)
                {
                    totalCost += ComputeSegmentRunningCost(z, k, runningCostEvaluator);
                }
                return totalCost;
            }
        }

        /// <summary>
        /// Computes the terminal cost (Mayer term) at the final time.
        /// J_terminal = Φ(x(tf), tf)
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="terminalCostEvaluator">Terminal cost function: Φ(x, t).</param>
        /// <returns>Terminal cost.</returns>
        public double ComputeTerminalCost(double[] z, Func<double[], double, double> terminalCostEvaluator)
        {
            var xf = GetState(z, _totalPoints - 1);
            var tf = _grid.FinalTime;
            return terminalCostEvaluator(xf, tf);
        }

        /// <summary>
        /// Computes the total objective function value (Lagrange + Mayer).
        /// J = Φ(x(tf), tf) + ∫[t0, tf] L(x(t), u(t), t) dt
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="runningCostEvaluator">Running cost function (null to skip).</param>
        /// <param name="terminalCostEvaluator">Terminal cost function (null to skip).</param>
        /// <returns>Total objective value.</returns>
        public double ComputeTotalCost(
            double[] z,
            Func<double[], double[], double, double>? runningCostEvaluator,
            Func<double[], double, double>? terminalCostEvaluator)
        {
            var cost = 0.0;

            if (runningCostEvaluator != null)
            {
                cost += ComputeRunningCost(z, runningCostEvaluator);
            }

            if (terminalCostEvaluator != null)
            {
                cost += ComputeTerminalCost(z, terminalCostEvaluator);
            }

            return cost;
        }
    }
}
