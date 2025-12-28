/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control
{
    /// <summary>
    /// Hermite-Simpson collocation transcription for optimal control problems.
    /// Converts a continuous-time ODE into a discrete NLP via collocation.
    /// </summary>
    public sealed class HermiteSimpsonTranscription
    {
#pragma warning disable IDE0052 // Remove unread private members
        private readonly ControlProblem _problem;
#pragma warning restore IDE0052 // Remove unread private members
        private readonly CollocationGrid _grid;
        private readonly int _stateDim;
        private readonly int _controlDim;
        private readonly int _decisionVectorSize;

        /// <summary>
        /// Gets the total size of the decision vector.
        /// Layout: [x0, u0, x1, u1, ..., xN, uN]
        /// </summary>
        public int DecisionVectorSize => _decisionVectorSize;

        /// <summary>
        /// Gets the number of collocation segments.
        /// </summary>
        public int Segments => _grid.Segments;

        /// <summary>
        /// Creates a Hermite-Simpson transcription.
        /// </summary>
        /// <param name="problem">The control problem to transcribe.</param>
        /// <param name="grid">The collocation grid.</param>
        public HermiteSimpsonTranscription(ControlProblem problem, CollocationGrid grid)
        {
            _problem = problem ?? throw new ArgumentNullException(nameof(problem));
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));
            _stateDim = problem.StateDim;
            _controlDim = problem.ControlDim;
            _decisionVectorSize = (grid.Segments + 1) * (_stateDim + _controlDim);
        }

        /// <summary>
        /// Extracts state vector at a given node from the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="nodeIndex">Node index (0 to Segments).</param>
        /// <returns>State vector at the node.</returns>
        public double[] GetState(double[] z, int nodeIndex)
        {
            if (nodeIndex < 0 || nodeIndex > _grid.Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(nodeIndex));
            }

            var state = new double[_stateDim];
            var offset = nodeIndex * (_stateDim + _controlDim);
            Array.Copy(z, offset, state, 0, _stateDim);
            return state;
        }

        /// <summary>
        /// Extracts control vector at a given node from the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="nodeIndex">Node index (0 to Segments).</param>
        /// <returns>Control vector at the node.</returns>
        public double[] GetControl(double[] z, int nodeIndex)
        {
            if (nodeIndex < 0 || nodeIndex > _grid.Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(nodeIndex));
            }

            var control = new double[_controlDim];
            var offset = nodeIndex * (_stateDim + _controlDim) + _stateDim;
            Array.Copy(z, offset, control, 0, _controlDim);
            return control;
        }

        /// <summary>
        /// Sets state vector at a given node in the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="nodeIndex">Node index (0 to Segments).</param>
        /// <param name="state">State vector to set.</param>
        public void SetState(double[] z, int nodeIndex, double[] state)
        {
            if (nodeIndex < 0 || nodeIndex > _grid.Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(nodeIndex));
            }

            if (state.Length != _stateDim)
            {
                throw new ArgumentException($"State must have {_stateDim} elements.", nameof(state));
            }

            var offset = nodeIndex * (_stateDim + _controlDim);
            Array.Copy(state, 0, z, offset, _stateDim);
        }

        /// <summary>
        /// Sets control vector at a given node in the decision vector.
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="nodeIndex">Node index (0 to Segments).</param>
        /// <param name="control">Control vector to set.</param>
        public void SetControl(double[] z, int nodeIndex, double[] control)
        {
            if (nodeIndex < 0 || nodeIndex > _grid.Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(nodeIndex));
            }

            if (control.Length != _controlDim)
            {
                throw new ArgumentException($"Control must have {_controlDim} elements.", nameof(control));
            }

            var offset = nodeIndex * (_stateDim + _controlDim) + _stateDim;
            Array.Copy(control, 0, z, offset, _controlDim);
        }

        /// <summary>
        /// Computes the Hermite interpolation for the state at the midpoint of a segment.
        /// Formula: x_mid = (x_k + x_{k+1})/2 + h/8 * (f_k - f_{k+1})
        /// </summary>
        /// <param name="xk">State at node k.</param>
        /// <param name="xk1">State at node k+1.</param>
        /// <param name="fk">Dynamics at node k: f(x_k, u_k, t_k).</param>
        /// <param name="fk1">Dynamics at node k+1: f(x_{k+1}, u_{k+1}, t_{k+1}).</param>
        /// <param name="h">Time step.</param>
        /// <returns>Interpolated state at midpoint.</returns>
        public static double[] HermiteInterpolation(double[] xk, double[] xk1, double[] fk, double[] fk1, double h)
        {
            var n = xk.Length;
            var xMid = new double[n];

            for (var i = 0; i < n; i++)
            {
                xMid[i] = 0.5 * (xk[i] + xk1[i]) + (h / 8.0) * (fk[i] - fk1[i]);
            }

            return xMid;
        }

        /// <summary>
        /// Computes the control at the midpoint (simple average).
        /// Formula: u_mid = (u_k + u_{k+1})/2
        /// </summary>
        /// <param name="uk">Control at node k.</param>
        /// <param name="uk1">Control at node k+1.</param>
        /// <returns>Interpolated control at midpoint.</returns>
        public static double[] ControlMidpoint(double[] uk, double[] uk1)
        {
            var n = uk.Length;
            var uMid = new double[n];

            for (var i = 0; i < n; i++)
            {
                uMid[i] = 0.5 * (uk[i] + uk1[i]);
            }

            return uMid;
        }

        /// <summary>
        /// Computes the defect constraint for a single segment using Hermite-Simpson.
        /// Defect = x_{k+1} - x_k - h/6 * (f_k + 4*f_mid + f_{k+1})
        /// </summary>
        /// <param name="xk">State at node k.</param>
        /// <param name="xk1">State at node k+1.</param>
        /// <param name="fk">Dynamics at node k.</param>
        /// <param name="fMid">Dynamics at midpoint.</param>
        /// <param name="fk1">Dynamics at node k+1.</param>
        /// <param name="h">Time step.</param>
        /// <returns>Defect vector (should be zero for feasible trajectory).</returns>
        public static double[] ComputeDefect(double[] xk, double[] xk1, double[] fk, double[] fMid, double[] fk1, double h)
        {
            var n = xk.Length;
            var defect = new double[n];

            for (var i = 0; i < n; i++)
            {
                defect[i] = xk1[i] - xk[i] - (h / 6.0) * (fk[i] + 4.0 * fMid[i] + fk1[i]);
            }

            return defect;
        }

        /// <summary>
        /// Computes all defect constraints for the entire trajectory.
        /// Returns a flat array of all defects: [defect_0, defect_1, ..., defect_{N-1}]
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="dynamicsEvaluator">Function to evaluate dynamics: f(x, u, t).</param>
        /// <returns>Flat array of all defect constraints.</returns>
        public double[] ComputeAllDefects(double[] z, Func<double[], double[], double, double[]> dynamicsEvaluator)
        {
            var totalDefects = _grid.Segments * _stateDim;
            var defects = new double[totalDefects];

            for (var k = 0; k < _grid.Segments; k++)
            {
                var tk = _grid.TimePoints[k];
                var tk1 = _grid.TimePoints[k + 1];
                var tMid = _grid.GetMidpoint(k);
                var h = _grid.GetTimeStep(k);

                var xk = GetState(z, k);
                var xk1 = GetState(z, k + 1);
                var uk = GetControl(z, k);
                var uk1 = GetControl(z, k + 1);

                var fk = dynamicsEvaluator(xk, uk, tk);
                var fk1 = dynamicsEvaluator(xk1, uk1, tk1);

                var xMid = HermiteInterpolation(xk, xk1, fk, fk1, h);
                var uMid = ControlMidpoint(uk, uk1);
                var fMid = dynamicsEvaluator(xMid, uMid, tMid);

                var segmentDefect = ComputeDefect(xk, xk1, fk, fMid, fk1, h);

                Array.Copy(segmentDefect, 0, defects, k * _stateDim, _stateDim);
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
        /// Creates an initial guess for the decision vector using linear interpolation.
        /// </summary>
        /// <param name="initialState">Initial state.</param>
        /// <param name="finalState">Final state.</param>
        /// <param name="constantControl">Constant control value.</param>
        /// <returns>Initial decision vector.</returns>
        public double[] CreateInitialGuess(double[] initialState, double[] finalState, double[] constantControl)
        {
            var z = new double[_decisionVectorSize];

            for (var k = 0; k <= _grid.Segments; k++)
            {
                var alpha = (double)k / _grid.Segments;
                var state = new double[_stateDim];

                for (var i = 0; i < _stateDim; i++)
                {
                    // Handle NaN in finalState (free/unspecified terminal condition)
                    if (double.IsNaN(finalState[i]))
                    {
                        state[i] = initialState[i];
                    }
                    else
                    {
                        state[i] = (1.0 - alpha) * initialState[i] + alpha * finalState[i];
                    }
                }

                SetState(z, k, state);
                SetControl(z, k, constantControl);
            }

            return z;
        }

        /// <summary>
        /// Computes the running cost (Lagrange term) integrated over the trajectory using Simpson's rule.
        /// J_running = ∫[t0, tf] L(x(t), u(t), t) dt
        /// Approximated as: Σ_k h/6 * (L_k + 4*L_mid + L_{k+1})
        /// </summary>
        /// <param name="z">Decision vector.</param>
        /// <param name="runningCostEvaluator">Running cost function: L(x, u, t).</param>
        /// <returns>Integrated running cost.</returns>
        public double ComputeRunningCost(double[] z, Func<double[], double[], double, double> runningCostEvaluator)
        {
            var totalCost = 0.0;

            for (var k = 0; k < _grid.Segments; k++)
            {
                var tk = _grid.TimePoints[k];
                var tk1 = _grid.TimePoints[k + 1];
                var tMid = _grid.GetMidpoint(k);
                var h = _grid.GetTimeStep(k);

                var xk = GetState(z, k);
                var xk1 = GetState(z, k + 1);
                var uk = GetControl(z, k);
                var uk1 = GetControl(z, k + 1);

                var Lk = runningCostEvaluator(xk, uk, tk);
                var Lk1 = runningCostEvaluator(xk1, uk1, tk1);

                // For midpoint evaluation, need dynamics to compute x_mid
                // We'll use a simpler approach: evaluate at (x_mid, u_mid) directly
                var xMid = new double[_stateDim];
                var uMid = new double[_controlDim];

                for (var i = 0; i < _stateDim; i++)
                {
                    xMid[i] = 0.5 * (xk[i] + xk1[i]);
                }

                for (var i = 0; i < _controlDim; i++)
                {
                    uMid[i] = 0.5 * (uk[i] + uk1[i]);
                }

                var LMid = runningCostEvaluator(xMid, uMid, tMid);

                // Simpson's rule: h/6 * (L_k + 4*L_mid + L_{k+1})
                totalCost += (h / 6.0) * (Lk + 4.0 * LMid + Lk1);
            }

            return totalCost;
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
            var xf = GetState(z, _grid.Segments);
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
