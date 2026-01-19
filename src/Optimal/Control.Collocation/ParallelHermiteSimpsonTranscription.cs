/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Threading.Tasks;
using Optimal.Control.Core;

namespace Optimal.Control.Collocation
{
    /// <summary>
    /// Parallel version of Hermite-Simpson transcription with optimized constraint and gradient evaluation.
    /// </summary>
    public sealed class ParallelHermiteSimpsonTranscription : ICollocationTranscription
    {
#pragma warning disable IDE0052 // Remove unread private members
        private readonly ControlProblem _problem;
#pragma warning restore IDE0052 // Remove unread private members
        private readonly CollocationGrid _grid;
        private readonly int _stateDim;
        private readonly int _controlDim;
        private readonly int _decisionVectorSize;
        private readonly bool _enableParallelization;

        /// <summary>
        /// Gets the total size of the decision vector.
        /// </summary>
        public int DecisionVectorSize => _decisionVectorSize;

        /// <summary>
        /// Gets the number of collocation segments.
        /// </summary>
        public int Segments => _grid.Segments;

        /// <summary>
        /// Gets the total number of unique collocation points.
        /// For Hermite-Simpson, this is Segments + 1 (the node points).
        /// </summary>
        public int TotalPoints => _grid.Segments + 1;

        /// <summary>
        /// Gets the order of the transcription method.
        /// For Hermite-Simpson, this is 2 (two endpoints per segment).
        /// </summary>
        public int Order => 2;

        /// <summary>
        /// Creates a parallel transcription.
        /// </summary>
        /// <param name="problem">The control problem to transcribe.</param>
        /// <param name="grid">The collocation grid.</param>
        /// <param name="enableParallelization">Enable parallel computation (default: true).</param>
        public ParallelHermiteSimpsonTranscription(ControlProblem problem, CollocationGrid grid, bool enableParallelization = true)
        {
            _problem = problem ?? throw new ArgumentNullException(nameof(problem));
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));
            _stateDim = problem.StateDim;
            _controlDim = problem.ControlDim;
            _decisionVectorSize = (grid.Segments + 1) * (_stateDim + _controlDim);
            _enableParallelization = enableParallelization;
        }

        /// <summary>
        /// Extracts state vector at a given node from the decision vector.
        /// </summary>
        public double[] GetState(double[] z, int globalPointIndex)
        {
            if (globalPointIndex < 0 || globalPointIndex > _grid.Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(globalPointIndex));
            }

            var state = new double[_stateDim];
            var offset = globalPointIndex * (_stateDim + _controlDim);
            Array.Copy(z, offset, state, 0, _stateDim);
            return state;
        }

        /// <summary>
        /// Extracts control vector at a given node from the decision vector.
        /// </summary>
        public double[] GetControl(double[] z, int globalPointIndex)
        {
            if (globalPointIndex < 0 || globalPointIndex > _grid.Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(globalPointIndex));
            }

            var control = new double[_controlDim];
            var offset = globalPointIndex * (_stateDim + _controlDim) + _stateDim;
            Array.Copy(z, offset, control, 0, _controlDim);
            return control;
        }

        /// <summary>
        /// Sets state vector at a given node in the decision vector.
        /// </summary>
        public void SetState(double[] z, int globalPointIndex, double[] state)
        {
            if (globalPointIndex < 0 || globalPointIndex > _grid.Segments)
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
        /// Sets control vector at a given node in the decision vector.
        /// </summary>
        public void SetControl(double[] z, int globalPointIndex, double[] control)
        {
            if (globalPointIndex < 0 || globalPointIndex > _grid.Segments)
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
        /// Computes all defect constraints with optional parallelization.
        /// </summary>
        public double[] ComputeAllDefects(double[] z, Func<DynamicsInput, DynamicsResult> dynamicsEvaluator)
        {
            var totalDefects = _grid.Segments * _stateDim;
            var defects = new double[totalDefects];

            if (_enableParallelization && _grid.Segments >= 4)
            {
                // Parallel computation for larger problems
                Parallel.For(0, _grid.Segments, k =>
                {
                    var segmentDefect = ComputeSegmentDefect(z, k, dynamicsEvaluator);
                    Array.Copy(segmentDefect, 0, defects, k * _stateDim, _stateDim);
                });
            }
            else
            {
                // Sequential computation for small problems or when disabled
                for (var k = 0; k < _grid.Segments; k++)
                {
                    var segmentDefect = ComputeSegmentDefect(z, k, dynamicsEvaluator);
                    Array.Copy(segmentDefect, 0, defects, k * _stateDim, _stateDim);
                }
            }

            return defects;
        }

        /// <summary>
        /// Computes defect constraint for a single segment.
        /// </summary>
        private double[] ComputeSegmentDefect(double[] z, int k, Func<DynamicsInput, DynamicsResult> dynamicsEvaluator)
        {
            var tk = _grid.TimePoints[k];
            var tk1 = _grid.TimePoints[k + 1];
            var tMid = _grid.GetMidpoint(k);
            var h = _grid.GetTimeStep(k);

            var xk = GetState(z, k);
            var xk1 = GetState(z, k + 1);
            var uk = GetControl(z, k);
            var uk1 = GetControl(z, k + 1);

            var fk = dynamicsEvaluator(new DynamicsInput(xk, uk, tk, k, _grid.Segments)).Value;
            var fk1 = dynamicsEvaluator(new DynamicsInput(xk1, uk1, tk1, k + 1, _grid.Segments)).Value;

            var xMid = HermiteInterpolation(xk, xk1, fk, fk1, h);
            var uMid = ControlMidpoint(uk, uk1);
            var fMid = dynamicsEvaluator(new DynamicsInput(xMid, uMid, tMid, k + 0.5, _grid.Segments)).Value;
            return ComputeDefect(xk, xk1, fk, fMid, fk1, h);
        }

        /// <summary>
        /// Computes numerical gradient of defects with respect to decision variables using finite differences.
        /// Returns sparse Jacobian structure optimized for parallel computation.
        /// </summary>
        public double[,] ComputeDefectJacobian(double[] z, Func<DynamicsInput, DynamicsResult> dynamicsEvaluator, double epsilon = 1e-8)
        {
            var numConstraints = _grid.Segments * _stateDim;
            var jacobian = new double[numConstraints, _decisionVectorSize];

            if (_enableParallelization && _decisionVectorSize >= 20)
            {
                // Parallel gradient computation
                Parallel.For(0, _decisionVectorSize, j =>
                {
                    var zPerturbed = new double[_decisionVectorSize];
                    Array.Copy(z, zPerturbed, _decisionVectorSize);
                    zPerturbed[j] += epsilon;

                    var defectsOriginal = ComputeAllDefects(z, dynamicsEvaluator);
                    var defectsPerturbed = ComputeAllDefects(zPerturbed, dynamicsEvaluator);

                    for (var i = 0; i < numConstraints; i++)
                    {
                        jacobian[i, j] = (defectsPerturbed[i] - defectsOriginal[i]) / epsilon;
                    }
                });
            }
            else
            {
                // Sequential computation
                for (var j = 0; j < _decisionVectorSize; j++)
                {
                    var zPerturbed = new double[_decisionVectorSize];
                    Array.Copy(z, zPerturbed, _decisionVectorSize);
                    zPerturbed[j] += epsilon;

                    var defectsOriginal = ComputeAllDefects(z, dynamicsEvaluator);
                    var defectsPerturbed = ComputeAllDefects(zPerturbed, dynamicsEvaluator);

                    for (var i = 0; i < numConstraints; i++)
                    {
                        jacobian[i, j] = (defectsPerturbed[i] - defectsOriginal[i]) / epsilon;
                    }
                }
            }

            return jacobian;
        }

        /// <summary>
        /// Computes numerical gradient of objective function with optional parallelization.
        /// </summary>
        public double[] ComputeObjectiveGradient(
            double[] z,
            Func<double[], double[], double, double>? runningCostEvaluator,
            Func<double[], double, double>? terminalCostEvaluator,
            double epsilon = 1e-8)
        {
            var gradient = new double[_decisionVectorSize];

            if (_enableParallelization && _decisionVectorSize >= 20)
            {
                // Parallel gradient computation
                Parallel.For(0, _decisionVectorSize, j =>
                {
                    var zPerturbed = new double[_decisionVectorSize];
                    Array.Copy(z, zPerturbed, _decisionVectorSize);
                    zPerturbed[j] += epsilon;

                    var costOriginal = ComputeTotalCost(z, runningCostEvaluator, terminalCostEvaluator);
                    var costPerturbed = ComputeTotalCost(zPerturbed, runningCostEvaluator, terminalCostEvaluator);

                    gradient[j] = (costPerturbed - costOriginal) / epsilon;
                });
            }
            else
            {
                // Sequential computation
                for (var j = 0; j < _decisionVectorSize; j++)
                {
                    var zPerturbed = new double[_decisionVectorSize];
                    Array.Copy(z, zPerturbed, _decisionVectorSize);
                    zPerturbed[j] += epsilon;

                    var costOriginal = ComputeTotalCost(z, runningCostEvaluator, terminalCostEvaluator);
                    var costPerturbed = ComputeTotalCost(zPerturbed, runningCostEvaluator, terminalCostEvaluator);

                    gradient[j] = (costPerturbed - costOriginal) / epsilon;
                }
            }

            return gradient;
        }

        /// <summary>
        /// Computes the running cost using Simpson's rule with optional parallelization.
        /// </summary>
        public double ComputeRunningCost(double[] z, Func<double[], double[], double, double> runningCostEvaluator)
        {
            if (_enableParallelization && _grid.Segments >= 10)
            {
                // Parallel cost computation
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
                // Sequential computation
                var totalCost = 0.0;
                for (var k = 0; k < _grid.Segments; k++)
                {
                    totalCost += ComputeSegmentRunningCost(z, k, runningCostEvaluator);
                }
                return totalCost;
            }
        }

        /// <summary>
        /// Computes running cost for a single segment.
        /// </summary>
        private double ComputeSegmentRunningCost(double[] z, int k, Func<double[], double[], double, double> runningCostEvaluator)
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

            return (h / 6.0) * (Lk + 4.0 * LMid + Lk1);
        }

        /// <summary>
        /// Computes the terminal cost.
        /// </summary>
        public double ComputeTerminalCost(double[] z, Func<double[], double, double> terminalCostEvaluator)
        {
            var xf = GetState(z, _grid.Segments);
            var tf = _grid.FinalTime;
            return terminalCostEvaluator(xf, tf);
        }

        /// <summary>
        /// Computes the total objective function value.
        /// </summary>
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

        /// <summary>
        /// Creates an initial guess for the decision vector.
        /// </summary>
        public double[] CreateInitialGuess(double[] initialState, double[] finalState, double[] constantControl)
        {
            var z = new double[_decisionVectorSize];

            for (var k = 0; k <= _grid.Segments; k++)
            {
                var alpha = (double)k / _grid.Segments;
                var state = new double[_stateDim];

                for (var i = 0; i < _stateDim; i++)
                {
                    var x0 = initialState[i];
                    var xf = finalState[i];
                    var x0IsNaN = double.IsNaN(x0);
                    var xfIsNaN = double.IsNaN(xf);

                    if (x0IsNaN && xfIsNaN)
                    {
                        // Both free - use 0 as default
                        state[i] = 0.0;
                    }
                    else if (x0IsNaN)
                    {
                        // Initial free - use final value
                        state[i] = xf;
                    }
                    else if (xfIsNaN)
                    {
                        // Final free - use initial value
                        state[i] = x0;
                    }
                    else
                    {
                        // Linear interpolation
                        state[i] = (1.0 - alpha) * x0 + alpha * xf;
                    }
                }

                SetState(z, k, state);
                SetControl(z, k, constantControl);
            }

            return z;
        }

        /// <summary>
        /// Converts an InitialGuess to a decision vector.
        /// </summary>
        /// <param name="initialGuess">The initial guess containing state and control trajectories.</param>
        /// <returns>A decision vector suitable for the optimizer.</returns>
        public double[] ToDecisionVector(InitialGuess initialGuess)
        {
            ArgumentNullException.ThrowIfNull(initialGuess);

            var z = new double[_decisionVectorSize];

            for (var k = 0; k <= _grid.Segments; k++)
            {
                SetState(z, k, initialGuess.StateTrajectory[k]);
                SetControl(z, k, initialGuess.ControlTrajectory[k]);
            }

            return z;
        }

        // Static helper methods
        private static double[] HermiteInterpolation(double[] xk, double[] xk1, double[] fk, double[] fk1, double h)
        {
            var n = xk.Length;
            var xMid = new double[n];

            for (var i = 0; i < n; i++)
            {
                xMid[i] = 0.5 * (xk[i] + xk1[i]) + (h / 8.0) * (fk[i] - fk1[i]);
            }

            return xMid;
        }

        private static double[] ControlMidpoint(double[] uk, double[] uk1)
        {
            var n = uk.Length;
            var uMid = new double[n];

            for (var i = 0; i < n; i++)
            {
                uMid[i] = 0.5 * (uk[i] + uk1[i]);
            }

            return uMid;
        }

        private static double[] ComputeDefect(double[] xk, double[] xk1, double[] fk, double[] fMid, double[] fk1, double h)
        {
            var n = xk.Length;
            var defect = new double[n];

            for (var i = 0; i < n; i++)
            {
                defect[i] = xk1[i] - xk[i] - (h / 6.0) * (fk[i] + 4.0 * fMid[i] + fk1[i]);
            }

            return defect;
        }
    }
}
