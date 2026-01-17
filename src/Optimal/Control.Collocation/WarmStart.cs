/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Core;

namespace Optimal.Control.Collocation
{
    /// <summary>
    /// Provides warm starting capabilities for optimal control problems.
    /// Allows reusing solutions from similar problems to improve convergence.
    /// </summary>
    public sealed class WarmStart
    {
        /// <summary>
        /// Creates an initial guess by interpolating from a previous solution.
        /// Handles different grid sizes between previous and current problem.
        /// </summary>
        /// <param name="previousResult">Previous solution result.</param>
        /// <param name="newGrid">New collocation grid.</param>
        /// <returns>Initial guess for new problem.</returns>
        public static InitialGuess InterpolateFromPrevious(
            CollocationResult previousResult,
            CollocationGrid newGrid)
        {
            ArgumentNullException.ThrowIfNull(previousResult);
            ArgumentNullException.ThrowIfNull(newGrid);

            var numPoints = newGrid.Segments + 1;
            var stateTrajectory = new double[numPoints][];
            var controlTrajectory = new double[numPoints][];

            for (var k = 0; k <= newGrid.Segments; k++)
            {
                var t = newGrid.TimePoints[k];
                var (state, control) = InterpolateAtTime(previousResult, t);
                stateTrajectory[k] = state;
                controlTrajectory[k] = control;
            }

            return new InitialGuess(stateTrajectory, controlTrajectory);
        }

        /// <summary>
        /// Creates an initial guess by interpolating from a previous solution for LGL solvers.
        /// </summary>
        /// <param name="previousResult">Previous solution result.</param>
        /// <param name="newGrid">New collocation grid.</param>
        /// <param name="order">LGL order.</param>
        /// <returns>Initial guess for new problem.</returns>
        public static InitialGuess InterpolateFromPreviousForLGL(
            CollocationResult previousResult,
            CollocationGrid newGrid,
            int order)
        {
            ArgumentNullException.ThrowIfNull(previousResult);
            ArgumentNullException.ThrowIfNull(newGrid);

            var numPoints = (order - 1) * newGrid.Segments + 1;
            var stateTrajectory = new double[numPoints][];
            var controlTrajectory = new double[numPoints][];

            // Generate LGL time points
            var (lglPoints, _) = LegendreGaussLobatto.GetPointsAndWeights(order);
            var pointIndex = 0;

            for (var k = 0; k < newGrid.Segments; k++)
            {
                var tk = newGrid.TimePoints[k];
                var tk1 = newGrid.TimePoints[k + 1];
                var h = tk1 - tk;

                for (var i = 0; i < order; i++)
                {
                    if (k > 0 && i == 0)
                    {
                        continue;
                    }

                    var t = tk + 0.5 * (1 + lglPoints[i]) * h;
                    var (state, control) = InterpolateAtTime(previousResult, t);
                    stateTrajectory[pointIndex] = state;
                    controlTrajectory[pointIndex] = control;
                    pointIndex++;
                }
            }

            return new InitialGuess(stateTrajectory, controlTrajectory);
        }

        /// <summary>
        /// Interpolates state and control at a specific time from a previous solution.
        /// </summary>
        /// <param name="result">Previous solution result.</param>
        /// <param name="t">Time at which to interpolate.</param>
        /// <returns>Interpolated state and control vectors.</returns>
        private static (double[] state, double[] control) InterpolateAtTime(CollocationResult result, double t)
        {
            var tClamped = Math.Max(result.Times[0], Math.Min(result.Times[result.Times.Length - 1], t));

            var idx = 0;
            for (var i = 0; i < result.Times.Length - 1; i++)
            {
                if (tClamped >= result.Times[i] && tClamped <= result.Times[i + 1])
                {
                    idx = i;
                    break;
                }
            }

            var t0 = result.Times[idx];
            var t1 = result.Times[idx + 1];
            var alpha = (t1 - t0) < 1e-12 ? 0.0 : (tClamped - t0) / (t1 - t0);

            var x0 = result.States[idx];
            var x1 = result.States[idx + 1];
            var state = new double[x0.Length];
            for (var i = 0; i < state.Length; i++)
            {
                state[i] = (1 - alpha) * x0[i] + alpha * x1[i];
            }

            var u0 = result.Controls[idx];
            var u1 = result.Controls[idx + 1];
            var control = new double[u0.Length];
            for (var i = 0; i < control.Length; i++)
            {
                control[i] = (1 - alpha) * u0[i] + alpha * u1[i];
            }

            return (state, control);
        }

        /// <summary>
        /// Scales a previous solution to a new time horizon.
        /// Useful when varying the time duration of a problem.
        /// </summary>
        /// <param name="previousResult">Previous solution result.</param>
        /// <param name="newInitialTime">New initial time.</param>
        /// <param name="newFinalTime">New final time.</param>
        /// <param name="newGrid">New collocation grid.</param>
        /// <returns>Initial guess scaled to new time horizon.</returns>
        public static InitialGuess ScaleTimeHorizon(
            CollocationResult previousResult,
            double newInitialTime,
            double newFinalTime,
            CollocationGrid newGrid)
        {
            ArgumentNullException.ThrowIfNull(previousResult);
            ArgumentNullException.ThrowIfNull(newGrid);

            var oldT0 = previousResult.Times[0];
            var oldTf = previousResult.Times[previousResult.Times.Length - 1];
            var oldDuration = oldTf - oldT0;
            var newDuration = newFinalTime - newInitialTime;

            var numPoints = newGrid.Segments + 1;
            var stateTrajectory = new double[numPoints][];
            var controlTrajectory = new double[numPoints][];

            for (var k = 0; k <= newGrid.Segments; k++)
            {
                var t = newGrid.TimePoints[k];
                var normalizedTime = (t - newInitialTime) / newDuration;
                var oldTime = oldT0 + normalizedTime * oldDuration;

                var (state, control) = InterpolateAtTime(previousResult, oldTime);
                stateTrajectory[k] = state;
                controlTrajectory[k] = control;
            }

            return new InitialGuess(stateTrajectory, controlTrajectory);
        }

        /// <summary>
        /// Creates an initial guess by blending two solutions based on a parameter.
        /// Used for continuation methods where you gradually transition from one problem to another.
        /// </summary>
        /// <param name="result1">First solution (lambda = 0).</param>
        /// <param name="result2">Second solution (lambda = 1).</param>
        /// <param name="lambda">Blending parameter [0, 1].</param>
        /// <param name="newGrid">New collocation grid.</param>
        /// <returns>Blended initial guess.</returns>
        public static InitialGuess BlendSolutions(
            CollocationResult result1,
            CollocationResult result2,
            double lambda,
            CollocationGrid newGrid)
        {
            ArgumentNullException.ThrowIfNull(result1);
            ArgumentNullException.ThrowIfNull(result2);
            ArgumentNullException.ThrowIfNull(newGrid);

            if (lambda < 0.0 || lambda > 1.0)
            {
                throw new ArgumentOutOfRangeException(nameof(lambda), "Lambda must be in [0, 1].");
            }

            var numPoints = newGrid.Segments + 1;
            var stateTrajectory = new double[numPoints][];
            var controlTrajectory = new double[numPoints][];

            for (var k = 0; k <= newGrid.Segments; k++)
            {
                var t = newGrid.TimePoints[k];

                var (state1, control1) = InterpolateAtTime(result1, t);
                var (state2, control2) = InterpolateAtTime(result2, t);

                var state = new double[state1.Length];
                var control = new double[control1.Length];

                for (var i = 0; i < state.Length; i++)
                {
                    state[i] = (1 - lambda) * state1[i] + lambda * state2[i];
                }

                for (var i = 0; i < control.Length; i++)
                {
                    control[i] = (1 - lambda) * control1[i] + lambda * control2[i];
                }

                stateTrajectory[k] = state;
                controlTrajectory[k] = control;
            }

            return new InitialGuess(stateTrajectory, controlTrajectory);
        }
    }
}
