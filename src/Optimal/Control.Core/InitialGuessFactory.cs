/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control.Core
{
    /// <summary>
    /// Factory for creating initial guesses for optimal control problems.
    /// </summary>
    public static class InitialGuessFactory
    {
        private const double PositionTolerance = 1e-10;

        /// <summary>
        /// Creates an initial guess using linear interpolation for states and zero control.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="segments">Number of collocation segments.</param>
        /// <returns>An initial guess with linearly interpolated states and zero control.</returns>
        public static InitialGuess CreateLinearInterpolation(ControlProblem problem, int segments)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (segments <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(segments), "Segments must be positive");
            }

            var numPoints = segments + 1;
            var x0 = problem.InitialState ?? new double[problem.StateDim];
            var xf = problem.FinalState ?? new double[problem.StateDim];
            var u0 = new double[problem.ControlDim];

            return CreateFromBoundaryConditions(x0, xf, u0, numPoints, problem.StateDim, problem.ControlDim);
        }

        /// <summary>
        /// Creates an initial guess using linear interpolation for states with problem-specific control heuristics.
        /// For Brachistochrone-like problems (1D control, 2D+ state), computes an angle-based control guess.
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="segments">Number of collocation segments.</param>
        /// <returns>An initial guess with linearly interpolated states and heuristic-based control.</returns>
        public static InitialGuess CreateWithControlHeuristics(ControlProblem problem, int segments)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (segments <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(segments), "Segments must be positive");
            }

            var numPoints = segments + 1;
            var x0 = problem.InitialState ?? new double[problem.StateDim];
            var xf = problem.FinalState ?? new double[problem.StateDim];
            var u0 = ComputeInitialControlGuess(problem, x0, xf);

            return CreateFromBoundaryConditions(x0, xf, u0, numPoints, problem.StateDim, problem.ControlDim);
        }

        /// <summary>
        /// Creates an initial guess for LGL solvers with a specific order.
        /// The number of points depends on the order and segments: (order - 1) * segments + 1
        /// </summary>
        /// <param name="problem">The control problem.</param>
        /// <param name="segments">Number of collocation segments.</param>
        /// <param name="order">LGL order (number of points per segment).</param>
        /// <returns>An initial guess with the appropriate number of points for LGL.</returns>
        public static InitialGuess CreateForLGL(ControlProblem problem, int segments, int order)
        {
            ArgumentNullException.ThrowIfNull(problem);

            if (segments <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(segments), "Segments must be positive");
            }

            if (order < 2)
            {
                throw new ArgumentOutOfRangeException(nameof(order), "Order must be at least 2");
            }

            var numPoints = (order - 1) * segments + 1;
            var x0 = problem.InitialState ?? new double[problem.StateDim];
            var xf = problem.FinalState ?? new double[problem.StateDim];
            var u0 = ComputeInitialControlGuess(problem, x0, xf);

            return CreateFromBoundaryConditions(x0, xf, u0, numPoints, problem.StateDim, problem.ControlDim);
        }

        /// <summary>
        /// Creates an initial guess from explicit boundary conditions and constant control.
        /// </summary>
        /// <param name="initialState">Initial state vector.</param>
        /// <param name="finalState">Final state vector.</param>
        /// <param name="constantControl">Constant control applied at all points.</param>
        /// <param name="numPoints">Number of collocation points.</param>
        /// <param name="stateDim">State dimension.</param>
        /// <param name="controlDim">Control dimension.</param>
        /// <returns>An initial guess with linearly interpolated states.</returns>
        public static InitialGuess CreateFromBoundaryConditions(
            double[] initialState,
            double[] finalState,
            double[] constantControl,
            int numPoints,
            int stateDim,
            int controlDim)
        {
            ArgumentNullException.ThrowIfNull(initialState);
            ArgumentNullException.ThrowIfNull(finalState);
            ArgumentNullException.ThrowIfNull(constantControl);

            if (numPoints <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(numPoints), "Number of points must be positive");
            }

            var stateTrajectory = new double[numPoints][];
            var controlTrajectory = new double[numPoints][];

            for (var k = 0; k < numPoints; k++)
            {
                var alpha = numPoints > 1 ? (double)k / (numPoints - 1) : 0.0;
                var state = new double[stateDim];

                for (var i = 0; i < stateDim; i++)
                {
                    var x0 = initialState[i];
                    var xf = finalState[i];
                    var x0IsNaN = double.IsNaN(x0);
                    var xfIsNaN = double.IsNaN(xf);

                    if (x0IsNaN && xfIsNaN)
                    {
                        state[i] = 0.0;
                    }
                    else if (x0IsNaN)
                    {
                        state[i] = xf;
                    }
                    else if (xfIsNaN)
                    {
                        state[i] = x0;
                    }
                    else
                    {
                        state[i] = (1.0 - alpha) * x0 + alpha * xf;
                    }
                }

                stateTrajectory[k] = state;

                var control = new double[controlDim];
                Array.Copy(constantControl, control, controlDim);
                controlTrajectory[k] = control;
            }

            return new InitialGuess(stateTrajectory, controlTrajectory);
        }

        /// <summary>
        /// Computes a heuristic control guess for Brachistochrone-like problems.
        /// For problems with 1D control and 2D+ state, computes an angle based on displacement.
        /// </summary>
        private static double[] ComputeInitialControlGuess(ControlProblem problem, double[] x0, double[] xf)
        {
            var u0 = new double[problem.ControlDim];

            if (ShouldComputeAngleBasedGuess(problem))
            {
                var dx = xf[0] - x0[0];
                var dy = xf[1] - x0[1];
                if (HasSignificantDisplacement(dx, dy))
                {
                    var angle = Math.Atan2(-dy, dx);
                    u0[0] = Math.Clamp(angle, 0.0, Math.PI / 2.0);
                }
            }

            if (problem.ControlLowerBounds != null && problem.ControlUpperBounds != null)
            {
                for (var i = 0; i < problem.ControlDim; i++)
                {
                    u0[i] = Math.Max(problem.ControlLowerBounds[i],
                                     Math.Min(problem.ControlUpperBounds[i], u0[i]));
                }
            }

            return u0;
        }

        private static bool ShouldComputeAngleBasedGuess(ControlProblem problem) =>
            problem.ControlDim == 1 &&
            problem.StateDim >= 2 &&
            problem.InitialState != null &&
            problem.FinalState != null;

        private static bool HasSignificantDisplacement(double dx, double dy) =>
            Math.Abs(dx) > PositionTolerance || Math.Abs(dy) > PositionTolerance;
    }
}
