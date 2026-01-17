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
    /// Represents an initial guess for the state and control trajectories of an optimal control problem.
    /// Validates that trajectories are consistent in length and dimension.
    /// </summary>
    public sealed class InitialGuess
    {
        /// <summary>
        /// Gets the state trajectory. Each element is a state vector at a collocation point.
        /// </summary>
        public double[][] StateTrajectory { get; }

        /// <summary>
        /// Gets the control trajectory. Each element is a control vector at a collocation point.
        /// </summary>
        public double[][] ControlTrajectory { get; }

        /// <summary>
        /// Gets the number of collocation points in the trajectories.
        /// </summary>
        public int Points => StateTrajectory.Length;

        /// <summary>
        /// Gets the state dimension.
        /// </summary>
        public int StateDim { get; }

        /// <summary>
        /// Gets the control dimension.
        /// </summary>
        public int ControlDim { get; }

        /// <summary>
        /// Creates a new initial guess with the specified state and control trajectories.
        /// </summary>
        /// <param name="stateTrajectory">State trajectory where each element is a state vector.</param>
        /// <param name="controlTrajectory">Control trajectory where each element is a control vector.</param>
        /// <exception cref="ArgumentNullException">Thrown when either trajectory is null.</exception>
        /// <exception cref="ArgumentException">Thrown when trajectories are empty or have inconsistent dimensions.</exception>
        public InitialGuess(double[][] stateTrajectory, double[][] controlTrajectory)
        {
            ArgumentNullException.ThrowIfNull(stateTrajectory);
            ArgumentNullException.ThrowIfNull(controlTrajectory);

            if (stateTrajectory.Length == 0)
            {
                throw new ArgumentException("State trajectory must have at least one point", nameof(stateTrajectory));
            }

            if (controlTrajectory.Length != stateTrajectory.Length)
            {
                throw new ArgumentException("Control trajectory must have same length as state trajectory", nameof(controlTrajectory));
            }

            StateDim = stateTrajectory[0].Length;
            ControlDim = controlTrajectory[0].Length;

            for (var i = 0; i < stateTrajectory.Length; i++)
            {
                if (stateTrajectory[i].Length != StateDim)
                {
                    throw new ArgumentException($"State at point {i} has inconsistent dimension", nameof(stateTrajectory));
                }

                if (controlTrajectory[i].Length != ControlDim)
                {
                    throw new ArgumentException($"Control at point {i} has inconsistent dimension", nameof(controlTrajectory));
                }
            }

            StateTrajectory = stateTrajectory;
            ControlTrajectory = controlTrajectory;
        }
    }
}
