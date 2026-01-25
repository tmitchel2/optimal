/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.Control.Core
{
    /// <summary>
    /// Input arguments for the dynamics function: ẋ = f(x, u, t).
    /// </summary>
    public readonly struct DynamicsInput
    {
        /// <summary>
        /// Gets the state vector.
        /// </summary>
        public double[] State { get; init; }

        /// <summary>
        /// Gets the control vector.
        /// </summary>
        public double[] Control { get; init; }

        /// <summary>
        /// Gets the time.
        /// </summary>
        public double Time { get; init; }

        /// <summary>
        /// Gets the segment index (0-based). -1 if not applicable.
        /// </summary>
        public double SegmentIndex { get; init; }

        /// <summary>
        /// Gets the total number of segments. -1 if not applicable.
        /// </summary>
        public int SegmentCount { get; init; }

        /// <summary>
        /// Creates a new dynamics input.
        /// </summary>
        /// <param name="state">State vector.</param>
        /// <param name="control">Control vector.</param>
        /// <param name="time">Time.</param>
        /// <param name="segmentIndex">Segment index (0-based), or -1 if not applicable.</param>
        /// <param name="segmentCount">Total number of segments, or -1 if not applicable.</param>
        public DynamicsInput(double[] state, double[] control, double time, double segmentIndex, int segmentCount)
        {
            State = state;
            Control = control;
            Time = time;
            SegmentIndex = segmentIndex;
            SegmentCount = segmentCount;
        }
    }

    /// <summary>
    /// Result of the dynamics function: ẋ = f(x, u, t).
    /// </summary>
    public readonly struct DynamicsResult
    {
        /// <summary>
        /// Gets the state derivative (dx/dt).
        /// </summary>
        public double[] Value { get; init; }

        /// <summary>
        /// Gets the gradients: [0] = df/dx (flattened), [1] = df/du.
        /// </summary>
        public double[][] Gradients { get; init; }

        /// <summary>
        /// Creates a new dynamics result.
        /// </summary>
        /// <param name="value">State derivative.</param>
        /// <param name="gradients">Gradients [df/dx, df/du].</param>
        public DynamicsResult(double[] value, double[][] gradients)
        {
            Value = value;
            Gradients = gradients;
        }
    }

    /// <summary>
    /// Input arguments for the running cost function: L(x, u, t).
    /// </summary>
    public readonly struct RunningCostInput
    {
        /// <summary>
        /// Gets the state vector.
        /// </summary>
        public double[] State { get; init; }

        /// <summary>
        /// Gets the control vector.
        /// </summary>
        public double[] Control { get; init; }

        /// <summary>
        /// Gets the time.
        /// </summary>
        public double Time { get; init; }

        /// <summary>
        /// Creates a new running cost input.
        /// </summary>
        /// <param name="state">State vector.</param>
        /// <param name="control">Control vector.</param>
        /// <param name="time">Time.</param>
        public RunningCostInput(double[] state, double[] control, double time)
        {
            State = state;
            Control = control;
            Time = time;
        }
    }

    /// <summary>
    /// Result of the running cost function: L(x, u, t).
    /// </summary>
    public readonly struct RunningCostResult
    {
        /// <summary>
        /// Gets the running cost value.
        /// </summary>
        public double Value { get; init; }

        /// <summary>
        /// Gets the gradients: [dL/dx, dL/du, dL/dt].
        /// </summary>
        public double[] Gradients { get; init; }

        /// <summary>
        /// Creates a new running cost result.
        /// </summary>
        /// <param name="value">Running cost value.</param>
        /// <param name="gradients">Gradients [dL/dx, dL/du, dL/dt].</param>
        public RunningCostResult(double value, double[] gradients)
        {
            Value = value;
            Gradients = gradients;
        }
    }

    /// <summary>
    /// Input arguments for the terminal cost function: Φ(x, t).
    /// </summary>
    public readonly struct TerminalCostInput
    {
        /// <summary>
        /// Gets the state vector at final time.
        /// </summary>
        public double[] State { get; init; }

        /// <summary>
        /// Gets the final time.
        /// </summary>
        public double Time { get; init; }

        /// <summary>
        /// Creates a new terminal cost input.
        /// </summary>
        /// <param name="state">State vector at final time.</param>
        /// <param name="time">Final time.</param>
        public TerminalCostInput(double[] state, double time)
        {
            State = state;
            Time = time;
        }
    }

    /// <summary>
    /// Result of the terminal cost function: Φ(x, t).
    /// </summary>
    public readonly struct TerminalCostResult
    {
        /// <summary>
        /// Gets the terminal cost value.
        /// </summary>
        public double Value { get; init; }

        /// <summary>
        /// Gets the gradients: dΦ/dx.
        /// </summary>
        public double[] Gradients { get; init; }

        /// <summary>
        /// Creates a new terminal cost result.
        /// </summary>
        /// <param name="value">Terminal cost value.</param>
        /// <param name="gradients">Gradients dΦ/dx.</param>
        public TerminalCostResult(double value, double[] gradients)
        {
            Value = value;
            Gradients = gradients;
        }
    }

    /// <summary>
    /// Input arguments for path constraint functions: g(x, u, t) ≤ 0.
    /// </summary>
    public readonly struct PathConstraintInput
    {
        /// <summary>
        /// Gets the state vector.
        /// </summary>
        public double[] State { get; init; }

        /// <summary>
        /// Gets the control vector.
        /// </summary>
        public double[] Control { get; init; }

        /// <summary>
        /// Gets the time.
        /// </summary>
        public double Time { get; init; }

        /// <summary>
        /// Creates a new path constraint input.
        /// </summary>
        /// <param name="state">State vector.</param>
        /// <param name="control">Control vector.</param>
        /// <param name="time">Time.</param>
        public PathConstraintInput(double[] state, double[] control, double time)
        {
            State = state;
            Control = control;
            Time = time;
        }
    }

    /// <summary>
    /// Result of a path constraint function: g(x, u, t) ≤ 0.
    /// </summary>
    public readonly struct PathConstraintResult
    {
        /// <summary>
        /// Gets the constraint value. The constraint is satisfied when value ≤ 0.
        /// </summary>
        public double Value { get; init; }

        /// <summary>
        /// Gets the gradients: [dg/dx, dg/du, dg/dt].
        /// Length should be stateDim + controlDim + 1.
        /// </summary>
        public double[] Gradients { get; init; }

        /// <summary>
        /// Creates a new path constraint result.
        /// </summary>
        /// <param name="value">Constraint value (satisfied when ≤ 0).</param>
        /// <param name="gradients">Gradients [dg/dx, dg/du, dg/dt].</param>
        public PathConstraintResult(double value, double[] gradients)
        {
            Value = value;
            Gradients = gradients;
        }
    }
}
