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
    /// Defines an optimal control problem with dynamics, objective, and constraints.
    /// </summary>
    public sealed class ControlProblem
    {
        private Func<DynamicsInput, DynamicsResult>? _dynamics;
        private Func<RunningCostInput, RunningCostResult>? _runningCost;
        private Func<TerminalCostInput, TerminalCostResult>? _terminalCost;
        private int _stateDim;
        private int _controlDim;
        private double _t0;
        private double _tf;
        private double[]? _initialState;
        private double[]? _finalState;
        private double[]? _controlLowerBounds;
        private double[]? _controlUpperBounds;
        private double[]? _stateLowerBounds;
        private double[]? _stateUpperBounds;
        private readonly System.Collections.Generic.List<Func<PathConstraintInput, PathConstraintResult>> _pathConstraints = new();

        /// <summary>
        /// Gets the dynamics function: ẋ = f(x, u, t).
        /// Returns (value: dx/dt, gradients: [df/dx, df/du])
        /// </summary>
        public Func<DynamicsInput, DynamicsResult>? Dynamics => _dynamics;

        /// <summary>
        /// Gets the running cost function: L(x, u, t).
        /// </summary>
        public Func<RunningCostInput, RunningCostResult>? RunningCost => _runningCost;

        /// <summary>
        /// Gets the terminal cost function: Φ(x, t).
        /// </summary>
        public Func<TerminalCostInput, TerminalCostResult>? TerminalCost => _terminalCost;

        /// <summary>
        /// Gets the state dimension.
        /// </summary>
        public int StateDim => _stateDim;

        /// <summary>
        /// Gets the control dimension.
        /// </summary>
        public int ControlDim => _controlDim;

        /// <summary>
        /// Gets the initial time.
        /// </summary>
        public double InitialTime => _t0;

        /// <summary>
        /// Gets the final time.
        /// </summary>
        public double FinalTime => _tf;

        /// <summary>
        /// Gets the initial state condition (null if free).
        /// </summary>
        public double[]? InitialState => _initialState;

        /// <summary>
        /// Gets the final state condition (null if free).
        /// </summary>
        public double[]? FinalState => _finalState;

        /// <summary>
        /// Gets the control lower bounds (null if unconstrained).
        /// </summary>
        public double[]? ControlLowerBounds => _controlLowerBounds;

        /// <summary>
        /// Gets the control upper bounds (null if unconstrained).
        /// </summary>
        public double[]? ControlUpperBounds => _controlUpperBounds;

        /// <summary>
        /// Gets the state lower bounds (null if unconstrained).
        /// </summary>
        public double[]? StateLowerBounds => _stateLowerBounds;

        /// <summary>
        /// Gets the state upper bounds (null if unconstrained).
        /// </summary>
        public double[]? StateUpperBounds => _stateUpperBounds;

        /// <summary>
        /// Gets the path inequality constraints: g(x, u, t) ≤ 0.
        /// </summary>
        public System.Collections.Generic.IReadOnlyList<Func<PathConstraintInput, PathConstraintResult>> PathConstraints => _pathConstraints;

        /// <summary>
        /// Sets the dynamics function.
        /// </summary>
        /// <param name="dynamics">Dynamics function returning DynamicsResult.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithDynamics(Func<DynamicsInput, DynamicsResult> dynamics)
        {
            _dynamics = dynamics ?? throw new ArgumentNullException(nameof(dynamics));
            return this;
        }

        /// <summary>
        /// Sets the running cost function.
        /// </summary>
        /// <param name="runningCost">Running cost function returning RunningCostResult.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithRunningCost(Func<RunningCostInput, RunningCostResult> runningCost)
        {
            _runningCost = runningCost ?? throw new ArgumentNullException(nameof(runningCost));
            return this;
        }

        /// <summary>
        /// Sets the terminal cost function.
        /// </summary>
        /// <param name="terminalCost">Terminal cost function returning TerminalCostResult.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithTerminalCost(Func<TerminalCostInput, TerminalCostResult> terminalCost)
        {
            _terminalCost = terminalCost ?? throw new ArgumentNullException(nameof(terminalCost));
            return this;
        }

        /// <summary>
        /// Sets the state dimension.
        /// </summary>
        /// <param name="dim">State dimension.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithStateSize(int dim)
        {
            if (dim <= 0)
            {
                throw new ArgumentException("State dimension must be positive.", nameof(dim));
            }

            _stateDim = dim;
            return this;
        }

        /// <summary>
        /// Sets the control dimension.
        /// </summary>
        /// <param name="dim">Control dimension.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithControlSize(int dim)
        {
            if (dim <= 0)
            {
                throw new ArgumentException("Control dimension must be positive.", nameof(dim));
            }

            _controlDim = dim;
            return this;
        }

        /// <summary>
        /// Sets the time horizon.
        /// </summary>
        /// <param name="t0">Initial time.</param>
        /// <param name="tf">Final time.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithTimeHorizon(double t0, double tf)
        {
            if (tf <= t0)
            {
                throw new ArgumentException("Final time must be greater than initial time.");
            }

            _t0 = t0;
            _tf = tf;
            return this;
        }

        /// <summary>
        /// Sets the initial state condition.
        /// </summary>
        /// <param name="x0">Initial state.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithInitialCondition(double[] x0)
        {
            _initialState = x0 ?? throw new ArgumentNullException(nameof(x0));
            return this;
        }

        /// <summary>
        /// Sets the final state condition.
        /// </summary>
        /// <param name="xf">Final state.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithFinalCondition(double[] xf)
        {
            _finalState = xf ?? throw new ArgumentNullException(nameof(xf));
            return this;
        }

        /// <summary>
        /// Sets control bounds.
        /// </summary>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithControlBounds(double[] lower, double[] upper)
        {
            _controlLowerBounds = lower ?? throw new ArgumentNullException(nameof(lower));
            _controlUpperBounds = upper ?? throw new ArgumentNullException(nameof(upper));
            return this;
        }

        /// <summary>
        /// Sets state bounds.
        /// </summary>
        /// <param name="lower">Lower bounds.</param>
        /// <param name="upper">Upper bounds.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithStateBounds(double[] lower, double[] upper)
        {
            _stateLowerBounds = lower ?? throw new ArgumentNullException(nameof(lower));
            _stateUpperBounds = upper ?? throw new ArgumentNullException(nameof(upper));
            return this;
        }

        /// <summary>
        /// Adds a path inequality constraint: g(x, u, t) ≤ 0.
        /// The constraint is enforced at all collocation points.
        /// </summary>
        /// <param name="constraint">Constraint function returning PathConstraintResult.</param>
        /// <returns>This problem instance for method chaining.</returns>
        public ControlProblem WithPathConstraint(Func<PathConstraintInput, PathConstraintResult> constraint)
        {
            _pathConstraints.Add(constraint ?? throw new ArgumentNullException(nameof(constraint)));
            return this;
        }
    }
}
