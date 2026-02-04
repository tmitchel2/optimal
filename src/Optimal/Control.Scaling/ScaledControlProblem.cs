/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Optimal.Control.Core;

namespace Optimal.Control.Scaling
{
    /// <summary>
    /// Wraps a <see cref="ControlProblem"/> with variable scaling to improve
    /// optimization conditioning.
    /// </summary>
    /// <remarks>
    /// The scaling transforms state and control variables from original coordinates
    /// to normalized coordinates [-1, 1]. All functions (dynamics, costs, constraints)
    /// are transformed with appropriate chain rule applications to gradients.
    /// </remarks>
    public sealed class ScaledControlProblem
    {
        private readonly ControlProblem _original;
        private readonly VariableScaling _scaling;
        private readonly List<Func<PathConstraintInput, PathConstraintResult>> _scaledPathConstraints;

        /// <summary>
        /// Gets the variable scaling parameters.
        /// </summary>
        public VariableScaling Scaling => _scaling;

        /// <summary>
        /// Gets the state dimension.
        /// </summary>
        public int StateDim => _original.StateDim;

        /// <summary>
        /// Gets the control dimension.
        /// </summary>
        public int ControlDim => _original.ControlDim;

        /// <summary>
        /// Gets the initial time.
        /// </summary>
        public double InitialTime => _original.InitialTime;

        /// <summary>
        /// Gets the final time.
        /// </summary>
        public double FinalTime => _original.FinalTime;

        /// <summary>
        /// Gets the initial state condition in scaled coordinates (null if free).
        /// </summary>
        public double[]? InitialState { get; }

        /// <summary>
        /// Gets the final state condition in scaled coordinates (null if free).
        /// </summary>
        public double[]? FinalState { get; }

        /// <summary>
        /// Gets the control lower bounds in scaled coordinates (all -1 for bounded variables).
        /// </summary>
        public double[]? ControlLowerBounds { get; }

        /// <summary>
        /// Gets the control upper bounds in scaled coordinates (all +1 for bounded variables).
        /// </summary>
        public double[]? ControlUpperBounds { get; }

        /// <summary>
        /// Gets the state lower bounds in scaled coordinates (all -1 for bounded variables).
        /// </summary>
        public double[]? StateLowerBounds { get; }

        /// <summary>
        /// Gets the state upper bounds in scaled coordinates (all +1 for bounded variables).
        /// </summary>
        public double[]? StateUpperBounds { get; }

        /// <summary>
        /// Gets the scaled dynamics function.
        /// </summary>
        public Func<DynamicsInput, DynamicsResult>? Dynamics { get; }

        /// <summary>
        /// Gets the scaled running cost function.
        /// </summary>
        public Func<RunningCostInput, RunningCostResult>? RunningCost { get; }

        /// <summary>
        /// Gets the scaled terminal cost function.
        /// </summary>
        public Func<TerminalCostInput, TerminalCostResult>? TerminalCost { get; }

        /// <summary>
        /// Gets the scaled path constraints.
        /// </summary>
        public IReadOnlyList<Func<PathConstraintInput, PathConstraintResult>> PathConstraints => _scaledPathConstraints;

        /// <summary>
        /// Initializes a new instance of <see cref="ScaledControlProblem"/>.
        /// </summary>
        /// <param name="original">The original control problem.</param>
        /// <param name="scaling">The variable scaling parameters.</param>
        public ScaledControlProblem(ControlProblem original, VariableScaling scaling)
        {
            _original = original ?? throw new ArgumentNullException(nameof(original));
            _scaling = scaling ?? throw new ArgumentNullException(nameof(scaling));

            if (scaling.StateDim != original.StateDim)
            {
                throw new ArgumentException($"Scaling state dimension {scaling.StateDim} does not match problem state dimension {original.StateDim}.");
            }

            if (scaling.ControlDim != original.ControlDim)
            {
                throw new ArgumentException($"Scaling control dimension {scaling.ControlDim} does not match problem control dimension {original.ControlDim}.");
            }

            // Scale initial and final conditions
            InitialState = original.InitialState != null ? scaling.ScaleState(original.InitialState) : null;
            FinalState = original.FinalState != null ? scaling.ScaleState(original.FinalState) : null;

            // Scale bounds
            if (original.StateLowerBounds != null && original.StateUpperBounds != null)
            {
                (StateLowerBounds, StateUpperBounds) = scaling.ScaleStateBounds(
                    original.StateLowerBounds, original.StateUpperBounds);
            }

            if (original.ControlLowerBounds != null && original.ControlUpperBounds != null)
            {
                (ControlLowerBounds, ControlUpperBounds) = scaling.ScaleControlBounds(
                    original.ControlLowerBounds, original.ControlUpperBounds);
            }

            // Wrap dynamics
            if (original.Dynamics != null)
            {
                Dynamics = input => TransformDynamics(input, original.Dynamics);
            }

            // Wrap running cost
            if (original.RunningCost != null)
            {
                RunningCost = input => TransformRunningCost(input, original.RunningCost);
            }

            // Wrap terminal cost
            if (original.TerminalCost != null)
            {
                TerminalCost = input => TransformTerminalCost(input, original.TerminalCost);
            }

            // Wrap path constraints
            _scaledPathConstraints = new List<Func<PathConstraintInput, PathConstraintResult>>();
            foreach (var constraint in original.PathConstraints)
            {
                _scaledPathConstraints.Add(input => TransformPathConstraint(input, constraint));
            }
        }

        /// <summary>
        /// Creates a <see cref="ControlProblem"/> instance that uses this scaled problem's functions.
        /// This allows the scaled problem to be used with existing solvers.
        /// </summary>
        /// <returns>A ControlProblem configured with scaled functions.</returns>
        public ControlProblem ToControlProblem()
        {
            var problem = new ControlProblem()
                .WithStateSize(StateDim)
                .WithControlSize(ControlDim)
                .WithTimeHorizon(InitialTime, FinalTime);

            if (Dynamics != null)
            {
                problem.WithDynamics(Dynamics);
            }

            if (RunningCost != null)
            {
                problem.WithRunningCost(RunningCost);
            }

            if (TerminalCost != null)
            {
                problem.WithTerminalCost(TerminalCost);
            }

            if (InitialState != null)
            {
                problem.WithInitialCondition(InitialState);
            }

            if (FinalState != null)
            {
                problem.WithFinalCondition(FinalState);
            }

            if (StateLowerBounds != null && StateUpperBounds != null)
            {
                problem.WithStateBounds(StateLowerBounds, StateUpperBounds);
            }

            if (ControlLowerBounds != null && ControlUpperBounds != null)
            {
                problem.WithControlBounds(ControlLowerBounds, ControlUpperBounds);
            }

            foreach (var constraint in _scaledPathConstraints)
            {
                problem.WithPathConstraint(constraint);
            }

            return problem;
        }

        /// <summary>
        /// Scales an initial guess from original coordinates to scaled coordinates.
        /// </summary>
        /// <param name="original">Initial guess in original coordinates.</param>
        /// <returns>Initial guess in scaled coordinates.</returns>
        public InitialGuess ScaleInitialGuess(InitialGuess original)
        {
            var scaledStates = new double[original.StateTrajectory.Length][];
            var scaledControls = new double[original.ControlTrajectory.Length][];

            for (var i = 0; i < original.StateTrajectory.Length; i++)
            {
                scaledStates[i] = _scaling.ScaleState(original.StateTrajectory[i]);
            }

            for (var i = 0; i < original.ControlTrajectory.Length; i++)
            {
                scaledControls[i] = _scaling.ScaleControl(original.ControlTrajectory[i]);
            }

            return new InitialGuess(scaledStates, scaledControls);
        }

        /// <summary>
        /// Unscales state trajectories from scaled to original coordinates.
        /// </summary>
        /// <param name="scaledStates">State trajectory in scaled coordinates.</param>
        /// <returns>State trajectory in original coordinates.</returns>
        public double[][] UnscaleStates(double[][] scaledStates)
        {
            var original = new double[scaledStates.Length][];
            for (var i = 0; i < scaledStates.Length; i++)
            {
                original[i] = _scaling.UnscaleState(scaledStates[i]);
            }

            return original;
        }

        /// <summary>
        /// Unscales control trajectories from scaled to original coordinates.
        /// </summary>
        /// <param name="scaledControls">Control trajectory in scaled coordinates.</param>
        /// <returns>Control trajectory in original coordinates.</returns>
        public double[][] UnscaleControls(double[][] scaledControls)
        {
            var original = new double[scaledControls.Length][];
            for (var i = 0; i < scaledControls.Length; i++)
            {
                original[i] = _scaling.UnscaleControl(scaledControls[i]);
            }

            return original;
        }

        /// <summary>
        /// Unscales state derivatives from scaled to original coordinates.
        /// In scaled coordinates: dx_s/dt = S^(-1) * dx/dt, so dx/dt = S * dx_s/dt.
        /// </summary>
        /// <param name="scaledDerivatives">State derivatives in scaled coordinates.</param>
        /// <returns>State derivatives in original coordinates.</returns>
        public double[][] UnscaleDerivatives(double[][] scaledDerivatives)
        {
            var original = new double[scaledDerivatives.Length][];
            for (var i = 0; i < scaledDerivatives.Length; i++)
            {
                original[i] = new double[StateDim];
                for (var j = 0; j < StateDim; j++)
                {
                    original[i][j] = scaledDerivatives[i][j] * _scaling.StateScales[j];
                }
            }

            return original;
        }

        /// <summary>
        /// Transforms dynamics from scaled input to scaled output with correct gradient chain rule.
        /// </summary>
        /// <remarks>
        /// Given x = S*x_s + c, u = S_u*u_s + c_u, the dynamics transform as:
        /// dx_s/dt = S^(-1) * f(x, u, t)
        /// df_s/dx_s = S^(-1) * df/dx * S (matrix multiplication for row scaling, column scaling)
        /// df_s/du_s = S^(-1) * df/du * S_u
        /// </remarks>
        private DynamicsResult TransformDynamics(
            DynamicsInput scaledInput,
            Func<DynamicsInput, DynamicsResult> originalDynamics)
        {
            // Unscale inputs
            var originalState = _scaling.UnscaleState(scaledInput.State);
            var originalControl = _scaling.UnscaleControl(scaledInput.Control);

            var originalInput = new DynamicsInput(
                originalState,
                originalControl,
                scaledInput.Time,
                scaledInput.SegmentIndex,
                scaledInput.SegmentCount);

            var original = originalDynamics(originalInput);

            // Scale the dynamics value: dx_s/dt = S^(-1) * dx/dt
            var scaledValue = new double[StateDim];
            for (var i = 0; i < StateDim; i++)
            {
                scaledValue[i] = original.Value[i] / _scaling.StateScales[i];
            }

            // Transform gradients if present
            double[][]? scaledGradients = null;
            if (original.Gradients != null && original.Gradients.Length >= 2)
            {
                var dfdx = original.Gradients[0]; // Flattened StateDim x StateDim (row-major)
                var dfdu = original.Gradients[1]; // Flattened StateDim x ControlDim (row-major)

                // df_s/dx_s = S^(-1) * df/dx * S
                // For row i, col j: (df_s/dx_s)[i,j] = (1/S_i) * (df/dx)[i,j] * S_j
                var scaledDfdx = new double[StateDim * StateDim];
                for (var i = 0; i < StateDim; i++)
                {
                    var invSi = 1.0 / _scaling.StateScales[i];
                    for (var j = 0; j < StateDim; j++)
                    {
                        var idx = (i * StateDim) + j;
                        scaledDfdx[idx] = invSi * dfdx[idx] * _scaling.StateScales[j];
                    }
                }

                // df_s/du_s = S^(-1) * df/du * S_u
                // For row i, col j: (df_s/du_s)[i,j] = (1/S_i) * (df/du)[i,j] * S_u_j
                var scaledDfdu = new double[StateDim * ControlDim];
                for (var i = 0; i < StateDim; i++)
                {
                    var invSi = 1.0 / _scaling.StateScales[i];
                    for (var j = 0; j < ControlDim; j++)
                    {
                        var idx = (i * ControlDim) + j;
                        scaledDfdu[idx] = invSi * dfdu[idx] * _scaling.ControlScales[j];
                    }
                }

                scaledGradients = [scaledDfdx, scaledDfdu];
            }

            return new DynamicsResult(scaledValue, scaledGradients!);
        }

        /// <summary>
        /// Transforms running cost with correct gradient chain rule.
        /// </summary>
        /// <remarks>
        /// The cost value is unchanged (just evaluating L at transformed point).
        /// Gradients: dL_s/dx_s = dL/dx * S (chain rule)
        ///            dL_s/du_s = dL/du * S_u
        /// </remarks>
        private RunningCostResult TransformRunningCost(
            RunningCostInput scaledInput,
            Func<RunningCostInput, RunningCostResult> originalCost)
        {
            var originalState = _scaling.UnscaleState(scaledInput.State);
            var originalControl = _scaling.UnscaleControl(scaledInput.Control);

            var originalInput = new RunningCostInput(originalState, originalControl, scaledInput.Time);
            var original = originalCost(originalInput);

            // Value is unchanged
            var scaledValue = original.Value;

            // Transform gradients: [dL/dx_1, ..., dL/dx_n, dL/du_1, ..., dL/du_m, dL/dt]
            double[]? scaledGradients = null;
            if (original.Gradients != null)
            {
                scaledGradients = new double[original.Gradients.Length];

                // dL_s/dx_s = dL/dx * S (element-wise)
                for (var i = 0; i < StateDim; i++)
                {
                    scaledGradients[i] = original.Gradients[i] * _scaling.StateScales[i];
                }

                // dL_s/du_s = dL/du * S_u (element-wise)
                for (var i = 0; i < ControlDim; i++)
                {
                    scaledGradients[StateDim + i] = original.Gradients[StateDim + i] * _scaling.ControlScales[i];
                }

                // dL/dt is unchanged
                if (original.Gradients.Length > StateDim + ControlDim)
                {
                    scaledGradients[StateDim + ControlDim] = original.Gradients[StateDim + ControlDim];
                }
            }

            return new RunningCostResult(scaledValue, scaledGradients!);
        }

        /// <summary>
        /// Transforms terminal cost with correct gradient chain rule.
        /// </summary>
        private TerminalCostResult TransformTerminalCost(
            TerminalCostInput scaledInput,
            Func<TerminalCostInput, TerminalCostResult> originalCost)
        {
            var originalState = _scaling.UnscaleState(scaledInput.State);

            var originalInput = new TerminalCostInput(originalState, scaledInput.Time);
            var original = originalCost(originalInput);

            // Value is unchanged
            var scaledValue = original.Value;

            // Transform gradients: dΦ_s/dx_s = dΦ/dx * S (element-wise)
            double[]? scaledGradients = null;
            if (original.Gradients != null)
            {
                scaledGradients = new double[original.Gradients.Length];
                for (var i = 0; i < StateDim && i < original.Gradients.Length; i++)
                {
                    scaledGradients[i] = original.Gradients[i] * _scaling.StateScales[i];
                }
            }

            return new TerminalCostResult(scaledValue, scaledGradients!);
        }

        /// <summary>
        /// Transforms path constraint with correct gradient chain rule.
        /// </summary>
        private PathConstraintResult TransformPathConstraint(
            PathConstraintInput scaledInput,
            Func<PathConstraintInput, PathConstraintResult> originalConstraint)
        {
            var originalState = _scaling.UnscaleState(scaledInput.State);
            var originalControl = _scaling.UnscaleControl(scaledInput.Control);

            var originalInput = new PathConstraintInput(originalState, originalControl, scaledInput.Time);
            var original = originalConstraint(originalInput);

            // Constraint value is unchanged
            var scaledValue = original.Value;

            // Transform gradients: [dg/dx, dg/du, dg/dt]
            double[]? scaledGradients = null;
            if (original.Gradients != null)
            {
                scaledGradients = new double[original.Gradients.Length];

                // dg_s/dx_s = dg/dx * S (element-wise)
                for (var i = 0; i < StateDim; i++)
                {
                    scaledGradients[i] = original.Gradients[i] * _scaling.StateScales[i];
                }

                // dg_s/du_s = dg/du * S_u (element-wise)
                for (var i = 0; i < ControlDim; i++)
                {
                    scaledGradients[StateDim + i] = original.Gradients[StateDim + i] * _scaling.ControlScales[i];
                }

                // dg/dt is unchanged
                if (original.Gradients.Length > StateDim + ControlDim)
                {
                    scaledGradients[StateDim + ControlDim] = original.Gradients[StateDim + ControlDim];
                }
            }

            return new PathConstraintResult(scaledValue, scaledGradients!);
        }
    }
}
