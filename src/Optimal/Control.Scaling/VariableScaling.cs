/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control.Scaling
{
    /// <summary>
    /// Defines affine scaling parameters for normalizing state and control variables
    /// to the range [-1, 1] for improved optimization conditioning.
    /// </summary>
    /// <remarks>
    /// The affine transformation is: z_scaled = (z - center) / scale
    /// The inverse is: z = z_scaled * scale + center
    /// </remarks>
    public sealed class VariableScaling
    {
        private const double MinimumScale = 1e-10;

        /// <summary>
        /// Gets the scale factors for each state variable.
        /// </summary>
        public double[] StateScales { get; }

        /// <summary>
        /// Gets the center values for each state variable.
        /// </summary>
        public double[] StateCenters { get; }

        /// <summary>
        /// Gets the scale factors for each control variable.
        /// </summary>
        public double[] ControlScales { get; }

        /// <summary>
        /// Gets the center values for each control variable.
        /// </summary>
        public double[] ControlCenters { get; }

        /// <summary>
        /// Gets the state dimension.
        /// </summary>
        public int StateDim => StateScales.Length;

        /// <summary>
        /// Gets the control dimension.
        /// </summary>
        public int ControlDim => ControlScales.Length;

        /// <summary>
        /// Initializes a new instance of the <see cref="VariableScaling"/> class.
        /// </summary>
        /// <param name="stateScales">Scale factors for state variables.</param>
        /// <param name="stateCenters">Center values for state variables.</param>
        /// <param name="controlScales">Scale factors for control variables.</param>
        /// <param name="controlCenters">Center values for control variables.</param>
        public VariableScaling(
            double[] stateScales,
            double[] stateCenters,
            double[] controlScales,
            double[] controlCenters)
        {
            StateScales = stateScales ?? throw new ArgumentNullException(nameof(stateScales));
            StateCenters = stateCenters ?? throw new ArgumentNullException(nameof(stateCenters));
            ControlScales = controlScales ?? throw new ArgumentNullException(nameof(controlScales));
            ControlCenters = controlCenters ?? throw new ArgumentNullException(nameof(controlCenters));

            if (stateScales.Length != stateCenters.Length)
            {
                throw new ArgumentException("State scales and centers must have the same length.");
            }

            if (controlScales.Length != controlCenters.Length)
            {
                throw new ArgumentException("Control scales and centers must have the same length.");
            }
        }

        /// <summary>
        /// Creates scaling parameters from variable bounds.
        /// Variables are scaled to [-1, 1] using affine transformation.
        /// </summary>
        /// <param name="stateLower">Lower bounds for state variables.</param>
        /// <param name="stateUpper">Upper bounds for state variables.</param>
        /// <param name="controlLower">Lower bounds for control variables.</param>
        /// <param name="controlUpper">Upper bounds for control variables.</param>
        /// <returns>A new <see cref="VariableScaling"/> instance.</returns>
        public static VariableScaling FromBounds(
            double[] stateLower,
            double[] stateUpper,
            double[] controlLower,
            double[] controlUpper)
        {
            ArgumentNullException.ThrowIfNull(stateLower);
            ArgumentNullException.ThrowIfNull(stateUpper);
            ArgumentNullException.ThrowIfNull(controlLower);
            ArgumentNullException.ThrowIfNull(controlUpper);

            if (stateLower.Length != stateUpper.Length)
            {
                throw new ArgumentException("State bounds must have the same length.");
            }

            if (controlLower.Length != controlUpper.Length)
            {
                throw new ArgumentException("Control bounds must have the same length.");
            }

            var stateScales = new double[stateLower.Length];
            var stateCenters = new double[stateLower.Length];
            var controlScales = new double[controlLower.Length];
            var controlCenters = new double[controlLower.Length];

            for (var i = 0; i < stateLower.Length; i++)
            {
                (stateScales[i], stateCenters[i]) = ComputeScaleAndCenter(stateLower[i], stateUpper[i]);
            }

            for (var i = 0; i < controlLower.Length; i++)
            {
                (controlScales[i], controlCenters[i]) = ComputeScaleAndCenter(controlLower[i], controlUpper[i]);
            }

            return new VariableScaling(stateScales, stateCenters, controlScales, controlCenters);
        }

        /// <summary>
        /// Creates identity scaling (no transformation).
        /// </summary>
        /// <param name="stateDim">State dimension.</param>
        /// <param name="controlDim">Control dimension.</param>
        /// <returns>A scaling that leaves variables unchanged.</returns>
        public static VariableScaling Identity(int stateDim, int controlDim)
        {
            var stateScales = new double[stateDim];
            var stateCenters = new double[stateDim];
            var controlScales = new double[controlDim];
            var controlCenters = new double[controlDim];

            Array.Fill(stateScales, 1.0);
            Array.Fill(controlScales, 1.0);

            return new VariableScaling(stateScales, stateCenters, controlScales, controlCenters);
        }

        /// <summary>
        /// Scales a state vector from original coordinates to scaled coordinates.
        /// </summary>
        /// <param name="original">State in original coordinates.</param>
        /// <returns>State in scaled coordinates [-1, 1].</returns>
        public double[] ScaleState(double[] original)
        {
            ArgumentNullException.ThrowIfNull(original);

            if (original.Length != StateDim)
            {
                throw new ArgumentException($"Expected state dimension {StateDim}, got {original.Length}.");
            }

            var scaled = new double[StateDim];
            for (var i = 0; i < StateDim; i++)
            {
                scaled[i] = (original[i] - StateCenters[i]) / StateScales[i];
            }

            return scaled;
        }

        /// <summary>
        /// Unscales a state vector from scaled coordinates to original coordinates.
        /// </summary>
        /// <param name="scaled">State in scaled coordinates.</param>
        /// <returns>State in original coordinates.</returns>
        public double[] UnscaleState(double[] scaled)
        {
            ArgumentNullException.ThrowIfNull(scaled);

            if (scaled.Length != StateDim)
            {
                throw new ArgumentException($"Expected state dimension {StateDim}, got {scaled.Length}.");
            }

            var original = new double[StateDim];
            for (var i = 0; i < StateDim; i++)
            {
                original[i] = (scaled[i] * StateScales[i]) + StateCenters[i];
            }

            return original;
        }

        /// <summary>
        /// Scales a control vector from original coordinates to scaled coordinates.
        /// </summary>
        /// <param name="original">Control in original coordinates.</param>
        /// <returns>Control in scaled coordinates [-1, 1].</returns>
        public double[] ScaleControl(double[] original)
        {
            ArgumentNullException.ThrowIfNull(original);

            if (original.Length != ControlDim)
            {
                throw new ArgumentException($"Expected control dimension {ControlDim}, got {original.Length}.");
            }

            var scaled = new double[ControlDim];
            for (var i = 0; i < ControlDim; i++)
            {
                scaled[i] = (original[i] - ControlCenters[i]) / ControlScales[i];
            }

            return scaled;
        }

        /// <summary>
        /// Unscales a control vector from scaled coordinates to original coordinates.
        /// </summary>
        /// <param name="scaled">Control in scaled coordinates.</param>
        /// <returns>Control in original coordinates.</returns>
        public double[] UnscaleControl(double[] scaled)
        {
            ArgumentNullException.ThrowIfNull(scaled);

            if (scaled.Length != ControlDim)
            {
                throw new ArgumentException($"Expected control dimension {ControlDim}, got {scaled.Length}.");
            }

            var original = new double[ControlDim];
            for (var i = 0; i < ControlDim; i++)
            {
                original[i] = (scaled[i] * ControlScales[i]) + ControlCenters[i];
            }

            return original;
        }

        /// <summary>
        /// Scales state bounds to the normalized space.
        /// For properly scaled variables, this returns [-1, 1].
        /// </summary>
        /// <param name="lower">Original lower bounds.</param>
        /// <param name="upper">Original upper bounds.</param>
        /// <returns>Tuple of (scaled lower, scaled upper) bounds.</returns>
        public (double[] lower, double[] upper) ScaleStateBounds(double[] lower, double[] upper)
        {
            var scaledLower = ScaleState(lower);
            var scaledUpper = ScaleState(upper);
            return (scaledLower, scaledUpper);
        }

        /// <summary>
        /// Scales control bounds to the normalized space.
        /// For properly scaled variables, this returns [-1, 1].
        /// </summary>
        /// <param name="lower">Original lower bounds.</param>
        /// <param name="upper">Original upper bounds.</param>
        /// <returns>Tuple of (scaled lower, scaled upper) bounds.</returns>
        public (double[] lower, double[] upper) ScaleControlBounds(double[] lower, double[] upper)
        {
            var scaledLower = ScaleControl(lower);
            var scaledUpper = ScaleControl(upper);
            return (scaledLower, scaledUpper);
        }

        private static (double scale, double center) ComputeScaleAndCenter(double lower, double upper)
        {
            // Handle unbounded variables
            if (double.IsInfinity(lower) || double.IsInfinity(upper))
            {
                return (1.0, 0.0);
            }

            var range = upper - lower;

            // Handle fixed variables (zero or tiny range)
            if (range < MinimumScale)
            {
                return (1.0, (lower + upper) / 2.0);
            }

            var scale = range / 2.0;
            var center = (lower + upper) / 2.0;

            return (scale, center);
        }
    }
}
