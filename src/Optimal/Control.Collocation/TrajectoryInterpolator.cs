/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.Control.Collocation
{
    /// <summary>
    /// Interpolates trajectory states using cubic Hermite basis functions.
    /// Uses the same interpolation as the Hermite-Simpson collocation to produce
    /// mathematically accurate smooth curves between collocation nodes.
    /// </summary>
    public sealed class TrajectoryInterpolator
    {
        private readonly double[] _times;
        private readonly double[][] _states;
        private readonly double[][] _derivatives;

        /// <summary>
        /// Initializes a new instance of the <see cref="TrajectoryInterpolator"/> class from raw trajectory data.
        /// </summary>
        /// <param name="times">Time points for each trajectory node.</param>
        /// <param name="states">State vectors at each time point.</param>
        /// <param name="derivatives">State derivatives (dx/dt) at each time point.</param>
        /// <exception cref="ArgumentNullException">Thrown if any parameter is null.</exception>
        /// <exception cref="ArgumentException">Thrown if arrays have mismatched lengths.</exception>
        public TrajectoryInterpolator(double[] times, double[][] states, double[][] derivatives)
        {
            _times = times ?? throw new ArgumentNullException(nameof(times));
            _states = states ?? throw new ArgumentNullException(nameof(states));
            _derivatives = derivatives ?? throw new ArgumentNullException(nameof(derivatives));

            if (_times.Length != _states.Length || _times.Length != _derivatives.Length)
            {
                throw new ArgumentException("Times, states, and derivatives must have the same length.");
            }
        }

        /// <summary>
        /// Creates an interpolator from a CollocationResult (convenience method).
        /// </summary>
        /// <param name="result">The collocation result containing states, times, and derivatives.</param>
        /// <returns>A new TrajectoryInterpolator instance.</returns>
        /// <exception cref="ArgumentNullException">Thrown if result is null.</exception>
        /// <exception cref="ArgumentException">Thrown if result does not contain state derivatives.</exception>
        public static TrajectoryInterpolator FromResult(CollocationResult result)
        {
            ArgumentNullException.ThrowIfNull(result);

            if (result.StateDerivatives == null)
            {
                throw new ArgumentException("CollocationResult must contain StateDerivatives for interpolation.", nameof(result));
            }

            return new TrajectoryInterpolator(result.Times, result.States, result.StateDerivatives);
        }

        /// <summary>
        /// Gets the number of segments in the trajectory.
        /// </summary>
        public int Segments => _times.Length - 1;

        /// <summary>
        /// Gets the state dimension.
        /// </summary>
        public int StateDim => _states.Length > 0 ? _states[0].Length : 0;

        /// <summary>
        /// Interpolates the state at an arbitrary time using cubic Hermite basis functions.
        /// </summary>
        /// <param name="t">The time at which to interpolate. Must be within [t0, tf].</param>
        /// <returns>The interpolated state vector.</returns>
        /// <exception cref="ArgumentOutOfRangeException">Thrown if t is outside the time range.</exception>
        public double[] InterpolateState(double t)
        {
            var t0 = _times[0];
            var tf = _times[^1];

            // Handle boundary cases with small tolerance
            const double Tolerance = 1e-10;
            if (t < t0 - Tolerance || t > tf + Tolerance)
            {
                throw new ArgumentOutOfRangeException(nameof(t), $"Time {t} is outside the trajectory range [{t0}, {tf}].");
            }

            // Clamp to exact boundaries
            if (t <= t0)
            {
                return (double[])_states[0].Clone();
            }

            if (t >= tf)
            {
                return (double[])_states[^1].Clone();
            }

            // Find the segment containing t
            var k = FindSegment(t);
            return InterpolateOnSegment(k, t);
        }

        /// <summary>
        /// Gets interpolated points along the trajectory for smooth drawing.
        /// </summary>
        /// <param name="pointsPerSegment">Number of interpolated points per segment (default 10).</param>
        /// <returns>An enumerable of (time, state) pairs.</returns>
        public IEnumerable<(double t, double[] state)> GetInterpolatedPoints(int pointsPerSegment = 10)
        {
            if (pointsPerSegment < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(pointsPerSegment), "Must be at least 1.");
            }

            var segments = Segments;

            for (var k = 0; k < segments; k++)
            {
                var tk = _times[k];
                var tk1 = _times[k + 1];
                var h = tk1 - tk;

                for (var i = 0; i < pointsPerSegment; i++)
                {
                    var tau = (double)i / pointsPerSegment;
                    var t = tk + (tau * h);
                    yield return (t, InterpolateOnSegment(k, t));
                }
            }

            // Include the final point
            yield return (_times[^1], (double[])_states[^1].Clone());
        }

        /// <summary>
        /// Finds the segment index k such that t is in [t_k, t_{k+1}).
        /// </summary>
        private int FindSegment(double t)
        {
            // Binary search for the segment
            var lo = 0;
            var hi = _times.Length - 2; // Last valid segment index

            while (lo < hi)
            {
                var mid = (lo + hi + 1) / 2;
                if (_times[mid] <= t)
                {
                    lo = mid;
                }
                else
                {
                    hi = mid - 1;
                }
            }

            return lo;
        }

        /// <summary>
        /// Interpolates the state on segment k at time t using cubic Hermite basis functions.
        /// </summary>
        /// <remarks>
        /// Uses the cubic Hermite interpolation formula:
        /// <code>
        /// tau = (t - t_k) / h   (normalized time in [0, 1])
        /// H00 = 2*tau^3 - 3*tau^2 + 1
        /// H10 = tau^3 - 2*tau^2 + tau
        /// H01 = -2*tau^3 + 3*tau^2
        /// H11 = tau^3 - tau^2
        /// x(t) = H00*x_k + H10*h*f_k + H01*x_{k+1} + H11*h*f_{k+1}
        /// </code>
        /// </remarks>
        private double[] InterpolateOnSegment(int k, double t)
        {
            var tk = _times[k];
            var tk1 = _times[k + 1];
            var h = tk1 - tk;

            var xk = _states[k];
            var xk1 = _states[k + 1];
            var fk = _derivatives[k];
            var fk1 = _derivatives[k + 1];

            // Normalized time within segment
            var tau = (t - tk) / h;
            var tau2 = tau * tau;
            var tau3 = tau2 * tau;

            // Cubic Hermite basis functions
            var h00 = (2.0 * tau3) - (3.0 * tau2) + 1.0;
            var h10 = tau3 - (2.0 * tau2) + tau;
            var h01 = (-2.0 * tau3) + (3.0 * tau2);
            var h11 = tau3 - tau2;

            // Interpolate each state component
            var n = xk.Length;
            var result = new double[n];

            for (var i = 0; i < n; i++)
            {
                result[i] = (h00 * xk[i]) + (h10 * h * fk[i]) + (h01 * xk1[i]) + (h11 * h * fk1[i]);
            }

            return result;
        }
    }
}
