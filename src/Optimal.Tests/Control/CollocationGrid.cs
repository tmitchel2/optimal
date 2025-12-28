/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;

namespace Optimal.Control
{
    /// <summary>
    /// Represents a time discretization grid for collocation methods.
    /// </summary>
    public sealed class CollocationGrid
    {
        /// <summary>
        /// Gets the initial time.
        /// </summary>
        public double InitialTime { get; }

        /// <summary>
        /// Gets the final time.
        /// </summary>
        public double FinalTime { get; }

        /// <summary>
        /// Gets the number of collocation segments.
        /// </summary>
        public int Segments { get; }

        /// <summary>
        /// Gets the time points (nodes) including initial and final times.
        /// Length = Segments + 1
        /// </summary>
        public double[] TimePoints { get; }

        /// <summary>
        /// Gets the time step for each segment.
        /// </summary>
        public double TimeStep { get; }

        /// <summary>
        /// Creates a uniform collocation grid.
        /// </summary>
        /// <param name="t0">Initial time.</param>
        /// <param name="tf">Final time.</param>
        /// <param name="segments">Number of collocation segments.</param>
        public CollocationGrid(double t0, double tf, int segments)
        {
            if (segments <= 0)
            {
                throw new ArgumentException("Number of segments must be positive.", nameof(segments));
            }

            if (tf <= t0)
            {
                throw new ArgumentException("Final time must be greater than initial time.", nameof(tf));
            }

            InitialTime = t0;
            FinalTime = tf;
            Segments = segments;
            TimeStep = (tf - t0) / segments;

            TimePoints = new double[segments + 1];
            for (var i = 0; i <= segments; i++)
            {
                TimePoints[i] = t0 + i * TimeStep;
            }
        }

        /// <summary>
        /// Gets the midpoint time for a given segment.
        /// </summary>
        /// <param name="segment">Segment index (0 to Segments-1).</param>
        /// <returns>The midpoint time.</returns>
        public double GetMidpoint(int segment)
        {
            if (segment < 0 || segment >= Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(segment));
            }

            return 0.5 * (TimePoints[segment] + TimePoints[segment + 1]);
        }

        /// <summary>
        /// Gets the time step for a given segment (supports non-uniform grids in future).
        /// </summary>
        /// <param name="segment">Segment index (0 to Segments-1).</param>
        /// <returns>The time step.</returns>
        public double GetTimeStep(int segment)
        {
            if (segment < 0 || segment >= Segments)
            {
                throw new ArgumentOutOfRangeException(nameof(segment));
            }

            return TimePoints[segment + 1] - TimePoints[segment];
        }
    }
}
