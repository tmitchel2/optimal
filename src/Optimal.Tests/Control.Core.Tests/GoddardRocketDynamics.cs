/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.Control.Optimization;
using Optimal.Control.Solvers;

namespace Optimal.Control.Core.Tests
{
    /// <summary>
    /// Goddard Rocket problem dynamics with AutoDiff support.
    /// State: [h (altitude), v (velocity), m (mass)]
    /// Control: T (thrust)
    /// 
    /// Note: Mass clamping is handled outside AutoDiff functions to avoid unsupported operations.
    /// </summary>
    [OptimalCode]
    public static class GoddardRocketDynamics
    {
        /// <summary>
        /// Altitude rate: ḣ = v
        /// </summary>
        public static double AltitudeRate(double h, double v, double m, double T)
        {
            return v;
        }

        /// <summary>
        /// Velocity rate: v̇ = T/m - g
        /// No drag version for stability.
        /// WARNING: m must be clamped to positive value before calling this function.
        /// </summary>
        public static double VelocityRate(double h, double v, double m, double T, double g)
        {
            return T / m - g;
        }

        /// <summary>
        /// Mass rate: ṁ = -T/c (fuel consumption)
        /// </summary>
        public static double MassRate(double h, double v, double m, double T, double c)
        {
            return -T / c;
        }

        /// <summary>
        /// Terminal cost: Maximize altitude = minimize -h
        /// Φ(h, v, m) = -h
        /// </summary>
        public static double TerminalCost(double h, double v, double m)
        {
            return -h;
        }

        /// <summary>
        /// Running cost: Small regularization for smooth control
        /// L(h, v, m, T) = 0.001 * T²
        /// </summary>
        public static double RunningCost(double h, double v, double m, double T)
        {
            return 0.001 * T * T;
        }
    }
}
