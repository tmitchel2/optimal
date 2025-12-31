/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Goddard
{
    /// <summary>
    /// Goddard rocket problem dynamics with AutoDiff support.
    /// State: [h (altitude), v (velocity), m (mass)]
    /// Control: T (thrust)
    ///
    /// The Goddard rocket problem involves maximizing the final altitude of a vertically
    /// ascending rocket subject to drag, gravity, and fuel consumption constraints.
    /// This is a classic optimal control problem (Goddard, 1919).
    /// </summary>
    [OptimalCode]
    public static class GoddardRocketDynamics
    {
        /// <summary>
        /// Altitude rate: ḣ = v (velocity)
        /// </summary>
        public static double AltitudeRate(double h, double v, double m, double T, double g, double Dc, double c, double h0)
        {
            return v;
        }

        /// <summary>
        /// Velocity rate: v̇ = (T - D(h,v))/m - g
        /// where D(h,v) = Dc·v²·exp(-h/h0) is the drag force
        /// </summary>
        public static double VelocityRate(double h, double v, double m, double T, double g, double Dc, double c, double h0)
        {
            var drag = Dc * v * v * Math.Exp(-h / h0);
            return (T - drag) / m - g;
        }

        /// <summary>
        /// Mass rate: ṁ = -T/c (fuel consumption proportional to thrust)
        /// where c is the exhaust velocity (specific impulse × g)
        /// </summary>
        public static double MassRate(double h, double v, double m, double T, double g, double Dc, double c, double h0)
        {
            return -T / c;
        }

        /// <summary>
        /// Running cost: L = 0 (we maximize final altitude via terminal cost, no running cost)
        /// </summary>
        public static double RunningCost(double h, double v, double m, double T)
        {
            return 0.0;
        }

        /// <summary>
        /// Terminal cost: Φ = -h (negative altitude to maximize it via minimization)
        /// </summary>
        public static double TerminalCost(double h, double v, double m)
        {
            return -h;
        }
    }
}
