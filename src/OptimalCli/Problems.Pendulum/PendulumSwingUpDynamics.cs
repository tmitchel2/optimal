/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Pendulum
{
    /// <summary>
    /// Pendulum swing-up problem dynamics with AutoDiff support.
    /// State: [θ (angle), θ̇ (angular velocity)]
    /// Control: u (torque)
    ///
    /// The pendulum swing-up problem involves bringing a pendulum from hanging down
    /// (θ=0) to fully inverted upright position (θ=π) using minimal control effort.
    /// </summary>
    [OptimalCode]
    public static class PendulumSwingUpDynamics
    {
        /// <summary>
        /// Angular position rate: θ̇ (angular velocity)
        /// </summary>
#pragma warning disable RCS1163 // Unused parameter
        public static double AngleRate(double theta, double thetadot, double u, double g, double L, double m)
#pragma warning restore RCS1163 // Unused parameter
        {
            return thetadot;
        }

        /// <summary>
        /// Angular velocity rate (angular acceleration): θ̈ = -g/L·sin(θ) + u/(m·L²)
        /// where:
        /// - g is gravitational acceleration
        /// - L is pendulum length
        /// - m is pendulum mass
        /// - u is applied torque
        /// </summary>
#pragma warning disable RCS1163 // Unused parameter
        public static double AngularVelocityRate(double theta, double thetadot, double u, double g, double L, double m)
#pragma warning restore RCS1163 // Unused parameter
        {
            return -g / L * Math.Sin(theta) + u / (m * L * L);
        }

        /// <summary>
        /// Running cost: L = 0.5·u² (minimize control effort)
        /// </summary>
#pragma warning disable RCS1163 // Unused parameter
        public static double RunningCost(double theta, double thetadot, double u)
#pragma warning restore RCS1163 // Unused parameter
        {
            return 0.5 * u * u;
        }

        /// <summary>
        /// Terminal cost: Φ = 0 (no terminal cost, only running cost)
        /// </summary>
#pragma warning disable RCS1163 // Unused parameter
        public static double TerminalCost(double theta, double thetadot)
#pragma warning restore RCS1163 // Unused parameter
        {
            return 0.0;
        }
    }
}
