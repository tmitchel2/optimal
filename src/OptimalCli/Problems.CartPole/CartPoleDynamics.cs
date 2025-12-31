/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.CartPole
{
    /// <summary>
    /// Cart-pole problem dynamics with AutoDiff support.
    /// State: [x (cart position), ẋ (cart velocity), θ (pole angle), θ̇ (pole angular velocity)]
    /// Control: F (force on cart)
    ///
    /// The cart-pole problem involves balancing an inverted pendulum on a moving cart
    /// using forces applied to the cart. The dynamics are highly nonlinear.
    /// </summary>
    [OptimalCode]
    public static class CartPoleDynamics
    {
        /// <summary>
        /// Cart position rate: ẋ (trivial - just the velocity state)
        /// </summary>
        public static double XRate(double x, double xdot, double theta, double thetadot, double F, double M, double m, double L, double g)
        {
            return xdot;
        }

        /// <summary>
        /// Cart acceleration: ẍ = (F + m·L·θ̇²·sin(θ) - m·g·sin(θ)·cos(θ)) / (M + m·sin²(θ))
        /// Full nonlinear dynamics
        /// </summary>
        public static double XddotRate(double x, double xdot, double theta, double thetadot, double F, double M, double m, double L, double g)
        {
            var sinTheta = Math.Sin(theta);
            var cosTheta = Math.Cos(theta);
            var sin2Theta = sinTheta * sinTheta;

            var denom = M + m * sin2Theta;
            return (F + m * L * thetadot * thetadot * sinTheta - m * g * sinTheta * cosTheta) / denom;
        }

        /// <summary>
        /// Pole angular rate: θ̇ (trivial - just the angular velocity state)
        /// </summary>
        public static double ThetaRate(double x, double xdot, double theta, double thetadot, double F, double M, double m, double L, double g)
        {
            return thetadot;
        }

        /// <summary>
        /// Pole angular acceleration: θ̈ = (g·sin(θ) - cos(θ)·(F + m·L·θ̇²·sin(θ))/(M+m)) / (L·(4/3 - m·cos²(θ)/(M+m)))
        /// Full nonlinear dynamics
        /// </summary>
        public static double ThetaddotRate(double x, double xdot, double theta, double thetadot, double F, double M, double m, double L, double g)
        {
            var sinTheta = Math.Sin(theta);
            var cosTheta = Math.Cos(theta);
            var cos2Theta = cosTheta * cosTheta;

            var denom = L * (4.0 / 3.0 - m * cos2Theta / (M + m));
            return (g * sinTheta - cosTheta * (F + m * L * thetadot * thetadot * sinTheta) / (M + m)) / denom;
        }

        /// <summary>
        /// Running cost: L = 20·x² + 2·ẋ² + 50·θ² + 2·θ̇² + 0.5·F²
        /// Quadratic cost penalizing state deviations and control effort
        /// </summary>
        public static double RunningCost(double x, double xdot, double theta, double thetadot, double F)
        {
            return 20.0 * x * x + 2.0 * xdot * xdot + 50.0 * theta * theta + 2.0 * thetadot * thetadot + 0.5 * F * F;
        }
    }
}
