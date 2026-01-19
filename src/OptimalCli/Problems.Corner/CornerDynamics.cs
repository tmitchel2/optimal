/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Corner
{
    /// <summary>
    /// Corner problem dynamics with AutoDiff support using curvilinear coordinates.
    ///
    /// State: [n (lateral deviation), θ (heading angle), v (velocity), T_f (final time)]
    /// Control: [a (acceleration), ω (steering rate)]
    ///
    /// Coordinate conventions (LEFT-HAND RULE):
    /// - s: Distance along centerline (0 at start, increases monotonically)
    /// - n: Perpendicular distance from centerline (positive = right of centerline when facing forward)
    /// - θ: Heading angle where positive = clockwise rotation
    ///   - θ = 0: heading east
    ///   - θ = +π/2: heading south
    ///   - θ = -π/2: heading north
    ///
    /// The road geometry (thetaRoad, curvature) is provided by TrackGeometry and passed
    /// as parameters to the dynamics methods.
    /// </summary>
    [OptimalCode]
    public static class CornerDynamics
    {
        /// <summary>
        /// Progress rate along centerline: ṡ = v × cos(θ - θ_road) / (1 - n × κ)
        /// </summary>
        /// <param name="thetaRoad">Road heading at current position.</param>
        /// <param name="curvature">Road curvature at current position.</param>
        /// <param name="n">Lateral deviation from centerline.</param>
        /// <param name="theta">Vehicle heading angle.</param>
        /// <param name="v">Vehicle velocity.</param>
        public static double ProgressRate(double thetaRoad, double curvature, double n, double theta, double v)
        {
            var headingError = theta - thetaRoad;
            var denominator = 1.0 - n * curvature;

            // Prevent division by zero (shouldn't happen within road bounds)
            if (denominator < 0.1)
            {
                denominator = 0.1;
            }

            return v * Math.Cos(headingError) / denominator;
        }

        /// <summary>
        /// Lateral deviation rate: ṅ = v × sin(θ - θ_road)
        ///
        /// With left-hand rule (positive θ = clockwise, positive n = right of centerline):
        /// - When heading more right than road (θ > θ_road), headingError > 0, ṅ > 0, moving right
        /// - When heading more left than road (θ &lt; θ_road), headingError &lt; 0, ṅ &lt; 0, moving left
        /// </summary>
        /// <param name="thetaRoad">Road heading at current position.</param>
        /// <param name="theta">Vehicle heading angle.</param>
        /// <param name="v">Vehicle velocity.</param>
        public static double LateralRate(double thetaRoad, double theta, double v)
        {
            var headingError = theta - thetaRoad;
            return v * Math.Sin(headingError);
        }

        /// <summary>
        /// Heading rate: θ̇ = ω (steering rate)
        /// </summary>
        public static double ThetaRate(double omega)
        {
            return omega;
        }

        /// <summary>
        /// Velocity rate: v̇ = a (acceleration)
        /// </summary>
        public static double VelocityRate(double a)
        {
            return a;
        }

        /// <summary>
        /// Running cost: Minimize time (constant = 1)
        /// This minimizes total time when integrated over the trajectory.
        /// </summary>
        public static double RunningCost()
        {
            return 1.0;
        }
    }
}
