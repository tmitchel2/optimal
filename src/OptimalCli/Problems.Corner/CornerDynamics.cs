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
    /// Racecar dynamics with arc-length parameterization.
    ///
    /// State: [V, ax, ay, n, alpha, lambda, Omega, t]
    ///   - V: Speed (m/s)
    ///   - ax: Longitudinal acceleration (m/s^2)
    ///   - ay: Lateral acceleration (m/s^2)
    ///   - n: Lateral offset from centerline (m, positive = right)
    ///   - alpha: Heading angle relative to road (rad)
    ///   - lambda: Slip angle (rad)
    ///   - Omega: Yaw rate (rad/s)
    ///   - t: Elapsed time (s)
    ///
    /// Control: [delta, T]
    ///   - delta: Steering angle (rad)
    ///   - T: Thrust (normalized, -1 to 1)
    ///
    /// Independent variable: s (arc length along track centerline)
    ///
    /// Key insight: All dynamics are dx/ds = (dx/dt) * (dt/ds) where dt/ds = (1-n*kappa)/(V*cos(alpha))
    /// </summary>
    [OptimalCode]
    public static class CornerDynamics
    {
        // === Arc-length transformation ===

        /// <summary>
        /// Time rate: dt/ds = (1 - n*kappa) / (V * cos(alpha))
        /// This is the core transformation from time-parameterized to arc-length-parameterized dynamics.
        /// </summary>
        public static double TimeRate(double kappa, double n, double alpha, double V)
        {
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return safeDenom / (V * safeCos);
        }

        // === Normal forces (4 wheels with load transfer) ===

        /// <summary>
        /// Normal load on front-right wheel including static, dynamic, and aerodynamic contributions.
        /// </summary>
        public static double NormalForceFR(double M, double g, double a, double b, double tw, double h,
            double chi, double ax, double ay, double rho, double V, double ClA, double CoP)
        {
            var L = a + b;
            var Fz_static = M * g * b / (2 * L);
            var Fz_longitudinal = -M * ax * h / (2 * L);
            var Fz_lateral = M * ay * h * chi / (2 * tw);
            var Fz_aero = 0.5 * rho * V * V * ClA * (L - CoP) / (2 * L);
            return Fz_static + Fz_longitudinal + Fz_lateral + Fz_aero;
        }

        /// <summary>
        /// Normal load on front-left wheel.
        /// </summary>
        public static double NormalForceFL(double M, double g, double a, double b, double tw, double h,
            double chi, double ax, double ay, double rho, double V, double ClA, double CoP)
        {
            var L = a + b;
            var Fz_static = M * g * b / (2 * L);
            var Fz_longitudinal = -M * ax * h / (2 * L);
            var Fz_lateral = -M * ay * h * chi / (2 * tw);
            var Fz_aero = 0.5 * rho * V * V * ClA * (L - CoP) / (2 * L);
            return Fz_static + Fz_longitudinal + Fz_lateral + Fz_aero;
        }

        /// <summary>
        /// Normal load on rear-right wheel.
        /// </summary>
        public static double NormalForceRR(double M, double g, double a, double b, double tw, double h,
            double chi, double ax, double ay, double rho, double V, double ClA, double CoP)
        {
            var L = a + b;
            var Fz_static = M * g * a / (2 * L);
            var Fz_longitudinal = M * ax * h / (2 * L);
            var Fz_lateral = M * ay * h * (1 - chi) / (2 * tw);
            var Fz_aero = 0.5 * rho * V * V * ClA * CoP / (2 * L);
            return Fz_static + Fz_longitudinal + Fz_lateral + Fz_aero;
        }

        /// <summary>
        /// Normal load on rear-left wheel.
        /// </summary>
        public static double NormalForceRL(double M, double g, double a, double b, double tw, double h,
            double chi, double ax, double ay, double rho, double V, double ClA, double CoP)
        {
            var L = a + b;
            var Fz_static = M * g * a / (2 * L);
            var Fz_longitudinal = M * ax * h / (2 * L);
            var Fz_lateral = -M * ay * h * (1 - chi) / (2 * tw);
            var Fz_aero = 0.5 * rho * V * V * ClA * CoP / (2 * L);
            return Fz_static + Fz_longitudinal + Fz_lateral + Fz_aero;
        }

        // === Tire friction ===

        /// <summary>
        /// Load-dependent friction coefficient: mu = mu0 * (1 + Kmu * (Fz/Fz_nom - 1))
        /// </summary>
        public static double FrictionCoeff(double Fz, double Fz_nom, double mu0, double Kmu)
        {
            return mu0 * (1.0 + Kmu * (Fz / Fz_nom - 1.0));
        }

        // === State dynamics (derivatives w.r.t. arc length s) ===

        /// <summary>
        /// dV/ds - Speed rate including aerodynamic drag.
        /// </summary>
        public static double SpeedRateS(double kappa, double n, double alpha, double V, double ax,
            double rho, double CdA, double M)
        {
            var drag = 0.5 * rho * V * V * CdA / M;
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            var dtds = safeDenom / (V * safeCos);
            return (ax - drag) * dtds;
        }

        /// <summary>
        /// dax/ds - Longitudinal acceleration dynamics with first-order lag.
        /// </summary>
        public static double AxRateS(double kappa, double n, double alpha, double V, double ax,
            double T, double Fmax, double M, double tau_ax)
        {
            var ax_cmd = T * Fmax / M;
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            var dtds = safeDenom / (V * safeCos);
            return (ax_cmd - ax) / tau_ax * dtds;
        }

        /// <summary>
        /// day/ds - Lateral acceleration dynamics with first-order lag.
        /// </summary>
        public static double AyRateS(double kappa, double n, double alpha, double V, double ay,
            double Omega, double tau_ay)
        {
            var ay_kinematic = V * Omega;
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            var dtds = safeDenom / (V * safeCos);
            return (ay_kinematic - ay) / tau_ay * dtds;
        }

        /// <summary>
        /// dn/ds - Lateral offset rate.
        /// </summary>
        public static double LateralRateS(double kappa, double n, double alpha)
        {
            return (1.0 - n * kappa) * Math.Tan(alpha);
        }

        /// <summary>
        /// dalpha/ds - Heading rate relative to road.
        /// </summary>
        public static double AlphaRateS(double kappa, double n, double alpha, double V, double Omega)
        {
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            var dtds = safeDenom / (V * safeCos);
            return Omega * dtds - kappa;
        }

        /// <summary>
        /// dlambda/ds - Slip angle rate with relaxation dynamics.
        /// </summary>
#pragma warning disable RCS1163 // Unused parameter - needed for AutoDiff gradient generation
        public static double LambdaRateS(double kappa, double n, double alpha, double V, double lambda,
            double Omega, double delta, double a, double b, double tau_lambda)
#pragma warning restore RCS1163
        {
            _ = a; // Parameter reserved for future use in more complete slip angle model
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            var dtds = safeDenom / (V * safeCos);
            var lambda_target = Math.Atan2(b * Omega, V) - delta;
            return (lambda_target - lambda) / tau_lambda * dtds;
        }

        /// <summary>
        /// dOmega/ds - Yaw rate dynamics based on moment balance.
        /// </summary>
#pragma warning disable RCS1163 // Unused parameter - needed for AutoDiff gradient generation
        public static double OmegaRateS(double kappa, double n, double alpha, double V, double Omega,
            double delta, double ay, double a, double b, double Iz, double M)
#pragma warning restore RCS1163
        {
            _ = Omega; // Parameter reserved for yaw damping in more complete model
            var L = a + b;
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            var dtds = safeDenom / (V * safeCos);
            var Fy_front = M * ay * b / L;
            var moment = Fy_front * a * Math.Cos(delta);
            var Omega_dot = moment / Iz;
            return Omega_dot * dtds;
        }

        /// <summary>
        /// dt/ds - Time rate (same as TimeRate, explicit for state dynamics).
        /// </summary>
        public static double TimeRateS(double kappa, double n, double alpha, double V)
        {
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return safeDenom / (V * safeCos);
        }

        // === Path constraints ===

        /// <summary>
        /// Friction circle utilization for a wheel: (Fx/Fx_max)^2 + (Fy/Fy_max)^2
        /// Should be &lt;= 1 for the tire to maintain grip.
        /// </summary>
        public static double FrictionUtilization(double Fx, double Fy, double mu_x, double mu_y, double Fz)
        {
            var Fx_max = mu_x * Fz;
            var Fy_max = mu_y * Fz;
            var safeFxMax = Math.Abs(Fx_max) < 1.0 ? 1.0 : Fx_max;
            var safeFyMax = Math.Abs(Fy_max) < 1.0 ? 1.0 : Fy_max;
            return (Fx / safeFxMax) * (Fx / safeFxMax) + (Fy / safeFyMax) * (Fy / safeFyMax);
        }

        /// <summary>
        /// Power constraint: T * Fmax * V - Pmax &lt;= 0
        /// </summary>
        public static double PowerConstraint(double T, double V, double Fmax, double Pmax)
        {
            return T * Fmax * V - Pmax;
        }

        // === Running cost ===

        /// <summary>
        /// Running cost = dt/ds. Integrating this over s gives total elapsed time.
        /// </summary>
        public static double RunningCostS(double kappa, double n, double alpha, double V)
        {
            var denom = 1.0 - n * kappa;
            var safeDenom = denom < 0.1 ? 0.1 : denom;
            var cosAlpha = Math.Cos(alpha);
            var safeCos = Math.Abs(cosAlpha) < 0.1 ? (cosAlpha >= 0 ? 0.1 : -0.1) : cosAlpha;
            return safeDenom / (V * safeCos);
        }
    }
}
