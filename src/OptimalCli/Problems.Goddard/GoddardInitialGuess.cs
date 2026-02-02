/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace OptimalCli.Problems.Goddard;

/// <summary>
/// Generates physics-based initial guesses for the Goddard rocket problem.
/// Uses forward simulation with a heuristic thrust policy to create a 
/// dynamically feasible initial trajectory.
/// </summary>
public static class GoddardInitialGuess
{
    /// <summary>
    /// Parameters for the YOptimization Goddard rocket problem.
    /// </summary>
    public sealed record GoddardParameters(
        double D0,      // Drag coefficient at sea level
        double Beta,    // Inverse scale height (1/m)
        double C,       // Exhaust velocity (m/s)
        double G0,      // Gravitational acceleration (m/s²)
        double R0,      // Earth radius (m)
        double M0,      // Initial mass (kg)
        double Mf,      // Final/empty mass (kg)
        double Fm       // Maximum fuel mass flow rate (kg/s)
    );

    /// <summary>
    /// Creates the standard YOptimization parameters (PROPT examples 44/45).
    /// </summary>
    public static GoddardParameters CreateYOptParameters()
    {
        return new GoddardParameters(
            D0: 0.01227,
            Beta: 0.145e-3,
            C: 2060.0,
            G0: 9.81,
            R0: 6.371e6,
            M0: 214.839,
            Mf: 67.9833,
            Fm: 9.525515
        );
    }

    /// <summary>
    /// Generates an initial guess by simulating the rocket with a heuristic control policy.
    /// Samples the trajectory at arbitrary time points for compatibility with any solver.
    /// </summary>
    /// <param name="timePoints">Array of time points at which to sample the trajectory.</param>
    /// <param name="parameters">Goddard rocket parameters.</param>
    /// <returns>States and controls at each time point.</returns>
    public static (double[][] states, double[][] controls) GenerateSimulatedTrajectoryAtTimes(
        double[] timePoints,
        GoddardParameters parameters)
    {
        var nPoints = timePoints.Length;
        var states = new double[nPoints][];
        var controls = new double[nPoints][];

        // Initial state: [h, v, m] = [0, 0, m0]
        var h = 0.0;
        var v = 0.0;
        var m = parameters.M0;
        var currentTime = 0.0;
        var finalTime = timePoints[^1];

        for (var k = 0; k < nPoints; k++)
        {
            var targetTime = timePoints[k];

            // Integrate forward to this time point using very fine steps
            while (currentTime < targetTime - 1e-12)
            {
                var dt = Math.Min(0.01, targetTime - currentTime); // Use very small steps for accuracy
                var thrust = ComputeHeuristicThrust(h, v, m, currentTime, parameters, finalTime);
                (h, v, m) = IntegrateRK4(h, v, m, thrust, dt, parameters);

                // Clamp to physical bounds
                h = Math.Max(0, h);
                v = Math.Max(0, v);
                m = Math.Clamp(m, parameters.Mf, parameters.M0);

                currentTime += dt;
            }

            // Compute thrust at this time point
            var controlThrust = ComputeHeuristicThrust(h, v, m, targetTime, parameters, finalTime);

            // Store current state and control
            states[k] = [h, v, m];
            controls[k] = [controlThrust];
        }

        return (states, controls);
    }

    /// <summary>
    /// Generates an initial guess by simulating the rocket with a heuristic control policy.
    /// The policy uses maximum thrust until fuel is depleted, then coasts.
    /// </summary>
    /// <param name="grid">The collocation grid.</param>
    /// <param name="parameters">Goddard rocket parameters.</param>
    /// <returns>States and controls at each grid point.</returns>
    public static (double[][] states, double[][] controls) GenerateSimulatedTrajectory(
        CollocationGrid grid,
        GoddardParameters parameters)
    {
        return GenerateSimulatedTrajectoryAtTimes(grid.TimePoints, parameters);
    }

    /// <summary>
    /// Computes a heuristic thrust value based on current state and time.
    /// Uses max thrust until near burnout, then tapers smoothly to avoid
    /// discontinuities that create large collocation defects.
    /// </summary>
    private static double ComputeHeuristicThrust(
#pragma warning disable RCS1163 // Unused parameter
        double h, double v, double m, double t, GoddardParameters p, double tf)
#pragma warning restore RCS1163 // Unused parameter
    {
        var fuelRemaining = m - p.Mf;
        var Fm = p.Fm;

        // No thrust if no fuel
        if (fuelRemaining <= 1e-6)
        {
            return 0.0;
        }

        // Compute burn time remaining at max thrust
        var burnTimeRemaining = fuelRemaining / Fm;

        // If more than 1 second of burn time remains, use max thrust
        if (burnTimeRemaining > 1.0)
        {
            return Fm;
        }

        // Smooth taper over the last 1 second of burn
        // This reduces control discontinuity at burnout
        var taper = burnTimeRemaining; // Linear taper from 1.0 to 0.0
        return Fm * taper;
    }

    /// <summary>
    /// Integrates state forward using RK4 with given thrust.
    /// </summary>
    private static (double h, double v, double m) IntegrateRK4(
        double h, double v, double m, double thrust, double dt, GoddardParameters p)
    {
        (double dh, double dv, double dm) Derivatives(double hh, double vv, double mm, double T)
        {
            var D = p.D0 * Math.Exp(-p.Beta * hh);
            var dragForce = Math.Sign(vv) * D * vv * vv;
            var g = p.G0 * Math.Pow(p.R0 / (p.R0 + hh), 2);

            var dhdt = vv;
            var dvdt = (T * p.C - dragForce) / mm - g;
            var dmdt = -T;

            return (dhdt, dvdt, dmdt);
        }

        // RK4 integration
        var (k1h, k1v, k1m) = Derivatives(h, v, m, thrust);

        var h2 = h + 0.5 * dt * k1h;
        var v2 = v + 0.5 * dt * k1v;
        var m2 = m + 0.5 * dt * k1m;
        var (k2h, k2v, k2m) = Derivatives(h2, v2, m2, thrust);

        var h3 = h + 0.5 * dt * k2h;
        var v3 = v + 0.5 * dt * k2v;
        var m3 = m + 0.5 * dt * k2m;
        var (k3h, k3v, k3m) = Derivatives(h3, v3, m3, thrust);

        var h4 = h + dt * k3h;
        var v4 = v + dt * k3v;
        var m4 = m + dt * k3m;
        var (k4h, k4v, k4m) = Derivatives(h4, v4, m4, thrust);

        var hNew = h + dt / 6.0 * (k1h + 2 * k2h + 2 * k3h + k4h);
        var vNew = v + dt / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v);
        var mNew = m + dt / 6.0 * (k1m + 2 * k2m + 2 * k3m + k4m);

        return (hNew, vNew, mNew);
    }

    /// <summary>
    /// Creates an initial guess decision vector from simulated trajectory.
    /// </summary>
    public static double[] CreateDecisionVector(
        double[][] states,
        double[][] controls,
        int stateDim,
        int controlDim)
    {
        var nPoints = states.Length;
        var decisionVectorSize = nPoints * (stateDim + controlDim);
        var z = new double[decisionVectorSize];

        for (var k = 0; k < nPoints; k++)
        {
            var offset = k * (stateDim + controlDim);

            for (var i = 0; i < stateDim; i++)
            {
                z[offset + i] = states[k][i];
            }

            for (var i = 0; i < controlDim; i++)
            {
                z[offset + stateDim + i] = controls[k][i];
            }
        }

        return z;
    }

    /// <summary>
    /// Generates a complete initial guess for the Goddard rocket problem.
    /// Combines simulation with the correct decision vector format.
    /// </summary>
    public static double[] GenerateInitialGuess(
        ControlProblem problem,
        CollocationGrid grid,
        GoddardParameters parameters)
    {
        var (states, controls) = GenerateSimulatedTrajectory(grid, parameters);
        return CreateDecisionVector(states, controls, problem.StateDim, problem.ControlDim);
    }

    /// <summary>
    /// Generates the time points for LGL collocation.
    /// </summary>
    public static double[] GenerateLGLTimePoints(CollocationGrid grid, int order)
    {
        var segments = grid.Segments;
        var totalPoints = segments * (order - 1) + 1;
        var timePoints = new double[totalPoints];
        var lglPoints = GetLGLPoints(order);

        var pointIndex = 0;
        for (var k = 0; k < segments; k++)
        {
            var tk = grid.TimePoints[k];
            var h = grid.TimePoints[k + 1] - tk;

            for (var localIdx = 0; localIdx < order; localIdx++)
            {
                // Skip shared endpoint (already set by previous segment)
                if (k > 0 && localIdx == 0)
                {
                    continue;
                }

                var tau = lglPoints[localIdx];
                timePoints[pointIndex++] = tk + (tau + 1.0) * h / 2.0;
            }
        }

        return timePoints;
    }

    /// <summary>
    /// Gets the LGL points for a given order (memoized).
    /// </summary>
    private static double[] GetLGLPoints(int order)
    {
        // LGL points on [-1, 1] for common orders
        return order switch
        {
            2 => [-1.0, 1.0],
            3 => [-1.0, 0.0, 1.0],
            4 => [-1.0, -0.4472135954999579, 0.4472135954999579, 1.0],
            5 => [-1.0, -0.6546536707079771, 0.0, 0.6546536707079771, 1.0],
            6 => [-1.0, -0.7650553239294647, -0.2852315164806451, 0.2852315164806451, 0.7650553239294647, 1.0],
            7 => [-1.0, -0.8302238962785670, -0.4688487934707142, 0.0, 0.4688487934707142, 0.8302238962785670, 1.0],
            _ => ComputeLGLPoints(order)
        };
    }

    /// <summary>
    /// Computes LGL points numerically for arbitrary orders.
    /// </summary>
    private static double[] ComputeLGLPoints(int order)
    {
        var points = new double[order];
        points[0] = -1.0;
        points[order - 1] = 1.0;

        // Interior points are roots of P'_{n-1}(x)
        // Use Newton-Raphson for each root
        for (var i = 1; i < order - 1; i++)
        {
            // Initial guess from Chebyshev nodes
            var x = -Math.Cos(Math.PI * i / (order - 1));

            for (var iter = 0; iter < 20; iter++)
            {
                var (pn, dpn) = EvaluateLegendreDerivative(order - 1, x);
                var dx = pn / dpn;
                x -= dx;
                if (Math.Abs(dx) < 1e-14)
                {
                    break;
                }
            }

            points[i] = x;
        }

        Array.Sort(points);
        return points;
    }

    /// <summary>
    /// Evaluates Legendre polynomial derivative at x.
    /// </summary>
    private static (double value, double derivative) EvaluateLegendreDerivative(int n, double x)
    {
        if (n == 0)
        {
            return (1.0, 0.0);
        }

        if (n == 1)
        {
            return (x, 1.0);
        }

        var p0 = 1.0;
        var p1 = x;
        var dp0 = 0.0;
        var dp1 = 1.0;

        for (var k = 2; k <= n; k++)
        {
            var p2 = ((2.0 * k - 1) * x * p1 - (k - 1) * p0) / k;
            var dp2 = ((2.0 * k - 1) * (p1 + x * dp1) - (k - 1) * dp0) / k;
            p0 = p1;
            p1 = p2;
            dp0 = dp1;
            dp1 = dp2;
        }

        return (dp1, 0.0); // We only need the value of P'_n
    }

    /// <summary>
    /// Generates an initial guess for LGL collocation.
    /// </summary>
    public static double[] GenerateLGLInitialGuess(
        ControlProblem problem,
        CollocationGrid grid,
        int order,
        GoddardParameters parameters)
    {
        var timePoints = GenerateLGLTimePoints(grid, order);
        var (states, controls) = GenerateSimulatedTrajectoryAtTimes(timePoints, parameters);
        return CreateDecisionVector(states, controls, problem.StateDim, problem.ControlDim);
    }

    /// <summary>
    /// Generates an initial guess for the free final time variant by simulating
    /// the rocket with max thrust until fuel depletion, then coasting to apogee.
    /// The apogee time becomes the computed T_f.
    /// </summary>
    /// <param name="normalizedTimePoints">Array of normalized time points τ ∈ [0, 1].</param>
    /// <param name="parameters">Goddard rocket parameters.</param>
    /// <returns>
    /// States [h, v, m, t, T_f] and controls [F] at each time point,
    /// where t = elapsed time at this point, T_f = total mission time (constant).
    /// </returns>
    public static (double[][] states, double[][] controls)
        GenerateFreeFinalTimeInitialGuess(
            double[] normalizedTimePoints,
            GoddardParameters parameters)
    {
        // Simulate rocket to apogee
        var (trajectory, burnoutTime, apogeeTime) = SimulateToApogee(parameters);
        var computedTf = apogeeTime;

        var nPoints = normalizedTimePoints.Length;
        var states = new double[nPoints][];
        var controls = new double[nPoints][];

        for (var k = 0; k < nPoints; k++)
        {
            var tau = normalizedTimePoints[k];

            // Elapsed time at this normalized point: t = τ × T_f
            var elapsedTime = tau * computedTf;

            // Interpolate state at this physical time
            var (h, v, m) = InterpolateState(trajectory, elapsedTime);

            // Control: max thrust before burnout, zero after
            var thrust = elapsedTime < burnoutTime ? parameters.Fm : 0.0;

            states[k] = [h, v, m, elapsedTime, computedTf];
            controls[k] = [thrust];
        }

        return (states, controls);
    }

    /// <summary>
    /// Simulates the rocket from initial conditions with max thrust until fuel
    /// is depleted, then coasts until apogee (v ≤ 0).
    /// </summary>
    private static (List<(double t, double h, double v, double m, double F)> trajectory,
                    double burnoutTime, double apogeeTime)
        SimulateToApogee(GoddardParameters p)
    {
        var trajectory = new List<(double t, double h, double v, double m, double F)>();
        var dt = 0.01;
        var t = 0.0;
        var h = 0.0;
        var v = 0.0;
        var m = p.M0;

        // Phase 1: Max thrust until fuel depleted
        while (m > p.Mf && v >= 0)
        {
            trajectory.Add((t, h, v, m, p.Fm));
            (h, v, m) = IntegrateRK4(h, v, m, p.Fm, dt, p);
            t += dt;
            m = Math.Max(m, p.Mf);  // Clamp mass to minimum
        }

        var burnoutTime = t;

        // Phase 2: Coast until apogee (v ≤ 0)
        while (v > 0)
        {
            trajectory.Add((t, h, v, m, 0.0));
            (h, v, m) = IntegrateRK4(h, v, m, 0.0, dt, p);
            t += dt;
        }

        // Add final point at apogee
        trajectory.Add((t, h, v, m, 0.0));

        return (trajectory, burnoutTime, t);
    }

    /// <summary>
    /// Linearly interpolates state values at a target time from a trajectory.
    /// </summary>
    private static (double h, double v, double m) InterpolateState(
        List<(double t, double h, double v, double m, double F)> trajectory,
        double targetTime)
    {
        // Handle boundary cases
        if (targetTime <= trajectory[0].t)
        {
            var first = trajectory[0];
            return (first.h, first.v, first.m);
        }

        if (targetTime >= trajectory[^1].t)
        {
            var last = trajectory[^1];
            return (last.h, last.v, last.m);
        }

        // Find bracketing points
        for (var i = 0; i < trajectory.Count - 1; i++)
        {
            var p0 = trajectory[i];
            var p1 = trajectory[i + 1];

            if (targetTime >= p0.t && targetTime <= p1.t)
            {
                // Linear interpolation
                var alpha = (targetTime - p0.t) / (p1.t - p0.t);
                var h = p0.h + alpha * (p1.h - p0.h);
                var v = p0.v + alpha * (p1.v - p0.v);
                var m = p0.m + alpha * (p1.m - p0.m);
                return (h, v, m);
            }
        }

        // Fallback (should not reach here)
        var fallback = trajectory[^1];
        return (fallback.h, fallback.v, fallback.m);
    }
}
