/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using Optimal.Control.Collocation;
using Optimal.Control.Core;
using Optimal.Control.Solvers;
using Optimal.NonLinear.Constrained;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Monitoring;
using Optimal.NonLinear.Unconstrained;

namespace OptimalCli.Problems.Goddard;

/// <summary>
/// Solves the Goddard rocket problem (Goddard, 1919).
/// State: [h, v, m, t] for free final time (t = elapsed time)
/// Control: T (thrust)
/// Objective: Maximize final altitude subject to drag, gravity, and fuel constraints
///
/// </summary>
public sealed class GoddardRocketProblemSolver : ICommand
{
    public string Name => "goddard";

    public string Description => "Goddard rocket maximum altitude with drag and fuel constraints (Goddard, 1919)";

    public void Run(CommandOptions options)
    {
        PrintHeader();
        var (problem, problemParams) = CreateProblem();
        RunSolver(problem, problemParams, options);
    }

    private static void PrintHeader()
    {
        Console.WriteLine("=== GODDARD ROCKET PROBLEM ===");
        Console.WriteLine("Maximize final altitude of a vertically ascending rocket");
        Console.WriteLine();
        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Formulation: Fixed final time with terminal velocity constraint");
        Console.WriteLine("  State: [h, v, m] (altitude, velocity, mass)");
        Console.WriteLine("  Max iterations: 1000");
        Console.WriteLine("  Inner optimizer: L-BFGS with backtracking line search");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();
    }

    private static HermiteSimpsonSolver CreateSolver(
        int segments,
        ProgressCallback? progressCallback = null,
        Action<int, double, double, double, int, double[]>? innerProgressCallback = null,
        OptimisationMonitor? monitor = null)
    {
        var innerOptimizer = false
            ? (IOptimizer)new LBFGSBOptimizer(new LBFGSBOptions
            {
                MemorySize = 10,
                Tolerance = 1e-5,
                MaxIterations = 1000
            }, monitor)
            : new LBFGSOptimizer(new LBFGSOptions
            {
                MemorySize = 10,
                Preconditioning = new LBFGSPreconditioningOptions
                {
                    EnableAutomaticPreconditioning = true,
                    PreconditioningThreshold = 1e3,
                    Type = PreconditioningType.Diagonal
                },
                Tolerance = 1e-5,
                MaxIterations = 1000
            }, new BacktrackingLineSearch(), monitor);

        return new HermiteSimpsonSolver(
            new HermiteSimpsonSolverOptions
            {
                Segments = segments,
                Tolerance = 1e-4,
                MaxIterations = 1000,
                Verbose = true,
                ProgressCallback = progressCallback,
                // AutoScaling = true,
                // EnableMeshRefinement = true,
                // MaxRefinementIterations = 5,
                // RefinementDefectThreshold = 1e-4,
                InnerProgressCallback = innerProgressCallback
            },
            innerOptimizer,
            monitor);
    }

    private static InitialGuess CreateCustomInitialGuess(ControlProblem problem, int segments)
    {
        Console.WriteLine("Creating initial guess using physics-based simulation...");

        var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, segments);
        var goddardParams = GoddardInitialGuess.CreateYOptParameters();

        // Generate initial guess with physical time points
        var (states, controls) = GoddardInitialGuess.GenerateSimulatedTrajectory(grid, goddardParams);

        // Enforce terminal constraint: v_f = 0 (at apogee)
        // This helps the optimizer start from a point that satisfies the terminal condition
        var lastIdx = states.Length - 1;
        states[lastIdx][1] = 0.0;  // Set final velocity to 0

        // Diagnostic: print trajectory to verify physics
        Console.WriteLine();
        Console.WriteLine("  Initial guess trajectory (sampled points):");
        Console.WriteLine("    Time(s)   Alt(m)      Vel(m/s)    Mass(kg)    Thrust");
        for (var i = 0; i < states.Length; i += Math.Max(1, states.Length / 10))
        {
            var t = grid.TimePoints[i];
            var h = states[i][0];
            var v = states[i][1];
            var m = states[i][2];
            var thrust = controls[i][0];
            Console.WriteLine($"    {t,7:F1}   {h,10:F1}  {v,10:F1}  {m,10:F3}  {thrust,7:F3}");
        }
        // Always print last point
        var tLast = grid.TimePoints[lastIdx];
        Console.WriteLine($"    {tLast,7:F1}   {states[lastIdx][0],10:F1}  {states[lastIdx][1],10:F1}  {states[lastIdx][2],10:F3}  {controls[lastIdx][0],7:F3}");
        Console.WriteLine();

        var initialGuess = new InitialGuess(states, controls);

        Console.WriteLine($"  Segments: {segments}");
        Console.WriteLine($"  Initial guess created: {states.Length} collocation points");
        Console.WriteLine($"  Time horizon: [0, {problem.FinalTime:F1}] s");
        Console.WriteLine("  Method: Simulate max thrust to burnout, coast to apogee");
        Console.WriteLine($"  Simulated final altitude: {states[^1][0]:F1} m");
        Console.WriteLine($"  Simulated final velocity: {states[^1][1]:F1} m/s (enforced)");
        Console.WriteLine($"  Simulated final mass: {states[^1][2]:F3} kg");
        Console.WriteLine();

        return initialGuess;
    }

    private static ProgressCallback CreateProgressCallback(
        GoddardRocketParams problemParams,
        CancellationToken cancellationToken)
    {
        return (iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
        {
            if (cancellationToken.IsCancellationRequested)
            {
                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                throw new OperationCanceledException(cancellationToken);
            }
            RadiantGoddardRocketVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance, problemParams.H0);
        };
    }

    private static void RunSolver(
        ControlProblem problem,
        GoddardRocketParams problemParams,
        CommandOptions options)
    {
        var segments = 60;  // More segments = smaller defects from initial guess
        Console.WriteLine(problemParams.Description);
        Console.WriteLine();

        Console.WriteLine("Solving...");
        if (!options.Headless)
        {
            Console.WriteLine("Opening live visualization window...");
            Console.WriteLine("(Close window when done viewing)");
        }
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        using var cancellationTokenSource = new CancellationTokenSource();
        var cancellationToken = cancellationTokenSource.Token;

        var useMonitor = true;
        var monitor = new OptimisationMonitor()
            .WithGradientVerification(testStep: 1e-6)
            .WithSmoothnessMonitoring();

        var initialGuess = CreateCustomInitialGuess(problem, segments);

        if (options.Headless)
        {
            var solver = CreateSolver(segments, monitor: useMonitor ? monitor : null);
            var result = solver.Solve(problem, initialGuess, cancellationToken);
            PrintSolutionSummary(result, problemParams);
            return;
        }

        var solverWithCallback = CreateSolver(segments, CreateProgressCallback(problemParams, cancellationToken), null, monitor: useMonitor ? monitor : null);
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var taskResult = solverWithCallback.Solve(problem, initialGuess, cancellationToken);
                Console.WriteLine("[SOLVER] Optimization completed successfully");
                return taskResult;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("[SOLVER] Caught OperationCanceledException - optimization cancelled");
                throw;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SOLVER] Caught exception during solve: {ex.GetType().Name}: {ex.Message}");
                throw;
            }
        }, cancellationToken);

        RadiantGoddardRocketVisualizer.RunVisualizationWindow(cancellationTokenSource);

        if (!optimizationTask.IsCompleted)
        {
            Console.WriteLine();
            Console.WriteLine("Window closed - waiting for optimization to stop...");

            if (optimizationTask.Wait(TimeSpan.FromSeconds(5)))
            {
                Console.WriteLine("Optimization stopped gracefully");
            }
            else
            {
                Console.WriteLine("Optimization did not stop - returning to console");
                Console.WriteLine("(The background optimization will continue but results will be discarded)");
            }

            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("OPTIMIZATION CANCELLED");
            Console.WriteLine("  Window was closed before optimization completed");
            Console.WriteLine();
            return;
        }

        CollocationResult vizResult;
        try
        {
            vizResult = optimizationTask.Result;
        }
        catch (AggregateException ex) when (ex.InnerException is OperationCanceledException)
        {
            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("OPTIMIZATION CANCELLED");
            Console.WriteLine("  Optimization was cancelled");
            Console.WriteLine();
            return;
        }

        PrintSolutionSummary(vizResult, problemParams);
    }

    private static void PrintSolutionSummary(CollocationResult result, GoddardRocketParams problemParams)
    {
        Console.WriteLine();
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        var finalState = result.States[^1];
        Console.WriteLine($"  Final altitude: {finalState[0]:F2} m");
        Console.WriteLine($"  Final velocity: {finalState[1]:F2} m/s");
        Console.WriteLine($"  Final mass: {finalState[2]:F3} kg (fuel remaining: {finalState[2] - problemParams.MEmpty:F3} kg)");
        Console.WriteLine($"  Final time: {result.Times[^1]:F2} s");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6} (negative altitude)");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }

    /// <summary>
    /// Creates the fixed final time Goddard rocket problem.
    /// State: [h, v, m] (altitude, velocity, mass)
    /// Control: F (fuel mass flow rate)
    /// Objective: Maximize final altitude with terminal velocity constraint (v_f = 0 at apogee)
    /// </summary>
    private static (ControlProblem problem, GoddardRocketParams parameters) CreateProblem()
    {
        // Parameters from YOptimization / PROPT examples
        var D0 = 0.01227;      // Drag coefficient at sea level
        var beta = 0.145e-3;   // Inverse scale height (1/m)
        var c = 2060.0;        // Exhaust velocity (m/s)
        var g0 = 9.81;         // Gravitational acceleration (m/s²)
        var r0 = 6.371e6;      // Earth radius (m)
        var m0 = 214.839;      // Initial mass (kg)
        var mf = 67.9833;      // Final/empty mass (kg)
        var Fm = 9.525515;     // Maximum fuel mass flow rate (kg/s)

        var h0 = 1.0 / beta;   // Scale height for visualization

        // Compute final time from simulation (time to reach apogee)
        var goddardParams = GoddardInitialGuess.CreateYOptParameters();
        var tf = ComputeApogeeTime(goddardParams);

        var description = $"""
            Problem setup (Fixed Final Time - Simplified):
              Initial altitude: 0 m
              Initial velocity: 0 m/s
              Initial mass: {m0} kg (including fuel)
              Empty mass: {mf} kg (rocket structure)
              Fuel available: {m0 - mf:F3} kg
              Max fuel flow rate: {Fm} kg/s
              Exhaust velocity: c = {c} m/s
              Gravity (sea level): {g0} m/s²
              Earth radius: {r0 / 1e6:F3} × 10⁶ m
              Drag coefficient D0: {D0}
              Atmosphere scale (1/beta): {h0:F0} m
              Final time: {tf:F1} s (computed from simulation)
              Terminal constraint: None (optimizer finds apogee naturally)
              Objective: Maximize final altitude
            """;

        // Simulate to get reasonable bounds
        var (maxAltitude, maxVelocity) = SimulateForBounds(goddardParams);

        var problem = new ControlProblem()
            .WithStateSize(3)  // [h, v, m]
            .WithControlSize(1)
            .WithTimeHorizon(0.0, tf)  // Physical time t ∈ [0, T_f]
            .WithInitialCondition([0.0, 0.0, m0])
            .WithFinalCondition([double.NaN, double.NaN, double.NaN])  // All free - optimizer should find apogee naturally
            .WithControlBounds([0.0], [Fm])
            .WithStateBounds(
                [0.0, -50.0, mf],      // Allow small negative velocity during optimization
                [maxAltitude * 1.5, maxVelocity * 1.5, m0])  // Use realistic bounds from simulation
            .WithDynamics(input => CreateDynamics(input.State, input.Control, D0, beta, c, g0, r0))
            .WithRunningCost(CreateRunningCost)
            .WithTerminalCost(CreateTerminalCost)
            .WithPathConstraint(input => CreatePathConstraint(input, mf));

        return (problem, new GoddardRocketParams(description, h0, mf));
    }

    /// <summary>
    /// Simulates the rocket trajectory to determine reasonable state bounds.
    /// </summary>
    private static (double maxAltitude, double maxVelocity) SimulateForBounds(GoddardInitialGuess.GoddardParameters p)
    {
        var dt = 0.01;
        var t = 0.0;
        var h = 0.0;
        var v = 0.0;
        var m = p.M0;
        var maxH = 0.0;
        var maxV = 0.0;

        (double dh, double dv, double dm) Derivatives(double hh, double vv, double mm, double thrust)
        {
            var D = p.D0 * Math.Exp(-p.Beta * hh);
            var dragForce = Math.Sign(vv) * D * vv * vv;
            var g = p.G0 * Math.Pow(p.R0 / (p.R0 + hh), 2);
            return (vv, (thrust * p.C - dragForce) / mm - g, -thrust);
        }

        void Rk4Step(ref double hh, ref double vv, ref double mm, double thrust, double step)
        {
            var (k1h, k1v, k1m) = Derivatives(hh, vv, mm, thrust);
            var (k2h, k2v, k2m) = Derivatives(hh + 0.5 * step * k1h, vv + 0.5 * step * k1v, mm + 0.5 * step * k1m, thrust);
            var (k3h, k3v, k3m) = Derivatives(hh + 0.5 * step * k2h, vv + 0.5 * step * k2v, mm + 0.5 * step * k2m, thrust);
            var (k4h, k4v, k4m) = Derivatives(hh + step * k3h, vv + step * k3v, mm + step * k3m, thrust);
            hh += step / 6.0 * (k1h + 2 * k2h + 2 * k3h + k4h);
            vv += step / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v);
            mm += step / 6.0 * (k1m + 2 * k2m + 2 * k3m + k4m);
        }

        // Simulate until apogee
        while (t < 500 && (v >= 0 || t < 10))
        {
            var thrust = m > p.Mf + 0.001 ? p.Fm : 0.0;
            Rk4Step(ref h, ref v, ref m, thrust, dt);
            m = Math.Max(m, p.Mf);
            t += dt;
            maxH = Math.Max(maxH, h);
            maxV = Math.Max(maxV, v);
        }

        return (maxH, maxV);
    }

    /// <summary>
    /// Computes the time to reach apogee by simulating the rocket trajectory.
    /// Uses RK4 for better accuracy.
    /// </summary>
    private static double ComputeApogeeTime(GoddardInitialGuess.GoddardParameters p)
    {
        var dt = 0.01;
        var t = 0.0;
        var h = 0.0;
        var v = 0.0;
        var m = p.M0;

        (double dh, double dv, double dm) Derivatives(double hh, double vv, double mm, double thrust)
        {
            var D = p.D0 * Math.Exp(-p.Beta * hh);
            var dragForce = Math.Sign(vv) * D * vv * vv;
            var g = p.G0 * Math.Pow(p.R0 / (p.R0 + hh), 2);
            return (vv, (thrust * p.C - dragForce) / mm - g, -thrust);
        }

        void Rk4Step(ref double hh, ref double vv, ref double mm, double thrust, double step)
        {
            var (k1h, k1v, k1m) = Derivatives(hh, vv, mm, thrust);
            var (k2h, k2v, k2m) = Derivatives(hh + 0.5 * step * k1h, vv + 0.5 * step * k1v, mm + 0.5 * step * k1m, thrust);
            var (k3h, k3v, k3m) = Derivatives(hh + 0.5 * step * k2h, vv + 0.5 * step * k2v, mm + 0.5 * step * k2m, thrust);
            var (k4h, k4v, k4m) = Derivatives(hh + step * k3h, vv + step * k3v, mm + step * k3m, thrust);
            hh += step / 6.0 * (k1h + 2 * k2h + 2 * k3h + k4h);
            vv += step / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v);
            mm += step / 6.0 * (k1m + 2 * k2m + 2 * k3m + k4m);
        }

        // Phase 1: Max thrust until fuel depleted
        while (m > p.Mf + 0.001 && t < 500)
        {
            Rk4Step(ref h, ref v, ref m, p.Fm, dt);
            m = Math.Max(m, p.Mf);
            t += dt;
        }

        // Phase 2: Coast until apogee (v <= 0)
        while (v > 0 && t < 500)
        {
            Rk4Step(ref h, ref v, ref m, 0.0, dt);
            t += dt;
        }

        // Interpolate to find exact time when v = 0
        // Simple linear interpolation back
        if (v < 0)
        {
            var (_, prevV, _) = Derivatives(h, v, m, 0.0);
            t -= dt * v / (v - prevV * dt);  // Approximate
        }

        return t;
    }

    /// <summary>
    /// Creates dynamics for the Goddard rocket problem.
    /// State: [h, v, m], Control: [F]
    /// </summary>
    private static DynamicsResult CreateDynamics(
        double[] x, double[] u, double D0, double beta, double c, double g0, double r0)
    {
        var h = x[0];   // Altitude
        var v = x[1];   // Velocity
        var m = x[2];   // Mass
        var F = u[0];   // Fuel mass flow rate

        // Compute drag coefficient and gravity
        var D = D0 * Math.Exp(-beta * h);
        var signV = v >= 0 ? 1.0 : -1.0;
        var dragForce = signV * D * v * v;
        var g = g0 * Math.Pow(r0 / (r0 + h), 2);

        // Dynamics: dh/dt = v, dv/dt = (F*c - D)/m - g, dm/dt = -F
        var dhdt = v;
        var dvdt = (F * c - dragForce) / m - g;
        var dmdt = -F;

        var value = new[] { dhdt, dvdt, dmdt };

        // Analytical gradients
        var gradients = new double[2][];

        // Gradient of drag force w.r.t. h and v
        var dD_dh = -beta * D;
        var dDragForce_dh = signV * dD_dh * v * v;
        var dDragForce_dv = signV * D * 2 * v;

        // Gradient of gravity w.r.t. h
        var dg_dh = -2.0 * g0 * Math.Pow(r0, 2) / Math.Pow(r0 + h, 3);

        // State gradients: 3x3 matrix flattened row-major
        // df/dx = [∂f_i/∂x_j] where f = [dh/dt, dv/dt, dm/dt], x = [h, v, m]
        gradients[0] =
        [
            // Row 0: ∂(dh/dt)/∂[h, v, m]
            0.0, 1.0, 0.0,
            // Row 1: ∂(dv/dt)/∂[h, v, m]
            -dDragForce_dh / m - dg_dh,
            -dDragForce_dv / m,
            -(F * c - dragForce) / (m * m),
            // Row 2: ∂(dm/dt)/∂[h, v, m]
            0.0, 0.0, 0.0
        ];

        // Control gradients: 3x1 matrix
        // df/du = [∂f_i/∂u_j] where f = [dh/dt, dv/dt, dm/dt], u = [F]
        gradients[1] =
        [
            0.0,      // ∂(dh/dt)/∂F
            c / m,    // ∂(dv/dt)/∂F
            -1.0      // ∂(dm/dt)/∂F
        ];

        return new DynamicsResult(value, gradients);
    }

    /// <summary>
    /// Creates the running cost (zero - we only have terminal cost).
    /// Gradient array: [dL/dx (3), dL/du (1), dL/dt (1)] = 5 elements
    /// </summary>
    private static RunningCostResult CreateRunningCost(RunningCostInput input)
    {
        // Zero running cost - we only have terminal cost
        // Gradient: [∂L/∂h, ∂L/∂v, ∂L/∂m, ∂L/∂F, ∂L/∂t]
        var gradients = new double[5];  // All zeros
        return new RunningCostResult(0.0, gradients);
    }

    /// <summary>
    /// Creates the terminal cost (-h to maximize altitude).
    /// Gradient array: dΦ/dx = [∂Φ/∂h, ∂Φ/∂v, ∂Φ/∂m] = 3 elements (stateDim)
    /// </summary>
    private static TerminalCostResult CreateTerminalCost(TerminalCostInput input)
    {
        var h = input.State[0];

        // Terminal cost: -h (minimize negative altitude = maximize altitude)
        var cost = -h;

        // Gradient: dΦ/dx = [∂Φ/∂h, ∂Φ/∂v, ∂Φ/∂m]
        var gradients = new double[3];
        gradients[0] = -1.0;  // ∂Φ/∂h = -1
        gradients[1] = 0.0;   // ∂Φ/∂v = 0
        gradients[2] = 0.0;   // ∂Φ/∂m = 0

        return new TerminalCostResult(cost, gradients);
    }

    /// <summary>
    /// Creates the path constraint: prevents thrust when out of fuel.
    /// Constraint: F * (m - mEmpty) >= 0, rewritten as -F * (m - mEmpty) <= 0
    /// Gradient array: [dg/dx (3), dg/du (1), dg/dt (1)] = 5 elements
    /// </summary>
    private static PathConstraintResult CreatePathConstraint(PathConstraintInput input, double mEmpty)
    {
        var m = input.State[2];
        var F = input.Control[0];

        // Constraint: -F * (m - mEmpty) <= 0
        var constraint = -F * (m - mEmpty);

        // Gradient: [∂g/∂h, ∂g/∂v, ∂g/∂m, ∂g/∂F, ∂g/∂t]
        var gradients = new double[5];
        gradients[0] = 0.0;           // ∂g/∂h
        gradients[1] = 0.0;           // ∂g/∂v
        gradients[2] = -F;            // ∂g/∂m = -F
        gradients[3] = -(m - mEmpty); // ∂g/∂F = -(m - mEmpty)
        gradients[4] = 0.0;           // ∂g/∂t

        return new PathConstraintResult(constraint, gradients);
    }

    /// <summary>
    /// Parameters for the Goddard rocket problem.
    /// </summary>
    private sealed record GoddardRocketParams(string Description, double H0, double MEmpty);
}
