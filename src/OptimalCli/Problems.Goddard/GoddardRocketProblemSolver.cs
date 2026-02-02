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
/// State: [h, v, m] (altitude, velocity, mass) for fixed final time
/// State: [h, v, m, t, T_f] for free final time (t = elapsed time, T_f = total mission time)
/// Control: T (thrust)
/// Objective: Maximize final altitude subject to drag, gravity, and fuel constraints
///
/// Supports multiple variants:
/// - Default: Normalized parameters (original implementation)
/// - FixedFinalTime: PROPT example 45 with tf = 100s (from YOptimization)
/// - FreeFinalTime: PROPT example 44 with free final time (from YOptimization)
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
        Console.WriteLine("  Max iterations: 300");
        Console.WriteLine("  Inner optimizer: L-BFGS-B with parallel line search");
        Console.WriteLine("  Parallel line search: 4 candidates per batch");
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
                Tolerance = 1e-1,
                MaxIterations = 1000
            }, monitor)
            : new LBFGSOptimizer(new LBFGSOptions
            {
                MemorySize = 10,
                Preconditioning = new LBFGSPreconditioningOptions
                {
                    EnableAutomaticPreconditioning = true,
                    PreconditioningThreshold = 1e3,
                    Type = PreconditioningType.Regularization
                },
                Tolerance = 1e-2,
                MaxIterations = 1000
            }, new BacktrackingLineSearch(), monitor);

        return new HermiteSimpsonSolver(
            new HermiteSimpsonSolverOptions
            {
                Segments = segments,
                Tolerance = 1e-3,
                MaxIterations = 1000,
                Verbose = true,
                ProgressCallback = progressCallback,
                AutoScaling = true,
                EnableMeshRefinement = true,
                MaxRefinementIterations = 5,
                RefinementDefectThreshold = 1e-4,
                InnerProgressCallback = innerProgressCallback
            },
            innerOptimizer,
            monitor);
    }

    private static InitialGuess CreateCustomInitialGuess(ControlProblem problem, int segments)
    {
        Console.WriteLine("Creating initial guess using physics-based simulation...");

        var grid = new CollocationGrid(0.0, problem.FinalTime, segments);
        var goddardParams = GoddardInitialGuess.CreateYOptParameters();

        // For free final time (5 states), simulate to apogee to compute T_f
        if (problem.StateDim == 5)
        {
            var tauPoints = grid.TimePoints;  // Already [0, 1] for normalized time
            var (augmentedStates, physControls) =
                GoddardInitialGuess.GenerateFreeFinalTimeInitialGuess(tauPoints, goddardParams);

            var initialGuess = new InitialGuess(augmentedStates, physControls);

            Console.WriteLine($"  Segments: {segments}");
            Console.WriteLine($"  Initial guess created: {augmentedStates.Length} collocation points");
            Console.WriteLine("  Method: Simulate max thrust to burnout, coast to apogee (free final time)");
            Console.WriteLine($"  Simulated final altitude: {augmentedStates[^1][0]:F1} m");
            Console.WriteLine($"  Simulated final velocity: {augmentedStates[^1][1]:F1} m/s");
            Console.WriteLine($"  Simulated final mass: {augmentedStates[^1][2]:F3} kg");
            Console.WriteLine($"  Computed T_f (apogee time): {augmentedStates[^1][4]:F1} s");
            Console.WriteLine();

            return initialGuess;
        }

        var (initialStates, initialControls) = GoddardInitialGuess.GenerateSimulatedTrajectory(grid, goddardParams);
        var fixedTimeGuess = new InitialGuess(initialStates, initialControls);

        Console.WriteLine($"  Segments: {segments}");
        Console.WriteLine($"  Initial guess created: {initialStates.Length} collocation points");
        Console.WriteLine("  Method: Forward simulation with heuristic thrust policy");
        Console.WriteLine($"  Simulated final altitude: {initialStates[^1][0]:F1} m");
        Console.WriteLine($"  Simulated final velocity: {initialStates[^1][1]:F1} m/s");
        Console.WriteLine($"  Simulated final mass: {initialStates[^1][2]:F3} kg");
        Console.WriteLine();

        return fixedTimeGuess;
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
        var segments = 30;
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

        var initialGuess = CreateCustomInitialGuess(problem, segments);

        if (options.Headless)
        {
            var solver = CreateSolver(segments);
            var result = solver.Solve(problem, initialGuess, cancellationToken);
            PrintSolutionSummary(result, problemParams);
            return;
        }

        var solverWithCallback = CreateSolver(segments, CreateProgressCallback(problemParams, cancellationToken));
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
        if (finalState.Length == 5)
        {
            Console.WriteLine($"  Elapsed time t: {finalState[3]:F2} s");
            Console.WriteLine($"  Optimal final time T_f: {finalState[4]:F2} s");
        }
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6} (negative altitude)");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }

    /// <summary>
    /// Creates the free final time variant based on PROPT example 44 / YOptimization.
    /// Final time is free (optimized) using time-scaling transformation.
    /// State: [h, v, m, t, T_f], Control: F
    /// where t = elapsed time (evolves from 0 to T_f), T_f = total mission time (constant).
    /// Normalized time: τ ∈ [0, 1], with dt/dτ = T_f.
    /// </summary>
    private static (ControlProblem problem, GoddardRocketParams parameters) CreateProblem()
    {
        // Parameters from YOptimization goddardRocketFreeTf (PROPT example 44)
        var D0 = 0.01227;      // Drag coefficient at sea level
        var beta = 0.145e-3;   // Inverse scale height (1/m)
        var c = 2060.0;        // Exhaust velocity (m/s)
        var g0 = 9.81;         // Gravitational acceleration (m/s²)
        var r0 = 6.371e6;      // Earth radius (m)
        var m0 = 214.839;      // Initial mass (kg)
        var mf = 67.9833;      // Final/empty mass (kg)
        var Fm = 9.525515;     // Maximum fuel mass flow rate (kg/s)

        var tfGuess = 150.0;   // Initial guess for final time
        var h0 = 1.0 / beta;   // Scale height for visualization

        var description = $"""
            Problem setup (PROPT example 44 - Free Final Time):
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
              Time horizon: FREE (guess {tfGuess} s, optimized as state variable)
              Normalized time: τ ∈ [0, 1]
              Final condition: free
              Objective: Maximize final altitude
            """;

        var problem = new ControlProblem()
            .WithStateSize(5)  // [h, v, m, t, T_f]
            .WithControlSize(1)
            .WithTimeHorizon(0.0, 1.0)  // Normalized time τ ∈ [0, 1]
            .WithInitialCondition([0.0, 0.0, m0, 0.0, tfGuess])  // t starts at 0
            .WithFinalCondition([double.NaN, double.NaN, double.NaN, double.NaN, double.NaN])  // All states free
            .WithControlBounds([0.0], [Fm])
            .WithStateBounds(
                [0.0, 0.0, mf, 0.0, 10.0],      // t >= 0, T_f >= 10s
                [1e6, 1e4, m0, 500.0, 500.0])   // t <= 500s, T_f <= 500s
            .WithDynamics(input => CreateFreeFinalTimeDynamics(input.State, input.Control, D0, beta, c, g0, r0))
            .WithRunningCost(CreateFreeFinalTimeRunningCost)
            .WithTerminalCost(CreateFreeFinalTimeTerminalCost)
            .WithPathConstraint(input => CreateFreeFinalTimePathConstraint(input, mf));

        return (problem, new GoddardRocketParams(description, h0, mf));
    }

    /// <summary>
    /// Creates dynamics for the free final time Goddard rocket problem.
    /// Uses time-scaling: dx/dτ = T_f × (dx/dt), where τ ∈ [0, 1].
    /// State: [h, v, m, t, T_f], Control: [F]
    /// where t = elapsed time (evolves from 0 to T_f), T_f = total mission time (constant).
    /// </summary>
    private static DynamicsResult CreateFreeFinalTimeDynamics(
        double[] x, double[] u, double D0, double beta, double c, double g0, double r0)
    {
        var h = x[0];   // Altitude
        var v = x[1];   // Velocity
        var m = x[2];   // Mass
        var t = x[3];   // Elapsed time
        var Tf = x[4];  // Final time (for scaling)
        var F = u[0];   // Fuel mass flow rate

        // Suppress unused variable warning - t is part of the state but not used in dynamics
        _ = t;

        // Physical dynamics (same as CreateYOptDynamics)
        var D = D0 * Math.Exp(-beta * h);
        var signV = double.IsNaN(v) ? 0.0 : (v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0));
        var F_D = signV * D * v * v;  // Drag force
        var g = g0 * Math.Pow(r0 / (r0 + h), 2);  // Gravity with altitude variation

        // Physical time derivatives
        var dhdt = v;
        var dvdt = ((F * c) - F_D) / m - g;
        var dmdt = -F;

        // Time-scaled dynamics: dx/dτ = T_f × (dx/dt)
        var dhdtau = Tf * dhdt;
        var dvdtau = Tf * dvdt;
        var dmdtau = Tf * dmdt;
        var dtdtau = Tf;    // dt/dτ = T_f (elapsed time evolves)
        var dTfdtau = 0.0;  // T_f is constant over trajectory

        var value = new[] { dhdtau, dvdtau, dmdtau, dtdtau, dTfdtau };

        // Compute gradients analytically with chain rule
        // For scaled dynamics f_scaled = T_f × f_phys:
        //   ∂f_scaled/∂x_i = T_f × (∂f_phys/∂x_i)
        //   ∂f_scaled/∂T_f = f_phys
        var gradients = new double[2][];

        // Physical gradients
        var dFD_dh = signV * (-beta) * D0 * Math.Exp(-beta * h) * v * v;
        var dFD_dv = signV * D * 2 * v;
        var dg_dh = g0 * (-2) * Math.Pow(r0, 2) / Math.Pow(r0 + h, 3);

        // State gradients: 5x5 matrix flattened row-major
        // [∂(dh/dτ)/∂h, ∂(dh/dτ)/∂v, ∂(dh/dτ)/∂m, ∂(dh/dτ)/∂t, ∂(dh/dτ)/∂T_f;
        //  ∂(dv/dτ)/∂h, ∂(dv/dτ)/∂v, ∂(dv/dτ)/∂m, ∂(dv/dτ)/∂t, ∂(dv/dτ)/∂T_f;
        //  ∂(dm/dτ)/∂h, ∂(dm/dτ)/∂v, ∂(dm/dτ)/∂m, ∂(dm/dτ)/∂t, ∂(dm/dτ)/∂T_f;
        //  ∂(dt/dτ)/∂h, ∂(dt/dτ)/∂v, ∂(dt/dτ)/∂m, ∂(dt/dτ)/∂t, ∂(dt/dτ)/∂T_f;
        //  ∂(dT_f/dτ)/∂h, ∂(dT_f/dτ)/∂v, ∂(dT_f/dτ)/∂m, ∂(dT_f/dτ)/∂t, ∂(dT_f/dτ)/∂T_f]
        gradients[0] = [
            0.0, Tf * 1.0, 0.0, 0.0, dhdt,                                               // dh/dτ gradients
            Tf * ((-dFD_dh / m) - dg_dh), Tf * (-dFD_dv / m), Tf * (-((F * c) - F_D) / (m * m)), 0.0, dvdt,  // dv/dτ gradients
            0.0, 0.0, 0.0, 0.0, dmdt,                                                    // dm/dτ gradients
            0.0, 0.0, 0.0, 0.0, 1.0,                                                     // dt/dτ gradients (∂(T_f)/∂T_f = 1)
            0.0, 0.0, 0.0, 0.0, 0.0                                                      // dT_f/dτ gradients
        ];

        // Control gradients: 5x1 matrix
        gradients[1] = [
            0.0,           // ∂(dh/dτ)/∂F
            Tf * c / m,    // ∂(dv/dτ)/∂F
            Tf * (-1.0),   // ∂(dm/dτ)/∂F
            0.0,           // ∂(dt/dτ)/∂F
            0.0            // ∂(dT_f/dτ)/∂F
        ];

        return new DynamicsResult(value, gradients);
    }

    /// <summary>
    /// Creates the running cost for free final time variant (zero cost).
    /// Gradient array: [h, v, m, t, T_f, F, τ] = 7 elements
    /// </summary>
    private static RunningCostResult CreateFreeFinalTimeRunningCost(RunningCostInput input)
    {
        // Zero running cost - we only have terminal cost
        var gradients = new double[7];  // [h, v, m, t, T_f, F, τ]
        return new RunningCostResult(0.0, gradients);
    }

    /// <summary>
    /// Creates the terminal cost for free final time variant (-h to maximize altitude).
    /// Gradient array: [h, v, m, t, T_f, τ] = 6 elements
    /// </summary>
    private static TerminalCostResult CreateFreeFinalTimeTerminalCost(TerminalCostInput input)
    {
        var h = input.State[0];

        // Terminal cost: -h (minimize negative altitude = maximize altitude)
        var cost = -h;

        var gradients = new double[6];  // [h, v, m, t, T_f, τ]
        gradients[0] = -1.0;  // ∂Φ/∂h = -1
        gradients[1] = 0.0;   // ∂Φ/∂v
        gradients[2] = 0.0;   // ∂Φ/∂m
        gradients[3] = 0.0;   // ∂Φ/∂t
        gradients[4] = 0.0;   // ∂Φ/∂T_f
        gradients[5] = 0.0;   // ∂Φ/∂τ

        return new TerminalCostResult(cost, gradients);
    }

    /// <summary>
    /// Creates the path constraint for free final time variant.
    /// Prevents thrust when out of fuel.
    /// Gradient array: [h, v, m, t, T_f, F, τ] = 7 elements
    /// </summary>
    private static PathConstraintResult CreateFreeFinalTimePathConstraint(PathConstraintInput input, double mEmpty)
    {
        var m = input.State[2];
        var F = input.Control[0];

        // Constraint: -F * (m - mEmpty) <= 0  (equivalent to F * (m - mEmpty) >= 0)
        var constraint = -F * (m - mEmpty);

        var gradients = new double[7];  // [h, v, m, t, T_f, F, τ]
        gradients[0] = 0.0;           // ∂c/∂h
        gradients[1] = 0.0;           // ∂c/∂v
        gradients[2] = -F;            // ∂c/∂m = -F
        gradients[3] = 0.0;           // ∂c/∂t
        gradients[4] = 0.0;           // ∂c/∂T_f
        gradients[5] = -(m - mEmpty); // ∂c/∂F = -(m - mEmpty)
        gradients[6] = 0.0;           // ∂c/∂τ

        return new PathConstraintResult(constraint, gradients);
    }

    /// <summary>
    /// Parameters for the Goddard rocket problem variants.
    /// </summary>
    private sealed record GoddardRocketParams(string Description, double H0, double MEmpty);
}
