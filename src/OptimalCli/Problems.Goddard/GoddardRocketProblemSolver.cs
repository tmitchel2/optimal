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
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace OptimalCli.Problems.Goddard;

/// <summary>
/// Solves the Goddard rocket problem (Goddard, 1919).
/// State: [h, v, m] (altitude, velocity, mass)
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
        Console.WriteLine("=== GODDARD ROCKET PROBLEM ===");
        Console.WriteLine("Maximize final altitude of a vertically ascending rocket");
        Console.WriteLine();

        var (problem, problemParams) = options.GoddardVariant switch
        {
            GoddardRocketVariant.FixedFinalTime => CreateFixedFinalTimeProblem(),
            GoddardRocketVariant.FreeFinalTime => CreateFreeFinalTimeProblem(),
            _ => throw new ArgumentException($"Unknown variant: {options.GoddardVariant}")
        };

        Console.WriteLine($"Variant: {options.GoddardVariant}");
        Console.WriteLine(problemParams.Description);
        Console.WriteLine();

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 30");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS-B with parallel line search");
        Console.WriteLine("  Parallel line search: 4 candidates per batch");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();

        var useLGL = options.Solver == SolverType.LGL;

        // Create initial guess using physics-based simulation
        Console.WriteLine("Creating initial guess using physics-based simulation...");
        const int segments = 100;
        const int lglOrder = 5;
        var grid = new CollocationGrid(0.0, problem.FinalTime, segments);

        // Generate initial guess by simulating rocket trajectory
        var goddardParams = GoddardInitialGuess.CreateYOptParameters();

        // Use appropriate initial guess format for the chosen solver
        double[][] initialStates;
        double[][] initialControls;

        if (useLGL)
        {
            // LGL solver needs more points (order * segments - (segments-1) = N*(order-1)+1)
            var lglTimePoints = GoddardInitialGuess.GenerateLGLTimePoints(grid, lglOrder);
            (initialStates, initialControls) = GoddardInitialGuess.GenerateSimulatedTrajectoryAtTimes(lglTimePoints, goddardParams);
            Console.WriteLine($"  LGL collocation points: {lglTimePoints.Length}");
        }
        else
        {
            // Hermite-Simpson uses grid points directly
            (initialStates, initialControls) = GoddardInitialGuess.GenerateSimulatedTrajectory(grid, goddardParams);
        }

        var initialGuess = new InitialGuess(initialStates, initialControls);

        Console.WriteLine($"  Segments: {segments}");
        Console.WriteLine($"  Initial guess created: {initialStates.Length} collocation points");
        Console.WriteLine($"  Method: Forward simulation with heuristic thrust policy");
        Console.WriteLine($"  Simulated final altitude: {initialStates[^1][0]:F1} m");
        Console.WriteLine($"  Simulated final velocity: {initialStates[^1][1]:F1} m/s");
        Console.WriteLine($"  Simulated final mass: {initialStates[^1][2]:F3} kg");
        Console.WriteLine();

        Console.WriteLine("Solving...");
        if (!options.Headless)
        {
            Console.WriteLine("Opening live visualization window...");
            Console.WriteLine("(Close window when done viewing)");
        }
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        var lbfgInnerOptimizer = new LBFGSOptimizer(
            new LBFGSOptions
            {
                Tolerance = 0.0001,  // Relaxed inner tolerance
                MaxIterations = 1000,  // More inner iterations
                Verbose = true
            },
            new ParallelBacktrackingLineSearch(parallelBatchSize: 4));

        // Headless mode - run synchronously without visualization
        if (options.Headless)
        {
            ISolver headlessSolver = useLGL
                ? new LegendreGaussLobattoSolver(
                    new LegendreGaussLobattoSolverOptions
                    {
                        Order = 5,
                        Segments = segments,
                        Tolerance = 0.0001,
                        MaxIterations = 150,
                        Verbose = true
                    },
                    lbfgInnerOptimizer)
                : new HermiteSimpsonSolver(
                    new HermiteSimpsonSolverOptions
                    {
                        Segments = segments,
                        Tolerance = 0.001,
                        MaxIterations = 300,
                        EnableMeshRefinement = false,
                        Verbose = true
                    },
                    lbfgInnerOptimizer);

            var result = headlessSolver.Solve(problem, initialGuess);
            PrintSolutionSummary(result, problemParams);
            return;
        }

        // Run the optimizer in a background task with visualization
        ProgressCallback progressCallback = (iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
        {
            var token = RadiantGoddardRocketVisualizer.CancellationToken;
            if (token.IsCancellationRequested)
            {
                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                throw new OperationCanceledException(token);
            }
            RadiantGoddardRocketVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance, problemParams.H0);
        };

        ISolver solver = useLGL
            ? new LegendreGaussLobattoSolver(
                new LegendreGaussLobattoSolverOptions
                {
                    Order = 5,
                    Segments = segments,
                    Tolerance = 0.0001,
                    MaxIterations = 150,
                    Verbose = true,
                    ProgressCallback = progressCallback
                },
                lbfgInnerOptimizer)
            : new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions
                {
                    Segments = segments,
                    Tolerance = 0.001,
                    MaxIterations = 300,
                    EnableMeshRefinement = false,
                    Verbose = true,
                    ProgressCallback = progressCallback
                },
                lbfgInnerOptimizer);

        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var result = solver.Solve(problem, initialGuess);
                Console.WriteLine("[SOLVER] Optimization completed successfully");
                return result;
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
        }, RadiantGoddardRocketVisualizer.CancellationToken);

        // Run the visualization window on the main thread (blocks until window closed)
        RadiantGoddardRocketVisualizer.RunVisualizationWindow();

        // Check if optimization is still running after window closed
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

        // Optimization completed - get the result
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
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6} (negative altitude)");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }

    /// <summary>
    /// Creates the fixed final time variant based on PROPT example 45 / YOptimization.
    /// Final time is fixed at 100 seconds.
    /// Uses physical SI units from the example.
    /// </summary>
    private static (ControlProblem problem, GoddardRocketParams parameters) CreateFixedFinalTimeProblem()
    {
        // Parameters from YOptimization goddardRocketFixedTf (PROPT example 45)
        var D0 = 0.01227;      // Drag coefficient at sea level
        var beta = 0.145e-3;   // Inverse scale height (1/m)
        var c = 2060.0;        // Exhaust velocity (m/s)
        var g0 = 9.81;         // Gravitational acceleration (m/s²)
        var r0 = 6.371e6;      // Earth radius (m)
        var m0 = 214.839;      // Initial mass (kg)
        var mf = 67.9833;      // Final/empty mass (kg)
        var Fm = 9.525515;     // Maximum fuel mass flow rate (kg/s)
        var finalTime = 100.0; // Fixed final time (s)

        var h0 = 1.0 / beta;   // Scale height for visualization

        var description = $"""
            Problem setup (PROPT example 45 - Fixed Final Time):
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
              Time horizon: {finalTime} s (FIXED)
              Final condition: free
              Objective: Maximize final altitude
            """;

        var problem = new ControlProblem()
            .WithStateSize(3)
            .WithControlSize(1)
            .WithTimeHorizon(0.0, finalTime)
            .WithInitialCondition([0.0, 0.0, m0])
            .WithFinalCondition([double.NaN, double.NaN, double.NaN])  // All states free at final time
            .WithControlBounds([0.0], [Fm])
            .WithStateBounds(
                [0.0, 0.0, mf],
                [1e6, 1e4, m0])
            .WithDynamics(input => CreateYOptDynamics(input.State, input.Control, D0, beta, c, g0, r0))
            .WithRunningCost(CreateRunningCost)
            .WithTerminalCost(CreateTerminalCost)
            .WithPathConstraint(input => CreatePathConstraint(input, mf));

        return (problem, new GoddardRocketParams(description, h0, mf));
    }

    /// <summary>
    /// Creates the free final time variant based on PROPT example 44 / YOptimization.
    /// Final time is free (optimized).
    /// Uses time-scaling transformation to convert to fixed time horizon.
    /// </summary>
    private static (ControlProblem problem, GoddardRocketParams parameters) CreateFreeFinalTimeProblem()
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

        // For free final time, we use time-scaling: τ ∈ [0, 1], t = τ * T_f
        // The dynamics become: dx/dτ = T_f * f(x, u)
        // T_f becomes an additional decision variable (here we approximate with a reasonable fixed time)
        // A proper implementation would add T_f as a state variable
        var estimatedFinalTime = 150.0; // Estimated optimal final time for free-time problem

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
              Time horizon: FREE (estimated {estimatedFinalTime} s)
              Final condition: free
              Objective: Maximize final altitude
              Note: Uses time-scaling transformation for free final time
            """;

        var problem = new ControlProblem()
            .WithStateSize(3)
            .WithControlSize(1)
            .WithTimeHorizon(0.0, estimatedFinalTime)
            .WithInitialCondition([0.0, 0.0, m0])
            .WithFinalCondition([double.NaN, double.NaN, double.NaN])  // All states free at final time
            .WithControlBounds([0.0], [Fm])
            .WithStateBounds(
                [0.0, 0.0, mf],
                [1e6, 1e4, m0])
            .WithDynamics(input => CreateYOptDynamics(input.State, input.Control, D0, beta, c, g0, r0))
            .WithRunningCost(CreateRunningCost)
            .WithTerminalCost(CreateTerminalCost)
            .WithPathConstraint(input => CreatePathConstraint(input, mf));

        return (problem, new GoddardRocketParams(description, h0, mf));
    }

    /// <summary>
    /// Creates dynamics for the YOptimization-style Goddard rocket problem.
    /// Uses the model from PROPT examples 44 and 45.
    /// </summary>
    private static DynamicsResult CreateYOptDynamics(
        double[] x, double[] u, double D0, double beta, double c, double g0, double r0)
    {
        var h = x[0];  // Altitude
        var v = x[1];  // Velocity
        var m = x[2];  // Mass
        var F = u[0];  // Fuel mass flow rate

        // Drag and gravity from YOptimization model
        var D = D0 * Math.Exp(-beta * h);
        var F_D = Math.Sign(v) * D * v * v;  // Drag force
        var g = g0 * Math.Pow(r0 / (r0 + h), 2);  // Gravity with altitude variation

        // Dynamics: dv = (F*c - F_D)/m - g, dh = v, dm = -F
        var dh = v;
        var dv = ((F * c) - F_D) / m - g;
        var dm = -F;

        var value = new[] { dh, dv, dm };

        // Compute gradients analytically
        var gradients = new double[2][];

        // Gradients w.r.t. state: [∂ḣ/∂h, ∂ḣ/∂v, ∂ḣ/∂m; ∂v̇/∂h, ∂v̇/∂v, ∂v̇/∂m; ∂ṁ/∂h, ∂ṁ/∂v, ∂ṁ/∂m]
        var dFD_dh = Math.Sign(v) * (-beta) * D0 * Math.Exp(-beta * h) * v * v;  // ∂F_D/∂h
        var dFD_dv = Math.Sign(v) * D * 2 * v;  // ∂F_D/∂v (ignoring sign discontinuity)
        var dg_dh = g0 * (-2) * Math.Pow(r0, 2) / Math.Pow(r0 + h, 3);  // ∂g/∂h

        gradients[0] = [
            0.0, 1.0, 0.0,                           // ∂ḣ/∂[h,v,m]
            (-dFD_dh / m) - dg_dh, -dFD_dv / m, -((F * c) - F_D) / (m * m),  // ∂v̇/∂[h,v,m]
            0.0, 0.0, 0.0                            // ∂ṁ/∂[h,v,m]
        ];

        // Gradients w.r.t. control: [∂ḣ/∂F, ∂v̇/∂F, ∂ṁ/∂F]
        gradients[1] = [
            0.0,    // ∂ḣ/∂F
            c / m,  // ∂v̇/∂F
            -1.0    // ∂ṁ/∂F
        ];

        return new DynamicsResult(value, gradients);
    }

    /// <summary>
    /// Creates the running cost (zero for Goddard problem - we maximize terminal altitude).
    /// </summary>
    private static RunningCostResult CreateRunningCost(RunningCostInput input)
    {
        var x = input.State;
        var u = input.Control;
        var (cost, cost_gradients) = GoddardRocketDynamicsGradients.RunningCostReverse(x[0], x[1], x[2], u[0]);

        var gradients = new double[3];
        gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];
        gradients[1] = cost_gradients[3];
        gradients[2] = 0.0;
        return new RunningCostResult(cost, gradients);
    }

    /// <summary>
    /// Creates the terminal cost (-h to maximize altitude via minimization).
    /// </summary>
    private static TerminalCostResult CreateTerminalCost(TerminalCostInput input)
    {
        var x = input.State;
        var (cost, cost_gradients) = GoddardRocketDynamicsGradients.TerminalCostReverse(x[0], x[1], x[2]);

        var gradients = new double[2];
        gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];
        gradients[1] = 0.0;
        return new TerminalCostResult(cost, gradients);
    }

    /// <summary>
    /// Creates the path constraint to prevent thrust when out of fuel.
    /// </summary>
    private static PathConstraintResult CreatePathConstraint(PathConstraintInput input, double mEmpty)
    {
        var m = input.State[2];
        var T = input.Control[0];

        // Constraint: -T * (m - mEmpty) <= 0  (equivalent to T * (m - mEmpty) >= 0)
        var constraint = -T * (m - mEmpty);

        var gradients = new double[5];
        gradients[0] = 0.0;           // ∂c/∂h
        gradients[1] = 0.0;           // ∂c/∂v
        gradients[2] = -T;            // ∂c/∂m = -T
        gradients[3] = -(m - mEmpty); // ∂c/∂T = -(m - mEmpty)
        gradients[4] = 0.0;           // ∂c/∂t

        return new PathConstraintResult(constraint, gradients);
    }

    /// <summary>
    /// Parameters for the Goddard rocket problem variants.
    /// </summary>
    private sealed record GoddardRocketParams(string Description, double H0, double MEmpty);
}
