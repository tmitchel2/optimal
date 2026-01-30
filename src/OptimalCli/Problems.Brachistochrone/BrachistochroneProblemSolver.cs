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

namespace OptimalCli.Problems.Brachistochrone;

/// <summary>
/// Solves the Brachistochrone problem (Johann Bernoulli, 1696).
/// Question: What curve gives the fastest descent under gravity between two points?
/// Answer: A cycloid - the curve traced by a point on a rolling circle.
/// State: [x, y, v] where x is horizontal, y is vertical (down), v is velocity
/// Control: θ (angle from horizontal, 0 to π/2)
/// Objective: Minimize time of descent
/// </summary>
public sealed class BrachistochroneProblemSolver : ICommand
{
    // Problem constants
    private const double Gravity = 9.80665; // Standard gravity m/s²
    private const double X0 = 0.0;          // Starting x position
    private const double Y0 = 10.0;         // Starting y position (top - y-axis points up)
    private const double Xf = 10.0;         // Final x position
    private const double Yf = 5.0;          // Final y position (bottom - lower than start)
    private const double V0 = 1e-6;         // Initial velocity (near zero, avoid singularity)

    public string Name => "brachistochrone";

    public string Description => "Curve of fastest descent under gravity (Johann Bernoulli, 1696)";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== BRACHISTOCHRONE PROBLEM ===");
        Console.WriteLine("Finding the curve of fastest descent under gravity");
        Console.WriteLine("(Johann Bernoulli, 1696)");
        Console.WriteLine();

        // Create problem based on variant
        var (problem, description) = options.Variant switch
        {
            BrachistochroneVariant.FixedTime => CreateFixedTimeProblem(),
            BrachistochroneVariant.FreeFinalTime => CreateFreeFinalTimeProblem(),
            BrachistochroneVariant.FreeFinalTimeRunningCost => CreateFreeFinalTimeRunningCostProblem(),
            _ => throw new ArgumentException($"Unknown variant: {options.Variant}")
        };

        Console.WriteLine($"Variant: {options.Variant}");
        Console.WriteLine(description);
        Console.WriteLine();

        // Create and run solver
        RunSolver(problem, options);
    }

    /// <summary>
    /// Creates the fixed final time problem formulation.
    /// State: [x, y, v], Control: θ
    /// Uses a pre-specified final time and minimizes path integral of 1 (which equals T_f).
    /// Simpler formulation useful for testing.
    /// </summary>
    private static (ControlProblem problem, string description) CreateFixedTimeProblem()
    {
        var tf = 1.8; // Fixed final time (near theoretical optimal)

        var description = $"""
            Problem setup (fixed time):
              Gravity: {Gravity} m/s²
              Start: ({X0}, {Y0}) m (top)
              End: ({Xf}, {Yf}) m (bottom)
              Initial velocity: {V0:E2} m/s (nearly at rest)
              Final velocity: free
              Final time: {tf} s (fixed)
            """;

        var problem = new ControlProblem()
            .WithStateSize(3) // [x, y, v]
            .WithControlSize(1) // theta
            .WithTimeHorizon(0.0, tf)
            .WithInitialCondition([X0, Y0, V0])
            .WithFinalCondition([Xf, Yf, double.NaN]) // Free final velocity
            .WithControlBounds([0.0], [Math.PI / 2.0])
            .WithStateBounds(
                [0.0, 0.0, 1e-6],
                [15.0, 15.0, 20.0])
            .WithDynamics(input =>
            {
                var x = input.State;
                var u = input.Control;
                var v = x[2];
                var theta = u[0];

                var (xrate, xrateGrad) = BrachistochroneDynamicsGradients.XRateReverse(v, theta);
                var (yrate, yrateGrad) = BrachistochroneDynamicsGradients.YRateReverse(x[0], x[1], v, theta, Gravity);
                var (vrate, vrateGrad) = BrachistochroneDynamicsGradients.VRateReverse(x[0], x[1], v, theta, Gravity);

                var value = new[] { xrate, yrate, vrate };

                // Gradients: [0]=state (3x3 flattened), [1]=control (3x1)
                var gradients = new double[2][];
                gradients[0] = [
                    xrateGrad[0], xrateGrad[1], xrateGrad[2],
                    yrateGrad[0], yrateGrad[1], yrateGrad[2],
                    vrateGrad[0], vrateGrad[1], vrateGrad[2]
                ];
                gradients[1] = [xrateGrad[3], yrateGrad[3], vrateGrad[3]];

                return new DynamicsResult(value, gradients);
            })
            .WithRunningCost(_ =>
            {
                // Running cost = 1 (integral equals time)
                var gradients = new double[5]; // [x, y, v, theta, t]
                return new RunningCostResult(1.0, gradients);
            });

        return (problem, description);
    }

    /// <summary>
    /// Creates the free final time problem using time-scaling transformation.
    /// State: [x, y, v, T_f], Control: θ
    /// Time is normalized to τ ∈ [0,1], T_f becomes a state variable.
    /// Terminal cost: Φ = T_f (minimize final time directly).
    /// </summary>
    private static (ControlProblem problem, string description) CreateFreeFinalTimeProblem()
    {
        var tfGuess = 1.8; // Initial guess for final time

        var description = $"""
            Problem setup (free final time with time-scaling):
              Gravity: {Gravity} m/s²
              Start: ({X0}, {Y0}) m (top)
              End: ({Xf}, {Yf}) m (bottom)
              Initial velocity: {V0:E2} m/s (nearly at rest)
              Final velocity: free
              Final time T_f: free (to be optimized, guess {tfGuess} s)
              Normalized time: τ ∈ [0, 1]
            """;

        var problem = new ControlProblem()
            .WithStateSize(4) // [x, y, v, T_f]
            .WithControlSize(1) // theta
            .WithTimeHorizon(0.0, 1.0) // Normalized time
            .WithInitialCondition([X0, Y0, V0, tfGuess])
            .WithFinalCondition([Xf, Yf, double.NaN, double.NaN]) // Free v and T_f
            .WithControlBounds([0.0], [Math.PI / 2.0])
            .WithStateBounds(
                [0.0, 0.0, 1e-6, 0.1],
                [15.0, 15.0, 20.0, 5.0])
            .WithDynamics(input =>
            {
                var x = input.State;
                var u = input.Control;
                var v = x[2];
                var Tf = x[3];
                var theta = u[0];

                // Time-scaled dynamics: dx/dτ = T_f · (dx/dt)
                var (xratePhys, xrateGrad) = BrachistochroneDynamicsGradients.XRateReverse(v, theta);
                var (yratePhys, yrateGrad) = BrachistochroneDynamicsGradients.YRateReverse(x[0], x[1], v, theta, Gravity);
                var (vratePhys, vrateGrad) = BrachistochroneDynamicsGradients.VRateReverse(x[0], x[1], v, theta, Gravity);

                var xrate = Tf * xratePhys;
                var yrate = Tf * yratePhys;
                var vrate = Tf * vratePhys;
                var Tfrate = 0.0; // T_f is constant

                var value = new[] { xrate, yrate, vrate, Tfrate };

                // Gradients w.r.t. state: chain rule ∂(T_f·f)/∂x = T_f·(∂f/∂x), ∂(T_f·f)/∂T_f = f
                // XRateReverse returns [∂/∂v, ∂/∂theta]
                // YRateReverse returns [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
                // VRateReverse returns [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
                var gradients = new double[2][];
                gradients[0] = [
                    0.0, 0.0, Tf * xrateGrad[0], xratePhys,                               // xrate: no x,y dep, v=xrateGrad[0], T_f=xratePhys
                    Tf * yrateGrad[0], Tf * yrateGrad[1], Tf * yrateGrad[2], yratePhys,   // yrate: x,y,v from yrateGrad, T_f=yratePhys
                    Tf * vrateGrad[0], Tf * vrateGrad[1], Tf * vrateGrad[2], vratePhys,   // vrate: x,y,v from vrateGrad, T_f=vratePhys
                    0.0, 0.0, 0.0, 0.0                                                    // T_frate = 0, all derivatives = 0
                ];
                gradients[1] = [Tf * xrateGrad[1], Tf * yrateGrad[3], Tf * vrateGrad[3], 0.0];

                return new DynamicsResult(value, gradients);
            })
            .WithTerminalCost(input =>
            {
                var Tf = input.State[3];
                var gradients = new double[5]; // [x, y, v, T_f, tau]
                gradients[3] = 1.0; // ∂Φ/∂T_f = 1
                return new TerminalCostResult(Tf, gradients);
            });

        return (problem, description);
    }

    /// <summary>
    /// Creates the free final time problem with running cost formulation.
    /// State: [x, y, v, T_f], Control: θ
    /// Running cost: L = T_f (integrating over τ ∈ [0,1] gives T_f).
    /// Alternative to terminal cost formulation.
    /// </summary>
    private static (ControlProblem problem, string description) CreateFreeFinalTimeRunningCostProblem()
    {
        var tfGuess = 1.8; // Initial guess for final time

        var description = $"""
            Problem setup (free final time with running cost):
              Gravity: {Gravity} m/s²
              Start: ({X0}, {Y0}) m (top)
              End: ({Xf}, {Yf}) m (bottom)
              Initial velocity: {V0:E2} m/s (nearly at rest)
              Final velocity: free
              Final time T_f: free (to be optimized, guess {tfGuess} s)
              Normalized time: τ ∈ [0, 1]
              Running cost: L = T_f (∫₀¹ T_f dτ = T_f)
            """;

        var problem = new ControlProblem()
            .WithStateSize(4) // [x, y, v, T_f]
            .WithControlSize(1) // theta
            .WithTimeHorizon(0.0, 1.0) // Normalized time
            .WithInitialCondition([X0, Y0, V0, tfGuess])
            .WithFinalCondition([Xf, Yf, double.NaN, double.NaN]) // Free v and T_f
            .WithControlBounds([0.0], [Math.PI / 2.0])
            .WithStateBounds(
                [0.0, 0.0, 1e-6, 0.1],
                [15.0, 15.0, 20.0, 5.0])
            .WithDynamics(input =>
            {
                var x = input.State;
                var u = input.Control;
                var v = x[2];
                var Tf = x[3];
                var theta = u[0];

                // Time-scaled dynamics (same as terminal cost variant)
                var (xratePhys, xrateGrad) = BrachistochroneDynamicsGradients.XRateReverse(v, theta);
                var (yratePhys, yrateGrad) = BrachistochroneDynamicsGradients.YRateReverse(x[0], x[1], v, theta, Gravity);
                var (vratePhys, vrateGrad) = BrachistochroneDynamicsGradients.VRateReverse(x[0], x[1], v, theta, Gravity);

                var xrate = Tf * xratePhys;
                var yrate = Tf * yratePhys;
                var vrate = Tf * vratePhys;
                var Tfrate = 0.0;

                var value = new[] { xrate, yrate, vrate, Tfrate };

                // Gradients w.r.t. state: chain rule ∂(T_f·f)/∂x = T_f·(∂f/∂x), ∂(T_f·f)/∂T_f = f
                // XRateReverse returns [∂/∂v, ∂/∂theta]
                // YRateReverse returns [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
                // VRateReverse returns [∂/∂x, ∂/∂y, ∂/∂v, ∂/∂theta, ∂/∂g]
                var gradients = new double[2][];
                gradients[0] = [
                    0.0, 0.0, Tf * xrateGrad[0], xratePhys,                               // xrate: no x,y dep, v=xrateGrad[0], T_f=xratePhys
                    Tf * yrateGrad[0], Tf * yrateGrad[1], Tf * yrateGrad[2], yratePhys,   // yrate: x,y,v from yrateGrad, T_f=yratePhys
                    Tf * vrateGrad[0], Tf * vrateGrad[1], Tf * vrateGrad[2], vratePhys,   // vrate: x,y,v from vrateGrad, T_f=vratePhys
                    0.0, 0.0, 0.0, 0.0                                                    // T_frate = 0, all derivatives = 0
                ];
                gradients[1] = [Tf * xrateGrad[1], Tf * yrateGrad[3], Tf * vrateGrad[3], 0.0];

                return new DynamicsResult(value, gradients);
            })
            .WithRunningCost(input =>
            {
                var Tf = input.State[3];
                // Running cost = T_f, so ∫₀¹ T_f dτ = T_f (since T_f is constant)
                var gradients = new double[6]; // [x, y, v, T_f, theta, tau]
                gradients[3] = 1.0; // ∂L/∂T_f = 1
                return new RunningCostResult(Tf, gradients);
            });

        return (problem, description);
    }

    private static ISolver CreateSolver(CommandOptions options, ProgressCallback? progressCallback = null)
    {
        var innerOptimizer = new LBFGSOptimizer(new LBFGSOptions
        {
            Tolerance = 1e-4,
            MaxIterations = 500
        }, new BacktrackingLineSearch());

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 20");
        Console.WriteLine("  Order: 5 (LGL only)");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS");
        Console.WriteLine("  Tolerance: 5e-3");
        Console.WriteLine();

        return options.Solver == SolverType.LGL
            ? new LegendreGaussLobattoSolver(
                new LegendreGaussLobattoSolverOptions
                {
                    Order = 5,
                    Segments = 20,
                    Tolerance = 5e-3,
                    MaxIterations = 100,
                    Verbose = true,
                    ProgressCallback = progressCallback
                },
                innerOptimizer)
            : new HermiteSimpsonSolver(
                new HermiteSimpsonSolverOptions
                {
                    Segments = 50,
                    Tolerance = 1e-2,
                    MaxIterations = 200,
                    Verbose = true,
                    ProgressCallback = progressCallback
                },
                innerOptimizer);
    }

    private static void RunSolver(ControlProblem problem, CommandOptions options)
    {
        Console.WriteLine("Solving...");
        if (!options.Headless)
        {
            Console.WriteLine("Opening live visualization window...");
            Console.WriteLine("(Close window when done viewing)");
        }
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        var segments = options.Solver == SolverType.LGL ? 20 : 50;
        var order = 5;
        var initialGuess = options.Solver == SolverType.LGL
            ? InitialGuessFactory.CreateForLGL(problem, segments, order)
            : InitialGuessFactory.CreateWithControlHeuristics(problem, segments);

        if (options.Headless)
        {
            var solver = CreateSolver(options);
            var headlessResult = solver.Solve(problem, initialGuess);
            PrintSolutionSummary(headlessResult, options.Variant);
            return;
        }

        var solverWithCallback = CreateSolver(options, CreateProgressCallback());
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var taskResult = solverWithCallback.Solve(problem, initialGuess);
                Console.WriteLine("[SOLVER] Optimization completed successfully");
                return taskResult;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("[SOLVER] Optimization cancelled");
                throw;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SOLVER] Exception during solve: {ex.GetType().Name}: {ex.Message}");
                throw;
            }
        }, RadiantBrachistochroneVisualizer.CancellationToken);

        RadiantBrachistochroneVisualizer.RunVisualizationWindow();

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
            }

            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("OPTIMIZATION CANCELLED");
            Console.WriteLine();
            return;
        }

        CollocationResult result;
        try
        {
            result = optimizationTask.Result;
        }
        catch (AggregateException ex) when (ex.InnerException is OperationCanceledException)
        {
            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("OPTIMIZATION CANCELLED");
            Console.WriteLine();
            return;
        }

        PrintSolutionSummary(result, options.Variant);
    }

    private static ProgressCallback CreateProgressCallback()
    {
        return (iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
        {
            var token = RadiantBrachistochroneVisualizer.CancellationToken;
            if (token.IsCancellationRequested)
            {
                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, stopping optimization...");
                throw new OperationCanceledException(token);
            }

            RadiantBrachistochroneVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
        };
    }

    private static void PrintSolutionSummary(CollocationResult result, BrachistochroneVariant variant)
    {
        Console.WriteLine();
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3}) m");
        Console.WriteLine($"  Final velocity: {result.States[^1][2]:F3} m/s");

        if (variant != BrachistochroneVariant.FixedTime)
        {
            Console.WriteLine($"  Optimal final time T_f: {result.States[^1][3]:F6} seconds");
        }

        Console.WriteLine($"  Objective value: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine($"  Max defect: {result.MaxDefect:E3}");
        Console.WriteLine();

        // Energy conservation check
        var y0 = result.States[0][1];
        var yf = result.States[^1][1];
        var vf = result.States[^1][2];
        var expectedVf = Math.Sqrt(2 * Gravity * (y0 - yf));
        Console.WriteLine("ENERGY CONSERVATION CHECK:");
        Console.WriteLine($"  Initial y: {y0:F3} m, Final y: {yf:F3} m");
        Console.WriteLine($"  Drop: {y0 - yf:F3} m");
        Console.WriteLine($"  Expected final velocity (energy): {expectedVf:F3} m/s");
        Console.WriteLine($"  Actual final velocity: {vf:F3} m/s");
        Console.WriteLine($"  Velocity error: {Math.Abs(vf - expectedVf):F3} m/s ({100.0 * Math.Abs(vf - expectedVf) / expectedVf:F1}%)");
        Console.WriteLine();

        // Print trajectory sample
        Console.WriteLine("TRAJECTORY (sample):");
        var hasTimeSaling = variant != BrachistochroneVariant.FixedTime;
        Console.WriteLine(hasTimeSaling ? "  τ       x        y        v        T_f      θ" : "  t       x        y        v        θ");
        var step = Math.Max(1, result.States.Length / 10);
        for (var i = 0; i < result.States.Length; i += step)
        {
            var x = result.States[i];
            var u = result.Controls[i];
            var t = result.Times[i];
            if (hasTimeSaling)
            {
                Console.WriteLine($"  {t:F3}   {x[0]:F3}   {x[1]:F3}   {x[2]:F3}   {x[3]:F3}   {u[0]:F3}");
            }
            else
            {
                Console.WriteLine($"  {t:F3}   {x[0]:F3}   {x[1]:F3}   {x[2]:F3}   {u[0]:F3}");
            }
        }
        Console.WriteLine();
    }
}
