/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using Optimal.Control;
using Optimal.NonLinear;

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
    public string Name => "brachistochrone";

    public string Description => "Curve of fastest descent under gravity (Johann Bernoulli, 1696)";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== BRACHISTOCHRONE PROBLEM ===");
        Console.WriteLine("Finding the curve of fastest descent under gravity");
        Console.WriteLine("(Johann Bernoulli, 1696)");
        Console.WriteLine();

        var g = 9.80665; // Standard gravity m/s²
        var x0 = 0.0;    // Starting x position
        var y0 = 10.0;   // Starting y position (top - y-axis points up)
        var xf = 10.0;   // Final x position
        var yf = 5.0;    // Final y position (bottom - lower than start)
        var v0 = 1e-6;   // Initial velocity (near zero, but not exactly zero to avoid singularity)

        // Time-scaling transformation for free final time optimization
        // Transform: τ ∈ [0,1], t = T_f·τ, dt = T_f·dτ
        // State becomes: [x, y, v, T_f] where T_f is the free final time
        var tfGuess = 1.5; // Initial guess for final time

        Console.WriteLine($"Problem setup (with time-scaling):");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Start: ({x0}, {y0}) m (top)");
        Console.WriteLine($"  End: ({xf}, {yf}) m (bottom)");
        Console.WriteLine($"  Initial velocity: {v0:E2} m/s (nearly at rest)");
        Console.WriteLine($"  Final velocity: free");
        Console.WriteLine($"  Final time T_f: free (to be optimized, initial guess {tfGuess} s)");
        Console.WriteLine($"  Normalized time: τ ∈ [0, 1]");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(4) // [x, y, v, T_f]
            .WithControlSize(1) // theta (angle)
            .WithTimeHorizon(0.0, 1.0) // Normalized time from 0 to 1
            .WithInitialCondition(new[] { x0, y0, v0, tfGuess }) // Start with T_f guess
            .WithFinalCondition(new[] { xf, yf, double.NaN, double.NaN }) // Free final velocity and T_f
            .WithControlBounds(new[] { 0.0 }, new[] { Math.PI / 2.0 }) // Angle between 0 (horizontal) and π/2 (vertical down)
            .WithStateBounds(
                new[] { 0.0, 0.0, 1e-6, 0.1 },     // x, y >= 0, v > 0, T_f > 0.1
                new[] { 10.0, 15.0, 20.0, 5.0 })   // Reasonable upper bounds including T_f
            .WithDynamics((x, u, tau) => // Note: tau is normalized time
            {
                var xPos = x[0];
                var y = x[1];
                var v = x[2];
                var Tf = x[3];  // Final time is now a state variable
                var theta = u[0];

                // Time-scaled dynamics: dx/dτ = T_f · (dx/dt)
                // Original dynamics in physical time:
                //   dx/dt = v·cos(θ), dy/dt = -v·sin(θ), dv/dt = g·sin(θ)
                // Transformed to normalized time:
                //   dx/dτ = T_f · v·cos(θ), dy/dτ = T_f · (-v·sin(θ)), dv/dτ = T_f · g·sin(θ)
                //   dT_f/dτ = 0 (T_f is constant along trajectory)

                var (xrate_physical, xrate_gradients) = BrachistochroneDynamicsGradients.XRateReverse(xPos, y, v, theta, g);
                var (yrate_physical, yrate_gradients) = BrachistochroneDynamicsGradients.YRateReverse(xPos, y, v, theta, g);
                var (vrate_physical, vrate_gradients) = BrachistochroneDynamicsGradients.VRateReverse(xPos, y, v, theta, g);

                // Scale by T_f to transform to normalized time
                var xrate = Tf * xrate_physical;
                var yrate = Tf * yrate_physical;
                var vrate = Tf * vrate_physical;
                var Tfrate = 0.0; // T_f is constant

                var value = new[] { xrate, yrate, vrate, Tfrate };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂(dx/dτ)/∂x, ∂(dx/dτ)/∂y, ∂(dx/dτ)/∂v, ∂(dx/dτ)/∂T_f; ...]
                // Using chain rule: ∂(T_f·f)/∂x = T_f·(∂f/∂x), ∂(T_f·f)/∂T_f = f
                gradients[0] = new[] {
                    Tf * xrate_gradients[0], Tf * xrate_gradients[1], Tf * xrate_gradients[2], xrate_physical,  // ∂(dx/dτ)/∂[x,y,v,T_f]
                    Tf * yrate_gradients[0], Tf * yrate_gradients[1], Tf * yrate_gradients[2], yrate_physical,  // ∂(dy/dτ)/∂[x,y,v,T_f]
                    Tf * vrate_gradients[0], Tf * vrate_gradients[1], Tf * vrate_gradients[2], vrate_physical,  // ∂(dv/dτ)/∂[x,y,v,T_f]
                    0.0, 0.0, 0.0, 0.0   // ∂(dT_f/dτ)/∂[x,y,v,T_f] = all zeros
                };

                // Gradients w.r.t. control: [∂(dx/dτ)/∂θ, ∂(dy/dτ)/∂θ, ∂(dv/dτ)/∂θ, ∂(dT_f/dτ)/∂θ]
                gradients[1] = new[] {
                    Tf * xrate_gradients[3],  // ∂(dx/dτ)/∂θ
                    Tf * yrate_gradients[3],  // ∂(dy/dτ)/∂θ
                    Tf * vrate_gradients[3],  // ∂(dv/dτ)/∂θ
                    0.0                        // ∂(dT_f/dτ)/∂θ
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, tau) =>
            {
                var Tf = x[3];  // Cost is the final time T_f

                // Running cost in normalized time: L = T_f
                // Integrating from τ=0 to τ=1 gives ∫T_f dτ = T_f·(1-0) = T_f
                var cost = Tf;

                var gradients = new double[6];  // [StateDim + ControlDim + 1]
                gradients[0] = 0.0;  // ∂L/∂x
                gradients[1] = 0.0;  // ∂L/∂y
                gradients[2] = 0.0;  // ∂L/∂v
                gradients[3] = 1.0;  // ∂L/∂T_f
                gradients[4] = 0.0;  // ∂L/∂θ
                gradients[5] = 0.0;  // ∂L/∂τ (cost doesn't depend on normalized time)
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Legendre-Gauss-Lobatto direct collocation");
        Console.WriteLine("  Segments: 30");
        Console.WriteLine("  Order: 3 (for stability)");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS");
        Console.WriteLine("  Tolerance: 1e-5");
        Console.WriteLine("  Initial guess: Physics-based with time-scaling");
        Console.WriteLine();

        Console.WriteLine("Solving...");
        if (!options.Headless)
        {
            Console.WriteLine("Opening live visualization window...");
            Console.WriteLine("(Close window when done viewing)");
        }
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        // Run the optimizer
        if (options.Headless)
        {
            // Headless mode - run solver directly without visualization
            var solver = new HermiteSimpsonSolver()
                .WithSegments(30)
                // .WithOrder(3)  // Use Order 3 for stability (higher orders can have spurious local minima)
                .WithTolerance(1e-2)
                .WithMaxIterations(200)
                .WithVerbose(true)
                .WithInnerOptimizer(new LBFGSOptimizer()
                    .WithTolerance(1e-2)
                    .WithMaxIterations(200)
                    .WithVerbose(false));  // Reduce verbosity of inner optimizer

            var result = solver.Solve(problem);

            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("SOLUTION SUMMARY:");
            Console.WriteLine($"  Success: {result.Success}");
            Console.WriteLine($"  Message: {result.Message}");
            Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3}) m");
            Console.WriteLine($"  Final velocity: {result.States[^1][2]:F3} m/s");
            Console.WriteLine($"  Optimal final time T_f: {result.States[^1][3]:F6} seconds");
            Console.WriteLine($"  Objective value: {result.OptimalCost:F6} (should equal T_f)");
            Console.WriteLine($"  Iterations: {result.Iterations}");
            Console.WriteLine($"  Max defect: {result.MaxDefect:E3}");
            Console.WriteLine();
        }
        else
        {
            // Interactive mode with visualization
            var optimizationTask = Task.Run(() =>
            {
                try
                {
                    var solver = new HermiteSimpsonSolver()
                        .WithSegments(30)
                        // .WithOrder(3)  // Use Order 3 for stability (higher orders can have spurious local minima)
                        .WithTolerance(1e-2)
                        .WithMaxIterations(200)
                        .WithMeshRefinement(true, 5, 1e-2)
                        .WithVerbose(true)
                        .WithInnerOptimizer(new LBFGSOptimizer()
                            .WithTolerance(1e-2)
                            .WithMaxIterations(200)
                            .WithVerbose(false))
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            // Check if visualization was closed
                            var token = RadiantBrachistochroneVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, stopping optimization...");
                                throw new OperationCanceledException(token);
                            }

                            // Update the live visualization with the current trajectory
                            RadiantBrachistochroneVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        });

                    var result = solver.Solve(problem);
                    Console.WriteLine("[SOLVER] Optimization completed successfully");
                    return result;
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

            // Run the visualization window on the main thread (blocks until window closed)
            RadiantBrachistochroneVisualizer.RunVisualizationWindow();

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

            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("SOLUTION SUMMARY:");
            Console.WriteLine($"  Success: {result.Success}");
            Console.WriteLine($"  Message: {result.Message}");
            Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3}) m");
            Console.WriteLine($"  Final velocity: {result.States[^1][2]:F3} m/s");
            Console.WriteLine($"  Optimal final time T_f: {result.States[^1][3]:F6} seconds");
            Console.WriteLine($"  Objective value: {result.OptimalCost:F6} (should equal T_f)");
            Console.WriteLine($"  Iterations: {result.Iterations}");
            Console.WriteLine($"  Max defect: {result.MaxDefect:E3}");
            Console.WriteLine();
        }
    }
}
