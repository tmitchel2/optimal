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

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Start: ({x0}, {y0}) m (top)");
        Console.WriteLine($"  End: ({xf}, {yf}) m (bottom)");
        Console.WriteLine($"  Initial velocity: {v0:E2} m/s (nearly at rest)");
        Console.WriteLine($"  Final velocity: free");
        Console.WriteLine($"  Time: free (to be optimized)");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(3) // [x, y, v]
            .WithControlSize(1) // theta (angle)
            .WithTimeHorizon(0.0, 1.2) // Initial guess for time horizon
                                       // .WithFreeFinalTime(0.1, 2.0) // Optimize final time within reasonable bounds
            .WithInitialCondition(new[] { x0, y0, v0 }) // Start at top-left
            .WithFinalCondition(new[] { xf, yf, double.NaN }) // End at bottom-right, free final velocity
            .WithControlBounds(new[] { 0.0 }, new[] { Math.PI / 2.0 }) // Angle between 0 (horizontal) and π/2 (vertical down)
            .WithStateBounds(
                new[] { 0.0, 0.0, 1e-6 },     // x free but reasonable, y >= 0, v > 0
                new[] { 10.0, 15.0, 20.0 })    // Reasonable upper bounds
            .WithDynamics((x, u, t) =>
            {
                var xPos = x[0];
                var y = x[1];
                var v = x[2];
                var theta = u[0];

                // Use AutoDiff-generated gradients for each state derivative
                var (xrate, xrate_gradients) = BrachistochroneDynamicsGradients.XRateReverse(xPos, y, v, theta, g);
                var (yrate, yrate_gradients) = BrachistochroneDynamicsGradients.YRateReverse(xPos, y, v, theta, g);
                var (vrate, vrate_gradients) = BrachistochroneDynamicsGradients.VRateReverse(xPos, y, v, theta, g);

                var value = new[] { xrate, yrate, vrate };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ẋ/∂x, ∂ẋ/∂y, ∂ẋ/∂v; ∂ẏ/∂x, ∂ẏ/∂y, ∂ẏ/∂v; ∂v̇/∂x, ∂v̇/∂y, ∂v̇/∂v]
                gradients[0] = new[] {
                    xrate_gradients[0], xrate_gradients[1], xrate_gradients[2],  // ∂ẋ/∂[x,y,v]
                    yrate_gradients[0], yrate_gradients[1], yrate_gradients[2],  // ∂ẏ/∂[x,y,v]
                    vrate_gradients[0], vrate_gradients[1], vrate_gradients[2]   // ∂v̇/∂[x,y,v]
                };

                // Gradients w.r.t. control: [∂ẋ/∂θ, ∂ẏ/∂θ, ∂v̇/∂θ]
                gradients[1] = new[] {
                    xrate_gradients[3],  // ∂ẋ/∂θ
                    yrate_gradients[3],  // ∂ẏ/∂θ
                    vrate_gradients[3]   // ∂v̇/∂θ
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var xPos = x[0];
                var y = x[1];
                var v = x[2];
                var theta = u[0];

                // Use AutoDiff-generated gradients for cost
                var (cost, costGrad) = BrachistochroneDynamicsGradients.RunningCostReverse(xPos, y, v, theta);

                var gradients = new double[4];
                gradients[0] = costGrad[0];  // ∂L/∂x
                gradients[1] = costGrad[1];  // ∂L/∂y
                gradients[2] = costGrad[2];  // ∂L/∂v
                gradients[3] = costGrad[3];  // ∂L/∂θ
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 30");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-5");
        Console.WriteLine("  Initial guess: Physics-based with increasing velocity");
        Console.WriteLine();

        // Create physics-based initial guess with increasing velocity
        var grid = new CollocationGrid(0.0, 2.0, 30);
        var transcription = new HermiteSimpsonTranscription(problem, grid);
        var initialGuess = new double[transcription.DecisionVectorSize];

        // Generate physically realistic initial trajectory
        for (var k = 0; k <= grid.Segments; k++)
        {
            var alpha = (double)k / grid.Segments;

            // Linearly interpolate position
            var x = (1.0 - alpha) * x0 + alpha * xf;
            var y = (1.0 - alpha) * y0 + alpha * yf;

            // Velocity increases based on height drop: v = sqrt(2*g*Δh)
            var heightDrop = y0 - y;
            var v = Math.Sqrt(2.0 * g * Math.Max(heightDrop, 0.0)) + v0;

            var state = new[] { x, y, v };
            transcription.SetState(initialGuess, k, state);

            // Control: angle toward target, adjusted for descent
            var dx = xf - x;
            var dy = yf - y;
            var theta = Math.Atan2(-dy, dx); // Note: -dy because y decreases when descending
            theta = Math.Max(0.0, Math.Min(Math.PI / 2.0, theta)); // Clamp to bounds

            var control = new[] { theta };
            transcription.SetControl(initialGuess, k, control);
        }

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
                .WithTolerance(1e-5)
                .WithMaxIterations(200)
                .WithMeshRefinement(true, 5, 1e-5)
                .WithVerbose(true)
                .WithInnerOptimizer(new LBFGSOptimizer()
                    .WithTolerance(1e-5)
                    .WithMaxIterations(200)
                    .WithVerbose(true));

            var result = solver.Solve(problem, initialGuess);

            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("SOLUTION SUMMARY:");
            Console.WriteLine($"  Success: {result.Success}");
            Console.WriteLine($"  Message: {result.Message}");
            Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3}) m");
            Console.WriteLine($"  Final velocity: {result.States[^1][2]:F3} m/s");
            Console.WriteLine($"  Optimal descent time: {result.OptimalCost:F6} seconds");
            Console.WriteLine($"  Iterations: {result.Iterations}");
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
                        .WithTolerance(1e-5)
                        .WithMaxIterations(200)
                        .WithMeshRefinement(true, 5, 1e-5)
                        .WithVerbose(true)
                        .WithInnerOptimizer(new LBFGSOptimizer()
                            .WithTolerance(1e-5)
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

                    var result = solver.Solve(problem, initialGuess);
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
            Console.WriteLine($"  Optimal descent time: {result.OptimalCost:F6} seconds");
            Console.WriteLine($"  Iterations: {result.Iterations}");
            Console.WriteLine();
        }
    }
}
