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
using Optimal.NonLinear.Unconstrained;

namespace OptimalCli.Problems.Dubin;

/// <summary>
/// Solves the Dubins car problem: Minimum path length with curvature constraint.
/// State: [x, y, θ] - position and heading
/// Control: ω - turning rate (with fixed velocity)
/// Minimize: Control effort (smooth path)
/// </summary>
public sealed class DubinsCarProblemSolver : ICommand
{
    public string Name => "dubins";

    public string Description => "Dubins car minimum path with curvature constraint";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== DUBINS CAR PROBLEM ===");
        Console.WriteLine("Finding minimum path with curvature constraint");
        Console.WriteLine();

        var v = 1.0; // Constant forward velocity

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Forward velocity: {v} m/s (constant)");
        Console.WriteLine($"  Initial state: (0, 0) facing east (θ=0)");
        Console.WriteLine($"  Target state: (2, 2) facing north (θ=π/2)");
        Console.WriteLine($"  Turn rate bounds: [-1.5, 1.5] rad/s");
        Console.WriteLine($"  Time horizon: 5.0 seconds");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(3) // [x, y, θ]
            .WithControlSize(1) // turning rate ω
            .WithTimeHorizon(0.0, 5.0)
            .WithInitialCondition([0.0, 0.0, 0.0]) // Origin, facing east
            .WithFinalCondition([2.0, 2.0, Math.PI / 2]) // Target position and heading
            .WithControlBounds([-1.5], [1.5]) // Turn rate limits
            .WithDynamics((x, u, _) =>
            {
                var xPos = x[0];
                var yPos = x[1];
                var theta = x[2];
                var omega = u[0];

                // Use AutoDiff-generated gradients for each state derivative
                var (xdot, xdot_gradients) = DubinsCarDynamicsGradients.XRateReverse(xPos, yPos, theta, v);
                var (ydot, ydot_gradients) = DubinsCarDynamicsGradients.YRateReverse(xPos, yPos, theta, v);
                var (thetadot, thetadot_gradients) = DubinsCarDynamicsGradients.ThetaRateReverse(xPos, yPos, theta, omega);

                var value = new[] { xdot, ydot, thetadot };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ẋ/∂x, ∂ẋ/∂y, ∂ẋ/∂θ; ∂ẏ/∂x, ∂ẏ/∂y, ∂ẏ/∂θ; ∂θ̇/∂x, ∂θ̇/∂y, ∂θ̇/∂θ]
                gradients[0] = [
                    xdot_gradients[0], xdot_gradients[1], xdot_gradients[2],        // ∂ẋ/∂[x,y,θ]
                    ydot_gradients[0], ydot_gradients[1], ydot_gradients[2],        // ∂ẏ/∂[x,y,θ]
                    thetadot_gradients[0], thetadot_gradients[1], thetadot_gradients[2]  // ∂θ̇/∂[x,y,θ]
                ];

                // Gradients w.r.t. control: [∂ẋ/∂ω, ∂ẏ/∂ω, ∂θ̇/∂ω]
                // Note: xdot and ydot don't depend on ω directly, so those gradients are 0
                // Only thetadot depends on ω
                gradients[1] = [
                    0.0,                      // ∂ẋ/∂ω = 0 (xdot doesn't depend on ω)
                    0.0,                      // ∂ẏ/∂ω = 0 (ydot doesn't depend on ω)
                    thetadot_gradients[3]     // ∂θ̇/∂ω
                ];

                return (value, gradients);
            })
            .WithRunningCost((x, u, _) =>
            {
                var xPos = x[0];
                var yPos = x[1];
                var theta = x[2];
                var omega = u[0];

                // Use AutoDiff for running cost gradients
                var (cost, cost_gradients) = DubinsCarDynamicsGradients.RunningCostReverse(xPos, yPos, theta, omega);

                var gradients = new double[3];
                // Running cost is 0.1 * omega^2, which doesn't depend on state (x, y, theta)
                gradients[0] = 0.0;                 // ∂L/∂x = 0 (cost independent of state)
                gradients[1] = cost_gradients[3];  // ∂L/∂u (∂L/∂omega)
                gradients[2] = 0.0;                 // ∂L/∂t = 0
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 20");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();
        Console.WriteLine("Solving...");
        Console.WriteLine("Opening live visualization window...");
        Console.WriteLine("(Close window when done viewing)");
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        var useLGL = options.Solver == SolverType.LGL;

        // Run the optimizer in a background task
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var innerOptimizer = new LBFGSOptimizer()
                    .WithTolerance(1e-5)
                    .WithMaxIterations(150)
                    .WithVerbose(false);

                ISolver solver = useLGL
                    ? new LegendreGaussLobattoSolver()
                        .WithOrder(5)
                        .WithSegments(20)
                        .WithTolerance(1e-5)
                        .WithMaxIterations(150)
                        .WithVerbose(true)
                        .WithInnerOptimizer(innerOptimizer)
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            var token = RadiantDubinsCarVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                                throw new OperationCanceledException(token);
                            }
                            RadiantDubinsCarVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        })
                    : new HermiteSimpsonSolver()
                        .WithSegments(20)
                        .WithTolerance(1e-5)
                        .WithMaxIterations(150)
                        .WithMeshRefinement(true, 5, 1e-5)
                        .WithVerbose(true)
                        .WithInnerOptimizer(innerOptimizer)
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            var token = RadiantDubinsCarVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                                throw new OperationCanceledException(token);
                            }
                            RadiantDubinsCarVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        });

                var result = solver.Solve(problem);
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
        }, RadiantDubinsCarVisualizer.CancellationToken);

        // Run the visualization window on the main thread (blocks until window closed)
        RadiantDubinsCarVisualizer.RunVisualizationWindow();

        // Check if optimization is still running after window closed
        if (!optimizationTask.IsCompleted)
        {
            Console.WriteLine();
            Console.WriteLine("Window closed - waiting for optimization to stop...");

            // Give the optimization a chance to respond to cancellation
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
            Console.WriteLine("  Optimization was cancelled");
            Console.WriteLine();
            return;
        }

        Console.WriteLine();
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3})");
        Console.WriteLine($"  Final heading: {result.States[^1][2]:F3} rad ({result.States[^1][2] * 180 / Math.PI:F1}°)");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
