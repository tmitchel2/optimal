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

namespace OptimalCli.Problems.Pendulum;

/// <summary>
/// Solves the pendulum swing-up problem.
/// Bring pendulum from hanging down to fully inverted (vertical upright).
/// State: [θ, θ̇] where θ=0 is down, θ=π is up
/// Dynamics: θ̈ = -g/L*sin(θ) + u/mL²
/// Minimize: control effort
/// </summary>
public sealed class PendulumSwingUpProblemSolver : ICommand
{
    public string Name => "pendulum";

    public string Description => "Pendulum swing-up from hanging down to fully inverted (vertical)";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== PENDULUM SWING-UP ===");
        Console.WriteLine("Swinging pendulum from hanging down to fully inverted (vertical upright)");
        Console.WriteLine();

        var g = 9.81;
        var L = 1.0;
        var m = 1.0;
        var targetAngle = Math.PI; // Full vertical - 180 degrees (inverted position)

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Length: {L} m");
        Console.WriteLine($"  Mass: {m} kg");
        Console.WriteLine($"  Initial angle: 0° (hanging down)");
        Console.WriteLine($"  Target angle: {targetAngle * 180 / Math.PI:F1}° (fully inverted)");
        Console.WriteLine($"  Time horizon: 5.0 seconds");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(2) // [angle, angular_velocity]
            .WithControlSize(1) // torque
            .WithTimeHorizon(0.0, 5.0) // Longer time for full swing
            .WithInitialCondition(new[] { 0.0, 0.0 }) // Hanging down at rest
            .WithFinalCondition(new[] { targetAngle, 0.0 }) // Fully inverted at rest
            .WithControlBounds(new[] { -10.0 }, new[] { 10.0 }) // Higher torque limits for full swing
            .WithDynamics((x, u, t) =>
            {
                var theta = x[0];
                var thetadot = x[1];

                // Use shared dynamics with AutoDiff gradients
                var (angleRate, angleRateGrad) =
                    PendulumSwingUpDynamicsGradients.AngleRateReverse(theta, thetadot, u[0], g, L, m);
                var (angVelRate, angVelRateGrad) =
                    PendulumSwingUpDynamicsGradients.AngularVelocityRateReverse(theta, thetadot, u[0], g, L, m);

                var value = new[] { angleRate, angVelRate };
                var gradients = new double[2][];
                gradients[0] = new[] { angleRateGrad[0], angleRateGrad[1] };  // ∂θ̇/∂[θ, θ̇]
                gradients[1] = new[] { angVelRateGrad[0], angVelRateGrad[1] };  // ∂θ̈/∂[θ, θ̇]
                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var theta = x[0];
                var thetadot = x[1];

                // Use shared cost function with AutoDiff gradients
                var (cost, costGrad) =
                    PendulumSwingUpDynamicsGradients.RunningCostReverse(theta, thetadot, u[0]);

                var gradients = new double[3];
                gradients[0] = costGrad[0];  // ∂L/∂θ
                gradients[1] = costGrad[1];  // ∂L/∂θ̇
                gradients[2] = costGrad[2];  // ∂L/∂u
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 25");
        Console.WriteLine("  Max iterations: 150");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-2");
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
                        .WithSegments(25)
                        .WithTolerance(1e-5)
                        .WithMaxIterations(150)
                        .WithVerbose(true)
                        .WithInnerOptimizer(innerOptimizer)
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            var token = RadiantPendulumVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                                throw new OperationCanceledException(token);
                            }
                            RadiantPendulumVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        })
                    : new HermiteSimpsonSolver()
                        .WithSegments(25)
                        .WithTolerance(1e-5)
                        .WithMaxIterations(150)
                        .WithMeshRefinement(true, 5, 1e-5)
                        .WithVerbose(true)
                        .WithInnerOptimizer(innerOptimizer)
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            var token = RadiantPendulumVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                                throw new OperationCanceledException(token);
                            }
                            RadiantPendulumVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
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
        }, RadiantPendulumVisualizer.CancellationToken);

        // Run the visualization window on the main thread (blocks until window closed)
        RadiantPendulumVisualizer.RunVisualizationWindow();

        // Check if optimization is still running after window closed
        if (!optimizationTask.IsCompleted)
        {
            Console.WriteLine();
            Console.WriteLine("Window closed - waiting for optimization to stop...");

            // Give the optimization a chance to respond to cancellation
            // Wait up to 5 seconds for graceful shutdown
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
        Console.WriteLine($"  Final angle: {result.States[^1][0] * 180 / Math.PI:F2}° (target: {targetAngle * 180 / Math.PI:F1}°)");
        Console.WriteLine($"  Final angular velocity: {result.States[^1][1]:F4} rad/s");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
