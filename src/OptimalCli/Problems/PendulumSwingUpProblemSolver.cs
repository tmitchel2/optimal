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

namespace OptimalCli.Problems;

/// <summary>
/// Solves the pendulum swing-up problem.
/// Bring pendulum from hanging down to fully inverted (vertical upright).
/// State: [θ, θ̇] where θ=0 is down, θ=π is up
/// Dynamics: θ̈ = -g/L*sin(θ) + u/mL²
/// Minimize: control effort
/// </summary>
public sealed class PendulumSwingUpProblemSolver : IProblemSolver
{
    public string Name => "pendulum";

    public string Description => "Pendulum swing-up from hanging down to fully inverted (vertical)";

    public void Solve()
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

                var thetaddot = -g / L * Math.Sin(theta) + u[0] / (m * L * L);

                var value = new[] { thetadot, thetaddot };
                var gradients = new double[2][];
                gradients[0] = new[] { 0.0, 1.0 };  // ∂θ̇/∂[θ, θ̇]
                gradients[1] = new[] {
                    -g / L * Math.Cos(theta),  // ∂θ̈/∂θ
                    0.0                         // ∂θ̈/∂θ̇
                };
                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                // Minimize control effort
                var value = 0.5 * u[0] * u[0];
                var gradients = new double[3];
                gradients[2] = u[0];
                return (value, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 25");
        Console.WriteLine("  Max iterations: 150");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-2");
        Console.WriteLine();
        Console.WriteLine("Solving... (real-time visualization will be shown below)");
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        // Initialize pendulum visualizer
        PendulumVisualizer.Initialize();

        var solver = new HermiteSimpsonSolver()
            .WithSegments(25) // More segments for complex trajectory
            .WithTolerance(1e-2) // Slightly relaxed for this difficult problem
            .WithMaxIterations(150) // More iterations for convergence
            .WithVerbose(false)  // Disable to avoid interference with visualizer
            .WithInnerOptimizer(new LBFGSOptimizer()
                .WithTolerance(1e-3)
                .WithMaxIterations(100)
                .WithVerbose(false))  // Disable verbose for cleaner visualization
            .WithProgressCallback((iteration, cost, states, controls, times) =>
            {
                // Extract final state (most interesting for visualization)
                var finalState = states[^1];
                var finalControl = controls[^1];
                var theta = finalState[0];      // Angle (rad)
                var thetaDot = finalState[1];   // Angular velocity (rad/s)
                var torque = finalControl[0];   // Torque (N·m)

                PendulumVisualizer.RenderPendulum(theta, thetaDot, torque, iteration, cost);

                // Throttle updates (every 5 iterations)
                if (iteration % 5 == 0)
                {
                    System.Threading.Thread.Sleep(50);
                }
            });

        var result = solver.Solve(problem);

        // Restore console
        PendulumVisualizer.Cleanup();

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

        // Generate visualization
        var htmlPath = ResultVisualizer.GenerateHtml(
            result,
            "Pendulum Swing-Up",
            new[] { "θ (rad)", "θ̇ (rad/s)" },
            new[] { "torque (N·m)" });
        Console.WriteLine($"Visualization saved to: file://{htmlPath}");
    }
}
