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
/// Solves the Dubins car problem: Minimum path length with curvature constraint.
/// State: [x, y, θ] - position and heading
/// Control: ω - turning rate (with fixed velocity)
/// Minimize: Control effort (smooth path)
/// </summary>
public sealed class DubinsCarProblemSolver : IProblemSolver
{
    public string Name => "dubins";

    public string Description => "Dubins car minimum path with curvature constraint";

    public void Solve()
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
            .WithInitialCondition(new[] { 0.0, 0.0, 0.0 }) // Origin, facing east
            .WithFinalCondition(new[] { 2.0, 2.0, Math.PI / 2 }) // Target position and heading
            .WithControlBounds(new[] { -1.5 }, new[] { 1.5 }) // Turn rate limits
            .WithDynamics((x, u, t) =>
            {
                var theta = x[2];
                var omega = u[0];

                var xdot = v * Math.Cos(theta);
                var ydot = v * Math.Sin(theta);
                var thetadot = omega;

                var value = new[] { xdot, ydot, thetadot };
                var gradients = new double[2][];
                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                // Minimize control effort (smooth path)
                var value = 0.1 * u[0] * u[0];
                var gradients = new double[4];
                return (value, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 20");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();
        Console.WriteLine("Solving...");

        var solver = new HermiteSimpsonSolver()
            .WithSegments(20)
            .WithTolerance(1e-3)
            .WithMaxIterations(100)
            .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

        var result = solver.Solve(problem);

        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3})");
        Console.WriteLine($"  Final heading: {result.States[^1][2]:F3} rad ({result.States[^1][2] * 180 / Math.PI:F1}°)");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();

        // Generate visualization
        var htmlPath = ResultVisualizer.GenerateHtml(
            result,
            "Dubins Car",
            new[] { "x (m)", "y (m)", "θ (rad)" },
            new[] { "ω (rad/s)" });
        Console.WriteLine($"Visualization saved to: file://{htmlPath}");
    }
}
