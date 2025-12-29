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
                gradients[0] = new[] {
                    xdot_gradients[0], xdot_gradients[1], xdot_gradients[2],        // ∂ẋ/∂[x,y,θ]
                    ydot_gradients[0], ydot_gradients[1], ydot_gradients[2],        // ∂ẏ/∂[x,y,θ]
                    thetadot_gradients[0], thetadot_gradients[1], thetadot_gradients[2]  // ∂θ̇/∂[x,y,θ]
                };

                // Gradients w.r.t. control: [∂ẋ/∂ω, ∂ẏ/∂ω, ∂θ̇/∂ω]
                // Note: xdot and ydot don't depend on ω directly, so those gradients are 0
                // Only thetadot depends on ω
                gradients[1] = new[] {
                    0.0,                      // ∂ẋ/∂ω = 0 (xdot doesn't depend on ω)
                    0.0,                      // ∂ẏ/∂ω = 0 (ydot doesn't depend on ω)
                    thetadot_gradients[3]     // ∂θ̇/∂ω
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var xPos = x[0];
                var yPos = x[1];
                var theta = x[2];
                var omega = u[0];

                // Use AutoDiff for running cost gradients
                var (cost, cost_gradients) = DubinsCarDynamicsGradients.RunningCostReverse(xPos, yPos, theta, omega);

                var gradients = new double[3];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];  // ∂L/∂x (sum over state components)
                gradients[1] = cost_gradients[3];  // ∂L/∂u
                gradients[2] = 0.0;                 // ∂L/∂t
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 20");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();
        Console.WriteLine("Solving... (real-time visualization will be shown below)");
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        // Initialize Dubins car visualizer
        DubinsCarVisualizer.Initialize();

        var solver = new HermiteSimpsonSolver()
            .WithSegments(20)
            .WithTolerance(1e-3)
            .WithMaxIterations(100)
            .WithVerbose(false)  // Disable to avoid interference with visualizer
            .WithInnerOptimizer(new LBFGSOptimizer()
                .WithTolerance(1e-5)
                .WithVerbose(false))  // Disable verbose for cleaner visualization
            .WithProgressCallback((iteration, cost, states, controls, times) =>
            {
                // Extract path data
                var xData = new double[states.Length];
                var yData = new double[states.Length];
                var headings = new double[states.Length];
                for (var i = 0; i < states.Length; i++)
                {
                    xData[i] = states[i][0];      // x position
                    yData[i] = states[i][1];      // y position
                    headings[i] = states[i][2];   // heading θ
                }

                var turnRate = controls[^1][0];  // Current turning rate ω

                // Render the path
                DubinsCarVisualizer.RenderPath(xData, yData, headings, turnRate, iteration, cost);

                // Throttle updates (every 5 iterations)
                if (iteration % 5 == 0)
                {
                    System.Threading.Thread.Sleep(50);
                }
            });

        var result = solver.Solve(problem);

        // Restore console
        DubinsCarVisualizer.Cleanup();

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

        // Generate visualization
        var htmlPath = ResultVisualizer.GenerateHtml(
            result,
            "Dubins Car",
            new[] { "x (m)", "y (m)", "θ (rad)" },
            new[] { "ω (rad/s)" });
        Console.WriteLine($"Visualization saved to: file://{htmlPath}");
    }
}
