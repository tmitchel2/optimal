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
/// Solves the cart-pole stabilization problem with full nonlinear dynamics.
/// State: [x, ẋ, θ, θ̇]
/// Control: Force on cart
/// Objective: Stabilize inverted pendulum from initial perturbation
/// </summary>
public sealed class CartPoleProblemSolver : IProblemSolver
{
    public string Name => "cartpole";

    public string Description => "Cart-pole stabilization with full nonlinear dynamics";

    public void Solve()
    {
        Console.WriteLine("=== CART-POLE STABILIZATION ===");
        Console.WriteLine("Stabilizing inverted pendulum on moving cart");
        Console.WriteLine();

        var M = 1.0;  // Cart mass
        var m = 0.1;  // Pole mass
        var L = 0.5;  // Pole length
        var g = 9.81; // Gravity

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Cart mass: {M} kg");
        Console.WriteLine($"  Pole mass: {m} kg");
        Console.WriteLine($"  Pole length: {L} m");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Initial state: x=0.1m, θ=0.08rad (4.6°)");
        Console.WriteLine($"  Target: Origin (all states zero)");
        Console.WriteLine($"  Time horizon: 1.5 seconds");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(4)
            .WithControlSize(1)
            .WithTimeHorizon(0.0, 1.5)
            .WithInitialCondition(new[] { 0.1, 0.0, 0.08, 0.0 }) // Maximum before timeout
            .WithFinalCondition(new[] { 0.0, 0.0, 0.0, 0.0 })
            .WithControlBounds(new[] { -3.0 }, new[] { 3.0 })
            .WithStateBounds(
                new[] { -1.0, -2.0, -0.3, -2.0 },
                new[] { 1.0, 2.0, 0.3, 2.0 })
            .WithDynamics((x, u, t) =>
            {
                var theta = x[2];
                var thetadot = x[3];
                var sinTheta = Math.Sin(theta);
                var cosTheta = Math.Cos(theta);
                var sin2Theta = sinTheta * sinTheta;
                var cos2Theta = cosTheta * cosTheta;

                // Full nonlinear denominators
                var denom_x = M + m * sin2Theta;
                var denom_theta = L * (4.0 / 3.0 - m * cos2Theta / (M + m));

                // Cart acceleration - full nonlinear form
                var xddot = (u[0] + m * L * thetadot * thetadot * sinTheta - m * g * sinTheta * cosTheta) / denom_x;
                // Pole angular acceleration - full nonlinear form
                var thetaddot = (g * sinTheta - cosTheta * (u[0] + m * L * thetadot * thetadot * sinTheta) / (M + m)) / denom_theta;

                var value = new[] { x[1], xddot, x[3], thetaddot };
                var gradients = new double[2][];

                // Simplified gradients (full derivatives are complex)
                gradients[0] = new[] {
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0,
                    // ∂ẍ/∂θ (simplified)
                    (m*L*thetadot*thetadot*cosTheta - m*g*(cos2Theta - sin2Theta))/denom_x,
                    // ∂ẍ/∂θ̇
                    2.0*m*L*thetadot*sinTheta/denom_x,
                    0.0, 0.0, 0.0, 1.0,
                    0.0, 0.0,
                    // ∂θ̈/∂θ (simplified)
                    (g*cosTheta + u[0]*sinTheta)/denom_theta,
                    // ∂θ̈/∂θ̇
                    0.0
                };

                // Control gradients
                gradients[1] = new[] {
                    0.0,
                    1.0/denom_x,
                    0.0,
                    -cosTheta/(denom_theta*(M+m))
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var value = 20.0 * x[0] * x[0] + 2.0 * x[1] * x[1] +
                            50.0 * x[2] * x[2] + 2.0 * x[3] * x[3] +
                            0.5 * u[0] * u[0];
                var gradients = new[] {
                    40.0 * x[0], 4.0 * x[1], 100.0 * x[2], 4.0 * x[3], 1.0 * u[0]
                };
                return (value, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 12");
        Console.WriteLine("  Max iterations: 30");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 2e-2");
        Console.WriteLine();
        Console.WriteLine("Solving...");

        var solver = new HermiteSimpsonSolver()
            .WithSegments(12)
            .WithTolerance(2e-2) // Very relaxed
            .WithMaxIterations(30)
            .WithInnerOptimizer(new LBFGSOptimizer()
                .WithTolerance(1e-2)
                .WithMaxIterations(40));

        var result = solver.Solve(problem);

        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        var finalState = result.States[^1];
        Console.WriteLine($"  Final cart position: {Math.Abs(finalState[0]):F4} m");
        Console.WriteLine($"  Final pole angle: {Math.Abs(finalState[2]):F4} rad ({Math.Abs(finalState[2]) * 180 / Math.PI:F2}°)");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();

        // Generate visualization
        var htmlPath = ResultVisualizer.GenerateHtml(
            result,
            "Cart-Pole Stabilization",
            new[] { "x (m)", "ẋ (m/s)", "θ (rad)", "θ̇ (rad/s)" },
            new[] { "Force (N)" });
        Console.WriteLine($"Visualization saved to: file://{htmlPath}");
    }
}
