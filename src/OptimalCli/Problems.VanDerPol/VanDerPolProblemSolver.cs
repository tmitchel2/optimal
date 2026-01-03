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

namespace OptimalCli.Problems.VanDerPol;

/// <summary>
/// Solves the Van der Pol oscillator problem with control.
/// ẋ₁ = x₂
/// ẋ₂ = -x₁ + μ(1 - x₁²)x₂ + u
/// Stabilize to origin with minimum control effort.
/// </summary>
public sealed class VanDerPolProblemSolver : ICommand
{
    public string Name => "vanderpol";

    public string Description => "Van der Pol oscillator stabilization with minimum control effort";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== VAN DER POL OSCILLATOR ===");
        Console.WriteLine("Stabilizing to origin with minimum control effort");
        Console.WriteLine();

        var mu = 1.0; // Nonlinearity parameter

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Nonlinearity parameter: μ = {mu}");
        Console.WriteLine($"  Initial state: (2.0, 0.0)");
        Console.WriteLine($"  Target state: (0.0, 0.0)");
        Console.WriteLine($"  Time horizon: 10.0 seconds");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(2)
            .WithControlSize(1)
            .WithTimeHorizon(0.0, 10.0)
            .WithInitialCondition(new[] { 2.0, 0.0 }) // Start away from origin
            .WithFinalCondition(new[] { 0.0, 0.0 }) // Stabilize to origin
            .WithControlBounds(new[] { -5.0 }, new[] { 5.0 })
            .WithDynamics((x, u, t) =>
            {
                var x1 = x[0];
                var x2 = x[1];

                var x1dot = x2;
                var x2dot = -x1 + mu * (1.0 - x1 * x1) * x2 + u[0];

                var value = new[] { x1dot, x2dot };
                var gradients = new double[2][];
                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                // Minimize control effort and state deviation
                var value = 0.1 * (x[0] * x[0] + x[1] * x[1]) + u[0] * u[0];
                var gradients = new double[4];
                return (value, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 25");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();
        Console.WriteLine("Solving...");

        var innerOptimizer = new LBFGSOptimizer().WithTolerance(1e-5);

        ISolver solver = options.Solver == SolverType.LGL
            ? new LegendreGaussLobattoSolver()
                .WithOrder(5)
                .WithSegments(25)
                .WithTolerance(1e-3)
                .WithMaxIterations(100)
                .WithInnerOptimizer(innerOptimizer)
            : new HermiteSimpsonSolver()
                .WithSegments(25)
                .WithTolerance(1e-3)
                .WithMaxIterations(100)
                .WithMeshRefinement(true, 5, 1e-3)
                .WithInnerOptimizer(innerOptimizer);

        var result = solver.Solve(problem);

        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final x₁: {result.States[^1][0]:F6}");
        Console.WriteLine($"  Final x₂: {result.States[^1][1]:F6}");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
