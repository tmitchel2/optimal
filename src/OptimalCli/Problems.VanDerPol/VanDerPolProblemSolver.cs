/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using Optimal.Control.Core;
using Optimal.Control.Solvers;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

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
            .WithInitialCondition([2.0, 0.0]) // Start away from origin
            .WithFinalCondition([0.0, 0.0]) // Stabilize to origin
            .WithControlBounds([-5.0], [5.0])
            .WithDynamics(input =>
            {
                var x = input.State;
                var u = input.Control;
                var x1 = x[0];
                var x2 = x[1];

                var x1dot = x2;
                var x2dot = -x1 + mu * (1.0 - x1 * x1) * x2 + u[0];

                var value = new[] { x1dot, x2dot };
                var gradients = new double[2][];
                return new DynamicsResult(value, gradients);
            })
            .WithRunningCost(input =>
            {
                // Minimize control effort and state deviation
                var x = input.State;
                var u = input.Control;
                var value = 0.1 * (x[0] * x[0] + x[1] * x[1]) + u[0] * u[0];
                var gradients = new double[4];
                return new RunningCostResult(value, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 25");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();
        Console.WriteLine("Solving...");

        var innerOptimizer = new LBFGSOptimizer(new LBFGSOptions { Tolerance = 1e-5 }, new BacktrackingLineSearch());

        var solver = new HermiteSimpsonSolver(
            new HermiteSimpsonSolverOptions
            {
                Segments = 25,
                Tolerance = 1e-3,
                MaxIterations = 100,
                EnableMeshRefinement = true,
                MaxRefinementIterations = 5,
                RefinementDefectThreshold = 1e-3
            },
            innerOptimizer);

        var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(problem, 25);
        var result = solver.Solve(problem, initialGuess);

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
