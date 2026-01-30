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
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Unconstrained;

namespace OptimalCli.Problems.CartPole;

/// <summary>
/// Solves the cart-pole stabilization problem with full nonlinear dynamics.
/// State: [x, ẋ, θ, θ̇]
/// Control: Force on cart
/// Objective: Stabilize inverted pendulum from initial perturbation
/// </summary>
public sealed class CartPoleProblemSolver : ICommand
{
    public string Name => "cartpole";

    public string Description => "Cart-pole stabilization with full nonlinear dynamics";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== CART-POLE STABILIZATION ===");
        Console.WriteLine("Stabilizing inverted pendulum on moving cart");
        Console.WriteLine();

        var M = 1.0;  // Cart mass
        var m = 0.1;  // Pole mass
        var L = 2;  // Pole length
        var g = 9.81; // Gravity

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Cart mass: {M} kg");
        Console.WriteLine($"  Pole mass: {m} kg");
        Console.WriteLine($"  Pole length: {L} m");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Initial state: x=0.1m, θ=0.08rad (4.6° from upright)");
        Console.WriteLine($"  Target: θ=0 (perfectly upright), x=0, all velocities zero");
        Console.WriteLine($"  Time horizon: 1.5 seconds");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(4)
            .WithControlSize(1)
            .WithTimeHorizon(0.0, 5)
            .WithInitialCondition([0.1, 0.0, 0.1, 0.0]) // Maximum before timeout
            .WithFinalCondition([0.0, 0.0, 0.0, 0.0])
            .WithControlBounds([-3.0], [3.0])
            .WithStateBounds(
                [-1.0, -2.0, -0.3, -2.0],
                [1.0, 2.0, 0.3, 2.0])
            .WithDynamics(input =>
            {
                var x = input.State;
                var u = input.Control;
                var xPos = x[0];
                var xdot = x[1];
                var theta = x[2];
                var thetadot = x[3];
                var force = u[0];

                // Use AutoDiff-generated gradients for each state derivative
                var (xrate, xrate_gradients) = CartPoleDynamicsGradients.XRateReverse(xPos, xdot, theta, thetadot, force, M, m, L, g);
                var (xddot, xddot_gradients) = CartPoleDynamicsGradients.XddotRateReverse(xPos, xdot, theta, thetadot, force, M, m, L, g);
                var (thetarate, thetarate_gradients) = CartPoleDynamicsGradients.ThetaRateReverse(xPos, xdot, theta, thetadot, force, M, m, L, g);
                var (thetaddot, thetaddot_gradients) = CartPoleDynamicsGradients.ThetaddotRateReverse(xPos, xdot, theta, thetadot, force, M, m, L, g);

                var value = new[] { xrate, xddot, thetarate, thetaddot };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ẋ/∂x, ∂ẋ/∂ẋ, ∂ẋ/∂θ, ∂ẋ/∂θ̇; ∂ẍ/∂x, ∂ẍ/∂ẋ, ∂ẍ/∂θ, ∂ẍ/∂θ̇; ...]
                gradients[0] = [
                    xrate_gradients[0], xrate_gradients[1], xrate_gradients[2], xrate_gradients[3],           // ∂ẋ/∂[x,ẋ,θ,θ̇]
                    xddot_gradients[0], xddot_gradients[1], xddot_gradients[2], xddot_gradients[3],           // ∂ẍ/∂[x,ẋ,θ,θ̇]
                    thetarate_gradients[0], thetarate_gradients[1], thetarate_gradients[2], thetarate_gradients[3],  // ∂θ̇/∂[x,ẋ,θ,θ̇]
                    thetaddot_gradients[0], thetaddot_gradients[1], thetaddot_gradients[2], thetaddot_gradients[3]   // ∂θ̈/∂[x,ẋ,θ,θ̇]
                ];

                // Gradients w.r.t. control: [∂ẋ/∂F, ∂ẍ/∂F, ∂θ̇/∂F, ∂θ̈/∂F]
                gradients[1] = [
                    xrate_gradients[4],      // ∂ẋ/∂F
                    xddot_gradients[4],      // ∂ẍ/∂F
                    thetarate_gradients[4],  // ∂θ̇/∂F
                    thetaddot_gradients[4]   // ∂θ̈/∂F
                ];

                return new DynamicsResult(value, gradients);
            })
            .WithRunningCost(input =>
            {
                var x = input.State;
                var u = input.Control;
                var xPos = x[0];
                var xdot = x[1];
                var theta = x[2];
                var thetadot = x[3];
                var force = u[0];

                // Use AutoDiff for running cost gradients
                var (cost, cost_gradients) = CartPoleDynamicsGradients.RunningCostReverse(xPos, xdot, theta, thetadot, force);

                var gradients = new double[3];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2] + cost_gradients[3];  // ∂L/∂x (sum over state components)
                gradients[1] = cost_gradients[4];  // ∂L/∂u
                gradients[2] = 0.0;                 // ∂L/∂t
                return new RunningCostResult(cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 20");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-1");
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
                var innerOptimizer = new LBFGSOptimizer(new LBFGSOptions
                {
                    Tolerance = 1e-6,
                    MaxIterations = 150,
                    Verbose = true
                }, new BacktrackingLineSearch());

                ProgressCallback progressCallback = (iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                {
                    var token = RadiantCartPoleVisualizer.CancellationToken;
                    if (token.IsCancellationRequested)
                    {
                        Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                        throw new OperationCanceledException(token);
                    }
                    RadiantCartPoleVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance, L);
                };

                ISolver solver = useLGL
                    ? new LegendreGaussLobattoSolver(
                        new LegendreGaussLobattoSolverOptions
                        {
                            Order = 5,
                            Segments = 20,
                            Tolerance = 1e-5,
                            MaxIterations = 100,
                            Verbose = true,
                            ProgressCallback = progressCallback
                        },
                        innerOptimizer)
                    : new HermiteSimpsonSolver(
                        new HermiteSimpsonSolverOptions
                        {
                            Segments = 20,
                            Tolerance = 1e-5,
                            MaxIterations = 100,
                            EnableMeshRefinement = true,
                            MaxRefinementIterations = 5,
                            RefinementDefectThreshold = 1e-5,
                            Verbose = true,
                            ProgressCallback = progressCallback
                        },
                        innerOptimizer);

                var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(problem, 20);
                var result = solver.Solve(problem, initialGuess);
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
        }, RadiantCartPoleVisualizer.CancellationToken);

        // Run the visualization window on the main thread (blocks until window closed)
        RadiantCartPoleVisualizer.RunVisualizationWindow();

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
        var finalState = result.States[^1];
        Console.WriteLine($"  Final cart position: {Math.Abs(finalState[0]):F4} m");
        Console.WriteLine($"  Final pole angle: {Math.Abs(finalState[2]):F4} rad ({Math.Abs(finalState[2]) * 180 / Math.PI:F2}°)");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
