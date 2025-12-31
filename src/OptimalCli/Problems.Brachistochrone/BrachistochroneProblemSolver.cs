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
/// </summary>
public sealed class BrachistochroneProblemSolver : IProblemSolver
{
    public string Name => "brachistochrone";

    public string Description => "Curve of fastest descent under gravity (Johann Bernoulli, 1696)";

    public void Solve()
    {
        Console.WriteLine("=== BRACHISTOCHRONE PROBLEM ===");
        Console.WriteLine("Find the curve of fastest descent under gravity between two points");
        Console.WriteLine();

        // Problem parameters
        var g = 9.81;           // Gravitational acceleration (m/s²)
        var xFinal = 2.0;       // Horizontal distance to target
        var yFinal = -2.0;      // Vertical distance to target (negative = down)

        // Calculate diagonal parameters
        var diagonalLength = Math.Sqrt(xFinal * xFinal + yFinal * yFinal);
        var alpha = Math.Atan2(yFinal, xFinal);  // Angle of diagonal from horizontal

        var finalTime = 1.5;    // Initial guess for final time (will be optimized)

        Console.WriteLine("Problem setup:");
        Console.WriteLine($"  Start point: (0, 0)");
        Console.WriteLine($"  End point: ({xFinal}, {yFinal})");
        Console.WriteLine($"  Diagonal length: {diagonalLength:F3} m");
        Console.WriteLine($"  Diagonal angle: {alpha * 180 / Math.PI:F1}° from horizontal");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Initial velocity: 0 m/s");
        Console.WriteLine($"  Objective: Minimize time of descent");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(3)  // [s, d, v]
            .WithControlSize(1)  // [theta]
            .WithTimeHorizon(0.0, finalTime)
            .WithInitialCondition(new[] { 0.0, 0.0, 0.0 })  // Start at origin with zero velocity
            .WithFinalCondition(new[] { diagonalLength, 0.0, double.NaN })  // End at target (on diagonal), free velocity
            .WithControlBounds(new[] { -Math.PI / 2 }, new[] { Math.PI / 2 })  // Theta between -90° and +90°
            .WithStateBounds(
                new[] { 0.0, -diagonalLength, 0.0 },  // s >= 0, reasonable d bounds, v >= 0
                new[] { diagonalLength * 1.5, diagonalLength, 20.0 })  // Reasonable upper bounds
            .WithDynamics((x, u, t) =>
            {
                var s = x[0];
                var d = x[1];
                var v = x[2];
                var theta = u[0];

                // Use AutoDiff-generated gradients
                var (srate, srate_gradients) = BrachistochroneDynamicsGradients.SRateReverse(s, d, v, theta, g, alpha);
                var (drate, drate_gradients) = BrachistochroneDynamicsGradients.DRateReverse(s, d, v, theta, g, alpha);
                var (vrate, vrate_gradients) = BrachistochroneDynamicsGradients.VRateReverse(s, d, v, theta, g, alpha);

                var value = new[] { srate, drate, vrate };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ṡ/∂s, ∂ṡ/∂d, ∂ṡ/∂v; ∂ḋ/∂s, ∂ḋ/∂d, ∂ḋ/∂v; ∂v̇/∂s, ∂v̇/∂d, ∂v̇/∂v]
                gradients[0] = new[] {
                    srate_gradients[0], srate_gradients[1], srate_gradients[2],  // ∂ṡ/∂[s,d,v]
                    drate_gradients[0], drate_gradients[1], drate_gradients[2],  // ∂ḋ/∂[s,d,v]
                    vrate_gradients[0], vrate_gradients[1], vrate_gradients[2]   // ∂v̇/∂[s,d,v]
                };

                // Gradients w.r.t. control: [∂ṡ/∂θ, ∂ḋ/∂θ, ∂v̇/∂θ]
                gradients[1] = new[] {
                    srate_gradients[3],  // ∂ṡ/∂θ
                    drate_gradients[3],  // ∂ḋ/∂θ
                    vrate_gradients[3]   // ∂v̇/∂θ
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var s = x[0];
                var d = x[1];
                var v = x[2];
                var theta = u[0];

                var (cost, cost_gradients) = BrachistochroneDynamicsGradients.RunningCostReverse(s, d, v, theta);

                var gradients = new double[3];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];  // ∂L/∂x (sum over state)
                gradients[1] = cost_gradients[3];  // ∂L/∂u
                gradients[2] = 0.0;                 // ∂L/∂t
                return (cost, gradients);
            })
            .WithTerminalCost((x, t) =>
            {
                var s = x[0];
                var d = x[1];
                var v = x[2];

                var (cost, cost_gradients) = BrachistochroneDynamicsGradients.TerminalCostReverse(s, d, v);

                var gradients = new double[2];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];  // ∂Φ/∂x
                gradients[1] = 1.0;  // ∂Φ/∂t = 1 (we want to minimize time)
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 40");
        Console.WriteLine("  Max iterations: 100");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();

        Console.WriteLine("Creating initial guess (straight diagonal path)...");
        const int segments = 40;
        var grid = new CollocationGrid(0.0, finalTime, segments);
        var transcription = new HermiteSimpsonTranscription(problem, grid);
        var initialGuess = new double[transcription.DecisionVectorSize];

        var nPoints = segments + 1;

        for (var k = 0; k < nPoints; k++)
        {
            var t = grid.TimePoints[k];
            var progress = t / finalTime;

            // Linear interpolation along diagonal
            var s = progress * diagonalLength;
            var d = 0.0;  // Start on diagonal
            var v = Math.Sqrt(2 * g * Math.Abs(yFinal) * progress);  // Approximate velocity from free fall

            // Initial theta guess: roughly aligned with diagonal
            var theta = 0.0;

            var state = new[] { s, d, v };
            var control = new[] { theta };

            transcription.SetState(initialGuess, k, state);
            transcription.SetControl(initialGuess, k, control);
        }

        Console.WriteLine($"  Initial guess created: {initialGuess.Length} decision variables");
        Console.WriteLine();

        Console.WriteLine("Solving...");
        Console.WriteLine("Opening live visualization window...");
        Console.WriteLine("(Close window when done viewing)");
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        // Run the optimizer in a background task
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var innerOptimizer = new LBFGSOptimizer()
                    .WithParallelLineSearch(enable: true, batchSize: 4)
                    .WithTolerance(0.01)
                    .WithMaxIterations(50)
                    .WithVerbose(false);

                // var innerOptimizer = new GradientDescentOptimizer()
                //     .WithTolerance(0.01)
                //     .WithMaxIterations(50)
                //     .WithVerbose(false);

                var solver = new HermiteSimpsonSolver()
                    .WithSegments(10)
                    .WithTolerance(0.001)
                    .WithMaxIterations(100)
                    .WithMeshRefinement(true, 5, 0.001)
                    .WithVerbose(true)
                    .WithInnerOptimizer(innerOptimizer)
                    .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                    {
                        // Check if visualization was closed
                        var token = RadiantBrachistochroneVisualizer.CancellationToken;
                        if (token.IsCancellationRequested)
                        {
                            Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, stopping...");
                            throw new OperationCanceledException(token);
                        }

                        // Update the live visualization with the current trajectory
                        RadiantBrachistochroneVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance, xFinal, yFinal, alpha);
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
            }

            Console.WriteLine();
            Console.WriteLine("=".PadRight(70, '='));
            Console.WriteLine();
            Console.WriteLine("OPTIMIZATION CANCELLED");
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
        var finalState = result.States[^1];
        Console.WriteLine($"  Final position along diagonal: {finalState[0]:F3} m (target: {diagonalLength:F3})");
        Console.WriteLine($"  Final perpendicular distance: {finalState[1]:F3} m (target: 0)");
        Console.WriteLine($"  Final velocity: {finalState[2]:F2} m/s");
        Console.WriteLine($"  Optimal time: {result.OptimalCost:F4} s");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
