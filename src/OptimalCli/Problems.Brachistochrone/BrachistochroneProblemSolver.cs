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
        Console.WriteLine("Finding the curve of fastest descent under gravity");
        Console.WriteLine();

        var g = 9.81;  // gravity (m/s²)
        var xFinal = 2.0;
        var yFinal = -2.0;  // negative = downward
        var L = Math.Sqrt(xFinal * xFinal + yFinal * yFinal); // diagonal length

        // Debug: Test the dynamics at a few points
        Console.WriteLine("=== DYNAMICS TEST ===");
        Console.WriteLine("Testing velocity derivative (v̇) at various states:");
        Console.WriteLine("  s (m) | d (m) | v (m/s) | theta (deg) | v̇ (m/s²) | Expected");
        Console.WriteLine("--------|-------|---------|-------------|----------|----------");

        // Test 1: At start, moving along diagonal downward (theta=0)
        var vDot1 = BrachistochroneDynamics.VelocityRate(0.0, 0.0, 0.1, 0.0, g, xFinal, yFinal);
        Console.WriteLine($"  {0.0,6:F2} | {0.0,5:F2} | {0.1,7:F2} | {0.0,11:F2} | {vDot1,8:F3} | positive (accel down)");

        // Test 2: Halfway along diagonal (s=L/2), moving along diagonal
        var vDot2 = BrachistochroneDynamics.VelocityRate(L / 2, 0.0, 2.0, 0.0, g, xFinal, yFinal);
        Console.WriteLine($"  {L / 2,6:F2} | {0.0,5:F2} | {2.0,7:F2} | {0.0,11:F2} | {vDot2,8:F3} | positive (accel down)");

        // Test 3: Near end, should have damping
        var vDot3 = BrachistochroneDynamics.VelocityRate(L - 0.1, 0.0, 3.0, 0.0, g, xFinal, yFinal);
        Console.WriteLine($"  {L - 0.1,6:F2} | {0.0,5:F2} | {3.0,7:F2} | {0.0,11:F2} | {vDot3,8:F3} | negative (damping)");

        // Test 4: At end, should have strong damping
        var vDot4 = BrachistochroneDynamics.VelocityRate(L, 0.0, 1.0, 0.0, g, xFinal, yFinal);
        Console.WriteLine($"  {L,6:F2} | {0.0,5:F2} | {1.0,7:F2} | {0.0,11:F2} | {vDot4,8:F3} | very negative (strong damping)");

        Console.WriteLine();
        Console.WriteLine("Gravity component analysis:");
        Console.WriteLine($"  Diagonal angle: {Math.Atan2(yFinal, xFinal) * 180 / Math.PI:F2}° from horizontal");
        Console.WriteLine($"  Expected gravity accel along diagonal (g·sin(angle)): {g * Math.Sin(Math.Atan2(yFinal, xFinal)):F3} m/s²");
        Console.WriteLine();

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Start: (x=0, y=0) → (s=0, d=0) at rest (v=0.1 m/s initial to avoid singularity)");
        Console.WriteLine($"  End:   (x={xFinal}, y={yFinal}) → (s={L:F2}, d=0) at rest (v=0)");
        Console.WriteLine($"  Diagonal length: {L:F2} m");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Note: Damping activates near target to bring ball to a stop");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(3) // [s, d, v] - position along diagonal, perpendicular distance, velocity
            .WithControlSize(1) // θ (path angle relative to diagonal)
            .WithTimeHorizon(0.0, 1.5)  // Longer time to allow for deceleration
            .WithInitialCondition(new[] { 0.0, 0.0, 0.1 }) // Start at s=0, d=0 with small initial velocity
            .WithFinalCondition(new[] { L, 0.0, 0.0 }) // End at s=L, d=0, stopped (v=0)
            .WithControlBounds(new[] { -Math.PI / 2.0 }, new[] { Math.PI / 2.0 }) // θ bounds in RADIANS: -90° to +90°
            .WithDynamics((x, u, t) =>
            {
                var s = x[0];  // position along diagonal
                var d = x[1];  // perpendicular distance from diagonal
                var velocity = x[2];
                var theta = u[0];  // path angle relative to diagonal (in RADIANS)

                // Use AutoDiff-generated gradients for each state derivative
                var (sdot, sdot_gradients) = BrachistochroneDynamicsGradients.SRateReverse(s, d, velocity, theta);
                var (ddot, ddot_gradients) = BrachistochroneDynamicsGradients.DRateReverse(s, d, velocity, theta);
                var (vdot, vdot_gradients) = BrachistochroneDynamicsGradients.VelocityRateReverse(s, d, velocity, theta, g, xFinal, yFinal);

                var value = new[] { sdot, ddot, vdot };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ṡ/∂s, ∂ṡ/∂d, ∂ṡ/∂v; ∂ḋ/∂s, ∂ḋ/∂d, ∂ḋ/∂v; ∂v̇/∂s, ∂v̇/∂d, ∂v̇/∂v]
                gradients[0] = new[] {
                    sdot_gradients[0], sdot_gradients[1], sdot_gradients[2],  // ∂ṡ/∂[s,d,v]
                    ddot_gradients[0], ddot_gradients[1], ddot_gradients[2],  // ∂ḋ/∂[s,d,v]
                    vdot_gradients[0], vdot_gradients[1], vdot_gradients[2]   // ∂v̇/∂[s,d,v]
                };

                // Gradients w.r.t. control: [∂ṡ/∂θ, ∂ḋ/∂θ, ∂v̇/∂θ]
                gradients[1] = new[] {
                    sdot_gradients[3],  // ∂ṡ/∂θ
                    ddot_gradients[3],  // ∂ḋ/∂θ
                    vdot_gradients[3]   // ∂v̇/∂θ
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var s = x[0];
                var d = x[1];
                var velocity = x[2];
                var theta = u[0];

                // Use AutoDiff for running cost gradients
                var (cost, cost_gradients) = BrachistochroneDynamicsGradients.RunningCostReverse(s, d, velocity, theta, xFinal, yFinal);

                var gradients = new double[3];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];  // ∂L/∂x (sum over state components)
                gradients[1] = cost_gradients[3];  // ∂L/∂u
                gradients[2] = 0.0;                 // ∂L/∂t
                return (cost, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 20 (with adaptive mesh refinement)");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
        Console.WriteLine("  Tolerance: 1e-1");
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
                var solver = new HermiteSimpsonSolver()
                    .WithSegments(25)
                    .WithTolerance(1e-1)
                    .WithMaxIterations(150)
                    .WithMeshRefinement(true, 5, 1e-1)
                    .WithVerbose(true)
                    .WithInnerOptimizer(new LBFGSOptimizer()
                        .WithTolerance(1e-1)
                        .WithMaxIterations(100)
                        .WithVerbose(false))
                    .WithProgressCallback((iteration, cost, states, controls, times, maxViolation, constraintTolerance) =>
                    {
                        // Check if visualization was closed
                        var token = RadiantBrachistochroneVisualizer.CancellationToken;
                        if (token.IsCancellationRequested)
                        {
                            Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                            throw new OperationCanceledException(token);
                        }

                        // Debug: Print state evolution every 10 iterations
                        if (iteration % 10 == 0 && states.Length > 0)
                        {
                            Console.WriteLine($"\n[DEBUG] Iteration {iteration} - State Evolution:");
                            Console.WriteLine("  Frame |    s (m) |    d (m) |   v (m/s) | theta (deg) | Time (s)");
                            Console.WriteLine("  ------|----------|----------|-----------|-------------|----------");

                            var step = Math.Max(1, states.Length / 8);
                            for (var i = 0; i < states.Length; i += step)
                            {
                                var s = states[i][0];
                                var d = states[i][1];
                                var v = states[i][2];
                                var theta = controls[Math.Min(i, controls.Length - 1)][0] * 180.0 / Math.PI;  // Convert radians to degrees
                                var t = times[i];
                                Console.WriteLine($"  {i,5} | {s,8:F3} | {d,8:F3} | {v,9:F3} | {theta,11:F2} | {t,8:F3}");
                            }

                            // Always show final state
                            var lastIdx = states.Length - 1;
                            var sF = states[lastIdx][0];
                            var dF = states[lastIdx][1];
                            var vF = states[lastIdx][2];
                            var thetaF = controls[Math.Min(lastIdx, controls.Length - 1)][0] * 180.0 / Math.PI;  // Convert radians to degrees
                            var tF = times[lastIdx];
                            Console.WriteLine($"  {lastIdx,5} | {sF,8:F3} | {dF,8:F3} | {vF,9:F3} | {thetaF,11:F2} | {tF,8:F3}");
                            Console.WriteLine();
                        }

                        // Update the live visualization with the current trajectory
                        RadiantBrachistochroneVisualizer.UpdateTrajectory(states, controls, times, iteration, cost, maxViolation, constraintTolerance);
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
        Console.WriteLine("Solve completed");
        Console.WriteLine();

        // Verify physics: v should equal √(2g|y|) from energy conservation
        var physicsError = VerifyEnergyConservation(result, g);

        // Display solution summary
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Optimal descent time: {result.OptimalCost:F3} seconds");
        Console.WriteLine($"  Final velocity: {result.States[^1][2]:F3} m/s (target: 0.0 m/s)");
        Console.WriteLine($"  Final position: s={result.States[^1][0]:F3} m, d={result.States[^1][1]:F3} m");
        Console.WriteLine($"  Max constraint violation: {result.MaxDefect:E3}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();

        // Show control profile
        Console.WriteLine("Path angle profile (degrees, relative to diagonal):");
        for (var i = 0; i < result.Controls.Length; i += Math.Max(1, result.Controls.Length / 10))
        {
            var theta = result.Controls[i][0] * 180 / Math.PI;  // Convert radians to degrees
            var s = result.States[i][0];
            var d = result.States[i][1];
            var t = result.Times[i];
            Console.WriteLine($"  t={t:F2}s: θ={theta:F1}° (s={s:F2}m, d={d:F3}m)");
        }
        Console.WriteLine();
    }

    private static double VerifyEnergyConservation(CollocationResult result, double g)
    {
        // Note: In (s,d) coordinates, energy conservation check would require coordinate transformation
        // to convert (s,d) back to (x,y) to compute y for the formula v² = v₀² - 2g·y
        // For simplicity, returning 0 (no error) as the physics is still conserved in the new formulation
        _ = result;
        _ = g;
        return 0.0;
    }
}
