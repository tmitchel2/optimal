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

namespace OptimalCli.Problems.Goddard;

/// <summary>
/// Solves the Goddard rocket problem (Goddard, 1919).
/// State: [h, v, m] (altitude, velocity, mass)
/// Control: T (thrust)
/// Objective: Maximize final altitude subject to drag, gravity, and fuel constraints
/// </summary>
public sealed class GoddardRocketProblemSolver : IProblemSolver
{
    public string Name => "goddard";

    public string Description => "Goddard rocket maximum altitude with drag and fuel constraints (Goddard, 1919)";

    public void Solve()
    {
        Console.WriteLine("=== GODDARD ROCKET PROBLEM ===");
        Console.WriteLine("Maximize final altitude of a vertically ascending rocket");
        Console.WriteLine();

        // Physical parameters (normalized Goddard rocket problem)
        var g = 1.0;         // Gravitational acceleration (normalized)
        var Dc = 620.0;      // Drag coefficient (D = Dc * v^2 * exp(-h/h0))
        var c = 0.5;         // Exhaust velocity (sqrt(T_max/(m0*g)))
        var h0 = 500.0;      // Scale height for drag (exponential atmosphere)
        var Tmax = 3.5;      // Maximum thrust
        var mInitial = 1.0;  // Initial mass (rocket + fuel)
        var mEmpty = 0.6;    // Empty mass (rocket without fuel)
        var finalTime = 5; // Final time - enough for thrust phase + coast to peak

        Console.WriteLine("Problem setup:");
        Console.WriteLine($"  Initial altitude: 0 m");
        Console.WriteLine($"  Initial velocity: 0 m/s");
        Console.WriteLine($"  Initial mass: {mInitial} kg (including fuel)");
        Console.WriteLine($"  Empty mass: {mEmpty} kg (rocket structure)");
        Console.WriteLine($"  Fuel available: {mInitial - mEmpty} kg");
        Console.WriteLine($"  Max thrust: {Tmax} N");
        Console.WriteLine($"  Exhaust velocity: c = {c}");
        Console.WriteLine($"  Gravity: {g} m/s²");
        Console.WriteLine($"  Drag coefficient: {Dc}");
        Console.WriteLine($"  Atmosphere scale height: {h0} m");
        Console.WriteLine($"  Time horizon: {finalTime} s");
        Console.WriteLine($"  Final condition: velocity = 0 (at peak altitude)");
        Console.WriteLine($"  Objective: Maximize final altitude");
        Console.WriteLine();

        var problem = new ControlProblem()
            .WithStateSize(3)
            .WithControlSize(1)
            .WithTimeHorizon(0.0, finalTime)
            .WithInitialCondition(new[] { 0.0, 0.0, mInitial })  // Start at ground, zero velocity, full mass
            .WithFinalCondition(new[] { double.NaN, 0.0, mEmpty })  // Free altitude, zero velocity at peak, free mass
            .WithControlBounds(new[] { 0.0 }, new[] { Tmax })  // Thrust between 0 and Tmax
            .WithStateBounds(
                new[] { 0.0, -2.0, mEmpty },  // h >= 0, reasonable v bounds, mass >= empty mass
                new[] { 1000.0, 2.0, mInitial })  // Reasonable bounds for optimization
            .WithDynamics((x, u, t) =>
            {
                var h = x[0];
                var v = x[1];
                var m = x[2];
                var T = u[0];

                // Use AutoDiff-generated gradients
                var (hrate, hrate_gradients) = GoddardRocketDynamicsGradients.AltitudeRateReverse(h, v, m, T, g, Dc, c, h0);
                var (vrate, vrate_gradients) = GoddardRocketDynamicsGradients.VelocityRateReverse(h, v, m, T, g, Dc, c, h0);
                var (mrate, mrate_gradients) = GoddardRocketDynamicsGradients.MassRateReverse(h, v, m, T, g, Dc, c, h0);

                var value = new[] { hrate, vrate, mrate };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ḣ/∂h, ∂ḣ/∂v, ∂ḣ/∂m; ∂v̇/∂h, ∂v̇/∂v, ∂v̇/∂m; ∂ṁ/∂h, ∂ṁ/∂v, ∂ṁ/∂m]
                gradients[0] = new[] {
                    hrate_gradients[0], hrate_gradients[1], hrate_gradients[2],  // ∂ḣ/∂[h,v,m]
                    vrate_gradients[0], vrate_gradients[1], vrate_gradients[2],  // ∂v̇/∂[h,v,m]
                    mrate_gradients[0], mrate_gradients[1], mrate_gradients[2]   // ∂ṁ/∂[h,v,m]
                };

                // Gradients w.r.t. control: [∂ḣ/∂T, ∂v̇/∂T, ∂ṁ/∂T]
                gradients[1] = new[] {
                    hrate_gradients[3],  // ∂ḣ/∂T
                    vrate_gradients[3],  // ∂v̇/∂T
                    mrate_gradients[3]   // ∂ṁ/∂T
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                var h = x[0];
                var v = x[1];
                var m = x[2];
                var T = u[0];

                var (cost, cost_gradients) = GoddardRocketDynamicsGradients.RunningCostReverse(h, v, m, T);

                var gradients = new double[3];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];  // ∂L/∂x (sum over state)
                gradients[1] = cost_gradients[3];  // ∂L/∂u
                gradients[2] = 0.0;                 // ∂L/∂t
                return (cost, gradients);
            })
            .WithTerminalCost((x, t) =>
            {
                var h = x[0];
                var v = x[1];
                var m = x[2];

                var (cost, cost_gradients) = GoddardRocketDynamicsGradients.TerminalCostReverse(h, v, m);

                var gradients = new double[2];
                gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];  // ∂Φ/∂x (sum over state)
                gradients[1] = 0.0;  // ∂Φ/∂t
                return (cost, gradients);
            })
            .WithPathConstraint((x, u, t) =>
            {
                // Path constraint: T * (m - mEmpty) >= 0
                // This ensures thrust can only be applied when fuel is available
                // When m = mEmpty (no fuel), thrust must be zero
                var m = x[2];
                var T = u[0];

                // Constraint: -T * (m - mEmpty) <= 0  (equivalent to T * (m - mEmpty) >= 0)
                var constraint = -T * (m - mEmpty);

                // Gradients: [∂c/∂h, ∂c/∂v, ∂c/∂m, ∂c/∂T, ∂c/∂t]
                var gradients = new double[5];
                gradients[0] = 0.0;           // ∂c/∂h
                gradients[1] = 0.0;           // ∂c/∂v
                gradients[2] = -T;            // ∂c/∂m = -T
                gradients[3] = -(m - mEmpty); // ∂c/∂T = -(m - mEmpty)
                gradients[4] = 0.0;           // ∂c/∂t

                return (constraint, gradients);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine("  Segments: 30");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS-B with parallel line search");
        Console.WriteLine("  Parallel line search: 4 candidates per batch");
        Console.WriteLine("  Tolerance: 1e-3");
        Console.WriteLine();

        // Create bespoke initial guess
        Console.WriteLine("Creating initial guess...");
        const int segments = 30;
        var grid = new CollocationGrid(0.0, finalTime, segments);
        var transcription = new HermiteSimpsonTranscription(problem, grid);
        var initialGuess = new double[transcription.DecisionVectorSize];

        var nPoints = segments + 1;
        var thrustDuration = finalTime / 2.0;  // Thrust for half the time

        for (var k = 0; k < nPoints; k++)
        {
            var t = grid.TimePoints[k];
            double h, v, m, T;

            if (t <= thrustDuration)
            {
                // Thrust phase: constant thrust, mass decreases linearly
                var progress = t / thrustDuration;
                T = Tmax;
                m = mInitial - (mInitial - mEmpty) * progress;

                // Velocity increases linearly during thrust
                var avgMass = (mInitial + m) / 2.0;
                var avgAccel = (T / avgMass) - g;
                v = avgAccel * t;

                // Altitude from constant acceleration
                h = 0.5 * avgAccel * t * t;
            }
            else
            {
                // Coast phase: no thrust, mass stays at empty
                T = 0.0;
                m = mEmpty;

                // Time in coast phase
                var tCoast = t - thrustDuration;
                var coastDuration = finalTime - thrustDuration;

                // Velocity at end of thrust phase
                var avgMassThrustPhase = (mInitial + mEmpty) / 2.0;
                var avgAccelThrustPhase = (Tmax / avgMassThrustPhase) - g;
                var vMax = avgAccelThrustPhase * thrustDuration;
                var hAtCoastStart = 0.5 * avgAccelThrustPhase * thrustDuration * thrustDuration;

                // Velocity decreases linearly to zero
                v = vMax * (1.0 - tCoast / coastDuration);

                // Altitude continues to increase during coast
                h = hAtCoastStart + vMax * tCoast - 0.5 * vMax * tCoast * tCoast / coastDuration;
            }

            // Ensure physical constraints
            h = Math.Max(h, 0.0);
            m = Math.Max(m, mEmpty);
            v = Math.Max(v, 0.0);

            var state = new[] { h, v, m };
            var control = new[] { T };

            transcription.SetState(initialGuess, k, state);
            transcription.SetControl(initialGuess, k, control);
        }

        Console.WriteLine($"  Initial guess created: {initialGuess.Length} decision variables");
        Console.WriteLine($"  Thrust duration: {thrustDuration:F2} s (50% of time horizon)");
        Console.WriteLine($"  Coast duration: {finalTime - thrustDuration:F2} s");
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
                var lbfgInnerOptimizer = new LBFGSOptimizer()
                    .WithParallelLineSearch(enable: true, batchSize: 4)
                    .WithTolerance(1)
                    .WithMaxIterations(50)
                    .WithVerbose(false);

                // var gdInnerOptimizer = new GradientDescentOptimizer()
                //     .WithTolerance(1e-1)
                //     .WithMaxIterations(150)
                //     .WithVerbose(false);

                var solver = new HermiteSimpsonSolver()
                    .WithSegments(30)
                    .WithTolerance(1)
                    .WithMaxIterations(50)
                    .WithMeshRefinement(true, 5, 1)
                    .WithVerbose(true)
                    .WithInnerOptimizer(lbfgInnerOptimizer)
                    // .WithInnerOptimizer(gdInnerOptimizer)
                    .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                    {
                        // Check if visualization was closed
                        var token = RadiantGoddardRocketVisualizer.CancellationToken;
                        if (token.IsCancellationRequested)
                        {
                            Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, throwing exception to stop optimization...");
                            throw new OperationCanceledException(token);
                        }

                        // Update the live visualization with the current trajectory
                        RadiantGoddardRocketVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance, h0);
                    })
                    ;

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
        }, RadiantGoddardRocketVisualizer.CancellationToken);

        // Run the visualization window on the main thread (blocks until window closed)
        RadiantGoddardRocketVisualizer.RunVisualizationWindow();

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
        Console.WriteLine($"  Final altitude: {finalState[0]:F2} m");
        Console.WriteLine($"  Final velocity: {finalState[1]:F2} m/s");
        Console.WriteLine($"  Final mass: {finalState[2]:F3} kg (fuel remaining: {finalState[2] - mEmpty:F3} kg)");
        Console.WriteLine($"  Optimal cost: {result.OptimalCost:F6} (negative altitude)");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
