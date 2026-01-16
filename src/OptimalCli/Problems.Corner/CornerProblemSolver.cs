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
using Optimal.NonLinear.Unconstrained;

namespace OptimalCli.Problems.Corner;

/// <summary>
/// Solves the Corner problem: Optimal racing line through a 90° turn using curvilinear coordinates.
/// State: [s, n, θ, v, T_f] - progress along centerline, lateral deviation, heading, velocity, final time
/// Control: [a, ω] - acceleration and steering rate
/// Minimize: Time to complete the corner
/// Constraints: Vehicle must stay within road boundaries (|n| ≤ RoadHalfWidth)
/// </summary>
public sealed class CornerProblemSolver : ICommand
{
    // Road geometry (from CornerDynamics)
    private static readonly double EntryLength = CornerDynamics.EntryLength;
    private static readonly double ArcLength = CornerDynamics.ArcLength;
    private static readonly double RoadHalfWidth = CornerDynamics.RoadHalfWidth;
    private static readonly double TotalLength = EntryLength + ArcLength + 20.0; // Exit length of 20m

    public string Name => "corner";

    public string Description => "Optimal racing line through 90° corner with road constraints (curvilinear coordinates)";

    public void Run(CommandOptions options)
    {
        // Debug visualization mode - just show track without optimization
        if (options.DebugViz)
        {
            RadiantCornerVisualizer.RunDebugVisualization();
            return;
        }

        Console.WriteLine("=== CORNER PROBLEM (CURVILINEAR COORDINATES) ===");
        Console.WriteLine("Finding optimal racing line through 90° right turn");
        Console.WriteLine("(Minimum time with free final time optimization)");
        Console.WriteLine();

        // Vehicle parameters
        var maxAccel = 5.0;      // Maximum acceleration (m/s²)
        var maxDecel = -8.0;     // Maximum braking (m/s²)
        var maxSteerRate = 1.0;  // Maximum steering rate (rad/s)
        var initialVelocity = 15.0;  // Entry speed (m/s)

        // Final position: at the end of the exit section
        var sInit = 0.0;           // Start at beginning of entry
        var sFinal = TotalLength;  // End at exit

        // Initial T_f guess: Based on variable velocity profile that slows to 5 m/s in the arc.
        // Average velocity ≈ 9 m/s, so T_f ≈ sFinal / v_avg = 42.9 / 9 ≈ 4.8 seconds
        var tfGuess = 5.0;         // Initial guess for final time

        Console.WriteLine($"Problem setup (curvilinear coordinates):");
        Console.WriteLine($"  Entry length: {EntryLength} m");
        Console.WriteLine($"  Arc length: {ArcLength:F3} m (π × {CornerDynamics.CenterlineRadius} / 2)");
        Console.WriteLine($"  Road half-width: {RoadHalfWidth} m");
        Console.WriteLine($"  Initial state: s=0, n=free, θ=0, v={initialVelocity} m/s");
        Console.WriteLine($"  Final state: s={sFinal:F1}, n=free, θ=-π/2, v=free");
        Console.WriteLine($"  Acceleration bounds: [{maxDecel}, {maxAccel}] m/s²");
        Console.WriteLine($"  Steering rate bounds: [-{maxSteerRate}, {maxSteerRate}] rad/s");
        Console.WriteLine($"  Final time T_f: free (to be optimized, guess {tfGuess} s)");
        Console.WriteLine();

        // State: [s, n, θ, v, T_f] where T_f is the free final time
        // Control: [a, ω] - acceleration and steering rate
        // Time is normalized to τ ∈ [0, 1]
        // Dynamics are scaled: dx/dτ = T_f · (dx/dt)

        var problem = new ControlProblem()
            .WithStateSize(5) // [s, n, θ, v, T_f]
            .WithControlSize(2) // [a, ω]
            .WithTimeHorizon(0.0, 1.0) // Normalized time τ ∈ [0, 1]
            .WithInitialCondition([sInit, double.NaN, 0.0, initialVelocity, tfGuess]) // Start at s=0, n=free, heading east
            .WithFinalCondition([sFinal, double.NaN, -Math.PI / 2.0, double.NaN, double.NaN]) // End at exit, n=free, heading south, free v, T_f
            .WithControlBounds([maxDecel, -maxSteerRate], [maxAccel, maxSteerRate])
            .WithStateBounds(
                [-1.0, -RoadHalfWidth, -Math.PI, 1.0, 0.5],  // Min: s >= -1, n >= -RoadHalfWidth, θ, v >= 1 m/s, T_f >= 0.5s
                [TotalLength + 5.0, RoadHalfWidth - 0.5, Math.PI, 30.0, 10.0])   // Max: s, n <= 4.5 (0.5m from apex), θ, v <= 30 m/s, T_f <= 10s
            .WithDynamics((x, u, _) =>
            {
                var s = x[0];
                var n = x[1];
                var theta = x[2];
                var v = x[3];
                var Tf = x[4];
                var a = u[0];
                var omega = u[1];

                // Physical dynamics using curvilinear coordinates
                var (sdotPhys, sdot_gradients) = CornerDynamicsGradients.ProgressRateReverse(s, n, theta, v);
                var (ndotPhys, ndot_gradients) = CornerDynamicsGradients.LateralRateReverse(s, n, theta, v);
                var (thetadotPhys, thetadot_gradients) = CornerDynamicsGradients.ThetaRateReverse(omega);
                var (vdotPhys, vdot_gradients) = CornerDynamicsGradients.VelocityRateReverse(a);

                // Time-scaled dynamics: dx/dτ = T_f · (dx/dt)
                var sdot = Tf * sdotPhys;
                var ndot = Tf * ndotPhys;
                var thetadot = Tf * thetadotPhys;
                var vdot = Tf * vdotPhys;
                var Tfdot = 0.0; // T_f is constant over the trajectory

                var value = new[] { sdot, ndot, thetadot, vdot, Tfdot };
                var gradients = new double[2][];

                // Gradients w.r.t. state: chain rule ∂(T_f·f)/∂x = T_f·(∂f/∂x), ∂(T_f·f)/∂T_f = f
                // State order: [s, n, θ, v, T_f]
                gradients[0] =
                [
                    // ∂ṡ/∂[s, n, θ, v, T_f]
                    Tf * sdot_gradients[0], Tf * sdot_gradients[1], Tf * sdot_gradients[2], Tf * sdot_gradients[3], sdotPhys,
                    // ∂ṅ/∂[s, n, θ, v, T_f]
                    Tf * ndot_gradients[0], Tf * ndot_gradients[1], Tf * ndot_gradients[2], Tf * ndot_gradients[3], ndotPhys,
                    // ∂θ̇/∂[s, n, θ, v, T_f] - thetadot only depends on omega, not state
                    0.0, 0.0, 0.0, 0.0, thetadotPhys,
                    // ∂v̇/∂[s, n, θ, v, T_f] - vdot only depends on a, not state
                    0.0, 0.0, 0.0, 0.0, vdotPhys,
                    // ∂Ṫf/∂[s, n, θ, v, T_f] = 0
                    0.0, 0.0, 0.0, 0.0, 0.0
                ];

                // Gradients w.r.t. control: [a, ω]
                gradients[1] =
                [
                    0.0, 0.0,  // ∂ṡ/∂[a, ω] = 0
                    0.0, 0.0,  // ∂ṅ/∂[a, ω] = 0
                    0.0, Tf * thetadot_gradients[0],  // ∂θ̇/∂[a, ω]
                    Tf * vdot_gradients[0], 0.0,  // ∂v̇/∂[a, ω]
                    0.0, 0.0   // ∂Ṫf/∂[a, ω] = 0
                ];

                return (value, gradients);
            })
            .WithTerminalCost((x, _) =>
            {
                // Minimize final time T_f
                var Tf = x[4];
                var gradients = new double[6]; // [s, n, θ, v, T_f, τ]
                gradients[4] = 1.0; // ∂Φ/∂T_f = 1
                return (Tf, gradients);
            })
            // Road boundary constraints with safety margin to avoid singularity at apex
            // The singularity occurs at n = CenterlineRadius = 5 where denominator 1 - n*κ = 0
            // Use a margin of 0.5m to keep away from the singularity
            // Left boundary: n >= -RoadHalfWidth  =>  -RoadHalfWidth - n <= 0
            .WithPathConstraint((x, _, _) =>
            {
                var n = x[1];
                var violation = -RoadHalfWidth - n;  // violation <= 0 when n >= -RoadHalfWidth
                var grads = new[] { 0.0, -1.0, 0.0, 0.0, 0.0 };
                return (violation, grads);
            })
            // Right boundary: n <= RoadHalfWidth - margin  =>  n - (RoadHalfWidth - margin) <= 0
            // This prevents getting too close to the apex singularity
            .WithPathConstraint((x, _, _) =>
            {
                var n = x[1];
                const double ApexMargin = 0.5;  // Keep 0.5m away from apex
                var maxN = RoadHalfWidth - ApexMargin;
                var violation = n - maxN;  // violation <= 0 when n <= maxN
                var grads = new[] { 0.0, 1.0, 0.0, 0.0, 0.0 };
                return (violation, grads);
            })
            // Time must be positive: T_f >= 0.5  =>  0.5 - T_f <= 0
            // This is needed because box constraints are only projected after inner optimization
            .WithPathConstraint((x, _, _) =>
            {
                var Tf = x[4];
                const double MinTime = 0.5;
                var violation = MinTime - Tf;  // violation <= 0 when T_f >= 0.5
                var grads = new[] { 0.0, 0.0, 0.0, 0.0, -1.0 };
                return (violation, grads);
            });

        Console.WriteLine("Solver configuration:");
        Console.WriteLine($"  Algorithm: {(options.Solver == SolverType.LGL ? "Legendre-Gauss-Lobatto" : "Hermite-Simpson")} direct collocation");
        Console.WriteLine("  Segments: 30");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS-B");
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
                var innerOptimizer = new LBFGSOptimizer()
                    .WithTolerance(1e-5)
                    .WithMaxIterations(200)
                    .WithVerbose(false);

                ISolver solver = useLGL
                    ? new LegendreGaussLobattoSolver()
                        .WithOrder(5)
                        .WithSegments(30)
                        .WithTolerance(1e-5)
                        .WithMaxIterations(200)
                        .WithVerbose(true)
                        .WithInnerOptimizer(innerOptimizer)
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            var token = RadiantCornerVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                throw new OperationCanceledException(token);
                            }
                            RadiantCornerVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        })
                    : new HermiteSimpsonSolver()
                        .WithSegments(30)
                        .WithTolerance(1e-3)  // Relaxed tolerance
                        .WithMaxIterations(200)
                        .WithMeshRefinement(true, 5, 1e-3)  // Relaxed mesh refinement threshold
                        .WithVerbose(true)
                        .WithInnerOptimizer(innerOptimizer)
                        .WithInitialPenalty(50.0) // Higher initial penalty to enforce constraints more strongly
                        .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
                        {
                            var token = RadiantCornerVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                throw new OperationCanceledException(token);
                            }
                            RadiantCornerVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        });

                // Create centerline initial guess (n=0 throughout)
                var initialGuess = CreateCenterlineInitialGuess(30, sInit, sFinal, initialVelocity, tfGuess);

                var result = solver.Solve(problem, initialGuess);
                Console.WriteLine("[SOLVER] Optimization completed successfully");
                return result;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("[SOLVER] Optimization cancelled");
                throw;
            }
        }, RadiantCornerVisualizer.CancellationToken);

        // Run the visualization window on the main thread
        RadiantCornerVisualizer.RunVisualizationWindow();

        // Check if optimization is still running after window closed
        if (!optimizationTask.IsCompleted)
        {
            Console.WriteLine("\nWindow closed - waiting for optimization to stop...");
            if (optimizationTask.Wait(TimeSpan.FromSeconds(5)))
            {
                Console.WriteLine("Optimization stopped gracefully");
            }
            else
            {
                Console.WriteLine("Optimization did not stop - returning to console");
            }
            Console.WriteLine("\n" + "=".PadRight(70, '=') + "\n");
            Console.WriteLine("OPTIMIZATION CANCELLED");
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
            Console.WriteLine("\n" + "=".PadRight(70, '=') + "\n");
            Console.WriteLine("OPTIMIZATION CANCELLED");
            return;
        }

        // Convert final state from curvilinear to Cartesian for display
        var finalS = result.States[^1][0];
        var finalN = result.States[^1][1];
        var (finalX, finalY) = CornerDynamicsHelpers.CurvilinearToCartesian(finalS, finalN);

        Console.WriteLine("\n" + "=".PadRight(70, '=') + "\n");
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final curvilinear: s={finalS:F3}, n={finalN:F3}");
        Console.WriteLine($"  Final Cartesian: ({finalX:F3}, {finalY:F3})");
        Console.WriteLine($"  Final heading: {result.States[^1][2]:F3} rad ({result.States[^1][2] * 180 / Math.PI:F1}°)");
        Console.WriteLine($"  Final velocity: {result.States[^1][3]:F3} m/s");
        Console.WriteLine($"  Optimal time T_f: {result.States[^1][4]:F3} seconds");
        Console.WriteLine($"  Objective value: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }

    /// <summary>
    /// Creates an initial guess using a racing line trajectory.
    /// The vehicle takes the outside line on entry, cuts toward the apex, then exits wide.
    /// This allows higher speed through the corner compared to the centerline.
    /// Decision vector layout: [x0, u0, x1, u1, ..., xN, uN]
    /// State: [s, n, θ, v, T_f], Control: [a, ω]
    /// </summary>
    private static double[] CreateCenterlineInitialGuess(
        int segments,
        double sInit,
        double sFinal,
        double initialVelocity,
        double tfGuess)
    {
        const int StateDim = 5;
        const int ControlDim = 2;
        var numNodes = segments + 1;
        var decisionVectorSize = numNodes * (StateDim + ControlDim);
        var z = new double[decisionVectorSize];

        // Racing line strategy:
        // - Entry: Move to outside of turn (n < 0 for right turn)
        // - Apex: Cut toward inside (n > 0), but stay away from singularity
        // - Exit: Return to outside (n < 0)
        //
        // With n ≠ 0, the effective curvature is κ_eff = κ / (1 - n × κ)
        // Outside (n < 0): κ_eff < κ, larger radius, allows higher speed
        // Inside (n > 0): κ_eff > κ, smaller radius, lower speed but shorter path
        //
        // The optimal racing line balances these effects for minimum time.

        var entryEnd = CornerDynamics.EntryLength;
        var arcLength = CornerDynamics.ArcLength;
        var arcEnd = entryEnd + arcLength;

        // Racing line lateral position profile
        // Use road half-width with margin to avoid singularity
        const double OutsideN = -3.5;  // Outside of turn (70% of road width)
        const double ApexN = 3.0;      // Inside toward apex (60% of road width, safe margin from singularity)

        // Maximum steering rate
        const double MaxSteerRate = 1.0;

        for (var k = 0; k < numNodes; k++)
        {
            var tau = (double)k / segments;

            // Compute s linearly first (will adjust velocity to match)
            var s = (1.0 - tau) * sInit + tau * sFinal;

            // Racing line lateral position profile based on s
            double n;
            if (s < entryEnd * 0.5)
            {
                // Early entry: stay on centerline, transition to outside
                var progress = s / (entryEnd * 0.5);
                n = progress * OutsideN;
            }
            else if (s < entryEnd)
            {
                // Late entry: on outside, preparing for turn
                n = OutsideN;
            }
            else if (s < entryEnd + arcLength * 0.5)
            {
                // First half of arc: transition from outside to apex
                var arcProgress = (s - entryEnd) / (arcLength * 0.5);
                n = OutsideN + arcProgress * (ApexN - OutsideN);
            }
            else if (s < arcEnd)
            {
                // Second half of arc: transition from apex back to outside
                var arcProgress = (s - entryEnd - arcLength * 0.5) / (arcLength * 0.5);
                n = ApexN + arcProgress * (OutsideN - ApexN);
            }
            else if (s < arcEnd + (sFinal - arcEnd) * 0.5)
            {
                // Early exit: still on outside
                n = OutsideN;
            }
            else
            {
                // Late exit: transition back to centerline
                var exitProgress = (s - arcEnd - (sFinal - arcEnd) * 0.5) / ((sFinal - arcEnd) * 0.5);
                n = OutsideN * (1.0 - exitProgress);
            }

            // Road curvature and effective curvature
            var kappa = CornerDynamics.RoadCurvature(s);
            var denominator = 1.0 - n * kappa;
            var kappaEff = denominator > 0.1 ? kappa / denominator : kappa / 0.1;  // Safety clamp

            // Velocity based on effective curvature to respect steering rate
            // ω = v × κ_eff, so v_max = MaxSteerRate / κ_eff
            var velocity = kappaEff > 0.01
                ? Math.Min(initialVelocity, MaxSteerRate / kappaEff)
                : initialVelocity;

            // Ensure minimum velocity
            velocity = Math.Max(velocity, 3.0);

            // Use actual road heading at position s (vehicle follows road direction)
            var theta = CornerDynamics.RoadHeading(s);

            // T_f = guess
            var state = new[] { s, n, theta, velocity, tfGuess };

            // Control: [a, ω]
            // ω = -κ_eff × v (physical steering rate for effective curvature)
            var omega = -kappaEff * velocity;

            // Clamp steering rate to bounds
            omega = Math.Clamp(omega, -MaxSteerRate, MaxSteerRate);

            // Acceleration: approximate from velocity changes
            // For simplicity, use 0 (let optimizer figure out the accelerations)
            var accel = 0.0;

            var control = new[] { accel, omega };

            // Set in decision vector
            var offset = k * (StateDim + ControlDim);
            Array.Copy(state, 0, z, offset, StateDim);
            Array.Copy(control, 0, z, offset + StateDim, ControlDim);
        }

        return z;
    }
}
