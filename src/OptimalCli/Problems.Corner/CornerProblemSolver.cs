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
        Console.WriteLine($"  Final state: s={sFinal:F1}, n=free, θ=+π/2, v=free");
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
            .WithFinalCondition([sFinal, double.NaN, Math.PI / 2.0, double.NaN, double.NaN]) // End at exit, n=free, heading south (θ=+π/2), free v, T_f
            .WithControlBounds([maxDecel, -maxSteerRate], [maxAccel, maxSteerRate])
            .WithStateBounds(
                [-1.0, -RoadHalfWidth, -Math.PI, 1.0, 0.5],  // Min: s >= -1, n >= -RoadHalfWidth, θ, v >= 1 m/s, T_f >= 0.5s
                [TotalLength + 5.0, RoadHalfWidth - 0.5, Math.PI, 30.0, 10.0])   // Max: s, n <= 4.5 (0.5m from apex), θ, v <= 30 m/s, T_f <= 10s
            .WithDynamics(input =>
            {
                var x = input.State;
                var u = input.Control;
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

                return new DynamicsResult(value, gradients);
            })
            .WithTerminalCost((x, _) =>
            {
                // Minimize final time T_f
                var Tf = x[4];
                var gradients = new double[6]; // [s, n, θ, v, T_f, τ]
                gradients[4] = 1.0; // ∂Φ/∂T_f = 1
                return (Tf, gradients);
            })
            // Road boundary constraints
            // The singularity occurs at n = CenterlineRadius = 5 where denominator 1 - n*κ = 0
            // This is on the INSIDE of the turn (positive n), so only the right boundary needs a margin
            // Left boundary (outside of turn): n >= -RoadHalfWidth (no margin needed, no singularity)
            .WithPathConstraint((x, _, _) =>
            {
                var n = x[1];
                var violation = -RoadHalfWidth - n;  // violation <= 0 when n >= -RoadHalfWidth
                var grads = new[] { 0.0, -1.0, 0.0, 0.0, 0.0 };
                return (violation, grads);
            })
            // Right boundary (inside of turn): n <= RoadHalfWidth - margin
            // Use margin of 0.5m to keep away from the apex singularity
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
                            if (iteration == 1857)
                            {
                                Console.WriteLine("Breakpoint hit at iteration 1861");
                            }

                            var token = RadiantCornerVisualizer.CancellationToken;
                            if (token.IsCancellationRequested)
                            {
                                throw new OperationCanceledException(token);
                            }
                            RadiantCornerVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
                        });

                // Create centerline initial guess (n=0 throughout)
                var initialGuess = CreateCenterlineInitialGuess(30, sInit, sFinal, initialVelocity);

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
    /// State: [s, n, θ, v, T_f], Control: [a, ω]
    /// </summary>
    private static InitialGuess CreateCenterlineInitialGuess(
        int segments,
        double sInit,
        double sFinal,
        double initialVelocity)
    {
        // Simple centerline initial guess:
        // - Follow exact centerline (n = 0)
        // - Match road heading exactly (θ = θ_road)
        // - Constant velocity throughout
        // - Integrate forward to get consistent state values
        //
        // On the centerline with θ = θ_road:
        // - ṡ = v (since cos(0) = 1 and denominator = 1 when n = 0)
        // - ṅ = 0 (since sin(0) = 0)
        // - θ̇ = ω = dθ_road/dt = (dθ_road/ds) × ṡ = (dθ_road/ds) × v

        var numNodes = segments + 1;
        var stateTrajectory = new double[numNodes][];
        var controlTrajectory = new double[numNodes][];

        var totalDistance = sFinal - sInit;
        var velocity = initialVelocity;

        // Time to traverse at constant velocity
        var Tf = totalDistance / velocity;

        // Arc geometry for steering rate calculation
        var arcLength = CornerDynamics.ArcLength;
        var entryEnd = CornerDynamics.EntryLength;
        var arcEnd = entryEnd + arcLength;

        // During arc: dθ_road/ds = +π / (2 × arcLength) = +1/R (left-hand rule: right turn increases θ)
        // So ω = dθ_road/ds × v = +v/R
        var arcSteeringRate = velocity / CornerDynamics.CenterlineRadius;

        for (var k = 0; k < numNodes; k++)
        {
            var tau = (double)k / segments;

            // Linear interpolation of s (exact for constant velocity on centerline)
            var s = sInit + tau * totalDistance;

            // Centerline: n = 0
            var n = 0.0;

            // Follow road heading exactly
            var theta = CornerDynamics.RoadHeading(s);

            // State: [s, n, θ, v, Tf]
            stateTrajectory[k] = new[] { s, n, theta, velocity, Tf };

            // Control: [a, ω]
            // - Acceleration = 0 (constant velocity)
            // - Steering rate = dθ_road/dt to follow road heading
            double omega;
            if (s < entryEnd)
            {
                // Entry straight: no steering needed
                omega = 0.0;
            }
            else if (s < arcEnd)
            {
                // Arc: steer at constant rate to follow centerline
                omega = arcSteeringRate;
            }
            else
            {
                // Exit straight: no steering needed
                omega = 0.0;
            }

            controlTrajectory[k] = new[] { 0.0, omega };
        }

        return new InitialGuess(stateTrajectory, controlTrajectory);
    }
}
