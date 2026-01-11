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
/// Solves the Corner problem: Optimal racing line through a 90° turn.
/// State: [x, y, θ, v] - position, heading, and velocity
/// Control: [a, ω] - acceleration and steering rate
/// Minimize: Time to complete the corner
/// Constraints: Vehicle must stay within road boundaries
/// </summary>
public sealed class CornerProblemSolver : ICommand
{
    // Road geometry: 90° right turn
    // The turn connects an entry straight (heading east) to an exit straight (heading south).
    // 
    // Layout (world coordinates, Y+ is up):
    //   - Entry: x <= 0, y ∈ [-HalfRoadWidth, +HalfRoadWidth] = [-5, +5]
    //   - Corner: The outer boundary is a quarter-circle arc centered at (HalfRoadWidth, -HalfRoadWidth) = (5, -5)
    //             with radius = RoadWidth = 10. The inner corner is the single point (5, -5).
    //   - Exit: y <= -HalfRoadWidth = -5, x ∈ [HalfRoadWidth, HalfRoadWidth + RoadWidth] = [5, 15]
    //
    // Region detection:
    //   - Entry region: x <= 0
    //   - Exit region: x > 0 AND y <= -HalfRoadWidth
    //   - Corner region: x > 0 AND y > -HalfRoadWidth
    private const double RoadWidth = 10.0;
    private const double HalfRoadWidth = RoadWidth / 2.0;
    // Corner center is at the inner corner where entry and exit meet
    private const double CornerCenterX = HalfRoadWidth;  // = 5
    private const double CornerCenterY = -HalfRoadWidth; // = -5
    private const double CornerRadius = RoadWidth;       // = 10 (outer boundary arc radius)

    public string Name => "corner";

    public string Description => "Optimal racing line through 90° corner with road constraints";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== CORNER PROBLEM ===");
        Console.WriteLine("Finding optimal racing line through 90° right turn");
        Console.WriteLine("(Minimum time with free final time optimization)");
        Console.WriteLine();

        // Vehicle parameters
        var maxAccel = 5.0;      // Maximum acceleration (m/s²)
        var maxDecel = -8.0;     // Maximum braking (m/s²)
        var maxSteerRate = 1.0;  // Maximum steering rate (rad/s)
        var initialVelocity = 15.0;  // Entry speed (m/s)

        // Final position: center of exit road at x = HalfRoadWidth + HalfRoadWidth = RoadWidth = 10
        var exitCenterX = HalfRoadWidth + HalfRoadWidth; // = 10
        var tfGuess = 3.0; // Initial guess for final time

        Console.WriteLine($"Problem setup (free final time with time-scaling):");
        Console.WriteLine($"  Road width: {RoadWidth} m");
        Console.WriteLine($"  Corner center: ({CornerCenterX}, {CornerCenterY})");
        Console.WriteLine($"  Corner outer radius: {CornerRadius} m");
        Console.WriteLine($"  Initial velocity: {initialVelocity} m/s");
        Console.WriteLine($"  Initial position: (-15, 0) heading east (θ=0)");
        Console.WriteLine($"  Final position: ({exitCenterX}, -25) heading south (θ=-π/2)");
        Console.WriteLine($"  Acceleration bounds: [{maxDecel}, {maxAccel}] m/s²");
        Console.WriteLine($"  Steering rate bounds: [-{maxSteerRate}, {maxSteerRate}] rad/s");
        Console.WriteLine($"  Final time T_f: free (to be optimized, guess {tfGuess} s)");
        Console.WriteLine($"  Normalized time: τ ∈ [0, 1]");
        Console.WriteLine();

        // State: [x, y, θ, v, T_f] where T_f is the free final time
        // Control: [a, ω] - acceleration and steering rate
        // Time is normalized to τ ∈ [0, 1]
        // Dynamics are scaled: dx/dτ = T_f · (dx/dt)

        var problem = new ControlProblem()
            .WithStateSize(5) // [x, y, θ, v, T_f]
            .WithControlSize(2) // [a, ω]
            .WithTimeHorizon(0.0, 1.0) // Normalized time τ ∈ [0, 1]
            .WithInitialCondition(new[] { -15.0, 0.0, 0.0, initialVelocity, tfGuess }) // Entry straight, heading east
            .WithFinalCondition(new[] { exitCenterX, -25.0, -Math.PI / 2, double.NaN, double.NaN }) // Exit straight, heading south, free v and T_f
            .WithControlBounds(new[] { maxDecel, -maxSteerRate }, new[] { maxAccel, maxSteerRate })
            .WithStateBounds(
                new[] { -20.0, -30.0, -Math.PI, 1.0, 0.5 },  // Min: x, y, θ, v >= 1 m/s, T_f >= 0.5s
                new[] { 20.0, 10.0, Math.PI, 30.0, 10.0 })   // Max: x, y, θ, v <= 30 m/s, T_f <= 10s
            .WithDynamics((x, u, tau) =>
            {
                var xPos = x[0];
                var yPos = x[1];
                var theta = x[2];
                var v = x[3];
                var Tf = x[4];
                var a = u[0];
                var omega = u[1];

                // Physical dynamics (before time scaling)
                var (xdotPhys, xdot_gradients) = CornerDynamicsGradients.XRateReverse(xPos, yPos, theta, v);
                var (ydotPhys, ydot_gradients) = CornerDynamicsGradients.YRateReverse(xPos, yPos, theta, v);
                var (thetadotPhys, thetadot_gradients) = CornerDynamicsGradients.ThetaRateReverse(omega);
                var (vdotPhys, vdot_gradients) = CornerDynamicsGradients.VelocityRateReverse(a);

                // Time-scaled dynamics: dx/dτ = T_f · (dx/dt)
                var xdot = Tf * xdotPhys;
                var ydot = Tf * ydotPhys;
                var thetadot = Tf * thetadotPhys;
                var vdot = Tf * vdotPhys;
                var Tfdot = 0.0; // T_f is constant over the trajectory

                var value = new[] { xdot, ydot, thetadot, vdot, Tfdot };
                var gradients = new double[2][];

                // Gradients w.r.t. state: chain rule ∂(T_f·f)/∂x = T_f·(∂f/∂x), ∂(T_f·f)/∂T_f = f
                // State order: [x, y, θ, v, T_f]
                gradients[0] = new[]
                {
                    // ∂ẋ/∂[x, y, θ, v, T_f]
                    Tf * xdot_gradients[0], Tf * xdot_gradients[1], Tf * xdot_gradients[2], Tf * xdot_gradients[3], xdotPhys,
                    // ∂ẏ/∂[x, y, θ, v, T_f]
                    Tf * ydot_gradients[0], Tf * ydot_gradients[1], Tf * ydot_gradients[2], Tf * ydot_gradients[3], ydotPhys,
                    // ∂θ̇/∂[x, y, θ, v, T_f] - thetadot only depends on omega, not state
                    0.0, 0.0, 0.0, 0.0, thetadotPhys,
                    // ∂v̇/∂[x, y, θ, v, T_f] - vdot only depends on a, not state
                    0.0, 0.0, 0.0, 0.0, vdotPhys,
                    // ∂Ṫf/∂[x, y, θ, v, T_f] = 0
                    0.0, 0.0, 0.0, 0.0, 0.0
                };

                // Gradients w.r.t. control: [a, ω]
                gradients[1] = new[]
                {
                    0.0, 0.0,  // ∂ẋ/∂[a, ω] = 0
                    0.0, 0.0,  // ∂ẏ/∂[a, ω] = 0
                    0.0, Tf * thetadot_gradients[0],  // ∂θ̇/∂[a, ω]
                    Tf * vdot_gradients[0], 0.0,  // ∂v̇/∂[a, ω]
                    0.0, 0.0   // ∂Ṫf/∂[a, ω] = 0
                };

                return (value, gradients);
            })
            .WithTerminalCost((x, tau) =>
            {
                // Minimize final time T_f
                var Tf = x[4];
                var gradients = new double[6]; // [x, y, θ, v, T_f, τ]
                gradients[4] = 1.0; // ∂Φ/∂T_f = 1
                return (Tf, gradients);
            })
            // Road boundary constraints
            // The road consists of:
            //   Entry section: y ∈ [-HalfRoadWidth, +HalfRoadWidth], for any x <= HalfRoadWidth
            //   Exit section: x ∈ [HalfRoadWidth, HalfRoadWidth + RoadWidth], for any y <= -HalfRoadWidth
            //   Corner section: outer arc (distance from (5,-5) <= 10) connecting entry upper to exit right
            //
            // Simplified inner constraint: vehicle cannot cut the corner
            // - If y > -HalfRoadWidth: must have x <= HalfRoadWidth (still on entry side)
            // - If x < HalfRoadWidth: must have y >= -HalfRoadWidth (still on entry side)  
            // - Combined: if in corner region, must satisfy y >= -HalfRoadWidth OR x >= HalfRoadWidth
            //   i.e., cannot be in the "cut corner" region where x < 5 AND y < -5
            //
            // Actually, let's define it region by region:
            // - Entry (x <= 0): y ∈ [-5, +5]
            // - Exit (y <= -5): x ∈ [5, 15]
            // - Transition (x > 0 AND y > -5): no inner constraint, but outer arc applies
            .WithPathConstraint((x, u, t) =>
            {
                var xPos = x[0];
                var yPos = x[1];

                double violation;
                double[] grads;

                if (xPos <= 0)
                {
                    // Entry region: lower boundary is y >= -HalfRoadWidth
                    violation = -HalfRoadWidth - yPos;
                    grads = new[] { 0.0, -1.0, 0.0, 0.0, 0.0 };
                }
                else if (yPos <= -HalfRoadWidth)
                {
                    // Exit region: left boundary is x >= HalfRoadWidth
                    violation = HalfRoadWidth - xPos;
                    grads = new[] { -1.0, 0.0, 0.0, 0.0, 0.0 };
                }
                else
                {
                    // Transition region (x > 0 AND y > -HalfRoadWidth): 
                    // Inner boundary is the corner point (5, -5)
                    // Must stay "outside" this corner: not (x >= 5 AND y <= -5)
                    // Since we're in y > -5, we're automatically satisfying this
                    // No active inner constraint in this region
                    violation = -1.0;  // Always satisfied
                    grads = new[] { 0.0, 0.0, 0.0, 0.0, 0.0 };
                }

                return (violation, grads);
            })
            // Outer boundary constraint (upper edge of entry, outer arc, right edge of exit)
            // The outer boundary is a quarter-circle arc centered at (HalfRoadWidth, -HalfRoadWidth) = (5, -5)
            // with radius = RoadWidth = 10
            .WithPathConstraint((x, u, t) =>
            {
                var xPos = x[0];
                var yPos = x[1];

                double violation;
                double[] grads;

                if (xPos <= 0)
                {
                    // Entry region: upper boundary is y <= HalfRoadWidth
                    violation = yPos - HalfRoadWidth;  // violation <= 0 when y <= HalfRoadWidth
                    grads = new[] { 0.0, 1.0, 0.0, 0.0, 0.0 };
                }
                else if (yPos > -HalfRoadWidth)
                {
                    // Corner region: outer boundary is distance from corner center <= CornerRadius
                    var dx = xPos - CornerCenterX;
                    var dy = yPos - CornerCenterY;
                    var dist = Math.Sqrt(dx * dx + dy * dy);
                    violation = dist - CornerRadius;  // violation <= 0 when dist <= CornerRadius
                    if (dist > 1e-6)
                    {
                        grads = new[] { dx / dist, dy / dist, 0.0, 0.0, 0.0 };
                    }
                    else
                    {
                        grads = new[] { 0.0, 0.0, 0.0, 0.0, 0.0 };
                    }
                }
                else
                {
                    // Exit region: right boundary is x <= HalfRoadWidth + RoadWidth = 15
                    var rightBoundary = HalfRoadWidth + RoadWidth;
                    violation = xPos - rightBoundary;  // violation <= 0 when x <= rightBoundary
                    grads = new[] { 1.0, 0.0, 0.0, 0.0, 0.0 };
                }

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
                        .WithTolerance(1e-5)
                        .WithMaxIterations(200)
                        .WithMeshRefinement(true, 5, 1e-5)
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
                        });

                var result = solver.Solve(problem);
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

        Console.WriteLine("\n" + "=".PadRight(70, '=') + "\n");
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final position: ({result.States[^1][0]:F3}, {result.States[^1][1]:F3})");
        Console.WriteLine($"  Final heading: {result.States[^1][2]:F3} rad ({result.States[^1][2] * 180 / Math.PI:F1}°)");
        Console.WriteLine($"  Final velocity: {result.States[^1][3]:F3} m/s");
        Console.WriteLine($"  Optimal time T_f: {result.States[^1][4]:F3} seconds");
        Console.WriteLine($"  Objective value: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
