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
        Console.WriteLine();

        // Vehicle parameters
        var maxAccel = 5.0;      // Maximum acceleration (m/s²)
        var maxDecel = -8.0;     // Maximum braking (m/s²)
        var maxSteerRate = 1.0;  // Maximum steering rate (rad/s)
        var initialVelocity = 15.0;  // Entry speed (m/s)

        // Final position: center of exit road at x = HalfRoadWidth + HalfRoadWidth = RoadWidth = 10
        var exitCenterX = HalfRoadWidth + HalfRoadWidth; // = 10

        Console.WriteLine($"Problem setup:");
        Console.WriteLine($"  Road width: {RoadWidth} m");
        Console.WriteLine($"  Corner center: ({CornerCenterX}, {CornerCenterY})");
        Console.WriteLine($"  Corner outer radius: {CornerRadius} m");
        Console.WriteLine($"  Initial velocity: {initialVelocity} m/s");
        Console.WriteLine($"  Initial position: (-15, 0) heading east (θ=0)");
        Console.WriteLine($"  Final position: ({exitCenterX}, -25) heading south (θ=-π/2)");
        Console.WriteLine($"  Acceleration bounds: [{maxDecel}, {maxAccel}] m/s²");
        Console.WriteLine($"  Steering rate bounds: [-{maxSteerRate}, {maxSteerRate}] rad/s");
        Console.WriteLine();

        // Time horizon - estimate based on distance and speed
        var estimatedTime = 5.0;

        var problem = new ControlProblem()
            .WithStateSize(4) // [x, y, θ, v]
            .WithControlSize(2) // [a, ω]
            .WithTimeHorizon(0.0, estimatedTime)
            .WithInitialCondition(new[] { -15.0, 0.0, 0.0, initialVelocity }) // Entry straight, heading east
            .WithFinalCondition(new[] { exitCenterX, -25.0, -Math.PI / 2, double.NaN }) // Exit straight, heading south, free final velocity
            .WithControlBounds(new[] { maxDecel, -maxSteerRate }, new[] { maxAccel, maxSteerRate })
            .WithStateBounds(
                new[] { double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity, 1.0 }, // Min velocity 1 m/s
                new[] { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity, 50.0 }) // Max velocity 50 m/s
            .WithDynamics((x, u, t) =>
            {
                var xPos = x[0];
                var yPos = x[1];
                var theta = x[2];
                var v = x[3];
                var a = u[0];
                var omega = u[1];

                // Use AutoDiff-generated gradients
                var (xdot, xdot_gradients) = CornerDynamicsGradients.XRateReverse(xPos, yPos, theta, v);
                var (ydot, ydot_gradients) = CornerDynamicsGradients.YRateReverse(xPos, yPos, theta, v);
                var (thetadot, thetadot_gradients) = CornerDynamicsGradients.ThetaRateReverse(omega);
                var (vdot, vdot_gradients) = CornerDynamicsGradients.VelocityRateReverse(a);

                var value = new[] { xdot, ydot, thetadot, vdot };
                var gradients = new double[2][];

                // Gradients w.r.t. state: [∂ẋ/∂x, ∂ẋ/∂y, ∂ẋ/∂θ, ∂ẋ/∂v; ∂ẏ/∂...; ∂θ̇/∂...; ∂v̇/∂...]
                gradients[0] = new[]
                {
                    xdot_gradients[0], xdot_gradients[1], xdot_gradients[2], xdot_gradients[3],  // ∂ẋ/∂[x,y,θ,v]
                    ydot_gradients[0], ydot_gradients[1], ydot_gradients[2], ydot_gradients[3],  // ∂ẏ/∂[x,y,θ,v]
                    0.0, 0.0, 0.0, 0.0,  // ∂θ̇/∂[x,y,θ,v] = 0 (thetadot only depends on omega)
                    0.0, 0.0, 0.0, 0.0   // ∂v̇/∂[x,y,θ,v] = 0 (vdot only depends on a)
                };

                // Gradients w.r.t. control: [∂ẋ/∂a, ∂ẋ/∂ω; ∂ẏ/∂a, ∂ẏ/∂ω; ∂θ̇/∂a, ∂θ̇/∂ω; ∂v̇/∂a, ∂v̇/∂ω]
                gradients[1] = new[]
                {
                    0.0, 0.0,  // ∂ẋ/∂[a,ω] = 0
                    0.0, 0.0,  // ∂ẏ/∂[a,ω] = 0
                    0.0, thetadot_gradients[0],  // ∂θ̇/∂[a,ω]
                    vdot_gradients[0], 0.0   // ∂v̇/∂[a,ω]
                };

                return (value, gradients);
            })
            .WithRunningCost((x, u, t) =>
            {
                // Minimize time
                var cost = 1.0;

                // Gradients: running cost is constant 1, no dependence on state/control/time
                var gradients = new double[3];
                gradients[0] = 0.0;  // ∂L/∂x = 0
                gradients[1] = 0.0;  // ∂L/∂u = 0
                gradients[2] = 0.0;  // ∂L/∂t = 0
                return (cost, gradients);
            })
            // Inner boundary constraint (lower edge of entry, left edge of exit, inner corner)
            // The inner corner is a sharp 90° turn at the point (HalfRoadWidth, -HalfRoadWidth) = (5, -5)
            .WithPathConstraint((x, u, t) =>
            {
                var xPos = x[0];
                var yPos = x[1];

                double violation;
                double[] grads;

                if (xPos <= 0)
                {
                    // Entry region: lower boundary is y >= -HalfRoadWidth
                    violation = -HalfRoadWidth - yPos;  // violation <= 0 when y >= -HalfRoadWidth
                    grads = new[] { 0.0, -1.0, 0.0, 0.0 };
                }
                else if (yPos > -HalfRoadWidth)
                {
                    // Corner region (x > 0 AND y > -HalfRoadWidth): left boundary is x >= 0 (already satisfied)
                    // but also need y >= -HalfRoadWidth... wait, we're in this region so y > -HalfRoadWidth
                    // The inner boundary here is actually x <= HalfRoadWidth AND y >= -HalfRoadWidth
                    // For simplicity, use the corner point constraint: can't go past x=HalfRoadWidth towards the corner
                    // Actually the inner corner is just the single point (5, -5) - the vehicle must stay "outside" this corner
                    // We need: not (x >= HalfRoadWidth AND y <= -HalfRoadWidth) in this region
                    // But we're already in y > -HalfRoadWidth, so just need x to be limited
                    // Let's use: x <= HalfRoadWidth (can't cut the corner)
                    violation = xPos - HalfRoadWidth;  // violation <= 0 when x <= HalfRoadWidth
                    grads = new[] { 1.0, 0.0, 0.0, 0.0 };
                }
                else
                {
                    // Exit region (y <= -HalfRoadWidth): left boundary is x >= HalfRoadWidth
                    violation = HalfRoadWidth - xPos;  // violation <= 0 when x >= HalfRoadWidth
                    grads = new[] { -1.0, 0.0, 0.0, 0.0 };
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
                    grads = new[] { 0.0, 1.0, 0.0, 0.0 };
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
                        grads = new[] { dx / dist, dy / dist, 0.0, 0.0 };
                    }
                    else
                    {
                        grads = new[] { 0.0, 0.0, 0.0, 0.0 };
                    }
                }
                else
                {
                    // Exit region: right boundary is x <= HalfRoadWidth + RoadWidth = 15
                    var rightBoundary = HalfRoadWidth + RoadWidth;
                    violation = xPos - rightBoundary;  // violation <= 0 when x <= rightBoundary
                    grads = new[] { 1.0, 0.0, 0.0, 0.0 };
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
        Console.WriteLine($"  Optimal time (cost): {result.OptimalCost:F3} seconds");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine();
    }
}
