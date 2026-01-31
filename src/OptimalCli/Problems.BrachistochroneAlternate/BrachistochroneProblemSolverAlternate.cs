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

namespace OptimalCli.Problems.BrachistochroneAlternate;

/// <summary>
/// Solves the Brachistochrone problem (Johann Bernoulli, 1696) using arc-length parameterization.
/// Question: What curve gives the fastest descent under gravity between two points?
/// Answer: A cycloid - the curve traced by a point on a rolling circle.
///
/// Arc-length parameterized formulation (similar to CornerProblemSolver):
/// - Independent variable: s (horizontal distance from start)
/// - State: [v, n, alpha, t]
///   - v: Speed (m/s)
///   - n: Vertical position below start (m, positive = descended)
///   - alpha: Heading angle from horizontal (rad, positive = descending)
///   - t: Elapsed time (s)
/// - Control: [k]
///   - k: Path curvature = dalpha/ds (rad/m)
///
/// Objective: Minimize elapsed time (integral of dt/ds over s)
/// </summary>
public sealed class BrachistochroneProblemSolverAlternate : ICommand
{
    // Problem constants
    private const double Gravity = 9.80665; // Standard gravity m/s²
    private const double V0 = 0.5;          // Initial velocity (m/s) - small but non-zero

    // Reference line geometry from dynamics
    private static readonly double STotal = BrachistochroneAlternateDynamics.STotal;
    private static readonly double ThetaRef = BrachistochroneAlternateDynamics.ThetaRef;

    // State indices
    private const int IdxV = 0;
    private const int IdxN = 1;
    private const int IdxAlpha = 2;
    private const int IdxT = 3;

    // Control indices
    private const int IdxK = 0;

    public string Name => "brachistochrone";

    public string Description => "Curve of fastest descent under gravity (Johann Bernoulli, 1696)";

    public void Run(CommandOptions options)
    {
        Console.WriteLine("=== BRACHISTOCHRONE PROBLEM ===");
        Console.WriteLine("Finding the curve of fastest descent under gravity");
        Console.WriteLine("(Johann Bernoulli, 1696)");
        Console.WriteLine();
        Console.WriteLine("Reference-line-aligned coordinate system:");
        Console.WriteLine($"  Gravity: {Gravity} m/s²");
        Console.WriteLine($"  Reference line distance (STotal): {STotal:F2} m");
        Console.WriteLine($"  Reference angle (ThetaRef): {ThetaRef * 180.0 / Math.PI:F2} degrees");
        Console.WriteLine($"  Initial velocity: {V0} m/s");
        Console.WriteLine();

        var problem = CreateProblem();
        RunSolver(problem, options);
    }

    /// <summary>
    /// Creates the arc-length parameterized Brachistochrone problem.
    /// </summary>
    private static ControlProblem CreateProblem()
    {
        // Initial angle - start with a moderate descent angle relative to reference line
        var alpha0 = Math.PI / 6.0; // 30 degrees relative to reference line

        // Initial state: [v, n, alpha, t] - on reference line (n=0)
        var initialState = new double[] { V0, 0.0, alpha0, 0.0 };

        // Final state: [v=free, n=0, alpha=free, t=free] - back on reference line
        var finalState = new double[] { double.NaN, 0.0, double.NaN, double.NaN };

        return new ControlProblem()
            .WithStateSize(4) // [v, n, alpha, t]
            .WithControlSize(1) // [k] curvature
            .WithTimeHorizon(0.0, STotal) // s from 0 to STotal (line distance)
            .WithInitialCondition(initialState)
            .WithFinalCondition(finalState)
            .WithControlBounds([-2.0], [2.0]) // Curvature bounds (rad/m)
            .WithStateBounds(
                [0.01, -5.0, -Math.PI / 2.5, 0.0],  // Lower bounds: v>0, n can be negative (above line), alpha bounded, t>=0
                [30.0,  5.0,  Math.PI / 2.5, 10.0]) // Upper bounds: n can be positive (below line)
            .WithDynamics(ComputeDynamics)
            .WithRunningCost(ComputeRunningCost);
    }

    private static DynamicsResult ComputeDynamics(DynamicsInput input)
    {
        var x = input.State;
        var u = input.Control;

        var v = x[IdxV];
        var alpha = x[IdxAlpha];
        var k = u[IdxK];

        // Compute state derivatives using BrachistochroneAlternateDynamics
        var dVds = BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef);
        var dNds = BrachistochroneAlternateDynamics.VerticalRateS(alpha);
        var dAlphads = BrachistochroneAlternateDynamics.AlphaRateS(k);
        var dTds = BrachistochroneAlternateDynamics.TimeRateS(v, alpha);

        var value = new[] { dVds, dNds, dAlphads, dTds };

        // Compute gradients numerically
        var gradients = ComputeDynamicsGradientsNumerically(x, u);

        return new DynamicsResult(value, gradients);
    }

    private static double[][] ComputeDynamicsGradientsNumerically(double[] x, double[] u)
    {
        const double eps = 1e-7;
        const int stateDim = 4;
        const int controlDim = 1;

        var stateGradients = new double[stateDim * stateDim];
        var controlGradients = new double[stateDim * controlDim];

        // Compute base derivatives
        var f0 = ComputeDerivatives(x, u);

        // State gradients (df/dx)
        for (var j = 0; j < stateDim; j++)
        {
            var xPerturbed = (double[])x.Clone();
            xPerturbed[j] += eps;
            var fPerturbed = ComputeDerivatives(xPerturbed, u);

            for (var i = 0; i < stateDim; i++)
            {
                stateGradients[i * stateDim + j] = (fPerturbed[i] - f0[i]) / eps;
            }
        }

        // Control gradients (df/du)
        for (var j = 0; j < controlDim; j++)
        {
            var uPerturbed = (double[])u.Clone();
            uPerturbed[j] += eps;
            var fPerturbed = ComputeDerivatives(x, uPerturbed);

            for (var i = 0; i < stateDim; i++)
            {
                controlGradients[i * controlDim + j] = (fPerturbed[i] - f0[i]) / eps;
            }
        }

        return [stateGradients, controlGradients];
    }

    private static double[] ComputeDerivatives(double[] x, double[] u)
    {
        var v = x[IdxV];
        var alpha = x[IdxAlpha];
        var k = u[IdxK];

        return
        [
            BrachistochroneAlternateDynamics.SpeedRateS(v, alpha, Gravity, ThetaRef),
            BrachistochroneAlternateDynamics.VerticalRateS(alpha),
            BrachistochroneAlternateDynamics.AlphaRateS(k),
            BrachistochroneAlternateDynamics.TimeRateS(v, alpha)
        ];
    }

    private static RunningCostResult ComputeRunningCost(RunningCostInput input)
    {
        var x = input.State;

        var v = x[IdxV];
        var alpha = x[IdxAlpha];

        // Running cost = dt/ds (integrates to total time)
        var cost = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);

        // Compute gradients numerically
        var gradients = ComputeRunningCostGradientsNumerically(x);

        return new RunningCostResult(cost, gradients);
    }

    private static double[] ComputeRunningCostGradientsNumerically(double[] x)
    {
        const double eps = 1e-7;
        const int stateDim = 4;
        const int controlDim = 1;

        // Gradients: [dL/dx (4), dL/du (1), dL/ds (1)]
        var gradients = new double[stateDim + controlDim + 1];

        var v = x[IdxV];
        var alpha = x[IdxAlpha];

        var L0 = BrachistochroneAlternateDynamics.RunningCostS(v, alpha);

        // dL/dv
        var Lp = BrachistochroneAlternateDynamics.RunningCostS(v + eps, alpha);
        gradients[IdxV] = (Lp - L0) / eps;

        // dL/dalpha
        Lp = BrachistochroneAlternateDynamics.RunningCostS(v, alpha + eps);
        gradients[IdxAlpha] = (Lp - L0) / eps;

        // dL/dn = 0, dL/dt = 0, dL/dk = 0, dL/ds = 0

        return gradients;
    }

    private static ISolver CreateSolver(int segments, ProgressCallback? progressCallback = null)
    {
        var innerOptimizer = new LBFGSOptimizer(new LBFGSOptions
        {
            Tolerance = 1e-4,
            MaxIterations = 500
        }, new BacktrackingLineSearch());

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine($"  Segments: {segments}");
        Console.WriteLine("  Max iterations: 200");
        Console.WriteLine("  Inner optimizer: L-BFGS");
        Console.WriteLine("  Tolerance: 1e-2");
        Console.WriteLine();

        return new HermiteSimpsonSolver(
            new HermiteSimpsonSolverOptions
            {
                Segments = segments,
                Tolerance = 1e-2,
                MaxIterations = 200,
                Verbose = true,
                ProgressCallback = progressCallback
            },
            innerOptimizer);
    }

    private static void RunSolver(ControlProblem problem, CommandOptions options)
    {
        Console.WriteLine("Solving...");
        if (!options.Headless)
        {
            Console.WriteLine("Opening live visualization window...");
            Console.WriteLine("(Close window when done viewing)");
        }
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();

        var segments = 5;
        var initialGuess = CreateCustomInitialGuess(problem, segments);

        if (options.Headless)
        {
            var solver = CreateSolver(segments);
            var headlessResult = solver.Solve(problem, initialGuess);
            PrintSolutionSummary(headlessResult);
            return;
        }

        var solverWithCallback = CreateSolver(segments, CreateProgressCallback());
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var taskResult = solverWithCallback.Solve(problem, initialGuess);
                Console.WriteLine("[SOLVER] Optimization completed successfully");
                return taskResult;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("[SOLVER] Optimization cancelled");
                throw;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[SOLVER] Exception during solve: {ex.GetType().Name}: {ex.Message}");
                throw;
            }
        }, RadiantBrachistochroneVisualizer.CancellationToken);

        RadiantBrachistochroneVisualizer.RunVisualizationWindow();

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

        PrintSolutionSummary(result);
    }

    /// <summary>
    /// Creates a custom initial guess for Hermite-Simpson solver.
    /// Hermite-Simpson requires 2*segments + 1 points.
    /// Uses a straight-line path along the reference line (n=0 throughout),
    /// with velocity from energy conservation and time integrated using the dynamics.
    /// </summary>
    private static InitialGuess CreateCustomInitialGuess(ControlProblem problem, int segments)
    {
        var numPoints = 2 * segments + 1;
        var states = new double[numPoints][];
        var controls = new double[numPoints][];

        // Get problem bounds
        var s0 = problem.InitialTime;
        var sf = problem.FinalTime;
        var totalLineDistance = sf - s0;
        var ds = totalLineDistance / (numPoints - 1);

        // Initial state values
        var v0 = problem.InitialState![IdxV];
        var t0 = problem.InitialState[IdxT];

        // State bounds
        var vMin = problem.StateLowerBounds![IdxV];
        var vMax = problem.StateUpperBounds![IdxV];
        var nMin = problem.StateLowerBounds[IdxN];
        var nMax = problem.StateUpperBounds[IdxN];
        var alphaMin = problem.StateLowerBounds[IdxAlpha];
        var alphaMax = problem.StateUpperBounds[IdxAlpha];
        var tMin = problem.StateLowerBounds[IdxT];
        var tMax = problem.StateUpperBounds[IdxT];

        // Control bounds
        var kMin = problem.ControlLowerBounds![IdxK];
        var kMax = problem.ControlUpperBounds![IdxK];

        // For a straight line along the reference line: alpha = 0 (following the line)
        var straightLineAlpha = 0.0;
        straightLineAlpha = Math.Clamp(straightLineAlpha, alphaMin, alphaMax);

        // For a straight line, curvature k = 0 (no change in angle)
        var k = Math.Clamp(0.0, kMin, kMax);

        // Initialize state for integration
        var v = v0;
        var n = 0.0; // On the reference line
        var alpha = straightLineAlpha;
        var t = t0;

        for (var i = 0; i < numPoints; i++)
        {
            var s = s0 + i * ds;

            // Clamp states to bounds
            var vClamped = Math.Clamp(v, vMin, vMax);
            var nClamped = Math.Clamp(n, nMin, nMax);
            var alphaClamped = Math.Clamp(alpha, alphaMin, alphaMax);
            var tClamped = Math.Clamp(t, tMin, tMax);

            states[i] = [vClamped, nClamped, alphaClamped, tClamped];
            controls[i] = [k];

            // Integrate to next point using the dynamics
            if (i < numPoints - 1)
            {
                var sNext = s0 + (i + 1) * ds;

                // For straight line along reference: n stays 0
                // Actual vertical drop from Cartesian perspective = s * sin(ThetaRef)
                var actualVerticalDrop = sNext * Math.Sin(ThetaRef);

                // Use energy conservation for velocity: v² = v0² + 2*g*(vertical drop)
                var vNext = Math.Sqrt(v0 * v0 + 2.0 * Gravity * actualVerticalDrop);
                vNext = Math.Max(vNext, vMin);

                // Integrate time using trapezoidal rule: dt/ds = 1/(v*cos(alpha))
                var dtdsCurrent = BrachistochroneAlternateDynamics.TimeRateS(vClamped, alphaClamped);
                var dtdsNext = BrachistochroneAlternateDynamics.TimeRateS(vNext, straightLineAlpha);
                var tNext = t + ds * (dtdsCurrent + dtdsNext) / 2.0;

                // Alpha stays constant for a straight line
                v = vNext;
                t = tNext;
            }
        }

        return new InitialGuess(states, controls);
    }

    private static ProgressCallback CreateProgressCallback()
    {
        return (iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
        {
            var token = RadiantBrachistochroneVisualizer.CancellationToken;
            if (token.IsCancellationRequested)
            {
                Console.WriteLine($"[SOLVER] Iteration {iteration}: Cancellation requested, stopping optimization...");
                throw new OperationCanceledException(token);
            }

            RadiantBrachistochroneVisualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
        };
    }

    private static void PrintSolutionSummary(CollocationResult result)
    {
        Console.WriteLine();
        Console.WriteLine("=".PadRight(70, '='));
        Console.WriteLine();
        Console.WriteLine("SOLUTION SUMMARY:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Final time (t): {result.States[^1][IdxT]:F6} seconds");
        Console.WriteLine($"  Objective value: {result.OptimalCost:F6}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine($"  Max defect: {result.MaxDefect:E3}");
        Console.WriteLine();

        // State summary
        Console.WriteLine("FINAL STATE:");
        Console.WriteLine($"  Speed (v): {result.States[^1][IdxV]:F3} m/s");
        Console.WriteLine($"  Perpendicular offset (n): {result.States[^1][IdxN]:F3} m");
        Console.WriteLine($"  Heading angle (alpha, rel to ref line): {result.States[^1][IdxAlpha] * 180.0 / Math.PI:F2} degrees");
        Console.WriteLine($"  Elapsed time (t): {result.States[^1][IdxT]:F4} s");
        Console.WriteLine();

        // Energy conservation check
        // In the rotated coordinate system, actual vertical drop from (s, n) is:
        // y_down = s * sin(ThetaRef) + n * cos(ThetaRef)
        var v0 = result.States[0][IdxV];
        var s0 = result.Times[0];
        var n0 = result.States[0][IdxN];
        var vf = result.States[^1][IdxV];
        var sf = result.Times[^1];
        var nf = result.States[^1][IdxN];

        var yDown0 = s0 * Math.Sin(ThetaRef) + n0 * Math.Cos(ThetaRef);
        var yDownF = sf * Math.Sin(ThetaRef) + nf * Math.Cos(ThetaRef);
        var actualVerticalDrop = yDownF - yDown0;
        var expectedVf = Math.Sqrt(v0 * v0 + 2 * Gravity * actualVerticalDrop);

        Console.WriteLine("ENERGY CONSERVATION CHECK:");
        Console.WriteLine($"  Initial (s={s0:F2}m, n={n0:F3}m), Final (s={sf:F2}m, n={nf:F3}m)");
        Console.WriteLine($"  Actual vertical drop (Cartesian): {actualVerticalDrop:F3} m");
        Console.WriteLine($"  Expected final velocity (energy): {expectedVf:F3} m/s");
        Console.WriteLine($"  Actual final velocity: {vf:F3} m/s");
        Console.WriteLine($"  Velocity error: {Math.Abs(vf - expectedVf):F3} m/s ({100.0 * Math.Abs(vf - expectedVf) / expectedVf:F1}%)");
        Console.WriteLine();

        // Print trajectory sample
        Console.WriteLine("TRAJECTORY (sample):");
        Console.WriteLine("  s(m)     v(m/s)   n(m)     alpha(deg)  t(s)     k(rad/m)");
        var step = Math.Max(1, result.States.Length / 10);
        for (var i = 0; i < result.States.Length; i += step)
        {
            var state = result.States[i];
            var control = result.Controls[i];
            var s = result.Times[i];
            Console.WriteLine($"  {s,6:F2}   {state[IdxV],6:F2}   {state[IdxN],6:F2}   {state[IdxAlpha] * 180.0 / Math.PI,8:F2}    {state[IdxT],6:F3}   {control[IdxK],8:F4}");
        }
        Console.WriteLine();
    }
}
