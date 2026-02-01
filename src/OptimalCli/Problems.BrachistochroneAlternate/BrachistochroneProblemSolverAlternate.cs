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
using Optimal.NonLinear.Constrained;
using Optimal.NonLinear.LineSearch;
using Optimal.NonLinear.Monitoring;
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
///   - n: Perpendicular distance from reference line (m, positive = below line)
///   - alpha: Heading angle relative to reference line (rad)
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
    private const double V0 = 1.0;          // Initial velocity (m/s) - above clamping threshold

    // Reference line geometry from dynamics
    private static readonly double STotal = BrachistochroneDynamicsAlternate.STotal;
    private static readonly double ThetaRef = BrachistochroneDynamicsAlternate.ThetaRef;

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
        // Initial angle - start along the reference line direction
        // This creates a feasible trajectory that stays near the reference line

        // Initial state: [v, n, alpha, t] - on reference line (n=0)
        var initialState = new double[] { V0, 0.0, double.NaN, 0.0 };

        // Final state: [v=free, n=0, alpha=free, t=free] - back on reference line
        var finalState = new double[] { double.NaN, 0.0, double.NaN, double.NaN };

        return new ControlProblem()
            .WithStateSize(4) // [v, n, alpha, t]
            .WithControlSize(1) // [k] curvature
            .WithTimeHorizon(0.0, STotal) // s from 0 to STotal (line distance)
            .WithInitialCondition(initialState)
            .WithFinalCondition(finalState)
            .WithControlBounds([-1.0], [1.0]) // Tighter curvature bounds (rad/m)
            .WithStateBounds(
                [0.5, -3.0, -Math.PI / 3.0, 0.0],   // Tighter lower bounds
                [15.0, 3.0, Math.PI / 3.0, 5.0])    // Tighter upper bounds
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

        // Compute state derivatives and gradients using analytical AutoDiff
        var (dVds, dVdsGrad) = BrachistochroneDynamicsAlternateGradients.SpeedRateSReverse(v, alpha, Gravity, ThetaRef);
        var (dNds, dNdsGrad) = BrachistochroneDynamicsAlternateGradients.VerticalRateSReverse(alpha);
        var (dAlphads, dAlphadsGrad) = BrachistochroneDynamicsAlternateGradients.AlphaRateSReverse(k);
        var (dTds, dTdsGrad) = BrachistochroneDynamicsAlternateGradients.TimeRateSReverse(v, alpha);

        var value = new[] { dVds, dNds, dAlphads, dTds };

        // Build gradient arrays
        // gradients[0] = df/dx (4x4 flattened row-major)
        // gradients[1] = df/du (4x1)
        var stateGradients = new double[16];
        var controlGradients = new double[4];

        // SpeedRateS(v, alpha, g, thetaRef) -> grad order: [v, alpha, g, thetaRef]
        stateGradients[0 * 4 + IdxV] = dVdsGrad[0];      // d(dVds)/dv
        stateGradients[0 * 4 + IdxAlpha] = dVdsGrad[1];  // d(dVds)/dalpha

        // VerticalRateS(alpha) -> grad order: [alpha]
        stateGradients[1 * 4 + IdxAlpha] = dNdsGrad[0];  // d(dNds)/dalpha

        // AlphaRateS(k) -> grad order: [k]
        controlGradients[IdxAlpha] = dAlphadsGrad[0];    // d(dAlphads)/dk

        // TimeRateS(v, alpha) -> grad order: [v, alpha]
        stateGradients[3 * 4 + IdxV] = dTdsGrad[0];      // d(dTds)/dv
        stateGradients[3 * 4 + IdxAlpha] = dTdsGrad[1];  // d(dTds)/dalpha

        return new DynamicsResult(value, [stateGradients, controlGradients]);
    }

    private static RunningCostResult ComputeRunningCost(RunningCostInput input)
    {
        var x = input.State;
        var v = x[IdxV];
        var alpha = x[IdxAlpha];

        // Running cost = dt/ds with analytical gradients
        var (cost, costGrad) = BrachistochroneDynamicsAlternateGradients.RunningCostSReverse(v, alpha);

        // Gradients: [dL/dx (4), dL/du (1), dL/ds (1)] = 6 elements
        var gradients = new double[6];
        gradients[IdxV] = costGrad[0];     // dL/dv
        gradients[IdxAlpha] = costGrad[1]; // dL/dalpha
        // Other gradients remain 0

        return new RunningCostResult(cost, gradients);
    }

    private static HermiteSimpsonSolver CreateSolver(
        int segments,
        ProgressCallback? progressCallback = null,
        Action<int, double, double, double, int, double[]>? innerProgressCallback = null,
        OptimisationMonitor? monitor = null)
    {
        var innerOptimizer = false ? (IOptimizer)
            new LBFGSBOptimizer(new LBFGSBOptions
            {
                MemorySize = 10,
                Tolerance = 1e-3,
                MaxIterations = 1000
            }, monitor) : new LBFGSOptimizer(new LBFGSOptions
            {
                MemorySize = 10,
                Preconditioning = new LBFGSPreconditioningOptions
                {
                    EnableAutomaticPreconditioning = true,
                    PreconditioningThreshold = 1e3,
                    Type = PreconditioningType.Regularization
                },
                Tolerance = 1e-4,
                MaxIterations = 1000
            }, new BacktrackingLineSearch(), monitor);

        Console.WriteLine("Solver configuration:");
        Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
        Console.WriteLine($"  Segments: {segments}");
        Console.WriteLine("  Max iterations: 1000");
        Console.WriteLine("  Inner optimizer: L-BFGS");
        Console.WriteLine("  Tolerance: 1e-2");
        Console.WriteLine();

        return new HermiteSimpsonSolver(
            new HermiteSimpsonSolverOptions
            {
                Segments = segments,
                Tolerance = 1e-3,
                MaxIterations = 1000,
                Verbose = true,
                ProgressCallback = progressCallback,
                AutoScaling = true,
                EnableMeshRefinement = true,
                MaxRefinementIterations = 5,
                RefinementDefectThreshold = 1e-4,
                InnerProgressCallback = innerProgressCallback
            },
            innerOptimizer,
            monitor);
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

        // Create optimization monitor for gradient verification, smoothness, and conditioning monitoring
        var useMonitor = false;
        var monitor = new OptimisationMonitor()
            .WithGradientVerification(testStep: 1e-6)
            .WithSmoothnessMonitoring()
            .WithConditioningMonitoring(threshold: 1e4);

        // Create cancellation token source for stopping optimization when visualization window closes
        using var cancellationTokenSource = new CancellationTokenSource();
        var cancellationToken = cancellationTokenSource.Token;

        if (options.Headless)
        {
            var solver = CreateSolver(segments, monitor: useMonitor ? monitor : null);
            var headlessResult = solver.Solve(problem, initialGuess, cancellationToken);
            PrintSolutionSummary(headlessResult);
            PrintMonitorReport(monitor);
            return;
        }

        var solverWithCallback = CreateSolver(segments, CreateProgressCallback(), monitor: useMonitor ? monitor : null);
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var taskResult = solverWithCallback.Solve(problem, initialGuess, cancellationToken);
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
        }, cancellationToken);

        RadiantBrachistochroneVisualizerAlternative.RunVisualizationWindow(cancellationTokenSource);

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
            PrintMonitorReport(monitor);
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
            return;
        }

        PrintSolutionSummary(result);
        PrintMonitorReport(monitor);
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
                var dtdsCurrent = BrachistochroneDynamicsAlternate.TimeRateS(vClamped, alphaClamped);
                var dtdsNext = BrachistochroneDynamicsAlternate.TimeRateS(vNext, straightLineAlpha);
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
            RadiantBrachistochroneVisualizerAlternative.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
        };
    }

    private static void PrintMonitorReport(OptimisationMonitor monitor)
    {
        var report = monitor.GenerateReport();

        Console.WriteLine();
        Console.WriteLine("-".PadRight(70, '-'));
        Console.WriteLine("OPTIMIZATION MONITOR REPORT:");
        Console.WriteLine($"  {report.Summary}");

        if (report.BadGradientSuspected)
        {
            Console.WriteLine();
            Console.WriteLine("  WARNING: Bad gradient detected!");
            Console.WriteLine($"    Function index: {report.BadGradientFunctionIndex} (-1 = objective)");
            Console.WriteLine($"    Variable index: {report.BadGradientVariableIndex}");
            if (report.ObjectiveGradientResult != null)
            {
                Console.WriteLine($"    Max relative error: {report.ObjectiveGradientResult.MaxRelativeError:E3}");
            }
        }

        if (report.NonC0Suspected)
        {
            Console.WriteLine();
            Console.WriteLine("  WARNING: C0 discontinuity suspected!");
            Console.WriteLine($"    Violations detected: {report.SmoothnessResult?.C0Violations.Count ?? 0}");
        }

        if (report.NonC1Suspected)
        {
            Console.WriteLine();
            Console.WriteLine("  WARNING: C1 non-smoothness suspected!");
            Console.WriteLine($"    Violations detected: {report.SmoothnessResult?.C1Violations.Count ?? 0}");
        }

        if (report.IllConditioningSuspected)
        {
            Console.WriteLine();
            Console.WriteLine("  WARNING: Ill-conditioning suspected!");
            if (report.ConditioningResult != null)
            {
                Console.WriteLine($"    Estimated condition number: {report.ConditioningResult.EstimatedConditionNumber:E2}");
                Console.WriteLine($"    Severity: {report.ConditioningResult.Severity}");
                Console.WriteLine($"    Correction pairs analyzed: {report.ConditioningResult.CorrectionPairsAnalyzed}");
                if (report.ConditioningResult.ProblematicVariableIndices.Count > 0)
                {
                    Console.WriteLine($"    Problematic variables: {string.Join(", ", report.ConditioningResult.ProblematicVariableIndices)}");
                }
            }
        }
        else if (report.ConditioningResult != null && report.ConditioningResult.CorrectionPairsAnalyzed > 0)
        {
            Console.WriteLine();
            Console.WriteLine("  Conditioning analysis:");
            Console.WriteLine($"    Estimated condition number: {report.ConditioningResult.EstimatedConditionNumber:E2}");
            Console.WriteLine($"    Severity: {report.ConditioningResult.Severity}");
        }

        Console.WriteLine();
        Console.WriteLine($"  Monitor function evaluations: {report.MonitorFunctionEvaluations}");
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
