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
/// Solves the minimum lap time problem for a racecar using the dymos model.
///
/// This is an arc-length parameterized optimal control problem where:
/// - Independent variable: s (arc length along track centerline)
/// - Objective: Minimize total elapsed time
///
/// State: [V, ax, ay, n, alpha, lambda, Omega, t]
/// Control: [delta, T]
/// </summary>
public sealed class CornerProblemSolver : ICommand
{
    // Vehicle parameters (from dymos racecar model)
    private const double M = 800.0;        // Vehicle mass (kg)
    private const double VehicleA = 1.404; // CoG to front axle (m)
    private const double VehicleB = 1.356; // CoG to rear axle (m)
    private const double L = VehicleA + VehicleB; // Wheelbase (m)
    private const double Tw = 0.807;       // Half track width (m)
    private const double H = 0.35;         // CoG height (m)
    private const double Iz = 1775.0;      // Yaw inertia (kg*m^2)
    private const double Chi = 0.5;        // Roll stiffness distribution
    private const double Rho = 1.2;        // Air density (kg/m^3)
    private const double Mu0 = 1.68;       // Base friction coefficient
    private const double Kmu = -0.5;       // Tire load sensitivity
    private const double TauAx = 0.2;      // Longitudinal load transfer time constant (s)
    private const double TauAy = 0.2;      // Lateral load transfer time constant (s)
    private const double TauLambda = 0.1;  // Slip angle relaxation time constant (s)
    private const double ClA = 4.0;        // Downforce coefficient * area (m^2)
    private const double CdA = 2.0;        // Drag coefficient * area (m^2)
    private const double CoP = 1.6;        // Center of pressure location (m)
    private const double Gravity = 9.81;   // Gravity (m/s^2)
    private const double Pmax = 960000.0;  // Max power (W)
    private const double Fmax = 12000.0;   // Max thrust force (N)

    // State indices
    private const int IdxV = 0;
    private const int IdxAx = 1;
    private const int IdxAy = 2;
    private const int IdxN = 3;
    private const int IdxAlpha = 4;
    private const int IdxLambda = 5;
    private const int IdxOmega = 6;
    private const int IdxT = 7;

    // Control indices
    private const int IdxDelta = 0;
    private const int IdxThrust = 1;

    public string Name => "corner";

    public string Description => "Optimal racing line";

    public void Run(CommandOptions options)
    {
        var trackGeometry = TrackGeometry
            .StartAt(x: 0, y: 0, heading: 0)
            .AddLine(distance: 10.0)
            .AddArc(radius: 10.0, angle: Math.PI / 1.5, turnRight: true)
            .AddLine(distance: 10.0)
            .AddArc(radius: 10.0, angle: Math.PI / 1.5, turnRight: false)
            .AddLine(distance: 10.0)
            .Build();

        var visualizer = new RadiantCornerVisualizer(trackGeometry);
        var problem = CreateProblem(trackGeometry);
        var initialGuess = CreateInitialGuess(trackGeometry);
        var optimizationTask = Task.Run(() =>
        {
            try
            {
                var solver = CreateSolver(visualizer);
                var result = solver.Solve(problem, initialGuess);
                Console.WriteLine("[SOLVER] Optimization completed successfully");
                return result;
            }
            catch (OperationCanceledException)
            {
                Console.WriteLine("[SOLVER] Optimization cancelled");
                throw;
            }
        }, visualizer.CancellationToken);

        if (!options.Headless)
        {
            // Run the visualization window on the main thread
            visualizer.RunVisualizationWindow();
        }

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

        PrintResults(result);
    }

    private static HermiteSimpsonSolver CreateSolver(RadiantCornerVisualizer visualizer)
    {
        var innerOptimizer = new LBFGSOptimizer()
            .WithTolerance(1e-5)
            .WithMaxIterations(500)
            .WithVerbose(false);

        return new HermiteSimpsonSolver()
            .WithSegments(30)
            .WithTolerance(1e-4)
            .WithMaxIterations(300)
            .WithMeshRefinement(true, 5, 1e-4)
            .WithVerbose(true)
            .WithInnerOptimizer(innerOptimizer)
            .WithInitialPenalty(50.0)
            .WithProgressCallback((iteration, cost, states, controls, _, maxViolation, constraintTolerance) =>
            {
                visualizer.CancellationToken.ThrowIfCancellationRequested();
                visualizer.UpdateTrajectory(states, controls, iteration, cost, maxViolation, constraintTolerance);
            });
    }

    private ControlProblem CreateProblem(TrackGeometry trackGeometry)
    {
        var totalLength = trackGeometry.TotalLength;
        var roadHalfWidth = TrackGeometry.RoadHalfWidth;

        // Initial conditions: starting at rest on centerline, aligned with road
        var initialState = new double[]
        {
            20.0,  // V: initial speed (m/s)
            0.0,   // ax: zero longitudinal acceleration
            0.0,   // ay: zero lateral acceleration
            0.0,   // n: on centerline
            0.0,   // alpha: aligned with road
            0.0,   // lambda: zero slip angle
            0.0,   // Omega: zero yaw rate
            0.0    // t: zero elapsed time
        };

        // Final conditions: all free (no terminal constraints)
        var finalState = new double[]
        {
            double.NaN, double.NaN, double.NaN, 0.0,
            0.0, double.NaN, double.NaN, double.NaN
        };

        return new ControlProblem()
            .WithStateSize(8)
            .WithControlSize(2)
            .WithTimeHorizon(0.0, totalLength)  // s from 0 to track length
            .WithInitialCondition(initialState)
            .WithFinalCondition(finalState)
            .WithControlBounds([-0.5, -1.0], [0.5, 1.0])  // steering +/- 0.5 rad, thrust -1 to 1
            .WithStateBounds(
                [1.0, -15.0, -15.0, -roadHalfWidth, -Math.PI / 3, -0.3, -2.0, 0.0],
                [70.0, 15.0, 15.0, roadHalfWidth, Math.PI / 3, 0.3, 2.0, 60.0])
            .WithDynamics(input => ComputeDynamics(input, trackGeometry))
            .WithRunningCost(input => ComputeRunningCost(input, trackGeometry))
            .WithPathConstraint((x, u, _) => ComputePowerConstraint(x, u))
            .WithPathConstraint((x, _, _2) => ComputeCombinedFrictionConstraint(x));
    }

    private static DynamicsResult ComputeDynamics(DynamicsInput input, TrackGeometry trackGeometry)
    {
        var x = input.State;
        var u = input.Control;
        var s = input.Time;  // "time" is actually arc length s

        // Extract state variables
        var V = x[IdxV];
        var ax = x[IdxAx];
        var ay = x[IdxAy];
        var n = x[IdxN];
        var alpha = x[IdxAlpha];
        var lambda = x[IdxLambda];
        var Omega = x[IdxOmega];

        // Extract control variables
        var delta = u[IdxDelta];
        var T = u[IdxThrust];

        // Get track curvature at current position
        var kappa = trackGeometry.RoadCurvature(s);

        // Compute state derivatives
        var dVds = CornerDynamics.SpeedRateS(kappa, n, alpha, V, ax, Rho, CdA, M);
        var dAxds = CornerDynamics.AxRateS(kappa, n, alpha, V, ax, T, Fmax, M, TauAx);
        var dAyds = CornerDynamics.AyRateS(kappa, n, alpha, V, ay, Omega, TauAy);
        var dNds = CornerDynamics.LateralRateS(kappa, n, alpha);
        var dAlphads = CornerDynamics.AlphaRateS(kappa, n, alpha, V, Omega);
        var dLambdads = CornerDynamics.LambdaRateS(kappa, n, alpha, V, lambda, Omega, delta, VehicleA, VehicleB, TauLambda);
        var dOmegads = CornerDynamics.OmegaRateS(kappa, n, alpha, V, Omega, delta, ay, VehicleA, VehicleB, Iz, M);
        var dTds = CornerDynamics.TimeRateS(kappa, n, alpha, V);

        var value = new[] { dVds, dAxds, dAyds, dNds, dAlphads, dLambdads, dOmegads, dTds };

        // Compute gradients numerically for now (AutoDiff will generate these)
        // Gradient layout: [0] = df/dx (8x8 flattened = 64 elements), [1] = df/du (8x2 flattened = 16 elements)
        var gradients = ComputeDynamicsGradientsNumerically(x, u, kappa, trackGeometry, s);

        return new DynamicsResult(value, gradients);
    }

    private static double[][] ComputeDynamicsGradientsNumerically(double[] x, double[] u, double kappa,
        TrackGeometry _, double _2)
    {
        const double eps = 1e-7;
        const int stateDim = 8;
        const int controlDim = 2;

        var stateGradients = new double[stateDim * stateDim];
        var controlGradients = new double[stateDim * controlDim];

        // Compute base derivatives
        var f0 = ComputeDerivatives(x, u, kappa);

        // State gradients (df/dx)
        for (var j = 0; j < stateDim; j++)
        {
            var xPerturbed = (double[])x.Clone();
            xPerturbed[j] += eps;
            var fPerturbed = ComputeDerivatives(xPerturbed, u, kappa);

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
            var fPerturbed = ComputeDerivatives(x, uPerturbed, kappa);

            for (var i = 0; i < stateDim; i++)
            {
                controlGradients[i * controlDim + j] = (fPerturbed[i] - f0[i]) / eps;
            }
        }

        return [stateGradients, controlGradients];
    }

    private static double[] ComputeDerivatives(double[] x, double[] u, double kappa)
    {
        var V = x[IdxV];
        var ax = x[IdxAx];
        var ay = x[IdxAy];
        var n = x[IdxN];
        var alpha = x[IdxAlpha];
        var lambda = x[IdxLambda];
        var Omega = x[IdxOmega];

        var delta = u[IdxDelta];
        var T = u[IdxThrust];

        return
        [
            CornerDynamics.SpeedRateS(kappa, n, alpha, V, ax, Rho, CdA, M),
            CornerDynamics.AxRateS(kappa, n, alpha, V, ax, T, Fmax, M, TauAx),
            CornerDynamics.AyRateS(kappa, n, alpha, V, ay, Omega, TauAy),
            CornerDynamics.LateralRateS(kappa, n, alpha),
            CornerDynamics.AlphaRateS(kappa, n, alpha, V, Omega),
            CornerDynamics.LambdaRateS(kappa, n, alpha, V, lambda, Omega, delta, VehicleA, VehicleB, TauLambda),
            CornerDynamics.OmegaRateS(kappa, n, alpha, V, Omega, delta, ay, VehicleA, VehicleB, Iz, M),
            CornerDynamics.TimeRateS(kappa, n, alpha, V)
        ];
    }

    private static RunningCostResult ComputeRunningCost(RunningCostInput input, TrackGeometry trackGeometry)
    {
        var x = input.State;
        var s = input.Time;

        var V = x[IdxV];
        var n = x[IdxN];
        var alpha = x[IdxAlpha];

        var kappa = trackGeometry.RoadCurvature(s);

        // Running cost = dt/ds (integrates to total time)
        var cost = CornerDynamics.RunningCostS(kappa, n, alpha, V);

        // Compute gradients numerically
        var gradients = ComputeRunningCostGradientsNumerically(x, kappa);

        return new RunningCostResult(cost, gradients);
    }

    private static double[] ComputeRunningCostGradientsNumerically(double[] x, double kappa)
    {
        const double eps = 1e-7;
        const int stateDim = 8;
        const int controlDim = 2;

        // Gradients: [dL/dx (8), dL/du (2), dL/dt (1)]
        var gradients = new double[stateDim + controlDim + 1];

        var V = x[IdxV];
        var n = x[IdxN];
        var alpha = x[IdxAlpha];

        var L0 = CornerDynamics.RunningCostS(kappa, n, alpha, V);

        // dL/dV
        var Lp = CornerDynamics.RunningCostS(kappa, n, alpha, V + eps);
        gradients[IdxV] = (Lp - L0) / eps;

        // dL/dn
        Lp = CornerDynamics.RunningCostS(kappa, n + eps, alpha, V);
        gradients[IdxN] = (Lp - L0) / eps;

        // dL/dalpha
        Lp = CornerDynamics.RunningCostS(kappa, n, alpha + eps, V);
        gradients[IdxAlpha] = (Lp - L0) / eps;

        // Other state derivatives are zero (ax, ay, lambda, Omega, t don't appear in running cost)
        // Control derivatives are zero
        // Time derivative is zero

        return gradients;
    }

    private static (double value, double[] gradients) ComputePowerConstraint(double[] x, double[] u)
    {
        var V = x[IdxV];
        var T = u[IdxThrust];

        // Power constraint: T * Fmax * V - Pmax <= 0
        var value = CornerDynamics.PowerConstraint(T, V, Fmax, Pmax);

        // Gradients: [dx (8), du (2)]
        var gradients = new double[10];
        gradients[IdxV] = T * Fmax;      // dC/dV
        gradients[8 + IdxThrust] = Fmax * V;  // dC/dT

        return (value, gradients);
    }

    private static (double value, double[] gradients) ComputeCombinedFrictionConstraint(double[] x)
    {
        var ax = x[IdxAx];
        var ay = x[IdxAy];

        // Simplified combined friction circle constraint
        const double axMax = 12.0;
        const double ayMax = 12.0;

        // (ax/axMax)^2 + (ay/ayMax)^2 - 1 <= 0
        var value = (ax * ax) / (axMax * axMax) + (ay * ay) / (ayMax * ayMax) - 1.0;

        // Gradients: [dx (8), du (2)]
        var gradients = new double[10];
        gradients[IdxAx] = 2.0 * ax / (axMax * axMax);  // dC/dax
        gradients[IdxAy] = 2.0 * ay / (ayMax * ayMax);  // dC/day

        return (value, gradients);
    }

    private static InitialGuess CreateInitialGuess(TrackGeometry trackGeometry)
    {
        const int numPoints = 31;
        var totalLength = trackGeometry.TotalLength;
        const double vInit = 20.0;

        var states = new double[numPoints][];
        var controls = new double[numPoints][];

        for (var i = 0; i < numPoints; i++)
        {
            var s = (double)i / (numPoints - 1) * totalLength;
            var kappa = trackGeometry.RoadCurvature(s);

            // Heuristic: offset toward inside of corner
            var n = -Math.Sign(kappa) * Math.Min(Math.Abs(kappa) * vInit * vInit / 15.0,
                TrackGeometry.RoadHalfWidth * 0.7);

            // Estimated lateral acceleration for cornering
            var ayEst = vInit * vInit * kappa;
            ayEst = Math.Clamp(ayEst, -10.0, 10.0);

            // Estimated yaw rate
            var OmegaEst = vInit * kappa;
            OmegaEst = Math.Clamp(OmegaEst, -1.5, 1.5);

            // State: [V, ax, ay, n, alpha, lambda, Omega, t]
            states[i] =
            [
                vInit,     // V
                0.0,       // ax
                ayEst,     // ay
                n,         // n
                0.0,       // alpha (aligned with road)
                0.0,       // lambda
                OmegaEst,  // Omega
                s / vInit  // t (rough time estimate)
            ];

            // Control: [delta, T]
            // Steer to approximately match curvature using bicycle model
            var deltaEst = Math.Atan(kappa * L);
            deltaEst = Math.Clamp(deltaEst, -0.4, 0.4);

            controls[i] = [deltaEst, 0.2];  // moderate thrust
        }

        return new InitialGuess(states, controls);
    }

    private static void PrintResults(CollocationResult result)
    {
        Console.WriteLine("\n" + "=".PadRight(70, '=') + "\n");
        Console.WriteLine("OPTIMIZATION RESULTS:");
        Console.WriteLine($"  Success: {result.Success}");
        Console.WriteLine($"  Message: {result.Message}");
        Console.WriteLine($"  Iterations: {result.Iterations}");
        Console.WriteLine($"  Final cost (total time): {result.OptimalCost:F3} s");
        Console.WriteLine($"  Max defect: {result.MaxDefect:E3}");

        if (result.States.Length > 0)
        {
            var finalState = result.States[^1];
            Console.WriteLine($"\n  Final state:");
            Console.WriteLine($"    Speed V: {finalState[IdxV]:F2} m/s");
            Console.WriteLine($"    Longitudinal accel ax: {finalState[IdxAx]:F2} m/s^2");
            Console.WriteLine($"    Lateral accel ay: {finalState[IdxAy]:F2} m/s^2");
            Console.WriteLine($"    Lateral offset n: {finalState[IdxN]:F2} m");
            Console.WriteLine($"    Heading alpha: {finalState[IdxAlpha] * 180 / Math.PI:F2} deg");
            Console.WriteLine($"    Elapsed time t: {finalState[IdxT]:F3} s");
        }

        Console.WriteLine();
    }
}
