/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using System;
using System.Diagnostics;
using Optimal.Control;
using Optimal.NonLinear;

// BRACHISTOCHRONE PROBLEM (Johann Bernoulli, 1696)
// Question: What curve gives the fastest descent under gravity between two points?
// Answer: A cycloid - the curve traced by a point on a rolling circle
//
// Formulation:
//   States: [x, y, v] - position (x,y) and speed v
//   Control: θ - path angle from horizontal
//   Dynamics: ẋ = v·cos(θ), ẏ = v·sin(θ), v̇ = -g·sin(θ)
//   Objective: Minimize time T = ∫ 1 dt
//
// Physics: Conservation of energy gives v = √(2g·h) where h is vertical drop
// Solution: Numerical solution should approximate a cycloid curve

Console.WriteLine("=== BRACHISTOCHRONE PROBLEM ===");
Console.WriteLine("Finding the curve of fastest descent under gravity");
Console.WriteLine();

var g = 9.81;  // gravity (m/s²)
var xFinal = 2.0;
var yFinal = -2.0;  // negative = downward

Console.WriteLine($"Problem setup:");
Console.WriteLine($"  Start: (0, 0) at rest (v=0.1 m/s initial to avoid singularity)");
Console.WriteLine($"  End:   ({xFinal}, {yFinal}) with free final velocity");
Console.WriteLine($"  Gravity: {g} m/s²");
Console.WriteLine();

var problem = new ControlProblem()
    .WithStateSize(3) // [x, y, v]
    .WithControlSize(1) // θ (path angle)
    .WithTimeHorizon(0.0, 1.5)  // Allow enough time
    .WithInitialCondition(new[] { 0.0, 0.0, 0.1 }) // Start at origin with small initial velocity
    .WithFinalCondition(new[] { xFinal, yFinal, double.NaN }) // End position, free velocity
    .WithControlBounds(new[] { -Math.PI / 2.0 }, new[] { 0.0 }) // Downward paths only
    .WithDynamics((x, u, t) =>
    {
        var velocity = x[2];
        var theta = u[0];
        var cosTheta = Math.Cos(theta);
        var sinTheta = Math.Sin(theta);

        // State derivatives
        var xdot = velocity * cosTheta;
        var ydot = velocity * sinTheta;
        var vdot = -g * sinTheta;  // Negative because gravity accelerates downward (θ < 0)

        var value = new[] { xdot, ydot, vdot };
        var gradients = new double[2][];

        // ∂f/∂x (state jacobian): [∂ẋ/∂x, ∂ẋ/∂y, ∂ẋ/∂v; ∂ẏ/∂x, ∂ẏ/∂y, ∂ẏ/∂v; ∂v̇/∂x, ∂v̇/∂y, ∂v̇/∂v]
        gradients[0] = new[] {
            0.0, 0.0, cosTheta,        // ∂ẋ/∂[x,y,v]
            0.0, 0.0, sinTheta,        // ∂ẏ/∂[x,y,v]
            0.0, 0.0, 0.0              // ∂v̇/∂[x,y,v]
        };

        // ∂f/∂u (control jacobian): [∂ẋ/∂θ, ∂ẏ/∂θ, ∂v̇/∂θ]
        gradients[1] = new[] {
            -velocity * sinTheta,      // ∂ẋ/∂θ
            velocity * cosTheta,       // ∂ẏ/∂θ
            -g * cosTheta              // ∂v̇/∂θ (derivative of -g·sin(θ))
        };

        return (value, gradients);
    })
    .WithRunningCost((x, u, t) =>
    {
        // Minimize time: L = 1
        var value = 1.0;
        var gradients = new double[3];
        gradients[0] = 0.0;  // ∂L/∂x (state components)
        gradients[1] = 0.0;  // ∂L/∂u
        gradients[2] = 0.0;  // ∂L/∂t
        return (value, gradients);
    });

Console.WriteLine("Solver configuration:");
Console.WriteLine("  Algorithm: Hermite-Simpson direct collocation");
Console.WriteLine("  Segments: 20 (with adaptive mesh refinement)");
Console.WriteLine("  Max iterations: 500");
Console.WriteLine("  Inner optimizer: L-BFGS-B");
Console.WriteLine("  Tolerance: 1e-3");
Console.WriteLine();
Console.WriteLine("Solving... (progress will be shown below)");
Console.WriteLine("=".PadRight(70, '='));

var solver = new HermiteSimpsonSolver()
    .WithSegments(20)  // Start with fewer segments
    .WithTolerance(1e-4)
    .WithMaxIterations(200)  // Increased significantly
    .WithMeshRefinement(enable: true, maxRefinementIterations: 3, defectThreshold: 1e-4)
    .WithVerbose(true)  // Enable progress output
    .WithInnerOptimizer(
        new LBFGSOptimizer()
            .WithTolerance(1e-6)
            .WithMaxIterations(200)
            .WithVerbose(true));  // Enable inner optimizer progress

var sw = Stopwatch.StartNew();
var result = solver.Solve(problem);
sw.Stop();

Console.WriteLine("=".PadRight(70, '='));
Console.WriteLine();
Console.WriteLine($"Solve completed in {sw.ElapsedMilliseconds}ms");
Console.WriteLine();

// Verify physics: v should equal √(2g|y|) from energy conservation
var physicsError = VerifyEnergyConservation(result, g);

// Display solution summary
Console.WriteLine("SOLUTION SUMMARY:");
Console.WriteLine($"  Success: {result.Success}");
Console.WriteLine($"  Message: {result.Message}");
Console.WriteLine($"  Optimal descent time: {result.OptimalCost:F3} seconds");
Console.WriteLine($"  Final velocity: {result.States[^1][2]:F3} m/s");
Console.WriteLine($"  Expected from energy: {Math.Sqrt(2 * g * Math.Abs(yFinal)):F3} m/s");
Console.WriteLine($"  Physics error: {physicsError * 100:F1}%");
Console.WriteLine($"  Max constraint violation: {result.MaxDefect:E3}");
Console.WriteLine($"  Iterations: {result.Iterations}");
Console.WriteLine();

// Show control profile
Console.WriteLine("Path angle profile (degrees):");
for (var i = 0; i < result.Controls.Length; i += Math.Max(1, result.Controls.Length / 10))
{
    var theta = result.Controls[i][0] * 180 / Math.PI;
    var t = result.Times[i];
    Console.WriteLine($"  t={t:F2}s: θ={theta:F1}°");
}
Console.WriteLine();

// Generate enhanced visualization with 2D path and physics verification
var htmlPath = ResultVisualizer.GenerateBrachistochroneHtml(
    result,
    g,
    xFinal,
    yFinal);
Console.WriteLine($"Visualization saved to: file://{htmlPath}");

static double VerifyEnergyConservation(CollocationResult result, double g)
{
    var maxError = 0.0;
    for (var i = 0; i < result.States.Length; i++)
    {
        var y = result.States[i][1];
        var v = result.States[i][2];
        var expectedV = Math.Sqrt(2.0 * g * Math.Abs(y));
        var error = Math.Abs(v - expectedV) / (expectedV + 1e-6);
        maxError = Math.Max(maxError, error);
    }
    return maxError;
}
