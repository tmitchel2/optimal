using System;
using Optimal.Control;
using Optimal.NonLinear;
using OptimalCli;

Console.WriteLine("=== DEBUGGING BRACHISTOCHRONE SOLVER ===");
Console.WriteLine();

var g = 9.81;
var xFinal = 2.0;
var yFinal = -2.0;

Console.WriteLine($"Problem setup:");
Console.WriteLine($"  Start: (0, 0) with v=0.1 m/s");
Console.WriteLine($"  End: ({xFinal}, {yFinal})");
Console.WriteLine($"  Time horizon: 0.0 to 1.2 s");
Console.WriteLine();

var problem = new ControlProblem()
    .WithStateSize(3)
    .WithControlSize(1)
    .WithTimeHorizon(0.0, 1.2)
    .WithInitialCondition(new[] { 0.0, 0.0, 0.1 })
    .WithFinalCondition(new[] { xFinal, yFinal, double.NaN })
    .WithControlBounds(new[] { -Math.PI / 2.0 }, new[] { 0.0 })
    .WithDynamics((x, u, t) =>
    {
        var xPos = x[0];
        var yPos = x[1];
        var velocity = x[2];
        var theta = u[0];

        var (xdot, xdot_gradients) = BrachistochroneDynamicsGradients.XRateReverse(xPos, yPos, velocity, theta);
        var (ydot, ydot_gradients) = BrachistochroneDynamicsGradients.YRateReverse(xPos, yPos, velocity, theta);
        var (vdot, vdot_gradients) = BrachistochroneDynamicsGradients.VelocityRateReverse(xPos, yPos, velocity, theta, g);

        var value = new[] { xdot, ydot, vdot };
        var gradients = new double[2][];

        gradients[0] = new[] {
            xdot_gradients[0], xdot_gradients[1], xdot_gradients[2],
            ydot_gradients[0], ydot_gradients[1], ydot_gradients[2],
            vdot_gradients[0], vdot_gradients[1], vdot_gradients[2]
        };

        gradients[1] = new[] {
            xdot_gradients[3],
            ydot_gradients[3],
            vdot_gradients[3]
        };

        return (value, gradients);
    })
    .WithRunningCost((x, u, t) =>
    {
        var xPos = x[0];
        var yPos = x[1];
        var velocity = x[2];
        var theta = u[0];

        var (cost, cost_gradients) = BrachistochroneDynamicsGradients.RunningCostReverse(xPos, yPos, velocity, theta, xFinal, yFinal);

        var gradients = new double[3];
        gradients[0] = cost_gradients[0] + cost_gradients[1] + cost_gradients[2];
        gradients[1] = cost_gradients[3];
        gradients[2] = 0.0;
        return (cost, gradients);
    });

Console.WriteLine("Solving with verbose output to see first few iterations...");
Console.WriteLine();

var iterationCount = 0;
var solver = new HermiteSimpsonSolver()
    .WithSegments(10)
    .WithTolerance(1e-1)
    .WithMaxIterations(5)  // Just a few iterations to see what's happening
    .WithVerbose(true)
    .WithInnerOptimizer(new LBFGSOptimizer()
        .WithTolerance(1e-4)
        .WithMaxIterations(20)
        .WithVerbose(false))
    .WithProgressCallback((iteration, cost, states, controls, times, maxViolation, constraintTolerance) =>
    {
        Console.WriteLine($"\n=== Iteration {iteration} ===");
        Console.WriteLine($"Cost: {cost:F6}, MaxViolation: {maxViolation:E3}");
        Console.WriteLine();
        Console.WriteLine("State trajectory (first 5 and last 2 points):");
        Console.WriteLine("Point  Time(s)  X(m)     Y(m)     V(m/s)   θ(deg)   ẋ       ẏ       v̇");
        Console.WriteLine("---------------------------------------------------------------------------------");
        
        for (var i = 0; i < Math.Min(5, states.Length); i++)
        {
            var state = states[i];
            var control = controls[i];
            var time = times[i];
            var theta = control[0] * 180.0 / Math.PI;
            
            // Compute rates to see what dynamics say
            var xdot = BrachistochroneDynamics.XRate(state[0], state[1], state[2], control[0]);
            var ydot = BrachistochroneDynamics.YRate(state[0], state[1], state[2], control[0]);
            var vdot = BrachistochroneDynamics.VelocityRate(state[0], state[1], state[2], control[0], g);
            
            Console.WriteLine($"{i,5}  {time:F4}   {state[0]:F4}   {state[1]:F4}   {state[2]:F4}   {theta:F2}   {xdot:F4}  {ydot:F4}  {vdot:F4}");
        }
        
        if (states.Length > 5)
        {
            Console.WriteLine("  ...");
            for (var i = states.Length - 2; i < states.Length; i++)
            {
                var state = states[i];
                var control = controls[i];
                var time = times[i];
                var theta = control[0] * 180.0 / Math.PI;
                
                var xdot = BrachistochroneDynamics.XRate(state[0], state[1], state[2], control[0]);
                var ydot = BrachistochroneDynamics.YRate(state[0], state[1], state[2], control[0]);
                var vdot = BrachistochroneDynamics.VelocityRate(state[0], state[1], state[2], control[0], g);
                
                Console.WriteLine($"{i,5}  {time:F4}   {state[0]:F4}   {state[1]:F4}   {state[2]:F4}   {theta:F2}   {xdot:F4}  {ydot:F4}  {vdot:F4}");
            }
        }
        
        // Check if velocity is changing
        var v0 = states[0][2];
        var v1 = states[Math.Min(1, states.Length - 1)][2];
        var vEnd = states[^1][2];
        Console.WriteLine();
        Console.WriteLine($"Velocity check: v[0]={v0:F4}, v[1]={v1:F4}, v[end]={vEnd:F4}");
        
        if (Math.Abs(v1 - v0) < 0.001)
        {
            Console.WriteLine("⚠️  WARNING: Velocity is NOT changing in first segment!");
            Console.WriteLine("    This suggests the dynamics may not be properly integrated.");
        }
    });

var result = solver.Solve(problem);

Console.WriteLine("\n=== FINAL RESULT ===");
Console.WriteLine($"Success: {result.Success}");
Console.WriteLine($"Message: {result.Message}");
Console.WriteLine($"Iterations: {result.Iterations}");
