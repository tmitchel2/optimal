/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using OptimalCli;
using OptimalCli.Problems.Brachistochrone;
using OptimalCli.Problems.CartPole;
using OptimalCli.Problems.Corner;
using OptimalCli.Problems.Dubin;
using OptimalCli.Problems.Goddard;
using OptimalCli.Problems.Pendulum;
using OptimalCli.Problems.VanDerPol;

// Parse command-line arguments
if (args.Length == 0 || args[0] == "--help" || args[0] == "-h")
{
    ShowHelp();
    return;
}

// Parse options
var headless = false;
var solver = SolverType.HS;
var variant = BrachistochroneVariant.FreeFinalTime;
var goddardVariant = GoddardRocketVariant.FreeFinalTime;
var debugViz = false;
var problemName = args[0].ToLowerInvariant();

for (var i = 1; i < args.Length; i++)
{
    if (args[i] == "--headless" || args[i] == "-H")
    {
        headless = true;
    }
    else if (args[i] == "--debug" || args[i] == "-d")
    {
        debugViz = true;
    }
    else if (args[i] == "--solver" || args[i] == "-s")
    {
        if (i + 1 < args.Length)
        {
            var solverArg = args[++i].ToLowerInvariant();
            solver = solverArg switch
            {
                "hs" or "hermite-simpson" => SolverType.HS,
                "lgl" => SolverType.LGL,
                _ => throw new ArgumentException($"Unknown solver type: {solverArg}. Use 'hs' or 'lgl'.")
            };
        }
    }
    else if (args[i] == "--variant" || args[i] == "-v")
    {
        if (i + 1 < args.Length)
        {
            var variantArg = args[++i].ToLowerInvariant();

            // Check if this is for Goddard or Brachistochrone
            if (problemName == "goddard")
            {
                goddardVariant = variantArg switch
                {
                    "fixed" or "fixed-time" or "fixed-tf" => GoddardRocketVariant.FixedFinalTime,
                    "free" or "free-time" or "free-tf" => GoddardRocketVariant.FreeFinalTime,
                    _ => throw new ArgumentException($"Unknown Goddard variant: {variantArg}. Use 'fixed' or 'free'.")
                };
            }
            else
            {
                variant = variantArg switch
                {
                    "fixed" or "fixed-time" => BrachistochroneVariant.FixedTime,
                    "free" or "free-time" => BrachistochroneVariant.FreeFinalTime,
                    "running" or "running-cost" => BrachistochroneVariant.FreeFinalTimeRunningCost,
                    _ => throw new ArgumentException($"Unknown variant: {variantArg}. Use 'fixed', 'free', or 'running'.")
                };
            }
        }
    }
}

var options = new CommandOptions(Headless: headless, Solver: solver, Variant: variant, GoddardVariant: goddardVariant, DebugViz: debugViz);

if (headless)
{
    Console.WriteLine("Running in headless mode (no visualization)");
}

Console.WriteLine($"Using solver: {options.Solver}");
Console.WriteLine();

// Create command registry
var commands = new Dictionary<string, ICommand>
{
    { "brachistochrone", new BrachistochroneProblemSolver() },
    { "vanderpol", new VanDerPolProblemSolver() },
    { "pendulum", new PendulumSwingUpProblemSolver() },
    { "cartpole", new CartPoleProblemSolver() },
    { "dubins", new DubinsCarProblemSolver() },
    { "goddard", new GoddardRocketProblemSolver() },
    { "corner", new CornerProblemSolver() }
};

// Find and run the requested command
if (commands.TryGetValue(problemName, out var command))
{
    command.Run(options);
}
else
{
    Console.WriteLine($"Error: Unknown problem '{args[0]}'");
    Console.WriteLine();
    ShowHelp();
    Environment.Exit(1);
}

static void ShowHelp()
{
    Console.WriteLine("OptimalCli - Classic Optimal Control Problems");
    Console.WriteLine();
    Console.WriteLine("Usage: OptimalCli <problem> [options]");
    Console.WriteLine();
    Console.WriteLine("Available problems:");
    Console.WriteLine();
    Console.WriteLine("  brachistochrone  - Curve of fastest descent under gravity (Johann Bernoulli, 1696)");
    Console.WriteLine("  vanderpol        - Van der Pol oscillator stabilization with minimum control effort");
    Console.WriteLine("  pendulum         - Pendulum swing-up from hanging down to fully inverted (vertical)");
    Console.WriteLine("  cartpole         - Cart-pole stabilization with full nonlinear dynamics");
    Console.WriteLine("  dubins           - Dubins car minimum path with curvature constraint");
    Console.WriteLine("  goddard          - Goddard rocket maximum altitude with drag and fuel constraints");
    Console.WriteLine("  corner           - Optimal racing line through 90° corner with road constraints");
    Console.WriteLine();
    Console.WriteLine("Options:");
    Console.WriteLine("  --headless, -H              Run without visualization windows");
    Console.WriteLine("  --solver, -s <type>         Solver type: 'hs' (default) or 'lgl'");
    Console.WriteLine("  --variant, -v <type>        Problem variant (see below)");
    Console.WriteLine();
    Console.WriteLine("Solvers:");
    Console.WriteLine("  hs                          Hermite-Simpson collocation (default, more robust)");
    Console.WriteLine("  lgl                         Legendre-Gauss-Lobatto collocation (higher order accuracy)");
    Console.WriteLine();
    Console.WriteLine("Brachistochrone variants:");
    Console.WriteLine("  fixed, fixed-time           Fixed final time formulation (simpler, for testing)");
    Console.WriteLine("  free, free-time             Free final time with time-scaling (classic formulation)");
    Console.WriteLine("  running, running-cost       Free final time with running cost (alternative)");
    Console.WriteLine();
    Console.WriteLine("Goddard rocket variants:");
    Console.WriteLine("  default                     Normalized parameters (original implementation)");
    Console.WriteLine("  fixed, fixed-time, fixed-tf Fixed final time (tf=100s, PROPT example 45)");
    Console.WriteLine("  free, free-time, free-tf    Free final time (PROPT example 44)");
    Console.WriteLine();
    Console.WriteLine("Examples:");
    Console.WriteLine("  OptimalCli brachistochrone");
    Console.WriteLine("  OptimalCli brachistochrone -v fixed");
    Console.WriteLine("  OptimalCli cartpole --headless");
    Console.WriteLine("  OptimalCli goddard -H --solver lgl");
    Console.WriteLine("  OptimalCli goddard -v free-tf");
    Console.WriteLine("  OptimalCli brachistochrone -s lgl -v running");
    Console.WriteLine();
}
