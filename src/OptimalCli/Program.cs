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
var problemName = args[0].ToLowerInvariant();

for (var i = 1; i < args.Length; i++)
{
    if (args[i] == "--headless" || args[i] == "-H")
    {
        headless = true;
    }
}

var options = new CommandOptions(Headless: headless);

if (headless)
{
    Console.WriteLine("Running in headless mode (no visualization)");
    Console.WriteLine();
}

// Create command registry
var commands = new Dictionary<string, ICommand>
{
    { "brachistochrone", new BrachistochroneProblemSolver() },
    { "vanderpol", new VanDerPolProblemSolver() },
    { "pendulum", new PendulumSwingUpProblemSolver() },
    { "cartpole", new CartPoleProblemSolver() },
    { "dubins", new DubinsCarProblemSolver() },
    { "goddard", new GoddardRocketProblemSolver() }
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
    Console.WriteLine();
    Console.WriteLine("Options:");
    Console.WriteLine("  --headless, -H   Run without visualization windows");
    Console.WriteLine();
    Console.WriteLine("Examples:");
    Console.WriteLine("  OptimalCli brachistochrone");
    Console.WriteLine("  OptimalCli cartpole --headless");
    Console.WriteLine("  OptimalCli goddard -H");
    Console.WriteLine();
}
