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

var problemName = args[0].ToLowerInvariant();

// Create problem solver registry
var solvers = new Dictionary<string, IProblemSolver>
{
    { "brachistochrone", new BrachistochroneProblemSolver() },
    { "vanderpol", new VanDerPolProblemSolver() },
    { "pendulum", new PendulumSwingUpProblemSolver() },
    { "cartpole", new CartPoleProblemSolver() },
    { "dubins", new DubinsCarProblemSolver() },
    { "goddard", new GoddardRocketProblemSolver() }
};

// Find and run the requested problem
if (solvers.TryGetValue(problemName, out var solver))
{
    solver.Solve();
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
    Console.WriteLine("Usage: OptimalCli <problem>");
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
    Console.WriteLine("Examples:");
    Console.WriteLine("  OptimalCli brachistochrone");
    Console.WriteLine("  OptimalCli cartpole");
    Console.WriteLine("  OptimalCli goddard");
    Console.WriteLine();
}
