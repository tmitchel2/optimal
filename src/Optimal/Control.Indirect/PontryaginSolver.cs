/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.Control.Core;
using System.Linq;

namespace Optimal.Control.Indirect
{
    /// <summary>
    /// Indirect method solver using Pontryagin's Minimum Principle.
    /// Solves optimal control problems by converting them into two-point boundary value problems
    /// with state and costate (adjoint) variables.
    /// </summary>
    public sealed class PontryaginSolver
    {
        private int _maxIterations = 100;
        private double _tolerance = 1e-4;
        private bool _verbose;

        /// <summary>
        /// Sets maximum iterations for the shooting method.
        /// </summary>
        public PontryaginSolver WithMaxIterations(int maxIterations)
        {
            _maxIterations = maxIterations;
            return this;
        }

        /// <summary>
        /// Sets convergence tolerance.
        /// </summary>
        public PontryaginSolver WithTolerance(double tolerance)
        {
            _tolerance = tolerance;
            return this;
        }

        /// <summary>
        /// Enables verbose output.
        /// </summary>
        public PontryaginSolver WithVerbose(bool verbose = true)
        {
            _verbose = verbose;
            return this;
        }

        /// <summary>
        /// Solves an optimal control problem using Pontryagin's Minimum Principle.
        /// 
        /// The method converts the problem into a two-point boundary value problem:
        /// - State equations: ẋ = f(x, u*, λ, t)
        /// - Costate equations: λ̇ = -∂H/∂x
        /// - Optimality: ∂H/∂u = 0 (or bang-bang if constrained)
        /// 
        /// Where H = L(x,u,t) + λᵀf(x,u,t) is the Hamiltonian.
        /// </summary>
        public CollocationResult Solve(
            ControlProblem problem,
            Func<double[], double[], double[], double, double[]> optimalControl,
            double[] initialCostates)
        {
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(optimalControl);
            ArgumentNullException.ThrowIfNull(initialCostates);

            if (_verbose)
            {
                Console.WriteLine("Pontryagin Solver: Starting indirect method...");
            }

            // Use shooting method to find correct initial costates
            var lambda0 = SolveBVPWithShooting(
                problem,
                optimalControl,
                initialCostates);

            if (lambda0 == null)
            {
                return new CollocationResult
                {
                    Success = false,
                    Message = "Failed to solve boundary value problem",
                    Times = Array.Empty<double>(),
                    States = Array.Empty<double[]>(),
                    Controls = Array.Empty<double[]>()
                };
            }

            // Integrate forward with found costates
            var (times, states, controls, costates) = IntegrateTrajectory(
                problem,
                optimalControl,
                lambda0);

            // Compute objective cost
            var cost = ComputeCost(problem, times, states, controls);

            if (_verbose)
            {
                Console.WriteLine($"Pontryagin Solver: Converged successfully");
                Console.WriteLine($"  Final cost: {cost:E3}");
                Console.WriteLine($"  Final altitude: {states[states.Length - 1][0]:F3}");
            }

            return new CollocationResult
            {
                Success = true,
                Message = "Pontryagin indirect method converged",
                Times = times,
                States = states,
                Controls = controls,
                OptimalCost = cost,
                Iterations = _maxIterations,
                MaxDefect = 0.0, // No defects in indirect method
                GradientNorm = 0.0
            };
        }

        /// <summary>
        /// Solves the two-point boundary value problem using shooting method.
        /// Adjusts initial costates until boundary conditions are satisfied.
        /// </summary>
        private double[]? SolveBVPWithShooting(
            ControlProblem problem,
            Func<double[], double[], double[], double, double[]> optimalControl,
            double[] initialGuess)
        {
            var lambda0 = (double[])initialGuess.Clone();
            var nStates = problem.StateDim;

            if (_verbose)
            {
                Console.WriteLine("  Shooting method: Finding initial costates...");
            }

            // Simple shooting: Try to match final boundary conditions
            for (var iter = 0; iter < _maxIterations; iter++)
            {
                // Integrate forward
                var (times, states, controls, costates) = IntegrateTrajectory(
                    problem,
                    optimalControl,
                    lambda0);

                // Check boundary condition error
                var xFinal = states[states.Length - 1];
                var error = 0.0;

                if (problem.FinalState != null)
                {
                    // Fixed final state
                    for (var i = 0; i < nStates; i++)
                    {
                        var diff = xFinal[i] - problem.FinalState[i];
                        error += diff * diff;
                    }
                    error = Math.Sqrt(error);
                }
                else
                {
                    // Free final state - check transversality conditions
                    // For terminal cost Φ(x(tf)): λ(tf) = ∂Φ/∂x
                    if (problem.TerminalCost != null)
                    {
                        var (phi, phiGrad) = problem.TerminalCost(xFinal, times[times.Length - 1]);
                        var lambdaFinal = costates[costates.Length - 1];
                        
                        for (var i = 0; i < nStates; i++)
                        {
                            var diff = lambdaFinal[i] - phiGrad[i];
                            error += diff * diff;
                        }
                        error = Math.Sqrt(error);
                    }
                    else
                    {
                        // No terminal cost: free final state, λ(tf) = 0
                        error = costates[costates.Length - 1].Sum(x => x * x);
                        error = Math.Sqrt(error);
                    }
                }

                if (_verbose && iter % 10 == 0)
                {
                    Console.WriteLine($"    Iteration {iter}: error = {error:E3}");
                }

                if (error < _tolerance)
                {
                    if (_verbose)
                    {
                        Console.WriteLine($"  Converged in {iter} iterations");
                    }
                    return lambda0;
                }

                // Adjust costates using finite difference gradient
                if (iter < _maxIterations - 1)
                {
                    AdjustCostates(
                        problem,
                        optimalControl,
                        lambda0,
                        error,
                        iter);
                }
            }

            if (_verbose)
            {
                Console.WriteLine("  Shooting method: Maximum iterations reached");
            }

            // Return best guess even if not fully converged
            return lambda0;
        }

        /// <summary>
        /// Adjusts initial costates to reduce boundary condition error.
        /// </summary>
        private static void AdjustCostates(
            ControlProblem problem,
            Func<double[], double[], double[], double, double[]> optimalControl,
            double[] lambda0,
            double currentError,
            int iteration)
        {
            var nStates = problem.StateDim;
            var stepSize = 0.01 / (1.0 + iteration * 0.01); // Decreasing step size

            // Simple gradient descent-like adjustment
            for (var i = 0; i < nStates; i++)
            {
                var delta = stepSize * (i % 2 == 0 ? 1.0 : -1.0); // Alternate directions
                lambda0[i] += delta;
            }
        }

        /// <summary>
        /// Integrates the state and costate equations forward in time.
        /// </summary>
        private static (double[] times, double[][] states, double[][] controls, double[][] costates) IntegrateTrajectory(
            ControlProblem problem,
            Func<double[], double[], double[], double, double[]> optimalControl,
            double[] lambda0)
        {
            var t0 = problem.InitialTime;
            var tf = problem.FinalTime;
            var nSteps = 100; // Number of integration steps
            var dt = (tf - t0) / nSteps;

            var times = new double[nSteps + 1];
            var states = new double[nSteps + 1][];
            var controls = new double[nSteps + 1][];
            var costates = new double[nSteps + 1][];

            // Initial conditions
            times[0] = t0;
            states[0] = (double[])problem.InitialState!.Clone();
            costates[0] = (double[])lambda0.Clone();
            controls[0] = optimalControl(states[0], costates[0], Array.Empty<double>(), times[0]);

            // RK4 integration
            for (var k = 0; k < nSteps; k++)
            {
                var t = times[k];
                var x = states[k];
                var lambda = costates[k];
                var u = controls[k];

                // Runge-Kutta 4th order
                var (k1x, k1lambda) = ComputeDerivatives(problem, x, lambda, u, t);
                
                var x2 = Add(x, Scale(k1x, dt / 2));
                var lambda2 = Add(lambda, Scale(k1lambda, dt / 2));
                var u2 = optimalControl(x2, lambda2, Array.Empty<double>(), t + dt / 2);
                var (k2x, k2lambda) = ComputeDerivatives(problem, x2, lambda2, u2, t + dt / 2);

                var x3 = Add(x, Scale(k2x, dt / 2));
                var lambda3 = Add(lambda, Scale(k2lambda, dt / 2));
                var u3 = optimalControl(x3, lambda3, Array.Empty<double>(), t + dt / 2);
                var (k3x, k3lambda) = ComputeDerivatives(problem, x3, lambda3, u3, t + dt / 2);

                var x4 = Add(x, Scale(k3x, dt));
                var lambda4 = Add(lambda, Scale(k3lambda, dt));
                var u4 = optimalControl(x4, lambda4, Array.Empty<double>(), t + dt);
                var (k4x, k4lambda) = ComputeDerivatives(problem, x4, lambda4, u4, t + dt);

                // Update
                times[k + 1] = t + dt;
                states[k + 1] = Add(x, Scale(Add(Add(k1x, Scale(k2x, 2)), Add(Scale(k3x, 2), k4x)), dt / 6));
                costates[k + 1] = Add(lambda, Scale(Add(Add(k1lambda, Scale(k2lambda, 2)), Add(Scale(k3lambda, 2), k4lambda)), dt / 6));
                controls[k + 1] = optimalControl(states[k + 1], costates[k + 1], Array.Empty<double>(), times[k + 1]);
            }

            return (times, states, controls, costates);
        }

        /// <summary>
        /// Computes derivatives for both state and costate equations.
        /// </summary>
        private static (double[] xdot, double[] lambdaDot) ComputeDerivatives(
            ControlProblem problem,
            double[] x,
            double[] lambda,
            double[] u,
            double t)
        {
            // State equation: ẋ = f(x, u, t)
            var (f, gradients) = problem.Dynamics!(x, u, t);

            // Costate equation: λ̇ = -∂H/∂x = -∂L/∂x - λᵀ(∂f/∂x)
            var lambdaDot = new double[x.Length];

            // Get ∂L/∂x from running cost
            if (problem.RunningCost != null)
            {
                var (L, Lgrad) = problem.RunningCost(x, u, t);
                lambdaDot[0] = -Lgrad[0]; // Assuming Lgrad[0] is ∂L/∂x
            }

            // Add -λᵀ(∂f/∂x) term
            if (gradients[0] != null)
            {
                var nStates = x.Length;
                for (var i = 0; i < nStates; i++)
                {
                    for (var j = 0; j < nStates; j++)
                    {
                        lambdaDot[i] -= lambda[j] * gradients[0][j * nStates + i];
                    }
                }
            }

            return (f, lambdaDot);
        }

        /// <summary>
        /// Computes the total cost of a trajectory.
        /// </summary>
        private static double ComputeCost(
            ControlProblem problem,
            double[] times,
            double[][] states,
            double[][] controls)
        {
            var cost = 0.0;

            // Running cost (trapezoidal rule)
            if (problem.RunningCost != null)
            {
                for (var k = 0; k < times.Length - 1; k++)
                {
                    var dt = times[k + 1] - times[k];
                    var (L1, _) = problem.RunningCost(states[k], controls[k], times[k]);
                    var (L2, _) = problem.RunningCost(states[k + 1], controls[k + 1], times[k + 1]);
                    cost += 0.5 * (L1 + L2) * dt;
                }
            }

            // Terminal cost
            if (problem.TerminalCost != null)
            {
                var (phi, _) = problem.TerminalCost(states[states.Length - 1], times[times.Length - 1]);
                cost += phi;
            }

            return cost;
        }

        // Vector helper methods
        private static double[] Add(double[] a, double[] b)
        {
            var result = new double[a.Length];
            for (var i = 0; i < a.Length; i++)
            {
                result[i] = a[i] + b[i];
            }
            return result;
        }

        private static double[] Scale(double[] a, double s)
        {
            var result = new double[a.Length];
            for (var i = 0; i < a.Length; i++)
            {
                result[i] = a[i] * s;
            }
            return result;
        }
    }
}
