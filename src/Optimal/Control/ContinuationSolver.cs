/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.Control
{
    /// <summary>
    /// Provides parameter continuation (homotopy) methods for solving difficult optimal control problems.
    /// Gradually transitions from an easy problem to the desired problem.
    /// </summary>
    public sealed class ContinuationSolver
    {
        private readonly HermiteSimpsonSolver _solver;
        private readonly List<double> _continuationParameters;
        private bool _verbose;

        /// <summary>
        /// Creates a continuation solver with the given base solver.
        /// </summary>
        /// <param name="solver">The Hermite-Simpson solver to use.</param>
        public ContinuationSolver(HermiteSimpsonSolver solver)
        {
            _solver = solver ?? throw new ArgumentNullException(nameof(solver));
            _continuationParameters = new List<double>();
        }

        /// <summary>
        /// Sets the continuation parameter values.
        /// Parameters should range from 0 (easy problem) to 1 (target problem).
        /// </summary>
        /// <param name="parameters">Sequence of parameter values, e.g., [0.0, 0.25, 0.5, 0.75, 1.0].</param>
        /// <returns>This continuation solver for method chaining.</returns>
        public ContinuationSolver WithParameters(params double[] parameters)
        {
            ArgumentNullException.ThrowIfNull(parameters);
            
            _continuationParameters.Clear();
            foreach (var p in parameters)
            {
                if (p < 0.0 || p > 1.0)
                {
                    throw new ArgumentException("Continuation parameters must be in [0, 1].", nameof(parameters));
                }
                _continuationParameters.Add(p);
            }

            // Sort parameters
            _continuationParameters.Sort();

            return this;
        }

        /// <summary>
        /// Automatically generates continuation parameters using linear spacing.
        /// </summary>
        /// <param name="steps">Number of continuation steps.</param>
        /// <returns>This continuation solver for method chaining.</returns>
        public ContinuationSolver WithLinearSteps(int steps)
        {
            if (steps < 2)
            {
                throw new ArgumentException("Must have at least 2 steps.", nameof(steps));
            }

            _continuationParameters.Clear();
            for (var i = 0; i < steps; i++)
            {
                var lambda = i / (double)(steps - 1);
                _continuationParameters.Add(lambda);
            }

            return this;
        }

        /// <summary>
        /// Enables verbose output.
        /// </summary>
        /// <param name="verbose">True to enable verbose output.</param>
        /// <returns>This continuation solver for method chaining.</returns>
        public ContinuationSolver WithVerbose(bool verbose = true)
        {
            _verbose = verbose;
            return this;
        }

        /// <summary>
        /// Solves a problem using parameter continuation.
        /// The problemGenerator function should create a control problem for a given parameter lambda in [0, 1].
        /// Lambda = 0 should be an easy problem, lambda = 1 should be the target problem.
        /// </summary>
        /// <param name="problemGenerator">Function that generates a control problem for a given lambda.</param>
        /// <returns>The solution to the target problem (lambda = 1).</returns>
        public CollocationResult Solve(Func<double, ControlProblem> problemGenerator)
        {
            ArgumentNullException.ThrowIfNull(problemGenerator);

            if (_continuationParameters.Count == 0)
            {
                // Default: single step (no continuation)
                _continuationParameters.Add(1.0);
            }

            CollocationResult? previousResult = null;

            foreach (var lambda in _continuationParameters)
            {
                if (_verbose)
                {
                    Console.WriteLine($"\nContinuation step: lambda = {lambda:F4}");
                }

                // Generate problem for this parameter
                var problem = problemGenerator(lambda);

                // Solve with warm start if available
                CollocationResult result;
                if (previousResult != null)
                {
                    result = SolveWithWarmStart(problem, previousResult);
                }
                else
                {
                    result = _solver.Solve(problem);
                }

                if (!result.Success)
                {
                    if (_verbose)
                    {
                        Console.WriteLine($"Continuation failed at lambda = {lambda:F4}");
                    }
                    return result; // Return failed result
                }

                if (_verbose)
                {
                    Console.WriteLine($"  Success: cost = {result.OptimalCost:E3}, defect = {result.MaxDefect:E3}");
                }

                previousResult = result;
            }

            return previousResult!;
        }

        /// <summary>
        /// Solves a problem with warm start from previous solution.
        /// </summary>
        private CollocationResult SolveWithWarmStart(ControlProblem problem, CollocationResult previousResult)
        {
            // Create grid and transcription for new problem
            var grid = new CollocationGrid(problem.InitialTime, problem.FinalTime, _solver.GetSegments());
            var transcription = new HermiteSimpsonTranscription(problem, grid);

            // Create warm start initial guess
            var initialGuess = WarmStart.InterpolateFromPrevious(previousResult, grid, transcription);

            // Solve using the warm start
            return _solver.SolveWithInitialGuess(problem, initialGuess);
        }

        /// <summary>
        /// Solves a sequence of problems, using each solution as warm start for the next.
        /// Useful for parameter sweeps or tracking solutions along a path.
        /// </summary>
        /// <param name="problems">Sequence of control problems to solve.</param>
        /// <returns>Array of solutions for each problem.</returns>
        public CollocationResult[] SolveSequence(params ControlProblem[] problems)
        {
            ArgumentNullException.ThrowIfNull(problems);

            if (problems.Length == 0)
            {
                return Array.Empty<CollocationResult>();
            }

            var results = new CollocationResult[problems.Length];
            CollocationResult? previousResult = null;

            for (var i = 0; i < problems.Length; i++)
            {
                if (_verbose)
                {
                    Console.WriteLine($"\nSolving problem {i + 1}/{problems.Length}");
                }

                var problem = problems[i];

                CollocationResult result;
                if (previousResult != null)
                {
                    result = SolveWithWarmStart(problem, previousResult);
                }
                else
                {
                    result = _solver.Solve(problem);
                }

                if (!result.Success && _verbose)
                {
                    Console.WriteLine($"  Failed to converge on problem {i + 1}");
                }

                results[i] = result;
                previousResult = result.Success ? result : null; // Only use successful results for warm start
            }

            return results;
        }
    }

    /// <summary>
    /// Extension methods for HermiteSimpsonSolver to support warm starting.
    /// </summary>
    public static class WarmStartExtensions
    {
        /// <summary>
        /// Solves a problem with a provided initial guess.
        /// </summary>
        /// <param name="solver">The solver instance.</param>
        /// <param name="problem">The control problem.</param>
        /// <param name="initialGuess">Initial guess for decision variables.</param>
        /// <returns>Solution result.</returns>
        public static CollocationResult SolveWithInitialGuess(
            this HermiteSimpsonSolver solver,
            ControlProblem problem,
            double[] initialGuess)
        {
            ArgumentNullException.ThrowIfNull(solver);
            ArgumentNullException.ThrowIfNull(problem);
            ArgumentNullException.ThrowIfNull(initialGuess);

            // Use reflection to call private method
            var method = typeof(HermiteSimpsonSolver).GetMethod(
                "SolveOnFixedGrid",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            if (method == null)
            {
                throw new InvalidOperationException("Could not find SolveOnFixedGrid method.");
            }

            var result = method.Invoke(solver, new object?[] { problem, solver.GetSegments(), initialGuess });
            return (CollocationResult)result!;
        }

        /// <summary>
        /// Gets the number of segments configured in the solver.
        /// </summary>
        /// <param name="solver">The solver instance.</param>
        /// <returns>Number of segments.</returns>
        public static int GetSegments(this HermiteSimpsonSolver solver)
        {
            ArgumentNullException.ThrowIfNull(solver);

            // Use reflection to access private field
            var field = typeof(HermiteSimpsonSolver).GetField(
                "_segments",
                System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);

            if (field == null)
            {
                throw new InvalidOperationException("Could not find _segments field.");
            }

            return (int)field.GetValue(solver)!;
        }
    }
}
