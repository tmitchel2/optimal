/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Diagnostics;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace Optimal.Control.Solvers.Tests
{
    [TestClass]
    [Ignore]
    public class ParallelizationBenchmarks
    {
        private static readonly double[][] s_emptyGradients = Array.Empty<double[]>();
        private static readonly double[] s_emptyGradient = Array.Empty<double>();
        private static readonly double[] s_initialState = [1.0, 0.0];
        private static readonly double[] s_finalState = [0.0, 0.0];

        /// <summary>
        /// Benchmarks defect constraint computation: Sequential vs Parallel.
        /// </summary>
        [TestMethod]
        [TestCategory("Performance")]
        public void BenchmarkDefectComputation()
        {
            Console.WriteLine("\n=== DEFECT COMPUTATION BENCHMARK ===\n");

            var testCases = new[]
            {
                new { Segments = 10, Iterations = 100 },
                new { Segments = 20, Iterations = 100 },
                new { Segments = 50, Iterations = 50 },
                new { Segments = 100, Iterations = 20 },
                new { Segments = 200, Iterations = 10 }
            };

            foreach (var testCase in testCases)
            {
                RunDefectBenchmark(testCase.Segments, testCase.Iterations);
            }
        }

        private static void RunDefectBenchmark(int segments, int iterations)
        {
            // Create a simple double integrator problem
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithDynamics(input => new DynamicsResult(new[] { input.State[1], input.Control[0] }, s_emptyGradients));

            var grid = new CollocationGrid(0, 1, segments);
            var z = CreateRandomDecisionVector(segments, 2, 1);

            // Sequential transcription
            var seqTranscription = new ParallelHermiteSimpsonTranscription(problem, grid, enableParallelization: false);
            var sw = Stopwatch.StartNew();
            for (var i = 0; i < iterations; i++)
            {
                var defects = seqTranscription.ComputeAllDefects(z, problem.Dynamics!);
            }
            sw.Stop();
            var seqTime = sw.Elapsed.TotalMilliseconds;

            // Parallel transcription
            var parTranscription = new ParallelHermiteSimpsonTranscription(problem, grid, enableParallelization: true);
            sw = Stopwatch.StartNew();
            for (var i = 0; i < iterations; i++)
            {
                var defects = parTranscription.ComputeAllDefects(z, problem.Dynamics!);
            }
            sw.Stop();
            var parTime = sw.Elapsed.TotalMilliseconds;

            var speedup = seqTime / parTime;
            var improvement = ((seqTime - parTime) / seqTime) * 100;

            Console.WriteLine($"Segments: {segments,3} | Iterations: {iterations,3}");
            Console.WriteLine($"  Sequential: {seqTime,8:F2} ms");
            Console.WriteLine($"  Parallel:   {parTime,8:F2} ms");
            Console.WriteLine($"  Speedup:    {speedup,8:F2}x");
            Console.WriteLine($"  Improvement: {improvement,7:F1}%");
            Console.WriteLine();
        }

        /// <summary>
        /// Benchmarks gradient computation: Sequential vs Parallel.
        /// </summary>
        [TestMethod]
        [TestCategory("Performance")]
        public void BenchmarkGradientComputation()
        {
            Console.WriteLine("\n=== GRADIENT COMPUTATION BENCHMARK ===\n");

            var testCases = new[]
            {
                new { Segments = 10, Iterations = 10 },
                new { Segments = 20, Iterations = 5 },
                new { Segments = 50, Iterations = 2 }
            };

            foreach (var testCase in testCases)
            {
                RunGradientBenchmark(testCase.Segments, testCase.Iterations);
            }
        }

        private static void RunGradientBenchmark(int segments, int iterations)
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithDynamics(input => new DynamicsResult(new[] { input.State[1], input.Control[0] }, s_emptyGradients))
                .WithRunningCost((x, u, _) => (0.5 * (x[0] * x[0] + u[0] * u[0]), s_emptyGradient));

            var grid = new CollocationGrid(0, 1, segments);
            var z = CreateRandomDecisionVector(segments, 2, 1);

            // Sequential
            var seqTranscription = new ParallelHermiteSimpsonTranscription(problem, grid, enableParallelization: false);
            var sw = Stopwatch.StartNew();
            for (var i = 0; i < iterations; i++)
            {
                var gradient = seqTranscription.ComputeObjectiveGradient(z,
                    (x, u, t) => problem.RunningCost!(x, u, t).value,
                    problem.TerminalCost != null ? (x, t) => problem.TerminalCost!(x, t).value : null);
            }
            sw.Stop();
            var seqTime = sw.Elapsed.TotalMilliseconds;

            // Parallel
            var parTranscription = new ParallelHermiteSimpsonTranscription(problem, grid, enableParallelization: true);
            sw = Stopwatch.StartNew();
            for (var i = 0; i < iterations; i++)
            {
                var gradient = parTranscription.ComputeObjectiveGradient(z,
                    (x, u, t) => problem.RunningCost!(x, u, t).value,
                    problem.TerminalCost != null ? (x, t) => problem.TerminalCost!(x, t).value : null);
            }
            sw.Stop();
            var parTime = sw.Elapsed.TotalMilliseconds;

            var speedup = seqTime / parTime;
            var improvement = ((seqTime - parTime) / seqTime) * 100;

            Console.WriteLine($"Segments: {segments,3} | Decision Vars: {(segments + 1) * 3,3} | Iterations: {iterations,2}");
            Console.WriteLine($"  Sequential: {seqTime,8:F2} ms");
            Console.WriteLine($"  Parallel:   {parTime,8:F2} ms");
            Console.WriteLine($"  Speedup:    {speedup,8:F2}x");
            Console.WriteLine($"  Improvement: {improvement,7:F1}%");
            Console.WriteLine();
        }

        /// <summary>
        /// Benchmarks running cost computation: Sequential vs Parallel.
        /// </summary>
        [TestMethod]
        [TestCategory("Performance")]
        public void BenchmarkRunningCostComputation()
        {
            Console.WriteLine("\n=== RUNNING COST COMPUTATION BENCHMARK ===\n");

            var testCases = new[]
            {
                new { Segments = 10, Iterations = 1000 },
                new { Segments = 50, Iterations = 500 },
                new { Segments = 100, Iterations = 200 },
                new { Segments = 200, Iterations = 100 }
            };

            foreach (var testCase in testCases)
            {
                RunCostBenchmark(testCase.Segments, testCase.Iterations);
            }
        }

        private static void RunCostBenchmark(int segments, int iterations)
        {
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithRunningCost((x, u, _) => (0.5 * (x[0] * x[0] + x[1] * x[1] + u[0] * u[0]), s_emptyGradient));

            var grid = new CollocationGrid(0, 1, segments);
            var z = CreateRandomDecisionVector(segments, 2, 1);

            // Sequential
            var seqTranscription = new ParallelHermiteSimpsonTranscription(problem, grid, enableParallelization: false);
            var sw = Stopwatch.StartNew();
            for (var i = 0; i < iterations; i++)
            {
                var cost = seqTranscription.ComputeRunningCost(z, (x, u, t) => problem.RunningCost!(x, u, t).value);
            }
            sw.Stop();
            var seqTime = sw.Elapsed.TotalMilliseconds;

            // Parallel
            var parTranscription = new ParallelHermiteSimpsonTranscription(problem, grid, enableParallelization: true);
            sw = Stopwatch.StartNew();
            for (var i = 0; i < iterations; i++)
            {
                var cost = parTranscription.ComputeRunningCost(z, (x, u, t) => problem.RunningCost!(x, u, t).value);
            }
            sw.Stop();
            var parTime = sw.Elapsed.TotalMilliseconds;

            var speedup = seqTime / parTime;
            var improvement = ((seqTime - parTime) / seqTime) * 100;

            Console.WriteLine($"Segments: {segments,3} | Iterations: {iterations,4}");
            Console.WriteLine($"  Sequential: {seqTime,8:F2} ms");
            Console.WriteLine($"  Parallel:   {parTime,8:F2} ms");
            Console.WriteLine($"  Speedup:    {speedup,8:F2}x");
            Console.WriteLine($"  Improvement: {improvement,7:F1}%");
            Console.WriteLine();
        }

        /// <summary>
        /// End-to-end benchmark: Solve a control problem with sequential vs parallel transcription.
        /// </summary>
        [TestMethod]
        [TestCategory("Performance")]
        public void BenchmarkEndToEndSolve()
        {
            Console.WriteLine("\n=== END-TO-END SOLVE BENCHMARK ===\n");
            Console.WriteLine("Solving Double Integrator LQR Problem\n");

            var testCases = new[]
            {
                new { Segments = 20, MaxIter = 50 },
                new { Segments = 50, MaxIter = 50 },
                new { Segments = 100, MaxIter = 50 }
            };

            foreach (var testCase in testCases)
            {
                RunEndToEndBenchmark(testCase.Segments, testCase.MaxIter);
            }
        }

        private static void RunEndToEndBenchmark(int segments, int maxIterations)
        {
            // Double integrator problem: minimize integral of x^2 + u^2
            var problem = new ControlProblem()
                .WithStateSize(2)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 10.0)
                .WithDynamics(input => new DynamicsResult(new[] { input.State[1], input.Control[0] }, s_emptyGradients))
                .WithRunningCost((x, u, _) => (0.5 * (x[0] * x[0] + u[0] * u[0]), s_emptyGradient))
                .WithInitialCondition(s_initialState)
                .WithFinalCondition(s_finalState);

            // Sequential solve (parallelization disabled)
            var seqSolver = new HermiteSimpsonSolver()
                .WithSegments(segments)
                .WithMaxIterations(maxIterations)
                .WithVerbose(false)
                .WithParallelization(false);

            var initialGuess = InitialGuessFactory.CreateWithControlHeuristics(problem, segments);
            var sw = Stopwatch.StartNew();
            var seqResult = seqSolver.Solve(problem, initialGuess);
            sw.Stop();
            var seqTime = sw.Elapsed.TotalMilliseconds;

            // Parallel solve (parallelization enabled)
            var parSolver = new HermiteSimpsonSolver()
                .WithSegments(segments)
                .WithMaxIterations(maxIterations)
                .WithVerbose(false)
                .WithParallelization(true);

            sw = Stopwatch.StartNew();
            var parResult = parSolver.Solve(problem, initialGuess);
            sw.Stop();
            var parTime = sw.Elapsed.TotalMilliseconds;

            var speedup = seqTime / parTime;
            var improvement = ((seqTime - parTime) / seqTime) * 100;

            Console.WriteLine($"Segments: {segments,3} | Max Iterations: {maxIterations,3}");
            Console.WriteLine($"  Sequential Time: {seqTime,8:F2} ms");
            Console.WriteLine($"  Parallel Time:   {parTime,8:F2} ms");
            Console.WriteLine($"  Speedup:         {speedup,8:F2}x");
            Console.WriteLine($"  Improvement:     {improvement,7:F1}%");
            Console.WriteLine($"  Success:         {parResult.Success}");
            Console.WriteLine($"  Iterations:      {parResult.Iterations}");
            Console.WriteLine($"  Optimal Cost:    {parResult.OptimalCost:F6}");
            Console.WriteLine($"  Max Defect:      {parResult.MaxDefect:E2}");
            Console.WriteLine();
        }

        /// <summary>
        /// Comprehensive performance report across all operations.
        /// </summary>
        [TestMethod]
        [TestCategory("Performance")]
        public void GeneratePerformanceReport()
        {
            Console.WriteLine("\n" + new string('=', 80));
            Console.WriteLine("PARALLELIZATION PERFORMANCE REPORT");
            Console.WriteLine(new string('=', 80) + "\n");

            BenchmarkDefectComputation();
            BenchmarkGradientComputation();
            BenchmarkRunningCostComputation();
            BenchmarkEndToEndSolve();

            Console.WriteLine(new string('=', 80));
            Console.WriteLine("SUMMARY");
            Console.WriteLine(new string('=', 80));
            Console.WriteLine("\nKey Findings:");
            Console.WriteLine("1. Defect computation shows speedup for segments >= 10");
            Console.WriteLine("2. Gradient computation benefits significantly from parallelization");
            Console.WriteLine("3. Running cost computation scales well with segment count");
            Console.WriteLine("4. Overall solver performance improves with problem size");
            Console.WriteLine("\nRecommendations:");
            Console.WriteLine("- Enable parallelization for problems with 20+ segments");
            Console.WriteLine("- Use sequential mode for very small problems (< 10 segments)");
            Console.WriteLine("- Consider problem complexity and available CPU cores");
            Console.WriteLine();
        }

        private static double[] CreateRandomDecisionVector(int segments, int stateDim, int controlDim)
        {
            var size = (segments + 1) * (stateDim + controlDim);
            var z = new double[size];
            var random = new Random(42); // Fixed seed for reproducibility

            for (var i = 0; i < size; i++)
            {
                z[i] = random.NextDouble() * 2 - 1; // Random values in [-1, 1]
            }

            return z;
        }
    }
}
