/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Mma.Tests
{
    /// <summary>
    /// The phase 21 gate: an MMA subproblem with n = 10⁵ variables and m = 10⁴ sparse constraints solves in under a
    /// second (<see cref="SyntheticSubproblem"/> on a 50 × 50 × 40 grid: 25 × 25 × 16 local volume windows).
    /// </summary>
    /// <remarks>
    /// The gate is judged on the process's CPU time in a Release build, the class running alone: wall time on a shared
    /// machine measures the scheduler as much as the solver. Both are reported.
    /// </remarks>
    [TestClass]
    [DoNotParallelize]
    public sealed class MmaSubproblemPerformanceTests
    {
        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        [TestCategory("LongRunning")]
        public void HundredThousandVariablesAndTenThousandSparseConstraintsSolveInUnderASecond()
        {
            AssertSolvesInUnderASecond(includeVolume: false);
        }

        [TestMethod]
        [TestCategory("LongRunning")]
        public void WithADenseVolumeRowAsWell()
        {
            AssertSolvesInUnderASecond(includeVolume: true);
        }

        private void AssertSolvesInUnderASecond(bool includeVolume)
        {
            var wall = new List<double>();
            var cpu = new List<double>();
            SyntheticSubproblem? synthetic = null;
            for (var run = 0; run < 3; run++)
            {
                synthetic = SyntheticSubproblem.Create(50, 50, 40, includeVolume, MmaLinearSolver.Automatic);
                var process = Process.GetCurrentProcess();
                var cpuBefore = process.TotalProcessorTime;
                var stopwatch = Stopwatch.StartNew();
                var solved = synthetic.Solve();
                stopwatch.Stop();
                process.Refresh();
                Assert.IsTrue(solved, "the subproblem failed");
                wall.Add(stopwatch.Elapsed.TotalSeconds);
                cpu.Add((process.TotalProcessorTime - cpuBefore).TotalSeconds);
            }

            var bestCpu = Math.Min(cpu[1], cpu[2]);
            var bestWall = Math.Min(wall[1], wall[2]);
            TestContext.WriteLine(
                $"n = {synthetic!.N}, m = {synthetic.M}, stored = {synthetic.Stored}: wall {wall[0]:F3} (cold), {wall[1]:F3}, {wall[2]:F3} s; CPU {cpu[0]:F3} (cold), {cpu[1]:F3}, {cpu[2]:F3} s; {synthetic.Describe()}");
            synthetic.AssertSolutionIsFeasible();
            if (IsJitOptimizerDisabled())
            {
                Assert.Inconclusive($"Timed in a Debug build, where the JIT does not optimise ({bestCpu:F3} s CPU); the gate is judged in Release.");
            }

            Assert.IsLessThan(1.0, bestCpu, $"best warm solve {bestCpu:F3} s CPU ({bestWall:F3} s wall)");
        }

        private static bool IsJitOptimizerDisabled()
        {
            return typeof(MmaSubproblemPerformanceTests).Assembly.GetCustomAttribute<DebuggableAttribute>()?.IsJITOptimizerDisabled == true;
        }
    }
}
