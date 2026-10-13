/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Mma.Tests
{
    [TestClass]
    public sealed class MmaOptimizerTests
    {
        private static readonly double[] s_cantileverStart = [5.0, 5.0, 5.0, 5.0, 5.0];
        private static readonly double[] s_cantileverPublished = [6.016, 5.309, 4.494, 3.502, 2.153];
        private static readonly double[] s_trussStart = [1.5, 0.5];

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void SvanbergCantileverReachesPublishedOptimum()
        {
            // Svanberg (1987), IJNME 24:359–373, and Svanberg (2007) "MMA and GCMMA": f* = 1.340 at
            // x* = (6.016, 5.309, 4.494, 3.502, 2.153).
            var result = Cantilever(MmaLinearSolver.Automatic);

            TestContext.WriteLine($"cantilever: f={result.OptimalValue:R} x=[{string.Join(", ", result.OptimalPoint)}] iterations={result.Iterations} kkt={result.KktResidualNorm:E3}");
            Assert.IsTrue(result.Success, result.Message);
            Assert.AreEqual(1.340, Math.Round(result.OptimalValue, 3), 1e-12, $"f* = {result.OptimalValue}");
            for (var j = 0; j < 5; j++)
            {
                Assert.AreEqual(s_cantileverPublished[j], result.OptimalPoint[j], 1e-2, $"x{j + 1}");
            }

            var analytic = MmaTestProblems.CantileverAnalyticOptimum();
            for (var j = 0; j < 5; j++)
            {
                Assert.AreEqual(analytic[j], result.OptimalPoint[j], 1e-4, $"x{j + 1} against the analytic optimum");
            }
        }

        [TestMethod]
        public void CantileverDirectAndConjugateGradientAgree()
        {
            var direct = Cantilever(MmaLinearSolver.Direct);
            var iterative = Cantilever(MmaLinearSolver.ConjugateGradient);

            Assert.IsTrue(iterative.Success, iterative.Message);
            for (var j = 0; j < 5; j++)
            {
                Assert.AreEqual(direct.OptimalPoint[j], iterative.OptimalPoint[j], 1e-6, $"x{j + 1}");
            }
        }

        [TestMethod]
        public void SvanbergTwoBarTrussReachesPublishedOptimum()
        {
            // Svanberg (1987), the two-bar truss: x* = (1.41, 0.38), f* = 1.51. With m = n = 2 the subproblem's
            // reduced system is the n×n one.
            var result = new MmaOptimizer(new MmaOptions { Tolerance = 1e-8, SubproblemTolerance = 1e-10, MaxIterations = 200 })
                .Minimize(MmaTestProblems.TwoBarTruss(), s_trussStart, CancellationToken.None);
            var (x1, x2, f) = MmaTestProblems.TwoBarTrussAnalyticOptimum();

            TestContext.WriteLine($"truss: f={result.OptimalValue:R} x=[{string.Join(", ", result.OptimalPoint)}] iterations={result.Iterations}");
            Assert.IsTrue(result.Success, result.Message);
            Assert.AreEqual(1.51, Math.Round(result.OptimalValue, 2), 1e-12);
            Assert.AreEqual(1.41, result.OptimalPoint[0], 5e-3);
            Assert.AreEqual(0.38, result.OptimalPoint[1], 5e-3);
            Assert.AreEqual(x1, result.OptimalPoint[0], 1e-5);
            Assert.AreEqual(x2, result.OptimalPoint[1], 1e-5);
            Assert.AreEqual(f, result.OptimalValue, 1e-6);
            Assert.IsLessThanOrEqualTo(1e-6, result.MaxConstraintViolation);
        }

        [TestMethod]
        public void SparseBlockAgreesWithDenseRows()
        {
            var options = new MmaOptions { Tolerance = 1e-9, SubproblemTolerance = 1e-11, MaxIterations = 500 };
            var start = Filled(40, 0.5);
            var dense = new MmaOptimizer(options).Minimize(MmaTestProblems.LocalWindows(40, 16, 6, sparse: false), start, CancellationToken.None);
            var sparse = new MmaOptimizer(options).Minimize(MmaTestProblems.LocalWindows(40, 16, 6, sparse: true), start, CancellationToken.None);

            TestContext.WriteLine($"dense: f={dense.OptimalValue:R} iterations={dense.Iterations}; sparse: f={sparse.OptimalValue:R} iterations={sparse.Iterations}");
            Assert.IsTrue(dense.Success, dense.Message);
            Assert.IsTrue(sparse.Success, sparse.Message);
            Assert.AreEqual(dense.OptimalValue, sparse.OptimalValue, 1e-8 * Math.Abs(dense.OptimalValue));
            for (var j = 0; j < 40; j++)
            {
                Assert.AreEqual(dense.OptimalPoint[j], sparse.OptimalPoint[j], 1e-6, $"x{j}");
            }

            for (var i = 0; i < 16; i++)
            {
                Assert.AreEqual(dense.Multipliers[i], sparse.Multipliers[i], 1e-5, $"λ{i}");
            }
        }

        [TestMethod]
        public void FullPatternCsrMatchesDenseRows()
        {
            // A CSR block whose pattern holds every column is the dense block written another way: the same
            // approximation, so the same iterates up to rounding.
            var problem = MmaTestProblems.Cantilever();
            var dense = problem.Constraints[0];
            var full = new ConstraintBlock(
                ConstraintSparsity.Csr(5, [0, 5], [0, 1, 2, 3, 4]),
                dense.Evaluate);
            var options = new MmaOptions { Tolerance = 1e-9, MaxIterations = 100 };
            var fromDense = new MmaOptimizer(options).Minimize(problem, s_cantileverStart, CancellationToken.None);
            var fromCsr = new MmaOptimizer(options).Minimize(problem with { Constraints = [full] }, s_cantileverStart, CancellationToken.None);

            Assert.AreEqual(fromDense.Iterations, fromCsr.Iterations);
            for (var j = 0; j < 5; j++)
            {
                Assert.AreEqual(fromDense.OptimalPoint[j], fromCsr.OptimalPoint[j], 1e-12, $"x{j + 1}");
            }
        }

        [TestMethod]
        public void ResultsAreDeterministic()
        {
            var options = new MmaOptions { Tolerance = 1e-8, MaxIterations = 300, LinearSolver = MmaLinearSolver.ConjugateGradient };
            var start = Filled(60, 0.5);
            var first = new MmaOptimizer(options).Minimize(MmaTestProblems.LocalWindows(60, 20, 7, sparse: true), start, CancellationToken.None);
            var second = new MmaOptimizer(options).Minimize(MmaTestProblems.LocalWindows(60, 20, 7, sparse: true), start, CancellationToken.None);

            Assert.AreEqual(first.Iterations, second.Iterations);
            Assert.AreEqual(BitConverter.DoubleToInt64Bits(first.OptimalValue), BitConverter.DoubleToInt64Bits(second.OptimalValue));
            for (var j = 0; j < 60; j++)
            {
                Assert.AreEqual(BitConverter.DoubleToInt64Bits(first.OptimalPoint[j]), BitConverter.DoubleToInt64Bits(second.OptimalPoint[j]), $"x{j}");
            }
        }

        [TestMethod]
        public void MinMaxFormWithConstraintAReachesTheMiddle()
        {
            // minimize max((x - 1)², (x + 1)²) as Svanberg's min-max form: f0 = 0, g_i = f_i(x) with a_i = 1, so z is
            // the bound on both; the optimum is x = 0 with z = 1.
            var block = new ConstraintBlock(ConstraintSparsity.Dense(2, 1), (x, values, jacobian) =>
            {
                values[0] = (x[0] - 1.0) * (x[0] - 1.0);
                values[1] = (x[0] + 1.0) * (x[0] + 1.0);
                if (!jacobian.IsEmpty)
                {
                    jacobian[0] = 2.0 * (x[0] - 1.0);
                    jacobian[1] = 2.0 * (x[0] + 1.0);
                }
            });
            var problem = new InequalityProblem
            {
                Objective = (_, gradient) =>
                {
                    gradient.Clear();
                    return 0.0;
                },
                Constraints = [block],
                LowerBounds = [-3.0],
                UpperBounds = [2.0],
            };
            var result = new MmaOptimizer(new MmaOptions { Tolerance = 1e-8, SubproblemTolerance = 1e-10, MaxIterations = 200, ConstraintA = [1.0, 1.0] })
                .Minimize(problem, [1.5], CancellationToken.None);

            Assert.IsTrue(result.Success, result.Message);
            Assert.AreEqual(0.0, result.OptimalPoint[0], 1e-5);
            Assert.AreEqual(1.0, result.ConstraintValues[0], 1e-5);
        }

        [TestMethod]
        [DataRow(MmaLinearSolver.Direct)]
        [DataRow(MmaLinearSolver.ConjugateGradient)]
        public void MinMaxDistanceReachesTheMidpointInBothReducedSpaces(MmaLinearSolver solver)
        {
            // minimize max(|x - p|², |x - q|²) over x in R³ (m = 2 < n = 3, the m×m system) and over x in R (m = 2 >= n = 1,
            // the n×n system): the optimum is the midpoint, with z = |p - q|²/4.
            foreach (var n in new[] { 3, 1 })
            {
                var p = new double[n];
                var q = new double[n];
                for (var j = 0; j < n; j++)
                {
                    p[j] = 1.0 + j;
                    q[j] = -1.0 + (0.5 * j);
                }

                var block = new ConstraintBlock(ConstraintSparsity.Dense(2, n), (x, values, jacobian) =>
                {
                    values[0] = 0.0;
                    values[1] = 0.0;
                    for (var j = 0; j < n; j++)
                    {
                        values[0] += (x[j] - p[j]) * (x[j] - p[j]);
                        values[1] += (x[j] - q[j]) * (x[j] - q[j]);
                        if (!jacobian.IsEmpty)
                        {
                            jacobian[j] = 2.0 * (x[j] - p[j]);
                            jacobian[n + j] = 2.0 * (x[j] - q[j]);
                        }
                    }
                });
                var lower = new double[n];
                var upper = new double[n];
                Array.Fill(lower, -5.0);
                Array.Fill(upper, 5.0);
                var problem = new InequalityProblem
                {
                    Objective = (_, gradient) =>
                    {
                        gradient.Clear();
                        return 0.0;
                    },
                    Constraints = [block],
                    LowerBounds = lower,
                    UpperBounds = upper,
                };
                var options = new MmaOptions { Tolerance = 1e-8, SubproblemTolerance = 1e-10, MaxIterations = 300, ConstraintA = [1.0, 1.0], LinearSolver = solver };
                var result = new MmaOptimizer(options).Minimize(problem, Filled(n, 2.5), CancellationToken.None);

                Assert.IsTrue(result.Success, $"n = {n}: {result.Message}");
                var distance = 0.0;
                for (var j = 0; j < n; j++)
                {
                    Assert.AreEqual(0.5 * (p[j] + q[j]), result.OptimalPoint[j], 1e-5, $"n = {n}, x{j}");
                    distance += (p[j] - q[j]) * (p[j] - q[j]);
                }

                Assert.AreEqual(distance / 4.0, result.ConstraintValues[0], 1e-5, $"n = {n}");
            }
        }

        [TestMethod]
        public void ProgressCallbackReportsEveryIteration()
        {
            var reported = new List<InequalityIteration>();
            var result = new MmaOptimizer(new MmaOptions { Tolerance = 1e-8, ProgressCallback = reported.Add })
                .Minimize(MmaTestProblems.Cantilever(), s_cantileverStart, CancellationToken.None);

            Assert.HasCount(result.Iterations, reported);
            for (var k = 0; k < reported.Count; k++)
            {
                Assert.AreEqual(k + 1, reported[k].Iteration);
                Assert.IsGreaterThan(0, reported[k].SubproblemIterations);
            }

            Assert.AreEqual(result.KktResidualNorm, reported[^1].KktResidualNorm);
        }

        [TestMethod]
        public void CancellationStopsTheRun()
        {
            using var source = new CancellationTokenSource();
            var iterations = 0;
            var options = new MmaOptions
            {
                Tolerance = 0.0,
                MaxIterations = 1000,
                ProgressCallback = progress =>
                {
                    iterations = progress.Iteration;
                    if (progress.Iteration == 3)
                    {
                        source.Cancel();
                    }
                },
            };
            var result = new MmaOptimizer(options).Minimize(MmaTestProblems.Cantilever(), s_cantileverStart, source.Token);

            Assert.AreEqual(StoppingReason.UserRequested, result.StoppingReason);
            Assert.IsFalse(result.Success);
            Assert.AreEqual(3, result.Iterations);
            Assert.AreEqual(3, iterations);
        }

        [TestMethod]
        public void NonFiniteObjectiveIsANumericalError()
        {
            var problem = MmaTestProblems.Cantilever() with { Objective = (_, _) => double.NaN };
            var result = new MmaOptimizer(new MmaOptions()).Minimize(problem, s_cantileverStart, CancellationToken.None);

            Assert.AreEqual(StoppingReason.NumericalError, result.StoppingReason);
            Assert.IsFalse(result.Success);
        }

        [TestMethod]
        public void RejectsMismatchedArtificialCoefficients()
        {
            var optimizer = new MmaOptimizer(new MmaOptions { ConstraintC = [1.0, 2.0] });

            Assert.ThrowsExactly<ArgumentException>(() => optimizer.Minimize(MmaTestProblems.Cantilever(), s_cantileverStart, CancellationToken.None));
        }

        [TestMethod]
        public void RejectsABlockOfTheWrongWidth()
        {
            var problem = MmaTestProblems.Cantilever() with { Constraints = [new LinearConstraintBlock([1.0, 1.0], 1.0)] };

            Assert.ThrowsExactly<ArgumentException>(() => new MmaOptimizer(new MmaOptions()).Minimize(problem, s_cantileverStart, CancellationToken.None));
        }

        private static InequalityOptimizerResult Cantilever(MmaLinearSolver solver)
        {
            return new MmaOptimizer(new MmaOptions { Tolerance = 1e-8, SubproblemTolerance = 1e-10, MaxIterations = 200, LinearSolver = solver })
                .Minimize(MmaTestProblems.Cantilever(), s_cantileverStart, CancellationToken.None);
        }

        private static double[] Filled(int n, double value)
        {
            var x = new double[n];
            Array.Fill(x, value);
            return x;
        }
    }
}
