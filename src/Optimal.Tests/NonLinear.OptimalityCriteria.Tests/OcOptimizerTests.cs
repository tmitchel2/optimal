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

namespace Optimal.NonLinear.OptimalityCriteria.Tests
{
    [TestClass]
    public sealed class OcOptimizerTests
    {
        private const int N = 12;

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void SeparableComplianceReachesAnalyticOptimumWithExpectedBisections()
        {
            // minimize Σ c_j/x_j subject to Σ v_j x_j <= V: stationarity gives x_j = √(c_j/(λ v_j)), so
            // x*_j = V √(c_j/v_j) / Σ_k √(c_k v_k) and √λ* = Σ_k √(c_k v_k) / V. With η = 1/2 the OC update is
            // x_j √(c_j/(x_j² λ v_j)) = √(c_j/(λ v_j)), independent of x, and the explicit bracket's upper end
            // Σ v_j P_j / V is √λ* itself. Bisection from [0, √λ*] then always moves the lower end, so after k steps
            // (u - l)/(u + l) = 1/(2^(k+1) - 1): the stop at 1e-4 takes k = 13, at 1e-10 k = 33.
            foreach (var (tolerance, expected) in new[] { (1e-4, 13), (1e-10, 33) })
            {
                var (c, v, limit) = Data(1.0);
                var history = new List<InequalityIteration>();
                var options = new OcOptions { MoveLimit = 1.0, BisectionTolerance = tolerance, ChangeTolerance = 1e-12, MaxIterations = 20, ProgressCallback = history.Add };
                var result = new OcOptimizer(options).Minimize(Problem(c, v, limit, 1e-3, 1.0), Filled(limit / Sum(v)), CancellationToken.None);

                var sum = 0.0;
                for (var j = 0; j < N; j++)
                {
                    sum += Math.Sqrt(c[j] * v[j]);
                }

                TestContext.WriteLine($"tolerance {tolerance:E0}: bisections {history[0].SubproblemIterations}, iterations {result.Iterations}, λ={result.Multipliers[0]:R} (analytic {sum * sum / (limit * limit):R})");
                Assert.IsTrue(result.Success, result.Message);
                Assert.AreEqual(expected, history[0].SubproblemIterations, $"bisections at tolerance {tolerance}");
                var relative = 1.5 * Math.Pow(2.0, -expected);
                for (var j = 0; j < N; j++)
                {
                    var analytic = limit * Math.Sqrt(c[j] / v[j]) / sum;
                    Assert.AreEqual(analytic, result.OptimalPoint[j], relative * analytic, $"x{j}");
                }

                var lambda = sum * sum / (limit * limit);
                Assert.AreEqual(lambda, result.Multipliers[0], 3.0 * relative * lambda);
            }
        }

        [TestMethod]
        public void ActiveBoundsMatchTheClampedAnalyticOptimum()
        {
            // With strongly varying c some variables sit at their bounds: x_j(λ) = clamp(√(c_j/(λ v_j)), lower, upper)
            // with λ the root of Σ v_j x_j(λ) = V. OC with a 0.2 move limit reaches it once the limits stop binding.
            var (c, v, limit) = Data(40.0);
            var options = new OcOptions { BisectionTolerance = 1e-13, ChangeTolerance = 1e-12, MaxIterations = 200 };
            var result = new OcOptimizer(options).Minimize(Problem(c, v, limit, 0.25, 0.55), Filled(limit / Sum(v)), CancellationToken.None);
            var lambda = ClampedRoot(c, v, limit, 0.25, 0.55);

            var atLower = 0;
            var atUpper = 0;
            for (var j = 0; j < N; j++)
            {
                var expected = Math.Clamp(Math.Sqrt(c[j] / (lambda * v[j])), 0.25, 0.55);
                atLower += expected == 0.25 ? 1 : 0;
                atUpper += expected == 0.55 ? 1 : 0;
                Assert.AreEqual(expected, result.OptimalPoint[j], 1e-9, $"x{j}");
            }

            TestContext.WriteLine($"iterations {result.Iterations}, {atLower} at the lower bound, {atUpper} at the upper, kkt {result.KktResidualMax:E3}");
            Assert.IsTrue(result.Success, result.Message);
            Assert.IsGreaterThan(0, atLower);
            Assert.IsGreaterThan(0, atUpper);
            Assert.AreEqual(lambda, result.Multipliers[0], 1e-8 * lambda);
            Assert.IsLessThanOrEqualTo(1e-9, result.KktResidualMax);
        }

        [TestMethod]
        public void InactiveConstraintSendsEveryVariableToItsUpperLimit()
        {
            var (c, v, _) = Data(1.0);
            var result = new OcOptimizer(new OcOptions { MoveLimit = 1.0 }).Minimize(Problem(c, v, 1e6, 1e-3, 1.0), Filled(0.5), CancellationToken.None);

            Assert.IsTrue(result.Success, result.Message);
            Assert.AreEqual(0.0, result.Multipliers[0]);
            foreach (var x in result.OptimalPoint)
            {
                Assert.AreEqual(1.0, x);
            }
        }

        [TestMethod]
        public void RejectsMoreThanOneConstraint()
        {
            var (c, v, limit) = Data(1.0);
            var problem = Problem(c, v, limit, 1e-3, 1.0);
            var twice = problem with { Constraints = [problem.Constraints[0], problem.Constraints[0]] };

            Assert.ThrowsExactly<ArgumentException>(() => new OcOptimizer(new OcOptions()).Minimize(twice, Filled(0.5), CancellationToken.None));
        }

        [TestMethod]
        public void RejectsNegativeLowerBounds()
        {
            var (c, v, limit) = Data(1.0);

            Assert.ThrowsExactly<ArgumentException>(() => new OcOptimizer(new OcOptions()).Minimize(Problem(c, v, limit, -1.0, 1.0), Filled(0.5), CancellationToken.None));
        }

        [TestMethod]
        public void CancellationStopsTheRun()
        {
            var (c, v, limit) = Data(40.0);
            using var source = new CancellationTokenSource();
            source.Cancel();
            var result = new OcOptimizer(new OcOptions()).Minimize(Problem(c, v, limit, 0.05, 1.0), Filled(0.5), source.Token);

            Assert.AreEqual(StoppingReason.UserRequested, result.StoppingReason);
            Assert.AreEqual(0, result.Iterations);
        }

        private static (double[] C, double[] V, double Limit) Data(double spread)
        {
            var c = new double[N];
            var v = new double[N];
            for (var j = 0; j < N; j++)
            {
                c[j] = 1.0 + (spread * j * j / (N * N)) + (0.3 * j);
                v[j] = 1.0 + (0.1 * j);
            }

            return (c, v, 0.4 * Sum(v));
        }

        private static InequalityProblem Problem(double[] c, double[] v, double limit, double lower, double upper)
        {
            var lowerBounds = new double[N];
            var upperBounds = new double[N];
            Array.Fill(lowerBounds, lower);
            Array.Fill(upperBounds, upper);
            return new InequalityProblem
            {
                Objective = (x, gradient) =>
                {
                    var value = 0.0;
                    for (var j = 0; j < x.Length; j++)
                    {
                        value += c[j] / x[j];
                        if (!gradient.IsEmpty)
                        {
                            gradient[j] = -c[j] / (x[j] * x[j]);
                        }
                    }

                    return value;
                },
                Constraints = [new LinearConstraintBlock(v, limit)],
                LowerBounds = lowerBounds,
                UpperBounds = upperBounds,
            };
        }

        private static double ClampedRoot(double[] c, double[] v, double limit, double lower, double upper)
        {
            double Volume(double lambda)
            {
                var volume = 0.0;
                for (var j = 0; j < N; j++)
                {
                    volume += v[j] * Math.Clamp(Math.Sqrt(c[j] / (lambda * v[j])), lower, upper);
                }

                return volume;
            }

            var low = 1e-12;
            var high = 1e12;
            for (var k = 0; k < 400; k++)
            {
                var middle = Math.Sqrt(low * high);
                if (Volume(middle) > limit)
                {
                    low = middle;
                }
                else
                {
                    high = middle;
                }
            }

            return Math.Sqrt(low * high);
        }

        private static double Sum(double[] values)
        {
            var sum = 0.0;
            foreach (var value in values)
            {
                sum += value;
            }

            return sum;
        }

        private static double[] Filled(double value)
        {
            var x = new double[N];
            Array.Fill(x, value);
            return x;
        }
    }
}
