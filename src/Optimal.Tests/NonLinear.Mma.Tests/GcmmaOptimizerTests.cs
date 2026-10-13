/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.NonLinear.Mma.Tests
{
    [TestClass]
    public sealed class GcmmaOptimizerTests
    {
        private static readonly double[] s_rosenbrockStart = [0.5, 1.0];
        private static readonly double[] s_cantileverStart = [5.0, 5.0, 5.0, 5.0, 5.0];

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void NonconvexProblemIsMonotoneAndFeasible()
        {
            // Rosenbrock's function constrained by a cubic and a line, from the feasible point (0.5, 1): GCMMA's
            // conservative approximations (Svanberg 2002, 2007) make every outer iterate feasible and the objective
            // non-increasing, up to the subproblem's tolerance.
            var problem = MmaTestProblems.ConstrainedRosenbrock();
            var history = new List<InequalityIteration>();
            var options = new GcmmaOptions { Tolerance = 1e-3, SubproblemTolerance = 1e-9, ConservativeTolerance = 1e-9, MaxIterations = 500, ProgressCallback = history.Add };
            var result = new GcmmaOptimizer(options).Minimize(problem, s_rosenbrockStart, CancellationToken.None);
            var f0 = Objective(problem, s_rosenbrockStart);

            var objectives = new[] { f0 }.Concat(history.Select(h => h.ObjectiveValue)).ToArray();
            var largestRise = Enumerable.Range(1, objectives.Length - 1).Max(k => objectives[k] - objectives[k - 1]);
            var largestViolation = history.Max(h => h.MaxConstraintViolation);
            TestContext.WriteLine($"GCMMA: f={result.OptimalValue:E3} x=[{string.Join(", ", result.OptimalPoint)}] outer={result.Iterations} inner={result.InnerIterations} evaluations={result.FunctionEvaluations} largest rise={largestRise:E3} largest violation={largestViolation:E3} non-conservative={result.NonConservativeIterations}");
            Assert.IsTrue(result.Success, result.Message);
            Assert.AreEqual(0, result.NonConservativeIterations);
            Assert.IsLessThanOrEqualTo(1e-9, largestRise, "the objective rose between outer iterates");
            Assert.IsLessThanOrEqualTo(1e-9, largestViolation, "an outer iterate was infeasible");
            Assert.AreEqual(1.0, result.OptimalPoint[0], 1e-4);
            Assert.AreEqual(1.0, result.OptimalPoint[1], 1e-4);
            Assert.AreEqual(0.0, result.OptimalValue, 1e-7);
            Assert.IsGreaterThan(0, result.InnerIterations, "the problem should need inner iterations, or it does not exercise them");
        }

        [TestMethod]
        public void CantileverReachesPublishedOptimumMonotonically()
        {
            var history = new List<InequalityIteration>();
            var options = new GcmmaOptions { Tolerance = 1e-8, SubproblemTolerance = 1e-10, ConservativeTolerance = 0.0, MaxIterations = 200, ProgressCallback = history.Add };
            var result = new GcmmaOptimizer(options).Minimize(MmaTestProblems.Cantilever(), s_cantileverStart, CancellationToken.None);

            TestContext.WriteLine($"GCMMA cantilever: f={result.OptimalValue:R} outer={result.Iterations} inner={result.InnerIterations}");
            Assert.IsTrue(result.Success, result.Message);
            Assert.AreEqual(1.340, Math.Round(result.OptimalValue, 3), 1e-12);
            for (var k = 1; k < history.Count; k++)
            {
                Assert.IsLessThanOrEqualTo(history[k - 1].ObjectiveValue + 1e-9, history[k].ObjectiveValue, $"iteration {k + 1}");
                Assert.IsLessThanOrEqualTo(1e-10, history[k].MaxConstraintViolation, $"iteration {k + 1}");
            }
        }

        [TestMethod]
        public void InnerIterationsEvaluateWithoutGradients()
        {
            var problem = MmaTestProblems.ConstrainedRosenbrock();
            var withGradient = 0;
            var withoutGradient = 0;
            var counted = problem with
            {
                Objective = (x, gradient) =>
                {
                    _ = gradient.IsEmpty ? withoutGradient++ : withGradient++;
                    return problem.Objective(x, gradient);
                },
            };
            var result = new GcmmaOptimizer(new GcmmaOptions { MaxIterations = 50 }).Minimize(counted, s_rosenbrockStart, CancellationToken.None);

            Assert.AreEqual(result.FunctionEvaluations, withGradient + withoutGradient);
            Assert.AreEqual(result.Iterations + result.InnerIterations, withoutGradient);
            Assert.AreEqual(result.Iterations + 1, withGradient);
        }

        private static double Objective(InequalityProblem problem, double[] x)
        {
            return problem.Objective(x, Span<double>.Empty);
        }
    }
}
