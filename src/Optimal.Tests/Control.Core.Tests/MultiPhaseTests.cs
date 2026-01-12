/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Solvers;
using Optimal.NonLinear.Unconstrained;

namespace Optimal.Control.Core.Tests
{
    [TestClass]
    public sealed class MultiPhaseTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_twoState1D = new[] { 2.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };

        [TestMethod]
        public void CanCreateMultiPhaseProblem()
        {
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = new ControlPhase
            {
                Name = "Ascent",
                Duration = 5.0,
                Segments = 10,
                Problem = new ControlProblem()
                    .WithStateSize(1)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 5.0)
                    .WithInitialCondition(s_zeroState1D)
                    .WithFinalCondition(s_oneState1D)
                    .WithDynamics((x, u, t) => (new[] { u[0] }, new double[2][]))
                    .WithRunningCost((x, u, t) => (u[0] * u[0], new double[3]))
            };

            multiPhase.AddPhase(phase1);

            Assert.AreEqual(1, multiPhase.Phases.Count);
            Assert.AreEqual("Ascent", multiPhase.Phases[0].Name);
        }

        [TestMethod]
        public void CanAddContinuityLinkage()
        {
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = CreateSimplePhase("Phase1", 0.0, 3.0, s_zeroState1D, s_oneState1D);
            var phase2 = CreateSimplePhase("Phase2", 3.0, 6.0, s_oneState1D, s_twoState1D);

            multiPhase.AddPhase(phase1).AddPhase(phase2);
            multiPhase.AddContinuityLinkage(0, 1);

            Assert.AreEqual(2, multiPhase.Phases.Count);
            Assert.AreEqual(1, multiPhase.Linkages.Count);
        }

        [TestMethod]
        public void CanSolveTwoPhaseSequential()
        {
            // Two-phase problem: go from 0 to 1 in phase 1, then 1 to 2 in phase 2
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = CreateSimplePhase("Boost", 0.0, 3.0, s_zeroState1D, s_oneState1D);
            var phase2 = CreateSimplePhase("Coast", 3.0, 6.0, s_oneState1D, s_twoState1D);

            multiPhase.AddPhase(phase1)
                     .AddPhase(phase2)
                     .AddContinuityLinkage(0, 1);

            var baseSolver = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var multiSolver = new MultiPhaseSolver(baseSolver);

            var result = multiSolver.Solve(multiPhase);

            Assert.IsTrue(result.Success, "Should solve two-phase problem");
            Assert.AreEqual(2, result.PhaseResults.Length);
            Assert.IsTrue(result.PhaseResults[0].Success, "Phase 1 should succeed");
            Assert.IsTrue(result.PhaseResults[1].Success, "Phase 2 should succeed");

            // Check continuity
            var phase1Final = result.PhaseResults[0].States[result.PhaseResults[0].States.Length - 1];
            var phase2Initial = result.PhaseResults[1].States[0];

            Assert.AreEqual(phase1Final[0], phase2Initial[0], 0.2, "States should be continuous");
        }

        [TestMethod]
        public void CanSolveThreePhaseSequential()
        {
            // Three-phase problem
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = CreateSimplePhase("Phase1", 0.0, 2.0, s_zeroState1D, new[] { 0.5 });
            var phase2 = CreateSimplePhase("Phase2", 2.0, 4.0, new[] { 0.5 }, s_oneState1D);
            var phase3 = CreateSimplePhase("Phase3", 4.0, 6.0, s_oneState1D, new[] { 1.5 });

            multiPhase.AddPhase(phase1)
                     .AddPhase(phase2)
                     .AddPhase(phase3)
                     .AddContinuityLinkage(0, 1)
                     .AddContinuityLinkage(1, 2);

            var baseSolver = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var multiSolver = new MultiPhaseSolver(baseSolver);

            var result = multiSolver.Solve(multiPhase);

            Assert.IsTrue(result.Success, "Should solve three-phase problem");
            Assert.AreEqual(3, result.PhaseResults.Length);

            // Verify all phases succeeded
            for (var i = 0; i < 3; i++)
            {
                Assert.IsTrue(result.PhaseResults[i].Success, $"Phase {i + 1} should succeed");
            }
        }

        [TestMethod]
        public void MultiPhaseComputesTotalCost()
        {
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = CreateSimplePhase("Phase1", 0.0, 2.0, s_zeroState1D, s_oneState1D);
            var phase2 = CreateSimplePhase("Phase2", 2.0, 4.0, s_oneState1D, s_twoState1D);

            multiPhase.AddPhase(phase1).AddPhase(phase2);

            var baseSolver = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var multiSolver = new MultiPhaseSolver(baseSolver);

            var result = multiSolver.Solve(multiPhase);

            Assert.IsTrue(result.Success);

            // Total cost should be sum of phase costs
            var expectedTotal = result.PhaseResults[0].OptimalCost + result.PhaseResults[1].OptimalCost;
            Assert.AreEqual(expectedTotal, result.TotalCost, 1e-6, "Total cost should be sum");
        }

        [TestMethod]
        [Ignore("Long running test - enable for full validation")]
        public void CanSolveDoubleIntegratorTwoPhase()
        {
            // Two-phase double integrator: acceleration then braking
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = new ControlPhase
            {
                Name = "Acceleration",
                Duration = 1.5,
                Segments = 10,
                Problem = new ControlProblem()
                    .WithStateSize(2)
                    .WithControlSize(1)
                    .WithTimeHorizon(0.0, 1.5)
                    .WithInitialCondition(s_zeroState2D)
                    .WithFinalCondition(new[] { 0.5, 0.5 }) // Some position and velocity
                    .WithDynamics((x, u, t) =>
                    {
                        var value = new[] { x[1], u[0] };
                        var gradients = new double[2][];
                        gradients[0] = new[] { 0.0, 1.0 };
                        gradients[1] = new[] { 1.0, 0.0 };
                        return (value, gradients);
                    })
                    .WithRunningCost((x, u, t) =>
                    {
                        var value = 0.5 * u[0] * u[0];
                        var gradients = new double[3];
                        gradients[0] = 0.0;
                        gradients[1] = 0.0;
                        gradients[2] = u[0];
                        return (value, gradients);
                    })
            };

            var phase2 = new ControlPhase
            {
                Name = "Braking",
                Duration = 1.5,
                Segments = 10,
                Problem = new ControlProblem()
                    .WithStateSize(2)
                    .WithControlSize(1)
                    .WithTimeHorizon(1.5, 3.0)
                    .WithInitialCondition(new[] { 0.5, 0.5 })
                    .WithFinalCondition(new[] { 1.0, 0.0 }) // Stop at position 1
                    .WithDynamics((x, u, t) =>
                    {
                        var value = new[] { x[1], u[0] };
                        var gradients = new double[2][];
                        gradients[0] = new[] { 0.0, 1.0 };
                        gradients[1] = new[] { 1.0, 0.0 };
                        return (value, gradients);
                    })
                    .WithRunningCost((x, u, t) =>
                    {
                        var value = 0.5 * u[0] * u[0];
                        var gradients = new double[3];
                        gradients[0] = 0.0;
                        gradients[1] = 0.0;
                        gradients[2] = u[0];
                        return (value, gradients);
                    })
            };

            multiPhase.AddPhase(phase1).AddPhase(phase2);
            multiPhase.AddContinuityLinkage(0, 1);

            var baseSolver = new HermiteSimpsonSolver()
                .WithSegments(12)
                .WithTolerance(1e-3)
                .WithMaxIterations(60)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-4));

            var multiSolver = new MultiPhaseSolver(baseSolver);

            var result = multiSolver.Solve(multiPhase);

            Assert.IsTrue(result.Success, "Should solve acceleration-braking problem");
            Assert.IsTrue(result.MaxLinkageViolation < 0.3, $"Linkage violation should be small, was {result.MaxLinkageViolation}");
        }

        [TestMethod]
        public void ValidateThrowsForEmptyProblem()
        {
            var multiPhase = new MultiPhaseControlProblem();

            Assert.ThrowsException<InvalidOperationException>(() => multiPhase.Validate());
        }

        [TestMethod]
        public void ValidateThrowsForPhaseWithoutProblem()
        {
            var multiPhase = new MultiPhaseControlProblem();

            var phase = new ControlPhase
            {
                Name = "Invalid",
                Duration = 5.0
                // No Problem defined
            };

            multiPhase.AddPhase(phase);

            Assert.ThrowsException<InvalidOperationException>(() => multiPhase.Validate());
        }

        [TestMethod]
        public void AddLinkageValidatesPhaseIndices()
        {
            var multiPhase = new MultiPhaseControlProblem();

            var phase1 = CreateSimplePhase("Phase1", 0.0, 2.0, s_zeroState1D, s_oneState1D);
            multiPhase.AddPhase(phase1);

            var linkage = new PhaseLinkage
            {
                Phase1Index = 0,
                Phase2Index = 1, // Invalid: phase 1 doesn't exist
                Type = PhaseLinkageType.Equality
            };

            Assert.ThrowsException<ArgumentException>(() => multiPhase.AddLinkage(linkage));
        }

        /// <summary>
        /// Helper to create a simple single integrator phase.
        /// </summary>
        private static ControlPhase CreateSimplePhase(
            string name,
            double t0,
            double tf,
            double[] initialState,
            double[] finalState)
        {
            return new ControlPhase
            {
                Name = name,
                Duration = tf - t0,
                Segments = 8,
                Problem = new ControlProblem()
                    .WithStateSize(1)
                    .WithControlSize(1)
                    .WithTimeHorizon(t0, tf)
                    .WithInitialCondition(initialState)
                    .WithFinalCondition(finalState)
                    .WithDynamics((x, u, t) =>
                    {
                        var value = new[] { u[0] };
                        var gradients = new double[2][];
                        gradients[0] = new[] { 0.0 };
                        gradients[1] = new[] { 1.0 };
                        return (value, gradients);
                    })
                    .WithRunningCost((x, u, t) =>
                    {
                        var value = 0.5 * u[0] * u[0];
                        var gradients = new double[3];
                        gradients[0] = 0.0;
                        gradients[1] = u[0];
                        gradients[2] = 0.0;
                        return (value, gradients);
                    })
            };
        }
    }
}
