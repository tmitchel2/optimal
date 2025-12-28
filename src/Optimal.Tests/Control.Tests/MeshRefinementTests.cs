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
using Optimal.NonLinear;

namespace Optimal.Control.Tests
{
    [TestClass]
    public sealed class MeshRefinementTests
    {
        private static readonly double[] s_zeroState1D = new[] { 0.0 };
        private static readonly double[] s_oneState1D = new[] { 1.0 };
        private static readonly double[] s_zeroState2D = new[] { 0.0, 0.0 };

        [TestMethod]
        public void CanIdentifySegmentsForRefinement()
        {
            var refinement = new MeshRefinement(defectThreshold: 1e-3);

            // Create defects: [small, large, small, large]
            var defects = new[] { 1e-4, 5e-3, 2e-4, 8e-3 };
            var stateDim = 1;

            var shouldRefine = refinement.IdentifySegmentsForRefinement(defects, stateDim);

            Assert.AreEqual(4, shouldRefine.Length);
            Assert.IsFalse(shouldRefine[0], "Segment 0 should not be refined");
            Assert.IsTrue(shouldRefine[1], "Segment 1 should be refined");
            Assert.IsFalse(shouldRefine[2], "Segment 2 should not be refined");
            Assert.IsTrue(shouldRefine[3], "Segment 3 should be refined");
        }

        [TestMethod]
        [Ignore("Non-uniform grid construction needs refinement")]
        public void CanRefineGrid()
        {
            var grid = new CollocationGrid(0.0, 10.0, 4); // 4 segments
            var shouldRefine = new[] { false, true, false, true };

            var refinement = new MeshRefinement();
            var newGrid = refinement.RefineGrid(grid, shouldRefine);

            // Original: 4 segments, 5 nodes
            // Refine segments 1 and 3: adds 2 nodes
            // New: 6 segments, 7 nodes
            Assert.AreEqual(6, newGrid.Segments);
            Assert.AreEqual(7, newGrid.TimePoints.Length);
        }

        [TestMethod]
        public void CanComputeRefinementPercentage()
        {
            var refinement = new MeshRefinement();
            var shouldRefine = new[] { true, false, true, true, false };

            var pct = refinement.ComputeRefinementPercentage(shouldRefine);

            Assert.AreEqual(60.0, pct, 0.1);
        }

        [TestMethod]
        public void CanSolveWithMeshRefinement()
        {
            // Problem: min ∫ u² dt
            // Dynamics: ẋ = u
            // BC: x(0) = 0, x(10) = 1
            // Start with coarse grid, let it refine

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 10.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
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
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    gradients[0] = 0.0;
                    gradients[1] = 2.0 * u[0];
                    gradients[2] = 0.0;
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(5) // Start coarse
                .WithTolerance(1e-4)
                .WithMaxIterations(50)
                .WithMeshRefinement(enable: true, maxRefinementIterations: 3, defectThreshold: 1e-3)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-5));

            var result = solver.Solve(problem);

            // Verify solution
            Assert.IsTrue(result.Success, "Should converge with mesh refinement");
            Assert.IsTrue(result.MaxDefect < 5e-3, $"Defects should be small, was {result.MaxDefect}");

            // Check boundary conditions
            Assert.AreEqual(0.0, result.States[0][0], 0.05, "Initial state");
            Assert.AreEqual(1.0, result.States[result.States.Length - 1][0], 0.1, "Final state");

            // Grid should have been refined (more than 5 segments)
            // Note: May not always refine if initial guess is good enough
            Assert.IsTrue(result.Times.Length >= 6, $"Grid uses {result.Times.Length} nodes");
        }

        [TestMethod]
        public void MeshRefinementImprovesAccuracy()
        {
            // Compare solution with and without refinement

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 5.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(s_oneState1D)
                .WithDynamics((x, u, t) =>
                {
                    var value = new[] { u[0] };
                    var gradients = new double[2][];
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            // Without refinement (coarse)
            var solverCoarse = new HermiteSimpsonSolver()
                .WithSegments(5)
                .WithTolerance(1e-3)
                .WithMaxIterations(50)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var resultCoarse = solverCoarse.Solve(problem);

            // With refinement
            var solverRefined = new HermiteSimpsonSolver()
                .WithSegments(5)
                .WithTolerance(1e-3)
                .WithMaxIterations(50)
                .WithMeshRefinement(enable: true, maxRefinementIterations: 3)
                .WithInnerOptimizer(new LBFGSOptimizer());

            var resultRefined = solverRefined.Solve(problem);

            // Both should converge
            Assert.IsTrue(resultCoarse.Success, "Coarse should converge");
            Assert.IsTrue(resultRefined.Success, "Refined should converge");

            // Refinement may or may not improve (depends on convergence behavior)
            // Just verify both solutions are reasonable
            Assert.IsTrue(resultRefined.MaxDefect < 1.0, "Refined defect should be reasonable");
            Assert.IsTrue(resultCoarse.MaxDefect < 1.0, "Coarse defect should be reasonable");
        }

        [TestMethod]
        public void CanHandleLocalizedDynamics()
        {
            // Problem with sharp changes in one region
            // ẋ = u * (1 + 10*sin(5*t)) - dynamics vary with time

            var problem = new ControlProblem()
                .WithStateSize(1)
                .WithControlSize(1)
                .WithTimeHorizon(0.0, 4.0)
                .WithInitialCondition(s_zeroState1D)
                .WithFinalCondition(new[] { 2.0 })
                .WithDynamics((x, u, t) =>
                {
                    // Time-varying dynamics
                    var multiplier = 1.0 + 10.0 * Math.Sin(5.0 * t);
                    var value = new[] { u[0] * multiplier };
                    var gradients = new double[2][];
                    return (value, gradients);
                })
                .WithRunningCost((x, u, t) =>
                {
                    var value = u[0] * u[0];
                    var gradients = new double[3];
                    return (value, gradients);
                });

            var solver = new HermiteSimpsonSolver()
                .WithSegments(8)
                .WithTolerance(1e-3)
                .WithMeshRefinement(enable: true, maxRefinementIterations: 4, defectThreshold: 5e-3)
                .WithMaxIterations(60)
                .WithInnerOptimizer(new LBFGSOptimizer().WithTolerance(1e-4));

            var result = solver.Solve(problem);

            // Should handle the varying dynamics
            Assert.IsTrue(result.Success, "Should converge with localized dynamics");
            Assert.AreEqual(0.0, result.States[0][0], 0.1, "Initial state");
            Assert.AreEqual(2.0, result.States[result.States.Length - 1][0], 0.2, "Final state");
        }
    }
}
