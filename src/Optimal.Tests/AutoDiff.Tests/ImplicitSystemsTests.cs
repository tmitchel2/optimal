/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// A three-unknown implicit solve: its run-time solution, its generated derivative against the implicit-function
    /// theorem worked by hand, and the Hessian of a scalar of the solution against central differences.
    /// </summary>
    [TestClass]
    public class ImplicitSystemsTests
    {
        // p₃ = 0 puts zeros on the diagonal of ∂g/∂y at the starting guess, so the elimination must pivot.
        private static readonly double[] Parameters = [0.3, -0.2, 0.5, 0.0];

        public TestContext TestContext { get; set; } = null!;

        [TestMethod]
        public void RunTimeSolutionSatisfiesTheResidual()
        {
            var y = new double[3];
            var g = new double[3];
            ImplicitSystems.Solve(Parameters, y);
            ImplicitSystems.Residual(y, Parameters, g);
            Assert.IsLessThanOrEqualTo(1e-15, g.Max(Math.Abs));
        }

        [TestMethod]
        public void JacobianIsTheImplicitFunctionTheorem()
        {
            var y = new double[3];
            var jacobian = new double[12];
            ImplicitSystems.Solve(Parameters, y);
            ImplicitSystemsKernels.SolveJacobian_p(Parameters, jacobian);
            var error = MaxRelative(jacobian, ImplicitFunctionTheorem(y, Parameters));
            TestContext.WriteLine($"ImplicitSolve dy/dp vs -A^-1 dg/dp: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-12, error);
        }

        [TestMethod]
        public void VjpEqualsForwardModeTransposeLambda()
        {
            var lambda = RandomVector(3, 71);
            var jacobian = new double[12];
            var gradient = new double[4];
            ImplicitSystemsKernels.SolveJacobian_p(Parameters, jacobian);
            ImplicitSystemsKernels.SolveVjp_p(Parameters, lambda, gradient);
            Assert.IsLessThanOrEqualTo(1e-13, MaxRelative(gradient, TransposeTimes(jacobian, lambda, 4)));
        }

        [TestMethod]
        public void EnergyHessianMatchesCentralDifferencesOfTheGradient()
        {
            double[] p = [0.3, -0.2, 0.5, 0.4];
            var hessian = new double[16];
            ImplicitSystemsKernels.EnergyHessian_p(p, hessian);
            var differenced = CentralDifferenceHessian((q, g) => ImplicitSystemsKernels.EnergyVjp_p(q, [1.0], g), p, 1e-6);
            var error = MaxRelative(hessian, differenced);
            TestContext.WriteLine($"Hessian through ImplicitSolve vs central FD of the VJP: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
        }

        [TestMethod]
        public void RunTimeSolveThatDoesNotConvergeThrows()
        {
            double[] y = [0.5];
            Assert.ThrowsExactly<InvalidOperationException>(() => KernelMath.ImplicitSolve(NoRoot, ReadOnlySpan<double>.Empty, y, 8));
        }

        // g = y² + 1 has no real root.
        private static void NoRoot(ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)
        {
            g[0] = (y[0] * y[0]) + 1;
        }

        // dy/dp = −A⁻¹ ∂g/∂p, with A = ∂g/∂y at the solution.
        private static double[] ImplicitFunctionTheorem(double[] y, double[] p)
        {
            double[] a =
            [
                p[3], 1 + (0.3 * y[1] * y[1]), 0,
                1 + (0.2 * y[0]), p[3], 0.2,
                0.5 * y[1], 0.5 * y[0], 1 + (0.3 * y[2] * y[2]),
            ];
            double[] dgdp =
            [
                -1, 0, 0, y[0],
                0, -1, 0, y[1],
                0, 0, -1, 0,
            ];
            var inverse = new double[9];
            _ = ElementGeometry.Inverse(a, inverse);
            return Enumerable.Range(0, 12)
                .Select(k => -Enumerable.Range(0, 3).Sum(j => inverse[(3 * (k / 4)) + j] * dgdp[(4 * j) + (k % 4)]))
                .ToArray();
        }
    }
}
