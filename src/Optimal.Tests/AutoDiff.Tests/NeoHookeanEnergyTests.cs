/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using static Optimal.AutoDiff.Tests.KernelTestData;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// The generated Hessian of a hyperelastic strain-energy density W(F) (nine inputs): against central differences of
    /// the generated gradient, against the closed-form material tangent, and as a Hessian–vector product.
    /// </summary>
    [TestClass]
    public class NeoHookeanEnergyTests
    {
        public TestContext TestContext { get; set; } = null!;

        // A finite, invertible deformation gradient.
        private static double[] Deformation() => RandomVector(9, 31).Select((v, k) => (k % 4 == 0 ? 1.0 : 0.0) + (0.2 * v)).ToArray();

        [TestMethod]
        public void HessianMatchesCentralDifferencesOfTheGradient()
        {
            var f = Deformation();
            var hessian = new double[81];
            NeoHookeanEnergyKernels.DensityHessian_F(f, E, Nu, hessian);
            var differenced = CentralDifferenceHessian(Gradient, f, 1e-6);
            var error = MaxRelative(hessian, differenced);
            TestContext.WriteLine($"Neo-Hookean W(F) Hessian vs central FD of the VJP: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-7, error);
        }

        [TestMethod]
        public void HessianIsTheClosedFormMaterialTangent()
        {
            var f = Deformation();
            var hessian = new double[81];
            NeoHookeanEnergyKernels.DensityHessian_F(f, E, Nu, hessian);
            var error = MaxRelative(hessian, ClosedFormTangent(f));
            TestContext.WriteLine($"Neo-Hookean W(F) Hessian vs closed-form dP/dF: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-12, error);
        }

        [TestMethod]
        public void HessianIsExactlySymmetric()
        {
            var hessian = new double[81];
            NeoHookeanEnergyKernels.DensityHessian_F(Deformation(), E, Nu, hessian);
            Assert.AreEqual(0.0, Asymmetry(hessian, 9));
        }

        [TestMethod]
        public void HvpIsTheHessianTimesTheDirection()
        {
            var f = Deformation();
            var direction = RandomVector(9, 32);
            var hessian = new double[81];
            var product = new double[9];
            NeoHookeanEnergyKernels.DensityHessian_F(f, E, Nu, hessian);
            NeoHookeanEnergyKernels.DensityHvp_F(f, E, Nu, direction, product);
            var error = MaxRelative(product, Times(hessian, direction));
            TestContext.WriteLine($"Neo-Hookean Hvp vs H v: max rel {error:E3}");
            Assert.IsLessThanOrEqualTo(1e-13, error);
        }

        private static void Gradient(double[] f, double[] gradient)
        {
            NeoHookeanEnergyKernels.DensityVjp_F(f, E, Nu, [1.0], gradient);
        }

        // ∂P_iJ/∂F_kL = μ δ_ik δ_JL + λ F⁻¹_Ji F⁻¹_Lk + (μ − λ ln J) F⁻¹_Li F⁻¹_Jk, for P = μ(F − F⁻ᵀ) + λ ln(J) F⁻ᵀ.
        private static double[] ClosedFormTangent(double[] f)
        {
            var inverse = new double[9];
            var det = ElementGeometry.Inverse(f, inverse);
            var mu = ElementGeometry.Shear(E, Nu);
            var lambda = ElementGeometry.Lame(E, Nu);
            var logJ = System.Math.Log(det);
            var tangent = new double[81];
            for (var i = 0; i < 3; i++)
            {
                for (var j = 0; j < 3; j++)
                {
                    for (var k = 0; k < 3; k++)
                    {
                        for (var l = 0; l < 3; l++)
                        {
                            var identity = i == k && j == l ? mu : 0.0;
                            tangent[(9 * ((3 * i) + j)) + (3 * k) + l] = identity
                                + (lambda * inverse[(3 * j) + i] * inverse[(3 * l) + k])
                                + ((mu - (lambda * logJ)) * inverse[(3 * l) + i] * inverse[(3 * j) + k]);
                        }
                    }
                }
            }

            return tangent;
        }
    }
}
