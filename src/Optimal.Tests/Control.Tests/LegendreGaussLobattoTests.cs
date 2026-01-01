/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control;

namespace Optimal.Tests.Control.Tests
{
    [TestClass]
    public class LegendreGaussLobattoTests
    {
        [TestMethod]
        public void GetPointsAndWeightsThrowsForInvalidOrder()
        {
            Assert.ThrowsException<ArgumentException>(() => LegendreGaussLobatto.GetPointsAndWeights(1));
            Assert.ThrowsException<ArgumentException>(() => LegendreGaussLobatto.GetPointsAndWeights(0));
            Assert.ThrowsException<ArgumentException>(() => LegendreGaussLobatto.GetPointsAndWeights(-1));
        }

        [TestMethod]
        public void GetDifferentiationMatrixThrowsForInvalidOrder()
        {
            Assert.ThrowsException<ArgumentException>(() => LegendreGaussLobatto.GetDifferentiationMatrix(1));
        }

        [TestMethod]
        public void PointsAreWithinBounds()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (points, _) = LegendreGaussLobatto.GetPointsAndWeights(order);

                Assert.AreEqual(order, points.Length, $"Order {order} should have {order} points");

                foreach (var point in points)
                {
                    Assert.IsTrue(point >= -1.0 && point <= 1.0,
                        $"Point {point} is outside [-1, 1] for order {order}");
                }
            }
        }

        [TestMethod]
        public void EndpointsAreExactlyPlusMinusOne()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (points, _) = LegendreGaussLobatto.GetPointsAndWeights(order);

                Assert.AreEqual(-1.0, points[0], 1e-14,
                    $"First point should be exactly -1 for order {order}");
                Assert.AreEqual(1.0, points[order - 1], 1e-14,
                    $"Last point should be exactly 1 for order {order}");
            }
        }

        [TestMethod]
        public void PointsAreSymmetric()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (points, _) = LegendreGaussLobatto.GetPointsAndWeights(order);

                for (var i = 0; i < order; i++)
                {
                    var symmetricIndex = order - 1 - i;
                    Assert.AreEqual(-points[i], points[symmetricIndex], 1e-13,
                        $"Points should be symmetric for order {order}: points[{i}] = {points[i]}, points[{symmetricIndex}] = {points[symmetricIndex]}");
                }
            }
        }

        [TestMethod]
        public void PointsAreMonotonicallyIncreasing()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (points, _) = LegendreGaussLobatto.GetPointsAndWeights(order);

                for (var i = 1; i < order; i++)
                {
                    Assert.IsTrue(points[i] > points[i - 1],
                        $"Points should be monotonically increasing for order {order}");
                }
            }
        }

        [TestMethod]
        public void WeightsSumToTwo()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (_, weights) = LegendreGaussLobatto.GetPointsAndWeights(order);

                var sum = 0.0;
                foreach (var weight in weights)
                {
                    sum += weight;
                }

                Assert.AreEqual(2.0, sum, 1e-13,
                    $"Weights should sum to 2 for order {order}, got {sum}");
            }
        }

        [TestMethod]
        public void WeightsArePositive()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (_, weights) = LegendreGaussLobatto.GetPointsAndWeights(order);

                foreach (var weight in weights)
                {
                    Assert.IsTrue(weight > 0,
                        $"All weights should be positive for order {order}, got {weight}");
                }
            }
        }

        [TestMethod]
        public void WeightsAreSymmetric()
        {
            for (var order = 2; order <= 10; order++)
            {
                var (_, weights) = LegendreGaussLobatto.GetPointsAndWeights(order);

                for (var i = 0; i < order; i++)
                {
                    var symmetricIndex = order - 1 - i;
                    Assert.AreEqual(weights[i], weights[symmetricIndex], 1e-14,
                        $"Weights should be symmetric for order {order}");
                }
            }
        }

        [TestMethod]
        public void QuadratureIsExactForPolynomials()
        {
            // LGL quadrature of order p should integrate polynomials of degree <= 2p-3 exactly
            for (var order = 2; order <= 8; order++)
            {
                var (points, weights) = LegendreGaussLobatto.GetPointsAndWeights(order);
                var maxDegree = 2 * order - 3;

                for (var degree = 0; degree <= maxDegree; degree++)
                {
                    // Compute ∫_{-1}^{1} x^degree dx = [x^{degree+1}/(degree+1)]_{-1}^{1}
                    double analyticalIntegral;
                    if (degree % 2 == 1)
                    {
                        // Odd powers integrate to 0
                        analyticalIntegral = 0.0;
                    }
                    else
                    {
                        // Even powers: 2/(degree+1)
                        analyticalIntegral = 2.0 / (degree + 1);
                    }

                    // Compute numerical integral using LGL quadrature
                    var numericalIntegral = 0.0;
                    for (var i = 0; i < order; i++)
                    {
                        var xPowDegree = Math.Pow(points[i], degree);
                        numericalIntegral += weights[i] * xPowDegree;
                    }

                    Assert.AreEqual(analyticalIntegral, numericalIntegral, 1e-12,
                        $"LGL quadrature of order {order} should exactly integrate x^{degree}");
                }
            }
        }

        [TestMethod]
        public void DifferentiationMatrixSize()
        {
            for (var order = 2; order <= 10; order++)
            {
                var D = LegendreGaussLobatto.GetDifferentiationMatrix(order);

                Assert.AreEqual(order, D.GetLength(0), $"Matrix should have {order} rows");
                Assert.AreEqual(order, D.GetLength(1), $"Matrix should have {order} columns");
            }
        }

        [TestMethod]
        public void DifferentiationMatrixDiagonalProperties()
        {
            for (var order = 3; order <= 10; order++)
            {
                var D = LegendreGaussLobatto.GetDifferentiationMatrix(order);
                var n = order - 1;

                // Check D[0,0] = -n(n+1)/4
                var expectedD00 = -n * (n + 1) / 4.0;
                Assert.AreEqual(expectedD00, D[0, 0], 1e-13,
                    $"D[0,0] should equal -n(n+1)/4 for order {order}");

                // Check D[n,n] = n(n+1)/4
                var expectedDnn = n * (n + 1) / 4.0;
                Assert.AreEqual(expectedDnn, D[order - 1, order - 1], 1e-13,
                    $"D[n,n] should equal n(n+1)/4 for order {order}");

                // Check interior diagonal entries are 0
                for (var i = 1; i < order - 1; i++)
                {
                    Assert.AreEqual(0.0, D[i, i], 1e-13,
                        $"D[{i},{i}] should be 0 for order {order}");
                }
            }
        }

        [TestMethod]
        public void DifferentiationMatrixAccuracyOnPolynomials()
        {
            // Test that D accurately differentiates polynomials
            for (var order = 3; order <= 8; order++)
            {
                var (points, _) = LegendreGaussLobatto.GetPointsAndWeights(order);
                var D = LegendreGaussLobatto.GetDifferentiationMatrix(order);

                // Test on polynomials up to degree order-1
                for (var degree = 1; degree <= order - 1; degree++)
                {
                    // f(x) = x^degree, f'(x) = degree * x^(degree-1)
                    var f = new double[order];
                    var dfAnalytical = new double[order];

                    for (var i = 0; i < order; i++)
                    {
                        f[i] = Math.Pow(points[i], degree);
                        dfAnalytical[i] = degree * Math.Pow(points[i], degree - 1);
                    }

                    // Compute df using differentiation matrix
                    var dfNumerical = new double[order];
                    for (var i = 0; i < order; i++)
                    {
                        for (var j = 0; j < order; j++)
                        {
                            dfNumerical[i] += D[i, j] * f[j];
                        }
                    }

                    // Compare
                    for (var i = 0; i < order; i++)
                    {
                        Assert.AreEqual(dfAnalytical[i], dfNumerical[i], 1e-10,
                            $"Differentiation matrix should accurately compute derivative of x^{degree} at point {i} for order {order}");
                    }
                }
            }
        }

        [TestMethod]
        public void CachingWorks()
        {
            // Call twice with same order and verify we get same reference (cached)
            var (points1, weights1) = LegendreGaussLobatto.GetPointsAndWeights(5);
            var (points2, weights2) = LegendreGaussLobatto.GetPointsAndWeights(5);

            // Arrays should be identical (same reference due to caching)
            Assert.AreSame(points1, points2, "Points should be cached");
            Assert.AreSame(weights1, weights2, "Weights should be cached");

            var D1 = LegendreGaussLobatto.GetDifferentiationMatrix(5);
            var D2 = LegendreGaussLobatto.GetDifferentiationMatrix(5);

            Assert.AreSame(D1, D2, "Differentiation matrix should be cached");
        }

        [TestMethod]
        public void KnownValuesForOrder2()
        {
            // For order 2, points are just the endpoints
            var (points, weights) = LegendreGaussLobatto.GetPointsAndWeights(2);

            Assert.AreEqual(-1.0, points[0], 1e-14);
            Assert.AreEqual(1.0, points[1], 1e-14);

            // Weights for 2-point LGL are both 1 (trapezoidal rule)
            Assert.AreEqual(1.0, weights[0], 1e-14);
            Assert.AreEqual(1.0, weights[1], 1e-14);
        }

        [TestMethod]
        public void KnownValuesForOrder3()
        {
            // For order 3, points are {-1, 0, 1}
            var (points, weights) = LegendreGaussLobatto.GetPointsAndWeights(3);

            Assert.AreEqual(-1.0, points[0], 1e-14);
            Assert.AreEqual(0.0, points[1], 1e-14);
            Assert.AreEqual(1.0, points[2], 1e-14);

            // Weights for 3-point LGL: {1/3, 4/3, 1/3}
            Assert.AreEqual(1.0 / 3.0, weights[0], 1e-13);
            Assert.AreEqual(4.0 / 3.0, weights[1], 1e-13);
            Assert.AreEqual(1.0 / 3.0, weights[2], 1e-13);
        }

        [TestMethod]
        public void KnownValuesForOrder4()
        {
            // For order 4, interior points are at ±√(1/5) = ±0.4472135955...
            var (points, _) = LegendreGaussLobatto.GetPointsAndWeights(4);

            Assert.AreEqual(-1.0, points[0], 1e-14);
            Assert.AreEqual(-Math.Sqrt(0.2), points[1], 1e-13);
            Assert.AreEqual(Math.Sqrt(0.2), points[2], 1e-13);
            Assert.AreEqual(1.0, points[3], 1e-14);
        }
    }
}
