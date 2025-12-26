/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal;

namespace Optimal.Tests
{
    [OptimalCode]
    public static class LoopFunctions
    {
        public static double SumOfSquares(double x, int n)
        {
            var sum = 0.0;
            for (var i = 0; i < n; i++)
            {
                sum += x * x;
            }
            return sum;
        }

        public static double SumOfCubes(double x, int n)
        {
            var sum = 0.0;
            for (var i = 0; i < n; i++)
            {
                sum += x * x * x;
            }
            return sum;
        }

        public static double LinearSum(double x, int n)
        {
            var sum = 0.0;
            for (var i = 0; i < n; i++)
            {
                sum += x;
            }
            return sum;
        }

        public static double ProductAccumulation(double x, int n)
        {
            var product = 1.0;
            for (var i = 0; i < n; i++)
            {
                product *= x;
            }
            return product;
        }

        public static double IndexedSum(double x, int n)
        {
            var sum = 0.0;
            for (var i = 0; i < n; i++)
            {
                sum += i * x;
            }
            return sum;
        }
    }
}
