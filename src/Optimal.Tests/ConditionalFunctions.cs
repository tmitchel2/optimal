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
    public static class ConditionalFunctions
    {
        public static double ReLU(double x) => x > 0.0 ? x : 0.0;

        public static double LeakyReLU(double x) => x > 0.0 ? x : 0.01 * x;

        public static double Max(double a, double b) => a > b ? a : b;

        public static double Min(double a, double b) => a < b ? a : b;

        public static double Clamp(double x, double min, double max)
        {
            if (x < min)
            {
                return min;
            }
            else if (x > max)
            {
                return max;
            }
            else
            {
                return x;
            }
        }

        public static double PiecewiseLinear(double x)
        {
            if (x < -1.0)
            {
                return -x - 1.0;
            }
            else if (x > 1.0)
            {
                return x - 1.0;
            }
            else
            {
                return 0.0;
            }
        }

        public static double Quadratic(double x)
        {
            if (x < 0.0)
            {
                return x * x;
            }
            else
            {
                return 2.0 * x;
            }
        }

        public static double SmoothStep(double x)
        {
            var result = 0.0;

            if (x <= 0.0)
            {
                result = 0.0;
            }
            else if (x >= 1.0)
            {
                result = 1.0;
            }
            else
            {
                result = 3.0 * x * x - 2.0 * x * x * x;
            }

            return result;
        }
    }
}
