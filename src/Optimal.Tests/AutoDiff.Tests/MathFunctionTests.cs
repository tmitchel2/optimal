/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Tests
{
    [OptimalCode]
    public static class MathFunctions
    {
        public static double SquareRoot(double x) => Math.Sqrt(x);

        public static double Sine(double x) => Math.Sin(x);

        public static double Cosine(double x) => Math.Cos(x);

        public static double Tangent(double x) => Math.Tan(x);

        public static double Exponential(double x) => Math.Exp(x);

        public static double Logarithm(double x) => Math.Log(x);

        public static double Power(double x, double n) => Math.Pow(x, n);

        public static double AbsoluteValue(double x) => Math.Abs(x);

        public static double Velocity(double height)
        {
            return Math.Sqrt(2.0 * 9.81 * height);
        }

        public static double SinPlusExp(double x)
        {
            return Math.Sin(x) + Math.Exp(x);
        }

        public static double ComplexFunction(double x)
        {
            return Math.Sqrt(x) * Math.Sin(x) + Math.Log(x);
        }

        public static double ComplexFunction(double x, double y)
        {
            return Math.Sin(x) * Math.Exp(y);
        }
    }
}
