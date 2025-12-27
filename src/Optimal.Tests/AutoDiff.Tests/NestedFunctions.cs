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
    public static class NestedFunctions
    {
        public static double Square(double x) => x * x;

        public static double SquareOfSquare(double x) => Square(Square(x));

        public static double ComplexNested(double x, double y)
        {
            var a = Square(x);
            var b = Square(y);
            return a + b;
        }

        public static double NestedWithMath(double x)
        {
            return Math.Sin(Square(x));
        }

        public static double DeepNesting(double x)
        {
            return Square(Square(Square(x)));
        }

        public static double Cube(double x) => x * x * x;

        public static double MixedNesting(double x, double y)
        {
            return Square(x) + Cube(y);
        }

        public static double ChainedFunctions(double x)
        {
            var a = Square(x);
            var b = Cube(a);
            return Math.Sqrt(b);
        }
    }
}
