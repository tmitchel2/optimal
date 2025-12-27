/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Tests
{
    [OptimalCode]
    public static class SimpleTestFunctions
    {
        public static double Square(double x) => x * x;

        public static double Add(double x, double y) => x + y;

        public static double Multiply(double x, double y) => x * y;

        public static double KineticEnergy(double mass, double velocity)
        {
            return 0.5 * mass * velocity * velocity;
        }
    }
}
