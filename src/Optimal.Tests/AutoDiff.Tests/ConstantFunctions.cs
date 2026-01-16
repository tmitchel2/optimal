/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable RCS1163 // Unused parameter

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// Test functions that return constants to verify AutoDiff handles them correctly.
    /// These are important for cost functions that return fixed values like "1.0" for time minimization.
    /// </summary>
    [OptimalCode]
    public static class ConstantFunctions
    {
        /// <summary>
        /// Returns constant 1.0 regardless of input - useful for time minimization in optimal control
        /// </summary>
        public static double One(double x)
        {
            return 1.0;
        }

        /// <summary>
        /// Returns constant 0.0 regardless of input
        /// </summary>
        public static double Zero(double x)
        {
            return 0.0;
        }

        /// <summary>
        /// Returns constant with multiple parameters - like running cost that ignores state
        /// </summary>
        public static double ConstantWithMultipleParams(double x, double y, double z)
        {
            return 1.0;
        }

        /// <summary>
        /// Returns constant computed from math expression that's constant
        /// </summary>
        public static double ComputedConstant(double x)
        {
            return 2.0 * 3.14159;
        }
    }
}
