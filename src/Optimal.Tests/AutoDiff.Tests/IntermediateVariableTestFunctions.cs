/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// Test case demonstrating intermediate variable issue in AutoDiff.
    /// This is a simplified version of CartPoleDynamics.XddotRate that initially fails,
    /// demonstrating that intermediate variables break the gradient chain.
    /// </summary>
    [OptimalCode]
    public static class IntermediateVariableTestFunctions
    {
        /// <summary>
        /// Simple function with intermediate variable: f(x) = sin(x) + cos(x)
        /// This should work but currently fails if we use intermediate variables.
        /// </summary>
        public static double SimpleWithIntermediate(double x)
        {
            var sinX = Math.Sin(x);
            var cosX = Math.Cos(x);
            return sinX + cosX;
        }

        /// <summary>
        /// Same function without intermediate variables: f(x) = sin(x) + cos(x)
        /// This is the workaround that currently works.
        /// </summary>
        public static double SimpleInlined(double x)
        {
            return Math.Sin(x) + Math.Cos(x);
        }

        /// <summary>
        /// CartPole-like function with intermediate variables.
        /// Simplified version: f(x, y) = (x + sin(y)) / (1 + sin²(y))
        /// </summary>
        public static double CartPoleLikeWithIntermediate(double x, double y)
        {
            var sinY = Math.Sin(y);
            var sin2Y = sinY * sinY;
            var denom = 1.0 + sin2Y;
            return (x + sinY) / denom;
        }

        /// <summary>
        /// Same function without intermediate variables (workaround).
        /// </summary>
        public static double CartPoleLikeInlined(double x, double y)
        {
            return (x + Math.Sin(y)) / (1.0 + Math.Sin(y) * Math.Sin(y));
        }

        /// <summary>
        /// More complex case with multiple dependent intermediate variables.
        /// f(x, y) = sin(x)·cos(y) + cos(x)·sin(y)
        /// </summary>
        public static double ComplexWithIntermediate(double x, double y)
        {
            var sinX = Math.Sin(x);
            var cosX = Math.Cos(x);
            var sinY = Math.Sin(y);
            var cosY = Math.Cos(y);
            return sinX * cosY + cosX * sinY;
        }

        /// <summary>
        /// Same function inlined.
        /// </summary>
        public static double ComplexInlined(double x, double y)
        {
            return Math.Sin(x) * Math.Cos(y) + Math.Cos(x) * Math.Sin(y);
        }

        /// <summary>
        /// Intermediate variable used multiple times.
        /// f(x) = sin(x) + sin(x)² + sin(x)³
        /// </summary>
        public static double MultipleUsesOfIntermediate(double x)
        {
            var sinX = Math.Sin(x);
            return sinX + sinX * sinX + sinX * sinX * sinX;
        }

        /// <summary>
        /// Same function inlined (inefficient - computes sin(x) multiple times).
        /// </summary>
        public static double MultipleUsesInlined(double x)
        {
            return Math.Sin(x) + Math.Sin(x) * Math.Sin(x) + Math.Sin(x) * Math.Sin(x) * Math.Sin(x);
        }

        /// <summary>
        /// Chain of intermediate variables.
        /// f(x) = ((x + 1)²)³
        /// </summary>
        public static double ChainedIntermediates(double x)
        {
            var a = x + 1.0;
            var b = a * a;
            var c = b * b * b;
            return c;
        }

        /// <summary>
        /// Same function inlined.
        /// </summary>
        public static double ChainedInlined(double x)
        {
            return (x + 1.0) * (x + 1.0) * (x + 1.0) * (x + 1.0) * (x + 1.0) * (x + 1.0);
        }
    }
}
