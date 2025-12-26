/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Diagnostics;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Tests
{
    [TestClass]
    public sealed class OptimalTests
    {
        [TestMethod]
        public void CanOptimiseBrachistochroneUsingAutoDiff()
        {
            var x = OptRange.Create(0, 1, 100).ToArray();
            var y = OptRange.Create(0.5, 0, x.Length).ToArray();
            var s1 = Stopwatch.StartNew();
            var totalTime = GetBrachistochroneTotalTime(x, y);
            Console.WriteLine();
            Console.WriteLine($"Total time = {totalTime} using interation took {s1.Elapsed} time to calculate.");
        }

        static double GetBrachistochroneTotalTime(
            double[] x, double[] y)
        {
            var totalTime = 0d;
            for (var i = 0; i < x.Length - 1; i++)
            {
                // Gravitational potential energy
                //   0.5 * mv^2 = mgy
                //   v = (2gy) ^ 0.5

                // Equation of motion 
                // -  Displacement = Velocity * Time
                // -  s = vt

                var startVelocity = Math.Pow(2.0d * 9.81d * (y[0] - y[i]), 0.5d);
                var endVelocity = Math.Pow(2.0d * 9.81d * (y[0] - y[i + 1]), 0.5d);
                var velocity = (startVelocity + endVelocity) / 2d;
                var dx = x[i + 1] - x[i];
                var dy = y[i + 1] - y[i];
                var distance = Math.Pow(Math.Pow(dx, 2d) + Math.Pow(dy, 2d), 0.5d);
                var time = distance / velocity;
                totalTime += time;
            }

            return totalTime;
        }
    }
}
