/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Tests
{
    [TestClass]
    public sealed class BrachistochroneTests
    {
        [TestMethod]
        public void CanCalculateDurationForArcPath()
        {
            var radius = 150.0;

            var duration = Brachistochrone.GetDuration(radius);

            Assert.IsTrue(duration.TotalSeconds > 0, "Duration should be positive");
            Assert.IsTrue(duration.TotalSeconds < 100, "Duration should be reasonable");
            Console.WriteLine($"Duration for radius {radius}: {duration.TotalSeconds:F3} seconds");
        }

        [TestMethod]
        public void DifferentRadiiProduceDifferentDurations()
        {
            var radius1 = 80.0;
            var radius2 = 150.0;

            var duration1 = Brachistochrone.GetDuration(radius1);
            var duration2 = Brachistochrone.GetDuration(radius2);

            Assert.AreNotEqual(duration1, duration2);
            Console.WriteLine($"Duration for radius {radius1}: {duration1.TotalSeconds:F3} seconds");
            Console.WriteLine($"Duration for radius {radius2}: {duration2.TotalSeconds:F3} seconds");
        }

        [TestMethod]
        public void GetDurationWithSmallerRadiusIsValid()
        {
            var minRadius = Math.Sqrt(100 * 100 + 100 * 100) / 2.0;
            var radius = minRadius + 1.0;

            var duration = Brachistochrone.GetDuration(radius);

            Assert.IsTrue(duration.TotalSeconds > 0);
            Console.WriteLine($"Duration for radius {radius:F2} (near minimum): {duration.TotalSeconds:F3} seconds");
        }
    }
}
