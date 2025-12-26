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
    public sealed class BrachistochroneTests
    {
        [TestMethod]
        public void CanOptimiseBrachistochroneUsingAutoDiff()
        {
            var x = OptRange.Create(0, 1, 100).ToArray();
            var y = OptRange.Create(0.5, 0, x.Length).ToArray();
            var s1 = Stopwatch.StartNew();
            var totalTime = Brachistochrone.GetTotalTime(x, y);
            Console.WriteLine();
            Console.WriteLine($"Total time = {totalTime} using interation took {s1.Elapsed} time to calculate.");
        }
    }
}
