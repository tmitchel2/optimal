/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for test clarity

using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace Optimal.Problems.Brachistochrone.Tests
{
    /// <summary>
    /// Integration tests for Brachistochrone problem solver with arc-length parameterization.
    ///
    /// Arc-length formulation:
    /// - Independent variable: s (horizontal distance from start)
    /// - State: [v, n, alpha, t]
    ///   - v: Speed (m/s)
    ///   - n: Vertical position below start (m, positive = descended)
    ///   - alpha: Heading angle from horizontal (rad, positive = descending)
    ///   - t: Elapsed time (s)
    /// - Control: [k]
    ///   - k: Path curvature = dalpha/ds (rad/m)
    /// </summary>
    [TestClass]
    [TestCategory("Integration")]
    public sealed class BrachistochroneSolverTests
    {
    }
}
