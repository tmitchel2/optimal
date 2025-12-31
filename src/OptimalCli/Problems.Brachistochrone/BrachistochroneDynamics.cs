/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal;

namespace OptimalCli.Problems.Brachistochrone
{
    /// <summary>
    /// Brachistochrone problem dynamics with AutoDiff support.
    /// State: [s (position along diagonal), d (perpendicular distance from diagonal), v (velocity)]
    /// Control: θ (path angle relative to diagonal, in RADIANS)
    ///
    /// The brachistochrone problem finds the curve of fastest descent under gravity
    /// between two points. The solution is a cycloid curve.
    ///
    /// Coordinate system: Instead of using (x,y), we use (s,d) where:
    /// - s is the position along the diagonal from start (0,0) to end (xFinal, yFinal)
    /// - d is the perpendicular distance from the diagonal line
    /// - Diagonal length L = sqrt(xFinal² + yFinal²)
    ///
    /// IMPORTANT: All angle parameters (theta) are in RADIANS, not degrees.
    /// </summary>
    [OptimalCode]
    public static class BrachistochroneDynamics
    {

    }
}
