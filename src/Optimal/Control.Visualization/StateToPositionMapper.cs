/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.Control.Visualization
{
    /// <summary>
    /// Converts a state vector to (x, y) screen coordinates for visualization.
    /// </summary>
    /// <param name="state">The state vector at the current time point.</param>
    /// <param name="t">The time at the current point.</param>
    /// <returns>A tuple containing the (x, y) position in world coordinates.</returns>
    public delegate (double x, double y) StateToPositionFunc(double[] state, double t);
}
