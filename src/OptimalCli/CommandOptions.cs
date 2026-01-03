/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace OptimalCli;

/// <summary>
/// Type of collocation solver to use.
/// </summary>
public enum SolverType
{
    /// <summary>
    /// Legendre-Gauss-Lobatto collocation (recommended for most problems).
    /// </summary>
    LGL,

    /// <summary>
    /// Hermite-Simpson collocation.
    /// </summary>
    HermiteSimpson
}

/// <summary>
/// Options for running CLI commands.
/// </summary>
/// <param name="Headless">Run without visualization windows.</param>
/// <param name="Solver">Type of collocation solver to use.</param>
public record CommandOptions(bool Headless = false, SolverType Solver = SolverType.LGL);
