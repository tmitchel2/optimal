/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace OptimalCli;

/// <summary>
/// Interface for optimization problem solvers that can be run from the CLI.
/// </summary>
public interface IProblemSolver
{
    /// <summary>
    /// Gets the name of the problem.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Gets a brief description of the problem.
    /// </summary>
    string Description { get; }

    /// <summary>
    /// Solves the optimization problem and generates visualizations.
    /// </summary>
    void Solve();
}
