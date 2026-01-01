/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace OptimalCli;

/// <summary>
/// Interface for CLI commands that solve optimization problems.
/// </summary>
public interface ICommand
{
    /// <summary>
    /// Gets the name of the command.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Gets a brief description of the command.
    /// </summary>
    string Description { get; }

    /// <summary>
    /// Runs the command with the specified options.
    /// </summary>
    /// <param name="options">Command execution options.</param>
    void Run(CommandOptions options);
}
