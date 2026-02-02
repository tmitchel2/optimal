/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace OptimalCli;

/// <summary>
/// Brachistochrone problem variant.
/// </summary>
public enum BrachistochroneVariant
{
    /// <summary>
    /// Fixed final time formulation (simpler, for testing).
    /// </summary>
    FixedTime,

    /// <summary>
    /// Free final time using time-scaling transformation (classic formulation).
    /// </summary>
    FreeFinalTime,

    /// <summary>
    /// Free final time with running cost (alternative formulation).
    /// </summary>
    FreeFinalTimeRunningCost
}

/// <summary>
/// Options for running CLI commands.
/// </summary>
/// <param name="Headless">Run without visualization windows.</param>
/// <param name="Variant">Brachistochrone problem variant.</param>
/// <param name="DebugViz">Debug visualization mode - shows track without optimization.</param>
public record CommandOptions(
    bool Headless = false,
    BrachistochroneVariant Variant = BrachistochroneVariant.FreeFinalTime,
    bool DebugViz = false);
