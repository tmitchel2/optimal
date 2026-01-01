/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace OptimalCli;

/// <summary>
/// Options for running CLI commands.
/// </summary>
/// <param name="Headless">Run without visualization windows.</param>
public record CommandOptions(bool Headless = false);
