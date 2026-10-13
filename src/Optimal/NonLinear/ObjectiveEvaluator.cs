/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.NonLinear
{
    /// <summary>
    /// Evaluates an objective f(x) and, unless <paramref name="gradient"/> is empty, writes ∇f(x) into it.
    /// </summary>
    /// <param name="x">The design variables.</param>
    /// <param name="gradient">Receives ∇f(x), or is empty when only the value is wanted.</param>
    /// <returns>f(x).</returns>
    public delegate double ObjectiveEvaluator(ReadOnlySpan<double> x, Span<double> gradient);
}
