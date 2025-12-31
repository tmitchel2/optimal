/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

#pragma warning disable CA1861 // Prefer static readonly fields - not applicable for lambda captures

namespace OptimalCli.Problems.Brachistochrone;

/// <summary>
/// Solves the Brachistochrone problem (Johann Bernoulli, 1696).
/// Question: What curve gives the fastest descent under gravity between two points?
/// Answer: A cycloid - the curve traced by a point on a rolling circle.
/// </summary>
public sealed class BrachistochroneProblemSolver : IProblemSolver
{
    public string Name => "brachistochrone";

    public string Description => "Curve of fastest descent under gravity (Johann Bernoulli, 1696)";

    public void Solve()
    {

    }
}
