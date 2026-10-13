/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels.Tests
{
    /// <summary>Source text of a one-kernel <c>[OptimalCode]</c> class, <c>Probes.Probe</c>, for the generator tests.</summary>
    public static class KernelSource
    {
        public const string DefaultSignature = "[FixedLength(3)] ReadOnlySpan<double> x, [FixedLength(2)] Span<double> y";

        public static string Kernel(
            string body,
            string signature = DefaultSignature,
            string attributes = "[Jacobian(nameof(x))]",
            string members = "",
            string types = "")
        {
            return $$"""
                using System;
                using Optimal;

                namespace Probes
                {
                    [OptimalCode]
                    public static class Probe
                    {
                {{members}}

                        {{attributes}}
                        public static void F({{signature}})
                        {
                {{body}}
                        }
                    }
                {{types}}
                }
                """;
        }
    }
}
