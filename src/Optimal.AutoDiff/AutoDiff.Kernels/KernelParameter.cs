/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// A parameter of a kernel as the graph sees it: a scalar (<see cref="IsArray"/> false, <see cref="Length"/> 1) or a
    /// span of <see cref="Length"/> elements. <see cref="Group"/> numbers the parameter's inputs in the graph; the one
    /// <see cref="IsOutput"/> parameter has no inputs.
    /// </summary>
    public sealed record KernelParameter(string Name, int Length, bool IsArray, bool IsOutput, int Group)
    {
        /// <summary>The C# type of the parameter in a generated method's signature.</summary>
        public string TypeName => IsOutput ? "Span<double>" : (IsArray ? "ReadOnlySpan<double>" : "double");
    }
}
