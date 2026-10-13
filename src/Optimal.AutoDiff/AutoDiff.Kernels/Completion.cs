/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>How the interpretation of a statement ended, so loops and calls can honour break, continue and return.</summary>
    public enum Completion
    {
        Normal,
        Break,
        Continue,
        Return
    }
}
