/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Microsoft.VisualStudio.TestTools.UnitTesting;
using Optimal.Control.Core;

namespace Optimal.Control.Collocation.Tests
{
    [TestClass]
    public sealed class HermiteSimpsonTranscriptionTestsAlt : TranscriptionTestsAlt
    {
        protected override ICollocationTranscription CreateTranscription(ControlProblem problem, CollocationGrid grid, int order)
        {
            return new HermiteSimpsonTranscription(problem, grid);
        }
    }
}
