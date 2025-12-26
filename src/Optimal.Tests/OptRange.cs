/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Generic;
using System.Linq;

namespace Optimal.Tests
{
    public static class OptRange
    {
        public static IEnumerable<double> Create(double start, double finish, int count)
        {
            var increment = (finish - start) / (count - 1);
            return Enumerable
                .Range(0, count)
                .Select(i => start + i * increment);
        }
    }
}
