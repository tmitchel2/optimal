/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Linq;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// Evaluates an <see cref="InequalityProblem"/> into buffers laid out by its <see cref="ConstraintMatrix"/>: the
    /// objective first, then each block in order, at the same point.
    /// </summary>
    internal sealed class ProblemEvaluator
    {
        private readonly InequalityProblem _problem;

        public ProblemEvaluator(InequalityProblem problem)
        {
            _problem = problem;
            Matrix = ConstraintMatrix.Create(problem.VariableCount, problem.Constraints.Select(block => block.Sparsity).ToArray());
        }

        /// <summary>Gets the concatenated constraint structure.</summary>
        public ConstraintMatrix Matrix { get; }

        /// <summary>Gets the number of evaluations so far.</summary>
        public int Evaluations { get; private set; }

        /// <summary>
        /// Evaluates f and g, and their gradients unless <paramref name="df0"/> and <paramref name="dg"/> are empty.
        /// Returns false if any value or gradient is not finite.
        /// </summary>
        public bool Evaluate(ReadOnlySpan<double> x, Span<double> df0, Span<double> g, Span<double> dg, out double f0)
        {
            Evaluations++;
            f0 = _problem.Objective(x, df0);
            for (var b = 0; b < Matrix.BlockCount; b++)
            {
                var values = g.Slice(Matrix.BlockRowOffset(b), Matrix.BlockRowCount(b));
                var jacobian = dg.IsEmpty ? Span<double>.Empty : dg.Slice(Matrix.BlockValueOffset(b), Matrix.BlockValueCount(b));
                _problem.Constraints[b].Evaluate(x, values, jacobian);
            }

            return double.IsFinite(f0) && AllFinite(g) && AllFinite(df0) && AllFinite(dg);
        }

        private static bool AllFinite(ReadOnlySpan<double> values)
        {
            foreach (var value in values)
            {
                if (!double.IsFinite(value))
                {
                    return false;
                }
            }

            return true;
        }
    }
}
