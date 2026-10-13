/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The graph of <c>KernelMath.ImplicitSolve</c>: the solution y*(p) of a small system g(y, p) = 0, unrolled at
    /// generation time, with its derivative by the implicit-function theorem rather than through the iterations.
    /// </summary>
    /// <remarks>
    /// <para>
    /// The residual is built once, symbolically, at placeholder unknowns Y: g(Y, p) and its Jacobian A(Y, p) = ∂g/∂Y
    /// (a forward sweep seeded at Y). Each Newton step instantiates both at the current iterate and solves
    /// A δ = g by Gaussian elimination with partial pivoting, unrolled, its pivot choices selects; y ← y − δ.
    /// </para>
    /// <para>
    /// The iterates are then detached (<see cref="ScalarOp.Detach"/>): no derivative flows through them. One more step
    /// with A detached too, ŷ − A(ŷ, p)⁻¹ g(ŷ, p), gives the value (one more Newton step, so no less accurate) and the
    /// whole first derivative: dy*/dp = −A⁻¹ ∂g/∂p, the implicit-function theorem, in forward and reverse mode alike.
    /// For second derivatives (a Hessian) a further Newton step with A not detached is taken: a Newton step's own
    /// derivative in y vanishes at the root, so applied to a first-order-exact iterate it is exact to second order.
    /// </para>
    /// </remarks>
    public static class ImplicitSolver
    {
        /// <summary>The most unknowns one implicit solve may have.</summary>
        public const int MaxUnknowns = 16;

        /// <summary>
        /// The solution, as graph nodes, of the system whose residual is <paramref name="residual"/> at the placeholder
        /// unknowns <paramref name="placeholders"/>, from <paramref name="guess"/>, after <paramref name="iterations"/>
        /// Newton steps and the implicit-function step; exact to <paramref name="order"/> (1 or 2) derivatives.
        /// <paramref name="countIteration"/> is called once per Newton step, so the unroller's limits apply.
        /// </summary>
        public static ImmutableArray<int> Solve(
            ScalarGraph g,
            IReadOnlyList<int> placeholders,
            IReadOnlyList<int> residual,
            IReadOnlyList<int> guess,
            int iterations,
            int order,
            Action countIteration)
        {
            var m = placeholders.Count;
            var symbolic = residual.Concat(Jacobian(g, placeholders, residual)).ToArray();

            int[] Step(int[] y, bool detachJacobian)
            {
                var values = Instantiate(g, placeholders, y, symbolic);
                var a = values.Skip(m).Select(v => detachJacobian ? g.Detach(v) : v).ToArray();
                var delta = GraphLinearAlgebra.Solve(g, a, values.Take(m).ToArray());
                return y.Zip(delta, g.Sub).ToArray();
            }

            var iterate = guess.ToArray();
            for (var k = 0; k < iterations; k++)
            {
                countIteration();
                iterate = Step(iterate, detachJacobian: false);
            }

            iterate = Step(iterate.Select(g.Detach).ToArray(), detachJacobian: true);
            return (order >= 2 ? Step(iterate, detachJacobian: false) : iterate).ToImmutableArray();
        }

        /// <summary>
        /// ∂residual_i/∂placeholder_j, row-major, by a forward sweep from the first placeholder. Nodes before it cannot
        /// depend on a placeholder.
        /// </summary>
        public static int[] Jacobian(ScalarGraph g, IReadOnlyList<int> placeholders, IReadOnlyList<int> residual)
        {
            var m = placeholders.Count;
            var zero = g.Const(0);
            var from = placeholders.Min();
            var count = g.Count;
            var live = DerivativeFlow.Live(g, count, residual);
            var seeds = placeholders.Select((id, j) => (id, j)).ToDictionary(p => p.id, p => p.j);
            var tangents = JacobianTransform.Sweep(
                g,
                from,
                count,
                id => live[id],
                id => seeds.TryGetValue(id, out var j) ? Enumerable.Range(0, m).Select(k => k == j ? g.Const(1) : zero).ToArray() : null,
                m,
                zero);
            return residual.SelectMany(id => tangents[id] ?? Enumerable.Repeat(zero, m).ToArray()).ToArray();
        }

        /// <summary>
        /// <paramref name="roots"/> with each placeholder replaced by the matching node of <paramref name="values"/>: every
        /// node between the first placeholder and the last root that depends on a placeholder, and that a root needs, is
        /// rebuilt through the graph's builders, so constants fold.
        /// </summary>
        public static int[] Instantiate(ScalarGraph g, IReadOnlyList<int> placeholders, IReadOnlyList<int> values, IReadOnlyList<int> roots)
        {
            var from = placeholders.Min();
            var to = roots.Max() + 1;
            if (to <= from)
            {
                return roots.ToArray();
            }

            var needed = new bool[to - from];
            foreach (var root in roots.Where(r => r >= from))
            {
                needed[root - from] = true;
            }

            for (var id = to - 1; id >= from; id--)
            {
                foreach (var operand in needed[id - from] ? g[id].Operands : Enumerable.Empty<int>())
                {
                    if (operand >= from)
                    {
                        needed[operand - from] = true;
                    }
                }
            }

            var substitute = placeholders.Select((id, k) => (id, k)).ToDictionary(p => p.id, p => values[p.k]);
            var mapped = new int[to - from];
            int Map(int id) => id < from ? id : mapped[id - from];
            for (var id = from; id < to; id++)
            {
                var node = g[id];
                mapped[id - from] = substitute.TryGetValue(id, out var value)
                    ? value
                    : needed[id - from] && node.Operands.Any(o => o >= from && Map(o) != o)
                        ? g.Rebuild(node, Map(node.A), Map(node.B), Map(node.C), Map(node.D))
                        : id;
            }

            return roots.Select(Map).ToArray();
        }
    }
}
