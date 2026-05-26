/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Differentiation
{
    /// <summary>
    /// Pure-IR forward-mode automatic differentiation expander. Given a
    /// <see cref="MethodBodyNode"/> and a parameter name to differentiate with
    /// respect to, returns a NEW <see cref="MethodBodyNode"/> whose statements
    /// interleave the original primal assignments with synthesised tangent
    /// assignments ("{var}_tan = d{var}/dwrt"). The final return statement is
    /// expanded into two assignments (the primal result and the tangent result)
    /// plus a <see cref="ReturnNode"/> of the primal result variable.
    ///
    /// This is the pure-IR sibling of <c>ForwardModeCodeGenerator</c>. The
    /// codegen path emits C# strings inline; this path produces IR that
    /// downstream consumers (e.g. WGSL emitters) can lower to other targets.
    /// Both reuse the same <see cref="ForwardModeDifferentiator"/> rules.
    /// </summary>
    public sealed class ForwardModeIRExpander
    {
        public const string TangentSuffix = "_tan";
        public const string PrimalResultName = "__primal";
        public const string TangentResultName = "__tangent";

        // Suffix/names used when ExpandSecondOrder needs a second pass that
        // can't reuse the defaults without colliding with the first pass's
        // emitted variable names.
        public const string TangentSuffixFirstPass = "_tan1";
        public const string PrimalResultNameFirstPass = "__primal1";
        public const string TangentResultNameFirstPass = "__tangent1";

        private readonly ForwardModeDifferentiator _differentiator;
        private readonly ITypeSymbol _doubleType;

        public ForwardModeIRExpander(ITypeSymbol doubleType, OptimalTransform? transform = null)
        {
            _doubleType = doubleType;
            _differentiator = new ForwardModeDifferentiator(doubleType, transform);
        }

        public ForwardModeIRExpander(ForwardModeDifferentiator differentiator, ITypeSymbol doubleType)
        {
            _doubleType = doubleType;
            _differentiator = differentiator;
        }

        /// <summary>
        /// Expand the given method body into a new body that produces both the
        /// primal value and its forward-mode derivative w.r.t.
        /// <paramref name="wrtParameter"/>. Other differentiable parameters,
        /// if listed in <paramref name="zeroTangentParameters"/>, are seeded
        /// with a zero tangent so references to them resolve cleanly.
        /// </summary>
        public ExpandedMethodBody Expand(
            MethodBodyNode body,
            string wrtParameter,
            int startNodeId,
            ImmutableArray<string> zeroTangentParameters = default)
        {
            return Expand(body, wrtParameter, startNodeId, zeroTangentParameters,
                TangentSuffix, PrimalResultName, TangentResultName);
        }

        /// <summary>
        /// Overridable-naming variant of <c>Expand</c> so two passes can be
        /// composed (<see cref="ExpandSecondOrder"/>) without their emitted
        /// variable names colliding.
        /// </summary>
        public ExpandedMethodBody Expand(
            MethodBodyNode body,
            string wrtParameter,
            int startNodeId,
            ImmutableArray<string> zeroTangentParameters,
            string tangentSuffix,
            string primalResultName,
            string tangentResultName)
        {
            var context = new ForwardModeContext(startNodeId, wrtParameter);

            context.SetTangent(wrtParameter, new ConstantNode(context.NewNodeId(), 1.0, _doubleType));

            if (!zeroTangentParameters.IsDefault)
            {
                foreach (var p in zeroTangentParameters)
                {
                    if (p != wrtParameter)
                    {
                        context.SetTangent(p, new ConstantNode(context.NewNodeId(), 0.0, _doubleType));
                    }
                }
            }

            var expanded = ImmutableArray.CreateBuilder<IRNode>();

            foreach (var stmt in body.Statements)
            {
                ExpandStatement(stmt, context, expanded, tangentSuffix, primalResultName, tangentResultName);
            }

            return new ExpandedMethodBody(
                new MethodBodyNode(context.NewNodeId(), expanded.ToImmutable()),
                primalResultName,
                tangentResultName);
        }

        /// <summary>
        /// Second-order forward-over-forward: compute ∂²f/∂firstWrt∂secondWrt
        /// by chaining two <c>Expand</c> passes. First pass with the
        /// "<c>_tan1</c>" suffix differentiates wrt <paramref name="firstWrt"/>
        /// and emits a <c>__tangent1</c> variable; we drop the original
        /// return and re-bind it to <c>__tangent1</c>; second pass with the
        /// default "<c>_tan</c>" suffix differentiates wrt
        /// <paramref name="secondWrt"/>. The result's <c>__tangent</c> is
        /// the second derivative.
        ///
        /// Symmetry note: ∂²f/∂x∂y = ∂²f/∂y∂x for smooth f, so callers can
        /// pick either order to share work across the 6 unique Hessian
        /// components.
        /// </summary>
        public ExpandedMethodBody ExpandSecondOrder(
            MethodBodyNode body,
            string firstWrt,
            string secondWrt,
            int startNodeId,
            ImmutableArray<string> zeroTangentParameters = default)
        {
            // First pass with "1"-suffixed names so the second pass's
            // default "_tan" / "__primal" / "__tangent" don't collide.
            var first = Expand(body, firstWrt, startNodeId, zeroTangentParameters,
                TangentSuffixFirstPass, PrimalResultNameFirstPass, TangentResultNameFirstPass);

            // Drop the first pass's terminal Return(__primal1) and re-emit
            // Return(__tangent1) so the second pass differentiates the
            // first derivative (which produces the second derivative).
            var firstStmts = first.Body.Statements;
            if (firstStmts.Length < 1 || firstStmts[firstStmts.Length - 1] is not ReturnNode)
            {
                throw new System.InvalidOperationException(
                    "First-pass body did not end with a ReturnNode — cannot compose second pass.");
            }
            var prelude = ImmutableArray.CreateBuilder<IRNode>(firstStmts.Length);
            for (var i = 0; i < firstStmts.Length - 1; i++)
            {
                prelude.Add(firstStmts[i]);
            }
            var idCounter = first.Body.NodeId + 1;
            prelude.Add(new ReturnNode(
                idCounter++,
                new VariableNode(idCounter++, TangentResultNameFirstPass, _doubleType)));
            var bodyForSecondPass = new MethodBodyNode(idCounter++, prelude.ToImmutable());

            // Second pass with default naming — produces __tangent which IS
            // the second derivative.
            return Expand(bodyForSecondPass, secondWrt, idCounter, zeroTangentParameters,
                TangentSuffix, PrimalResultName, TangentResultName);
        }

        private void ExpandStatement(
            IRNode stmt,
            ForwardModeContext context,
            ImmutableArray<IRNode>.Builder expanded,
            string tangentSuffix,
            string primalResultName,
            string tangentResultName)
        {
            switch (stmt)
            {
                case AssignmentNode assignment:
                {
                    var tangentExpr = _differentiator.Differentiate(assignment.Value, context);
                    var tangentVar = assignment.TargetVariable + tangentSuffix;

                    expanded.Add(assignment);
                    expanded.Add(new AssignmentNode(context.NewNodeId(), tangentVar, tangentExpr));

                    // Subsequent reads of `assignment.TargetVariable` need to find a tangent.
                    // Bind it to a VariableNode referencing the newly-introduced tangent name.
                    context.SetTangent(
                        assignment.TargetVariable,
                        new VariableNode(context.NewNodeId(), tangentVar, _doubleType));
                    break;
                }

                case ReturnNode returnNode:
                {
                    var tangentExpr = _differentiator.Differentiate(returnNode.Value, context);

                    expanded.Add(new AssignmentNode(context.NewNodeId(), primalResultName, returnNode.Value));
                    expanded.Add(new AssignmentNode(context.NewNodeId(), tangentResultName, tangentExpr));
                    expanded.Add(new ReturnNode(
                        context.NewNodeId(),
                        new VariableNode(context.NewNodeId(), primalResultName, _doubleType)));
                    break;
                }

                default:
                    // Conditionals, loops, and other control-flow statements are not
                    // expanded by v1 — Dynamis SDF primitives use straight-line code.
                    // Fall back to the existing differentiator's per-statement transform,
                    // which mutates the context but does not augment the body.
                    expanded.Add(stmt);
                    break;
            }
        }
    }

    /// <summary>
    /// Result of <c>ForwardModeIRExpander.Expand</c>: the augmented body
    /// plus the names of the two result variables it produces.
    /// </summary>
    public sealed record ExpandedMethodBody(
        MethodBodyNode Body,
        string PrimalResultVar,
        string TangentResultVar);
}
