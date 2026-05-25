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
                ExpandStatement(stmt, context, expanded);
            }

            return new ExpandedMethodBody(
                new MethodBodyNode(context.NewNodeId(), expanded.ToImmutable()),
                PrimalResultName,
                TangentResultName);
        }

        private void ExpandStatement(IRNode stmt, ForwardModeContext context, ImmutableArray<IRNode>.Builder expanded)
        {
            switch (stmt)
            {
                case AssignmentNode assignment:
                {
                    var tangentExpr = _differentiator.Differentiate(assignment.Value, context);
                    var tangentVar = assignment.TargetVariable + TangentSuffix;

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

                    expanded.Add(new AssignmentNode(context.NewNodeId(), PrimalResultName, returnNode.Value));
                    expanded.Add(new AssignmentNode(context.NewNodeId(), TangentResultName, tangentExpr));
                    expanded.Add(new ReturnNode(
                        context.NewNodeId(),
                        new VariableNode(context.NewNodeId(), PrimalResultName, _doubleType)));
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
    /// Result of <see cref="ForwardModeIRExpander.Expand"/>: the augmented
    /// body plus the names of the two result variables it produces.
    /// </summary>
    public sealed record ExpandedMethodBody(
        MethodBodyNode Body,
        string PrimalResultVar,
        string TangentResultVar);
}
