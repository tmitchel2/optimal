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
    /// Differentiates <see cref="MinReduceNode"/> by the subdifferential rule:
    /// the gradient of <c>min(b_0, b_1, …, b_{N-1})</c> is the gradient of whichever
    /// element <c>b_i</c> achieves the min. At the medial axis (where the argmin
    /// changes) this is a C0 discontinuity — propagates downstream into the sphere-tracer.
    ///
    /// Per-iteration emission: a <see cref="ConditionalExpressionNode"/> chain that
    /// selects each iteration's body-derivative based on whether that iteration achieved
    /// the running min. The chain is built right-to-left so the final selector picks the
    /// gradient of the winning iteration:
    /// <code>
    ///   grad = body_grad[N-1]
    ///   for i from N-2 down to 0:
    ///     grad = (body_value[i] &lt;= running_min[i+1]) ? body_grad[i] : grad
    /// </code>
    /// Body-value and body-grad are obtained per iteration by substituting the index
    /// literal into the body sub-expression and re-invoking the differentiator (same
    /// pattern as IFTRule for the Newton residual).
    /// </summary>
    public sealed class MinReduceRule : IDifferentiationRule
    {
        private readonly ForwardModeDifferentiator _differentiator;
        private readonly ITypeSymbol _doubleType;

        public MinReduceRule(ForwardModeDifferentiator differentiator, ITypeSymbol doubleType)
        {
            _differentiator = differentiator;
            _doubleType = doubleType;
        }

        public bool CanDifferentiate(IRNode node) => node is MinReduceNode;

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            var mr = (MinReduceNode)node;
            var wrtParam = context.WithRespectTo;

            if (mr.Count <= 0)
            {
                return new ConstantNode(context.NewNodeId(), 0.0, _doubleType);
            }

            // Compute per-iteration body values + derivatives by substituting the index literal
            // into the body and running AD on the substituted body. The index parameter itself
            // has zero tangent (it's an integer counter).
            var perIterValues = new IRNode[mr.Count];
            var perIterGrads = new IRNode[mr.Count];

            for (var i = 0; i < mr.Count; i++)
            {
                var indexLiteral = new ConstantNode(context.NewNodeId(), (double)i, _doubleType);
                var substitutedBody = SubstituteVariable(mr.Body, mr.IndexParamName, indexLiteral, context);

                perIterValues[i] = substitutedBody;

                // Differentiate the substituted body w.r.t. the outer wrt-parameter.
                var iterCtx = new ForwardModeContext(context.NewNodeId(), wrtParam);
                SeedTangents(iterCtx, wrtParam, mr.PParamNames);
                iterCtx.SetTangent(mr.IndexParamName, new ConstantNode(iterCtx.NewNodeId(), 0.0, _doubleType));
                perIterGrads[i] = _differentiator.Differentiate(substitutedBody, iterCtx);
            }

            // Build the running-min value chain to use as the selection condition.
            // runningMin[k] = min(perIterValues[k..N-1])
            var runningMin = new IRNode[mr.Count];
            runningMin[mr.Count - 1] = perIterValues[mr.Count - 1];
            for (var i = mr.Count - 2; i >= 0; i--)
            {
                runningMin[i] = MinCall(perIterValues[i], runningMin[i + 1], context);
            }

            // Build the gradient chain right-to-left:
            //   grad = perIterGrads[N-1]
            //   for i from N-2 down to 0:
            //     grad = (perIterValues[i] <= runningMin[i+1]) ? perIterGrads[i] : grad
            var grad = perIterGrads[mr.Count - 1];
            for (var i = mr.Count - 2; i >= 0; i--)
            {
                var cond = new BinaryOpNode(context.NewNodeId(), BinaryOperator.LessThanOrEqual,
                    perIterValues[i], runningMin[i + 1], _doubleType);
                grad = new ConditionalExpressionNode(context.NewNodeId(), cond, perIterGrads[i], grad, _doubleType);
            }

            return grad;
        }

        private static MethodCallNode MinCall(IRNode a, IRNode b, ForwardModeContext context)
        {
            // Emit System.Math.Min equivalent. MethodSymbol=null is fine here — we're
            // building a tangent expression that won't be further AD'd.
            return new MethodCallNode(
                context.NewNodeId(),
                "Min",
                MethodSymbol: null,
                Arguments: ImmutableArray.Create(a, b),
                Type: a is BinaryOpNode bin ? bin.Type : (a is ConstantNode c ? c.Type : (a is VariableNode v ? v.Type : null!)));
        }

        private static IRNode SubstituteVariable(IRNode node, string targetName, IRNode replacement, ForwardModeContext context)
        {
            return node switch
            {
                VariableNode v when v.Name == targetName => replacement,
                VariableNode or ConstantNode => node,
                BinaryOpNode bin => new BinaryOpNode(
                    context.NewNodeId(),
                    bin.Op,
                    SubstituteVariable(bin.Left, targetName, replacement, context),
                    SubstituteVariable(bin.Right, targetName, replacement, context),
                    bin.Type),
                UnaryOpNode un => new UnaryOpNode(
                    context.NewNodeId(),
                    un.Op,
                    SubstituteVariable(un.Operand, targetName, replacement, context),
                    un.Type),
                MethodCallNode mc => SubstituteCall(mc, targetName, replacement, context),
                ConditionalExpressionNode ce => new ConditionalExpressionNode(
                    context.NewNodeId(),
                    SubstituteVariable(ce.Condition, targetName, replacement, context),
                    SubstituteVariable(ce.TrueExpression, targetName, replacement, context),
                    SubstituteVariable(ce.FalseExpression, targetName, replacement, context),
                    ce.Type),
                _ => node
            };
        }

        private static MethodCallNode SubstituteCall(MethodCallNode mc, string targetName, IRNode replacement, ForwardModeContext context)
        {
            var newArgs = ImmutableArray.CreateBuilder<IRNode>(mc.Arguments.Length);
            foreach (var a in mc.Arguments)
            {
                newArgs.Add(SubstituteVariable(a, targetName, replacement, context));
            }
            return new MethodCallNode(context.NewNodeId(), mc.MethodName, mc.MethodSymbol, newArgs.ToImmutable(), mc.Type);
        }

        private void SeedTangents(ForwardModeContext ctx, string wrt, ImmutableArray<string> pParams)
        {
            ctx.SetTangent(wrt, new ConstantNode(ctx.NewNodeId(), 1.0, _doubleType));
            foreach (var p in pParams)
            {
                if (p != wrt)
                {
                    ctx.SetTangent(p, new ConstantNode(ctx.NewNodeId(), 0.0, _doubleType));
                }
            }
        }
    }
}
