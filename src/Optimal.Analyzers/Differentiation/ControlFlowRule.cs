/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Immutable;
using System.Linq;
using Optimal.Analyzers.IR;

namespace Optimal.Analyzers.Differentiation
{
    public sealed class ControlFlowRule : IDifferentiationRule
    {
        private readonly ForwardModeDifferentiator _differentiator;

        public ControlFlowRule(ForwardModeDifferentiator differentiator)
        {
            _differentiator = differentiator;
        }

        public bool CanDifferentiate(IRNode node)
        {
            return node is ConditionalNode or ConditionalExpressionNode or LoopNode;
        }

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            return node switch
            {
                ConditionalNode conditional => DifferentiateConditional(conditional, context),
                ConditionalExpressionNode conditionalExpr => DifferentiateConditionalExpression(conditionalExpr, context),
                LoopNode loop => DifferentiateLoop(loop, context),
                _ => throw new NotSupportedException($"Control flow node type not supported: {node.GetType().Name}")
            };
        }

        private IRNode DifferentiateConditional(ConditionalNode conditional, ForwardModeContext context)
        {
            var trueBranch = conditional.TrueBranch
                .Select(stmt => DifferentiateStatement(stmt, context))
                .ToImmutableArray();

            var falseBranch = conditional.FalseBranch
                .Select(stmt => DifferentiateStatement(stmt, context))
                .ToImmutableArray();

            return new ConditionalNode(
                context.NewNodeId(),
                conditional.Condition,
                trueBranch,
                falseBranch,
                conditional.Type);
        }

        private IRNode DifferentiateConditionalExpression(ConditionalExpressionNode conditional, ForwardModeContext context)
        {
            var trueExpr = _differentiator.Differentiate(conditional.TrueExpression, context);
            var falseExpr = _differentiator.Differentiate(conditional.FalseExpression, context);

            return new ConditionalExpressionNode(
                context.NewNodeId(),
                conditional.Condition,
                trueExpr,
                falseExpr,
                conditional.Type);
        }

        private IRNode DifferentiateLoop(LoopNode loop, ForwardModeContext context)
        {
            var body = loop.Body
                .Select(stmt => DifferentiateStatement(stmt, context))
                .ToImmutableArray();

            return new LoopNode(
                context.NewNodeId(),
                loop.IteratorVariable,
                loop.Initializer,
                loop.Condition,
                loop.Increment,
                body,
                loop.Type);
        }

        private IRNode DifferentiateStatement(IRNode statement, ForwardModeContext context)
        {
            return statement switch
            {
                AssignmentNode assignment => DifferentiateAssignment(assignment, context),
                ReturnNode returnNode => returnNode,
                ConditionalNode conditional => DifferentiateConditional(conditional, context),
                LoopNode loop => DifferentiateLoop(loop, context),
                _ => throw new NotSupportedException($"Statement type not supported for differentiation: {statement.GetType().Name}")
            };
        }

        private IRNode DifferentiateAssignment(AssignmentNode assignment, ForwardModeContext context)
        {
            var dValue = _differentiator.Differentiate(assignment.Value, context);
            context.SetTangent(assignment.TargetVariable, dValue);
            return assignment;
        }
    }
}
