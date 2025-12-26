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
using Microsoft.CodeAnalysis;
using Optimal.Analyzers.IR;

namespace Optimal.Analyzers.Differentiation
{
    public sealed class ForwardModeDifferentiator
    {
        private readonly List<IDifferentiationRule> _rules = new();
        private readonly ITypeSymbol _doubleType;

        public ForwardModeDifferentiator(ITypeSymbol doubleType)
        {
            _doubleType = doubleType;
            _rules.Add(new BinaryOpRule(this, doubleType));
            _rules.Add(new MathFunctionRule(this, doubleType));
            _rules.Add(new ControlFlowRule(this));
        }

        public MethodBodyNode DifferentiateMethod(MethodBodyNode methodBody, string wrtParameter, int startNodeId)
        {
            var context = new ForwardModeContext(startNodeId, wrtParameter);

            context.SetTangent(wrtParameter, new ConstantNode(context.NewNodeId(), 1.0, _doubleType));

            var differentiatedStatements = methodBody.Statements
                .Select(stmt => DifferentiateStatement(stmt, context))
                .ToImmutableArray();

            return new MethodBodyNode(context.NewNodeId(), differentiatedStatements);
        }

        private IRNode DifferentiateStatement(IRNode statement, ForwardModeContext context)
        {
            return statement switch
            {
                AssignmentNode assignment => DifferentiateAssignment(assignment, context),
                ReturnNode returnNode => DifferentiateReturn(returnNode, context),
                ConditionalNode => DifferentiateUsingRules(statement, context),
                _ => throw new NotSupportedException($"Statement type not supported for differentiation: {statement.GetType().Name}")
            };
        }

        private IRNode DifferentiateAssignment(AssignmentNode assignment, ForwardModeContext context)
        {
            var dValue = Differentiate(assignment.Value, context);
            context.SetTangent(assignment.TargetVariable, dValue);

            return assignment;
        }

        private IRNode DifferentiateReturn(ReturnNode returnNode, ForwardModeContext context)
        {
            return returnNode;
        }

        public IRNode Differentiate(IRNode node, ForwardModeContext context)
        {
            return node switch
            {
                ConstantNode => new ConstantNode(context.NewNodeId(), 0.0, _doubleType),
                VariableNode variable => DifferentiateVariable(variable, context),
                BinaryOpNode => DifferentiateUsingRules(node, context),
                UnaryOpNode unary => DifferentiateUnary(unary, context),
                MethodCallNode => DifferentiateUsingRules(node, context),
                ConditionalExpressionNode => DifferentiateUsingRules(node, context),
                _ => throw new NotSupportedException($"Node type not supported for differentiation: {node.GetType().Name}")
            };
        }

        private IRNode DifferentiateVariable(VariableNode variable, ForwardModeContext context)
        {
            if (variable.Name == context.WithRespectTo)
            {
                return new ConstantNode(context.NewNodeId(), 1.0, _doubleType);
            }

            if (context.HasTangent(variable.Name))
            {
                return context.GetTangent(variable.Name);
            }

            return new ConstantNode(context.NewNodeId(), 0.0, _doubleType);
        }

        private IRNode DifferentiateUnary(UnaryOpNode unary, ForwardModeContext context)
        {
            var dOperand = Differentiate(unary.Operand, context);

            return unary.Op switch
            {
                UnaryOperator.Negate => new UnaryOpNode(context.NewNodeId(), UnaryOperator.Negate, dOperand, unary.Type),
                UnaryOperator.Plus or UnaryOperator.Identity => dOperand,
                _ => throw new NotSupportedException($"Unary operator not supported: {unary.Op}")
            };
        }

        private IRNode DifferentiateUsingRules(IRNode node, ForwardModeContext context)
        {
            foreach (var rule in _rules)
            {
                if (rule.CanDifferentiate(node))
                {
                    return rule.DifferentiateForward(node, context);
                }
            }

            throw new NotSupportedException($"No differentiation rule found for node type: {node.GetType().Name}");
        }
    }
}
