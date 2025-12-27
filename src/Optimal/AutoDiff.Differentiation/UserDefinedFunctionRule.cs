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
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Differentiation
{
    public sealed class UserDefinedFunctionRule : IDifferentiationRule
    {
        private readonly ForwardModeDifferentiator _differentiator;
        private readonly Dictionary<string, MethodBodyNode> _methodCache;
        private readonly OptimalTransform _transform;

        public UserDefinedFunctionRule(
            ForwardModeDifferentiator differentiator,
            ITypeSymbol doubleType,
            OptimalTransform transform)
        {
            _differentiator = differentiator;
            _methodCache = new Dictionary<string, MethodBodyNode>();
            _transform = transform;
        }

        public bool CanDifferentiate(IRNode node)
        {
            if (node is not MethodCallNode methodCall)
            {
                return false;
            }

            // Check if this is a user-defined function from the current class
            var containingType = methodCall.MethodSymbol?.ContainingType;
            if (containingType == null)
            {
                return false;
            }

            // It's a user-defined function if it's in the same class we're processing
            return SymbolEqualityComparer.Default.Equals(containingType, _transform.ClassSymbol);
        }

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            var methodCall = (MethodCallNode)node;

            // Find the method definition
            var method = _transform.Methods.FirstOrDefault(m => m.MethodName == methodCall.MethodName);
            if (method == null)
            {
                throw new InvalidOperationException($"Method not found: {methodCall.MethodName}");
            }

            // Inline the function by substituting arguments
            var inlinedBody = InlineFunction(method, methodCall.Arguments, context);

            // The inlined body should contain a single return statement
            // Extract its value and differentiate it
            if (inlinedBody.Statements.Length == 0)
            {
                throw new InvalidOperationException($"Empty method body for: {methodCall.MethodName}");
            }

            var lastStatement = inlinedBody.Statements.Last();
            if (lastStatement is ReturnNode returnNode)
            {
                return _differentiator.Differentiate(returnNode.Value, context);
            }

            // If there are multiple statements, we need to process them in order
            // and track variable tangents
            foreach (var statement in inlinedBody.Statements.Take(inlinedBody.Statements.Length - 1))
            {
                if (statement is AssignmentNode assignment)
                {
                    var tangent = _differentiator.Differentiate(assignment.Value, context);
                    context.SetTangent(assignment.TargetVariable, tangent);
                }
            }

            // Now process the final statement
            lastStatement = inlinedBody.Statements.Last();
            if (lastStatement is ReturnNode finalReturn)
            {
                return _differentiator.Differentiate(finalReturn.Value, context);
            }

            throw new NotSupportedException($"Unsupported function body structure in: {methodCall.MethodName}");
        }

        private MethodBodyNode InlineFunction(
            MethodToTransform method,
            ImmutableArray<IRNode> arguments,
            ForwardModeContext context)
        {
            // Get or convert the method body to IR
            var cacheKey = method.MethodName;
            if (!_methodCache.TryGetValue(cacheKey, out var methodBody))
            {
                var converter = new ASTToIRConverter(method.SemanticModel);
                methodBody = converter.ConvertMethod(method.MethodSyntax, method.Parameters);
                _methodCache[cacheKey] = methodBody;
            }

            // Create a mapping from parameters to arguments
            var substitutions = new Dictionary<string, IRNode>();
            for (var i = 0; i < method.Parameters.Length; i++)
            {
                substitutions[method.Parameters[i].Name] = arguments[i];
            }

            // Substitute parameters with arguments in the method body
            var substitutedStatements = methodBody.Statements
                .Select(stmt => SubstituteVariables(stmt, substitutions))
                .ToImmutableArray();

            return new MethodBodyNode(context.NewNodeId(), substitutedStatements);
        }

        private IRNode SubstituteVariables(IRNode node, Dictionary<string, IRNode> substitutions)
        {
            return node switch
            {
                VariableNode variable => substitutions.TryGetValue(variable.Name, out var replacement)
                    ? replacement
                    : variable,
                ConstantNode constant => constant,
                BinaryOpNode binary => new BinaryOpNode(
                    binary.NodeId,
                    binary.Op,
                    SubstituteVariables(binary.Left, substitutions),
                    SubstituteVariables(binary.Right, substitutions),
                    binary.Type),
                UnaryOpNode unary => new UnaryOpNode(
                    unary.NodeId,
                    unary.Op,
                    SubstituteVariables(unary.Operand, substitutions),
                    unary.Type),
                MethodCallNode methodCall => new MethodCallNode(
                    methodCall.NodeId,
                    methodCall.MethodName,
                    methodCall.MethodSymbol,
                    methodCall.Arguments.Select(arg => SubstituteVariables(arg, substitutions)).ToImmutableArray(),
                    methodCall.Type),
                AssignmentNode assignment => new AssignmentNode(
                    assignment.NodeId,
                    assignment.TargetVariable,
                    SubstituteVariables(assignment.Value, substitutions)),
                ReturnNode returnNode => new ReturnNode(
                    returnNode.NodeId,
                    SubstituteVariables(returnNode.Value, substitutions)),
                ConditionalExpressionNode conditional => new ConditionalExpressionNode(
                    conditional.NodeId,
                    SubstituteVariables(conditional.Condition, substitutions),
                    SubstituteVariables(conditional.TrueExpression, substitutions),
                    SubstituteVariables(conditional.FalseExpression, substitutions),
                    conditional.Type),
                ConditionalNode conditionalNode => new ConditionalNode(
                    conditionalNode.NodeId,
                    SubstituteVariables(conditionalNode.Condition, substitutions),
                    conditionalNode.TrueBranch.Select(stmt => SubstituteVariables(stmt, substitutions)).ToImmutableArray(),
                    conditionalNode.FalseBranch.Select(stmt => SubstituteVariables(stmt, substitutions)).ToImmutableArray(),
                    conditionalNode.Type),
                LoopNode loop => new LoopNode(
                    loop.NodeId,
                    loop.IteratorVariable,
                    loop.Initializer != null ? SubstituteVariables(loop.Initializer, substitutions) : null,
                    loop.Condition != null ? SubstituteVariables(loop.Condition, substitutions) : null,
                    loop.Increment != null ? SubstituteVariables(loop.Increment, substitutions) : null,
                    loop.Body.Select(stmt => SubstituteVariables(stmt, substitutions)).ToImmutableArray(),
                    loop.Type),
                MethodBodyNode methodBody => new MethodBodyNode(
                    methodBody.NodeId,
                    methodBody.Statements.Select(stmt => SubstituteVariables(stmt, substitutions)).ToImmutableArray()),
                _ => throw new NotSupportedException($"Cannot substitute variables in node type: {node.GetType().Name}")
            };
        }
    }
}
