/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;

namespace Optimal.Analyzers.IR
{
    public abstract record IRNode(int NodeId);

    public sealed record VariableNode(int NodeId, string Name, ITypeSymbol Type) : IRNode(NodeId);

    public sealed record ConstantNode(int NodeId, object Value, ITypeSymbol Type) : IRNode(NodeId);

    public sealed record BinaryOpNode(
        int NodeId,
        BinaryOperator Op,
        IRNode Left,
        IRNode Right,
        ITypeSymbol Type) : IRNode(NodeId);

    public sealed record UnaryOpNode(
        int NodeId,
        UnaryOperator Op,
        IRNode Operand,
        ITypeSymbol Type) : IRNode(NodeId);

    public sealed record AssignmentNode(
        int NodeId,
        string TargetVariable,
        IRNode Value) : IRNode(NodeId);

    public sealed record ReturnNode(int NodeId, IRNode Value) : IRNode(NodeId);

    public sealed record MethodCallNode(
        int NodeId,
        string MethodName,
        IMethodSymbol? MethodSymbol,
        ImmutableArray<IRNode> Arguments,
        ITypeSymbol Type) : IRNode(NodeId);

    public sealed record ConditionalNode(
        int NodeId,
        IRNode Condition,
        ImmutableArray<IRNode> TrueBranch,
        ImmutableArray<IRNode> FalseBranch,
        ITypeSymbol Type) : IRNode(NodeId);

    public sealed record ConditionalExpressionNode(
        int NodeId,
        IRNode Condition,
        IRNode TrueExpression,
        IRNode FalseExpression,
        ITypeSymbol Type) : IRNode(NodeId);

    public sealed record LoopNode(
        int NodeId,
        string IteratorVariable,
        IRNode? Initializer,
        IRNode? Condition,
        IRNode? Increment,
        ImmutableArray<IRNode> Body,
        ITypeSymbol Type) : IRNode(NodeId);

    public sealed record MethodBodyNode(int NodeId, ImmutableArray<IRNode> Statements) : IRNode(NodeId);

    public enum BinaryOperator
    {
        Add,
        Subtract,
        Multiply,
        Divide,
        Power,
        GreaterThan,
        LessThan,
        GreaterThanOrEqual,
        LessThanOrEqual,
        Equal,
        NotEqual
    }

    public enum UnaryOperator
    {
        Negate,
        Identity,
        Plus
    }
}
