/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using Microsoft.CodeAnalysis;

namespace Optimal.AutoDiff.Analyzers.IR
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

    /// <summary>
    /// Models the converged result of a Newton iteration: <c>t* = NewtonSolve(initialGuess, residual)</c>
    /// where <c>residual(t, params...) = 0</c> at convergence. Used by SDFs that solve a closest-
    /// point problem (e.g. cubic Bezier distance via <c>dot(B(t)-p, B'(t)) = 0</c>) where the
    /// straightforward "AD through the Newton iteration" approach is both mathematically wrong
    /// (derivative depends on iteration count and starting guess) and impractically expensive
    /// (Hyperdual2-through-iterations balloons WGSL output by 100×+).
    ///
    /// The <see cref="Differentiation.IFTRule"/> differentiates this via the implicit-function
    /// theorem: <c>∂t*/∂p = -(∂f/∂t)⁻¹ · ∂f/∂p</c>, where <c>f</c> is the residual. This gives
    /// the closed-form derivative at the converged point in two scalar divisions plus the
    /// re-differentiation of the residual sub-expression, independent of <see cref="IterationCount"/>.
    /// </summary>
    public sealed record NewtonSolveNode(
        int NodeId,
        IRNode InitialGuess,
        string TParamName,
        ImmutableArray<string> PParamNames,
        IRNode ResidualBody,
        int IterationCount,
        ITypeSymbol Type) : IRNode(NodeId);

    /// <summary>
    /// Models <c>min(body(0), body(1), …, body(count − 1))</c> where <c>body</c> is an IR
    /// sub-expression parameterised by the index variable. Used by SDFs that select the
    /// closest of several candidate distances (e.g. Bezier curve = min over segments of
    /// per-segment closest-point distance).
    ///
    /// At AD time, the gradient is the gradient of whichever index achieves the min — the
    /// subdifferential. The C0 discontinuity at the medial axis (where the argmin changes)
    /// propagates into the sphere-tracer's step rule; that's the same overshoot the
    /// downstream Bezier-overshoot project addresses.
    /// </summary>
    public sealed record MinReduceNode(
        int NodeId,
        int Count,
        string IndexParamName,
        ImmutableArray<string> PParamNames,
        IRNode Body,
        ITypeSymbol Type) : IRNode(NodeId);

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
