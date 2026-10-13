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
    // The statement IR of array kernels, produced by KernelASTToIRConverter and consumed only by the
    // Kernels.LoopUnroller. The scalar pipeline (ASTToIRConverter, the differentiation rules and the code
    // generators) never sees these records, and bridges to other IRs reject them.

    /// <summary>
    /// A kernel or a helper it calls, as statement IR. <see cref="Parameters"/> holds a
    /// <see cref="VariableNode"/> for each scalar parameter and an <see cref="ArrayParameterNode"/> for each span.
    /// </summary>
    public sealed record KernelFunctionNode(
        int NodeId,
        IMethodSymbol Method,
        ImmutableArray<IRNode> Parameters,
        MethodBodyNode Body) : IRNode(NodeId);

    /// <summary>
    /// A span parameter. A kernel's spans declare <see cref="FixedLength"/> with <c>[FixedLength(n)]</c>; a helper's
    /// spans take the length of the caller's argument, so theirs is null.
    /// </summary>
    public sealed record ArrayParameterNode(
        int NodeId,
        string Name,
        ITypeSymbol ElementType,
        int? FixedLength,
        bool IsWritable) : IRNode(NodeId);

    /// <summary>
    /// A declared local: a scalar (an <c>int</c> or <c>bool</c> is a compile-time value to the unroller) or a span
    /// bound to an array expression. <see cref="Value"/> is null for a declaration without an initialiser.
    /// </summary>
    public sealed record LocalDeclarationNode(int NodeId, string Name, ITypeSymbol Type, IRNode? Value) : IRNode(NodeId);

    /// <summary>
    /// A new array: <c>stackalloc</c>, <c>new T[]</c>, a collection expression, or a static read-only table.
    /// <see cref="Dimensions"/> has one entry, or two for a rectangular table. <see cref="Elements"/> is empty for a
    /// zero-filled array, and otherwise lists every element in row-major order. <see cref="Name"/> is the table's
    /// name, or "array" for one created inline.
    /// </summary>
    public sealed record ArrayCreationNode(
        int NodeId,
        string Name,
        ITypeSymbol ElementType,
        ImmutableArray<IRNode> Dimensions,
        ImmutableArray<IRNode> Elements,
        bool IsReadOnly) : IRNode(NodeId);

    /// <summary>An element read <c>a[i]</c> or <c>a[i, j]</c>.</summary>
    public sealed record ElementAccessNode(
        int NodeId,
        IRNode Array,
        ImmutableArray<IRNode> Indices,
        ITypeSymbol Type) : IRNode(NodeId);

    /// <summary>An element write <c>a[i] = value</c>; compound assignments are lowered to a read and an operator.</summary>
    public sealed record ElementAssignmentNode(
        int NodeId,
        IRNode Array,
        ImmutableArray<IRNode> Indices,
        IRNode Value) : IRNode(NodeId);

    /// <summary>A view <c>a.Slice(start, length)</c>, <c>a.Slice(start)</c> or <c>a[start..end]</c> of a span.</summary>
    public sealed record ArraySliceNode(int NodeId, IRNode Array, IRNode Start, IRNode? Length) : IRNode(NodeId);

    /// <summary>A span's <c>Length</c>, which is always a compile-time integer.</summary>
    public sealed record ArrayLengthNode(int NodeId, IRNode Array) : IRNode(NodeId);

    /// <summary>A span method used as a statement: <c>Clear()</c>, <c>Fill(value)</c> or <c>CopyTo(destination)</c>.</summary>
    public sealed record ArrayOperationNode(
        int NodeId,
        ArrayOperation Operation,
        IRNode Array,
        ImmutableArray<IRNode> Arguments) : IRNode(NodeId);

    /// <summary><c>break</c> inside a loop.</summary>
    public sealed record BreakNode(int NodeId) : IRNode(NodeId);

    /// <summary><c>continue</c> inside a loop.</summary>
    public sealed record ContinueNode(int NodeId) : IRNode(NodeId);

    /// <summary><c>return;</c> from a void kernel or helper. A value return is a <see cref="ReturnNode"/>.</summary>
    public sealed record VoidReturnNode(int NodeId) : IRNode(NodeId);

    /// <summary>The span methods an array kernel may call as statements.</summary>
    public enum ArrayOperation
    {
        Clear,
        Fill,
        CopyTo
    }
}
