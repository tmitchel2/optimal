/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Differentiation
{
    /// <summary>
    /// Phase D: differentiate calls into subordinate SDF evaluators
    /// (<c>evalChild(arg)</c>) where the child's evaluator is supplied
    /// opaquely by the parent emitter. Matches <see cref="MethodCallNode"/>s
    /// whose <c>MethodName</c> carries the <c>__childEval_</c> prefix and
    /// whose <c>MethodSymbol</c> is null (the bridge-injected marker for
    /// child SDFs).
    ///
    /// Differentiation strategy:
    /// <list type="bullet">
    /// <item>The primal stays as the original <c>__childEval_NAME(arg)</c>
    ///   call. The downstream WGSL emitter substitutes it with a call into
    ///   the parent's child-value-emit helper.</item>
    /// <item>The tangent (w.r.t. some upstream parameter) follows the chain
    ///   rule: <c>∂child/∂p = ∇child(arg) · ∂arg/∂p</c>. The three
    ///   <c>∇child(arg)</c> components are emitted as opaque placeholder
    ///   calls <c>__childGrad_NAME_x/y/z(arg)</c>, which the WGSL emitter
    ///   substitutes with the parent's child-gradient-emit output.</item>
    /// </list>
    ///
    /// Argument shape: in practice the argument is a vector point (vec3 or
    /// vec2) that the upstream <c>VectorScalarizer</c> has already broken into
    /// per-axis scalar arguments. The single Argument expression we see here
    /// is the scalarised composite — typically a chain of additions and
    /// multiplications. <c>Differentiate(arg, context)</c> from the parent
    /// differentiator gives us the scalar tangent of that composite, which is
    /// dotted against a SINGLE child-grad placeholder via the
    /// <c>__childGrad_</c> identifier.
    /// </summary>
    public sealed class ChildEvalRule : IDifferentiationRule
    {
        public const string ChildEvalPrefix = "__childEval_";
        public const string ChildGradPrefix = "__childGrad_";

        private readonly ForwardModeDifferentiator _differentiator;
        private readonly ITypeSymbol _doubleType;

        public ChildEvalRule(ForwardModeDifferentiator differentiator, ITypeSymbol doubleType)
        {
            _differentiator = differentiator;
            _doubleType = doubleType;
        }

        public bool CanDifferentiate(IRNode node)
        {
            return node is MethodCallNode mc
                && mc.MethodSymbol is null
                && mc.MethodName.StartsWith(ChildEvalPrefix, StringComparison.Ordinal);
        }

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            var call = (MethodCallNode)node;
            var childName = call.MethodName.Substring(ChildEvalPrefix.Length);
            if (call.Arguments.Length != 1)
            {
                throw new NotSupportedException(
                    $"Child-eval call '{call.MethodName}' must take exactly one (scalarised) argument; got {call.Arguments.Length}.");
            }
            var arg = call.Arguments[0];

            // Chain rule: ∂child/∂p = childGrad(arg) · ∂arg/∂p.
            // For the v1 single-axis scalar form, that's just one product —
            // the parent's WGSL emitter expands the placeholder into the full
            // dot product when it knows the vec3 axis structure of `arg`.
            var dArg = _differentiator.Differentiate(arg, context);

            var gradPlaceholder = new MethodCallNode(
                context.NewNodeId(),
                ChildGradPrefix + childName,
                MethodSymbol: null,
                Arguments: ImmutableArray.Create(arg),
                Type: _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                gradPlaceholder,
                dArg,
                _doubleType);
        }
    }
}
