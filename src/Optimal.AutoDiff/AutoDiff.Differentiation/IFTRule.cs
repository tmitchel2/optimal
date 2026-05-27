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
    /// Differentiates <see cref="NewtonSolveNode"/> via the implicit-function theorem.
    /// At the converged point <c>f(t*, p) = 0</c>:
    /// <code>
    ///   ∂t*/∂p_i = -(∂f/∂t)⁻¹ · ∂f/∂p_i
    /// </code>
    /// Both partials are obtained by re-invoking the differentiator on the residual
    /// sub-expression with a fresh context — <c>t</c> tangent = 1 for the first, the
    /// outer wrt-parameter's tangent = 1 for the second. The remaining residual
    /// parameters get zero tangent so they don't pollute the derivative calculation.
    ///
    /// This is the principled answer to the "AD through Newton iteration" problem:
    /// the unrolled-AD tangent depends on iteration count and starting guess
    /// (mathematically wrong), and Hyperdual2-through-iterations balloons WGSL output
    /// (impractical — see the Bezier experiment in
    /// docs/plans/WGSL_ZERO_FD_PRODUCTION_PLAN.md). IFT gives the closed-form
    /// derivative at the converged point in O(re-differentiation of the residual).
    /// </summary>
    public sealed class IFTRule : IDifferentiationRule
    {
        private readonly ForwardModeDifferentiator _differentiator;
        private readonly ITypeSymbol _doubleType;

        public IFTRule(ForwardModeDifferentiator differentiator, ITypeSymbol doubleType)
        {
            _differentiator = differentiator;
            _doubleType = doubleType;
        }

        public bool CanDifferentiate(IRNode node) => node is NewtonSolveNode;

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            var ns = (NewtonSolveNode)node;
            var wrtParam = context.WithRespectTo;

            // ∂f/∂t: re-differentiate the residual with t as the wrt-parameter.
            var ctxT = new ForwardModeContext(context.NewNodeId(), ns.TParamName);
            SeedTangents(ctxT, ns.TParamName, ns.PParamNames);
            var dfDt = _differentiator.Differentiate(ns.ResidualBody, ctxT);

            // ∂f/∂wrt: re-differentiate with the outer wrt-parameter as the wrt-parameter.
            // If wrt isn't one of the residual's free parameters, the result is zero
            // (the converged t* doesn't depend on a variable the residual doesn't reference).
            if (!ContainsParameter(ns.PParamNames, wrtParam))
            {
                return new ConstantNode(context.NewNodeId(), 0.0, _doubleType);
            }

            var ctxP = new ForwardModeContext(context.NewNodeId(), wrtParam);
            SeedTangents(ctxP, wrtParam, ns.PParamNames);
            // Also seed the t parameter as zero — t doesn't move as we vary wrtParam at the
            // converged point (that's the whole point of IFT).
            ctxP.SetTangent(ns.TParamName, new ConstantNode(ctxP.NewNodeId(), 0.0, _doubleType));
            var dfDwrt = _differentiator.Differentiate(ns.ResidualBody, ctxP);

            // Emit: -dfDwrt / dfDt
            var negated = new UnaryOpNode(context.NewNodeId(), UnaryOperator.Negate, dfDwrt, _doubleType);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide, negated, dfDt, _doubleType);
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

        private static bool ContainsParameter(ImmutableArray<string> pParams, string name)
        {
            foreach (var p in pParams)
            {
                if (p == name) return true;
            }
            return false;
        }
    }
}
