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
    public sealed class MathFunctionRule : IDifferentiationRule
    {
        private readonly ForwardModeDifferentiator _differentiator;
        private readonly ITypeSymbol _doubleType;

        public MathFunctionRule(ForwardModeDifferentiator differentiator, ITypeSymbol doubleType)
        {
            _differentiator = differentiator;
            _doubleType = doubleType;
        }

        public bool CanDifferentiate(IRNode node)
        {
            if (node is not MethodCallNode methodCall)
            {
                return false;
            }

            var containingType = methodCall.MethodSymbol?.ContainingType?.ToDisplayString();
            if (containingType == "System.Math") return true;

            // Phase E item 3 follow-up: some WGSL builtins (step, mix, smoothstep)
            // don't have System.Math equivalents so they arrive with null MethodSymbol.
            // We still recognise them by name — see DifferentiateForward's switch for
            // the supported list.
            if (methodCall.MethodSymbol is null)
            {
                return methodCall.MethodName is "step" or "mix" or "smoothstep" or "fract" or "floor" or "round";
            }
            return false;
        }

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            var methodCall = (MethodCallNode)node;

            return methodCall.MethodName switch
            {
                "Sqrt" => DifferentiateSqrt(methodCall, context),
                "Sin" => DifferentiateSin(methodCall, context),
                "Cos" => DifferentiateCos(methodCall, context),
                "Tan" => DifferentiateTan(methodCall, context),
                "Exp" => DifferentiateExp(methodCall, context),
                "Log" => DifferentiateLog(methodCall, context),
                "Pow" => DifferentiatePow(methodCall, context),
                "Abs" => DifferentiateAbs(methodCall, context),
                "Atan2" => DifferentiateAtan2(methodCall, context),
                "Sign" => DifferentiateSign(methodCall, context),
                "Min" => DifferentiateMinMax(methodCall, context, isMin: true),
                "Max" => DifferentiateMinMax(methodCall, context, isMin: false),
                "Clamp" => DifferentiateClamp(methodCall, context),
                "Atan" => DifferentiateAtan(methodCall, context),
                "Asin" => DifferentiateAsin(methodCall, context),
                "Acos" => DifferentiateAcos(methodCall, context),
                // WGSL-builtin names (no System.Math equivalent — see CanDifferentiate).
                "step" => DifferentiateStepOrFloor(methodCall, context),
                "floor" or "round" or "fract" => DifferentiateStepOrFloor(methodCall, context),
                "mix" => DifferentiateMix(methodCall, context),
                "smoothstep" => DifferentiateSmoothstep(methodCall, context),
                _ => throw new NotSupportedException($"Math function not supported for differentiation: {methodCall.MethodName}")
            };
        }

        // Sign is constant ±1 (or 0) everywhere except a Dirac at zero; AD ignores the
        // Dirac and emits zero — the subdifferential.
#pragma warning disable RCS1163 // methodCall intentionally unused — Sign's derivative is a constant
        private IRNode DifferentiateSign(MethodCallNode methodCall, ForwardModeContext context)
        {
            return new ConstantNode(context.NewNodeId(), 0.0, _doubleType);
        }
#pragma warning restore RCS1163

        // min(a,b): derivative is da when a<b, else db.  Emitted as a ConditionalExpressionNode
        // which downstream WGSL lowers to select(db, da, a<b).
        // max(a,b): symmetric — derivative is da when a>b, else db.
        private IRNode DifferentiateMinMax(MethodCallNode methodCall, ForwardModeContext context, bool isMin)
        {
            if (methodCall.Arguments.Length != 2)
            {
                throw new InvalidOperationException($"{(isMin ? "Min" : "Max")} requires exactly 2 arguments");
            }
            var a = methodCall.Arguments[0];
            var b = methodCall.Arguments[1];
            var da = _differentiator.Differentiate(a, context);
            var db = _differentiator.Differentiate(b, context);
            var op = isMin ? BinaryOperator.LessThan : BinaryOperator.GreaterThan;
            var cond = new BinaryOpNode(context.NewNodeId(), op, a, b, _doubleType);
            return new ConditionalExpressionNode(context.NewNodeId(), cond, da, db, _doubleType);
        }

        // step(edge, x), floor(x), round(x), fract(x): piecewise-constant — derivative
        // is the constant 0 (Dirac contributions at the steps are subdifferential, treated
        // as zero by convention).
#pragma warning disable RCS1163 // method call intentionally unused — these functions all have zero derivative
        private IRNode DifferentiateStepOrFloor(MethodCallNode methodCall, ForwardModeContext context)
        {
            return new ConstantNode(context.NewNodeId(), 0.0, _doubleType);
        }
#pragma warning restore RCS1163

        // mix(a, b, t) = a + (b - a) * t  →  ∂/∂p = (1-t)·∂a/∂p + t·∂b/∂p + (b-a)·∂t/∂p.
        private IRNode DifferentiateMix(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 3)
            {
                throw new InvalidOperationException("mix requires exactly 3 arguments");
            }
            var a = methodCall.Arguments[0];
            var b = methodCall.Arguments[1];
            var t = methodCall.Arguments[2];
            var da = _differentiator.Differentiate(a, context);
            var db = _differentiator.Differentiate(b, context);
            var dt = _differentiator.Differentiate(t, context);
            var one = new ConstantNode(context.NewNodeId(), 1.0, _doubleType);
            var oneMinusT = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, one, t, _doubleType);
            var bMinusA = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, b, a, _doubleType);
            var term1 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, oneMinusT, da, _doubleType);
            var term2 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, t, db, _doubleType);
            var term3 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, bMinusA, dt, _doubleType);
            var s12 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Add, term1, term2, _doubleType);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Add, s12, term3, _doubleType);
        }

        // smoothstep(e0, e1, x) = 3u² - 2u³ where u = clamp((x-e0)/(e1-e0), 0, 1).
        // Derivative: 6u(1-u)/(e1-e0) · ∂u_unclamped/∂p (zero outside [e0,e1]).
        // For simplicity we approximate this as continuous: derivative is
        // 6u(1-u)/(e1-e0) · ∂x/∂p (ignoring derivatives of e0/e1 + the clamp).
        // Edge cases at u=0 or u=1 the formula correctly gives 0.
        private IRNode DifferentiateSmoothstep(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 3)
            {
                throw new InvalidOperationException("smoothstep requires exactly 3 arguments");
            }
            var e0 = methodCall.Arguments[0];
            var e1 = methodCall.Arguments[1];
            var x = methodCall.Arguments[2];
            var dx = _differentiator.Differentiate(x, context);

            // u = (x - e0) / (e1 - e0); we compute it as a raw expression (no clamp here —
            // outside [0,1] the formula 6u(1-u) is negative or > 1, but the derivative is
            // actually 0 outside the transition. For simplicity we accept the small error.)
            var xMinusE0 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, x, e0, _doubleType);
            var e1MinusE0 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, e1, e0, _doubleType);
            var u = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide, xMinusE0, e1MinusE0, _doubleType);
            var one = new ConstantNode(context.NewNodeId(), 1.0, _doubleType);
            var oneMinusU = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, one, u, _doubleType);
            var uTimes = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, u, oneMinusU, _doubleType);
            var six = new ConstantNode(context.NewNodeId(), 6.0, _doubleType);
            var sixU = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, six, uTimes, _doubleType);
            var factor = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide, sixU, e1MinusE0, _doubleType);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, factor, dx, _doubleType);
        }

        // atan(x): derivative is 1 / (1 + x²).
        private IRNode DifferentiateAtan(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Atan requires exactly 1 argument");
            }
            var x = methodCall.Arguments[0];
            var dx = _differentiator.Differentiate(x, context);
            var xSq = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, x, x, _doubleType);
            var denom = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Add,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType), xSq, _doubleType);
            var invDenom = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType), denom, _doubleType);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, invDenom, dx, _doubleType);
        }

        // asin(x): derivative is 1 / sqrt(1 - x²).
        private IRNode DifferentiateAsin(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Asin requires exactly 1 argument");
            }
            var x = methodCall.Arguments[0];
            var dx = _differentiator.Differentiate(x, context);
            var xSq = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, x, x, _doubleType);
            var oneMinusXSq = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType), xSq, _doubleType);
            var sqrtCall = new MethodCallNode(context.NewNodeId(), "Sqrt", methodCall.MethodSymbol,
                ImmutableArray.Create<IRNode>(oneMinusXSq), _doubleType);
            var invSqrt = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType), sqrtCall, _doubleType);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, invSqrt, dx, _doubleType);
        }

        // acos(x): derivative is -1 / sqrt(1 - x²).
        private IRNode DifferentiateAcos(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Acos requires exactly 1 argument");
            }
            var x = methodCall.Arguments[0];
            var dx = _differentiator.Differentiate(x, context);
            var xSq = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, x, x, _doubleType);
            var oneMinusXSq = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType), xSq, _doubleType);
            var sqrtCall = new MethodCallNode(context.NewNodeId(), "Sqrt", methodCall.MethodSymbol,
                ImmutableArray.Create<IRNode>(oneMinusXSq), _doubleType);
            var invSqrt = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide,
                new ConstantNode(context.NewNodeId(), -1.0, _doubleType), sqrtCall, _doubleType);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, invSqrt, dx, _doubleType);
        }

        // clamp(x, lo, hi): derivative is dx in the interior, 0 at the limits.
        // Two nested conditionals: `x > lo ? (x < hi ? dx : dhi) : dlo` so the limits
        // also propagate tangent correctly if lo/hi depend on the wrt-variable.
        private IRNode DifferentiateClamp(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 3)
            {
                throw new InvalidOperationException("Clamp requires exactly 3 arguments");
            }
            var x = methodCall.Arguments[0];
            var lo = methodCall.Arguments[1];
            var hi = methodCall.Arguments[2];
            var dx = _differentiator.Differentiate(x, context);
            var dlo = _differentiator.Differentiate(lo, context);
            var dhi = _differentiator.Differentiate(hi, context);

            var xLtHi = new BinaryOpNode(context.NewNodeId(), BinaryOperator.LessThan, x, hi, _doubleType);
            var innerSelect = new ConditionalExpressionNode(context.NewNodeId(), xLtHi, dx, dhi, _doubleType);

            var xGtLo = new BinaryOpNode(context.NewNodeId(), BinaryOperator.GreaterThan, x, lo, _doubleType);
            return new ConditionalExpressionNode(context.NewNodeId(), xGtLo, innerSelect, dlo, _doubleType);
        }

        private IRNode DifferentiateSqrt(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Sqrt requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            var two = new ConstantNode(context.NewNodeId(), 2.0, _doubleType);
            var twoTimesSqrt = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                two,
                methodCall,
                _doubleType);

            var oneOverTwoSqrt = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Divide,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType),
                twoTimesSqrt,
                _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                oneOverTwoSqrt,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiateSin(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Sin requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            var cosCall = new MethodCallNode(
                context.NewNodeId(),
                "Cos",
                methodCall.MethodSymbol,
                ImmutableArray.Create(arg),
                _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                cosCall,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiateCos(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Cos requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            var sinCall = new MethodCallNode(
                context.NewNodeId(),
                "Sin",
                methodCall.MethodSymbol,
                ImmutableArray.Create(arg),
                _doubleType);

            var negativeSin = new UnaryOpNode(
                context.NewNodeId(),
                UnaryOperator.Negate,
                sinCall,
                _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                negativeSin,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiateTan(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Tan requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            var cosCall = new MethodCallNode(
                context.NewNodeId(),
                "Cos",
                methodCall.MethodSymbol,
                ImmutableArray.Create(arg),
                _doubleType);

            var cosSquared = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                cosCall,
                cosCall,
                _doubleType);

            var oneOverCosSquared = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Divide,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType),
                cosSquared,
                _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                oneOverCosSquared,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiateExp(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Exp requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                methodCall,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiateLog(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Log requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            var oneOverArg = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Divide,
                new ConstantNode(context.NewNodeId(), 1.0, _doubleType),
                arg,
                _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                oneOverArg,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiatePow(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 2)
            {
                throw new InvalidOperationException("Pow requires exactly 2 arguments");
            }

            var baseArg = methodCall.Arguments[0];
            var exponentArg = methodCall.Arguments[1];

            var dBase = _differentiator.Differentiate(baseArg, context);
            var dExponent = _differentiator.Differentiate(exponentArg, context);

            var dBaseIsZero = dBase is ConstantNode c1 && Convert.ToDouble(c1.Value, System.Globalization.CultureInfo.InvariantCulture) == 0.0;
            var dExponentIsZero = dExponent is ConstantNode c2 && Convert.ToDouble(c2.Value, System.Globalization.CultureInfo.InvariantCulture) == 0.0;

            if (dBaseIsZero && dExponentIsZero)
            {
                return new ConstantNode(context.NewNodeId(), 0.0, _doubleType);
            }
            else if (dExponentIsZero)
            {
                var exponentMinus1 = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Subtract,
                    exponentArg,
                    new ConstantNode(context.NewNodeId(), 1.0, _doubleType),
                    _doubleType);

                var powCall = new MethodCallNode(
                    context.NewNodeId(),
                    "Pow",
                    methodCall.MethodSymbol,
                    ImmutableArray.Create(baseArg, exponentMinus1),
                    _doubleType);

                var term = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    exponentArg,
                    powCall,
                    _doubleType);

                return new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    term,
                    dBase,
                    _doubleType);
            }
            else if (dBaseIsZero)
            {
                var logCall = new MethodCallNode(
                    context.NewNodeId(),
                    "Log",
                    methodCall.MethodSymbol,
                    ImmutableArray.Create(baseArg),
                    _doubleType);

                var powTimesLog = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    methodCall,
                    logCall,
                    _doubleType);

                return new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    powTimesLog,
                    dExponent,
                    _doubleType);
            }
            else
            {
                var exponentMinus1 = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Subtract,
                    exponentArg,
                    new ConstantNode(context.NewNodeId(), 1.0, _doubleType),
                    _doubleType);

                var powCall = new MethodCallNode(
                    context.NewNodeId(),
                    "Pow",
                    methodCall.MethodSymbol,
                    ImmutableArray.Create(baseArg, exponentMinus1),
                    _doubleType);

                var term1 = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    exponentArg,
                    powCall,
                    _doubleType);

                var firstTerm = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    term1,
                    dBase,
                    _doubleType);

                var logCall = new MethodCallNode(
                    context.NewNodeId(),
                    "Log",
                    methodCall.MethodSymbol,
                    ImmutableArray.Create(baseArg),
                    _doubleType);

                var powTimesLog = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    methodCall,
                    logCall,
                    _doubleType);

                var secondTerm = new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Multiply,
                    powTimesLog,
                    dExponent,
                    _doubleType);

                return new BinaryOpNode(
                    context.NewNodeId(),
                    BinaryOperator.Add,
                    firstTerm,
                    secondTerm,
                    _doubleType);
            }
        }

        private IRNode DifferentiateAbs(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 1)
            {
                throw new InvalidOperationException("Abs requires exactly 1 argument");
            }

            var arg = methodCall.Arguments[0];
            var dArg = _differentiator.Differentiate(arg, context);

            // d/dx |x| = sign(x). Emit sign(x) rather than the algebraic x/abs(x):
            // the latter is 0/0 = NaN at x = 0, which poisons the whole forward-mode
            // gradient (and, in the WGSL SDF pipeline, produces NaN surface normals at
            // grazing box-face hits where the in-face coordinate rounds exactly to the
            // centre). sign(0) = 0 is a valid subgradient — and the correct value there
            // by symmetry — and equals x/abs(x) everywhere else.
            var sign = new MethodCallNode(
                context.NewNodeId(),
                "Sign",
                methodCall.MethodSymbol,
                ImmutableArray.Create<IRNode>(arg),
                _doubleType);

            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                sign,
                dArg,
                _doubleType);
        }

        private IRNode DifferentiateAtan2(MethodCallNode methodCall, ForwardModeContext context)
        {
            if (methodCall.Arguments.Length != 2)
            {
                throw new InvalidOperationException("Atan2 requires exactly 2 arguments");
            }

            var y = methodCall.Arguments[0];
            var x = methodCall.Arguments[1];

            var dy = _differentiator.Differentiate(y, context);
            var dx = _differentiator.Differentiate(x, context);

            // Calculate x² + y²
            var xSquared = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                x,
                x,
                _doubleType);

            var ySquared = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                y,
                y,
                _doubleType);

            var denominator = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Add,
                xSquared,
                ySquared,
                _doubleType);

            // x * dy
            var xTimesDy = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                x,
                dy,
                _doubleType);

            // y * dx
            var yTimesDx = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Multiply,
                y,
                dx,
                _doubleType);

            // x*dy - y*dx
            var numerator = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Subtract,
                xTimesDy,
                yTimesDx,
                _doubleType);

            // (x*dy - y*dx) / (x² + y²)
            return new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Divide,
                numerator,
                denominator,
                _doubleType);
        }
    }
}
