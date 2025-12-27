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
using Optimal.Analyzers.IR;

namespace Optimal.Analyzers.Differentiation
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
            return containingType == "System.Math";
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
                _ => throw new NotSupportedException($"Math function not supported for differentiation: {methodCall.MethodName}")
            };
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

            var sign = new BinaryOpNode(
                context.NewNodeId(),
                BinaryOperator.Divide,
                arg,
                methodCall,
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
