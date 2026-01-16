/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Differentiation
{
    public sealed class BinaryOpRule : IDifferentiationRule
    {
        private readonly ForwardModeDifferentiator _differentiator;
        private readonly Microsoft.CodeAnalysis.ITypeSymbol _doubleType;

        public BinaryOpRule(ForwardModeDifferentiator differentiator, Microsoft.CodeAnalysis.ITypeSymbol doubleType)
        {
            _differentiator = differentiator;
            _doubleType = doubleType;
        }

        public bool CanDifferentiate(IRNode node)
        {
            return node is BinaryOpNode;
        }

        public IRNode DifferentiateForward(IRNode node, ForwardModeContext context)
        {
            var binOp = (BinaryOpNode)node;

            var dLeft = _differentiator.Differentiate(binOp.Left, context);
            var dRight = _differentiator.Differentiate(binOp.Right, context);

            return binOp.Op switch
            {
                BinaryOperator.Add => DifferentiateAdd(binOp, dLeft, dRight, context),
                BinaryOperator.Subtract => DifferentiateSubtract(binOp, dLeft, dRight, context),
                BinaryOperator.Multiply => DifferentiateMultiply(binOp, dLeft, dRight, context),
                BinaryOperator.Divide => DifferentiateDivide(binOp, dLeft, dRight, context),
                BinaryOperator.Power => DifferentiatePower(binOp, dLeft, context),
                BinaryOperator.GreaterThan or
                BinaryOperator.LessThan or
                BinaryOperator.GreaterThanOrEqual or
                BinaryOperator.LessThanOrEqual or
                BinaryOperator.Equal or
                BinaryOperator.NotEqual => new ConstantNode(context.NewNodeId(), 0.0, _doubleType),
                _ => throw new NotSupportedException($"Binary operator not supported: {binOp.Op}")
            };
        }

        private IRNode DifferentiateAdd(BinaryOpNode node, IRNode dLeft, IRNode dRight, ForwardModeContext context)
        {
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Add, dLeft, dRight, node.Type);
        }

        private IRNode DifferentiateSubtract(BinaryOpNode node, IRNode dLeft, IRNode dRight, ForwardModeContext context)
        {
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, dLeft, dRight, node.Type);
        }

        private IRNode DifferentiateMultiply(BinaryOpNode node, IRNode dLeft, IRNode dRight, ForwardModeContext context)
        {
            var leftTerm = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, node.Left, dRight, node.Type);
            var rightTerm = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, node.Right, dLeft, node.Type);
            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Add, leftTerm, rightTerm, node.Type);
        }

        private IRNode DifferentiateDivide(BinaryOpNode node, IRNode dLeft, IRNode dRight, ForwardModeContext context)
        {
            var numeratorLeft = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, node.Right, dLeft, node.Type);
            var numeratorRight = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, node.Left, dRight, node.Type);
            var numerator = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Subtract, numeratorLeft, numeratorRight, node.Type);

            var denominator = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, node.Right, node.Right, node.Type);

            return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Divide, numerator, denominator, node.Type);
        }

        private IRNode DifferentiatePower(BinaryOpNode node, IRNode dLeft, ForwardModeContext context)
        {
            if (node.Right is ConstantNode constant)
            {
                var n = Convert.ToDouble(constant.Value, System.Globalization.CultureInfo.InvariantCulture);
                var nMinus1 = new ConstantNode(context.NewNodeId(), n - 1.0, constant.Type);

                var powerNode = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Power, node.Left, nMinus1, node.Type);

                var nNode = new ConstantNode(context.NewNodeId(), n, constant.Type);

                var term1 = new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, nNode, powerNode, node.Type);

                return new BinaryOpNode(context.NewNodeId(), BinaryOperator.Multiply, term1, dLeft, node.Type);
            }

            throw new NotSupportedException("Power with variable exponent not yet supported");
        }
    }
}
