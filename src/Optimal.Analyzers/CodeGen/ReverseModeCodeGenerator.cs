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
using System.Text;
using Microsoft.CodeAnalysis;
using Optimal.Analyzers.Differentiation;
using Optimal.Analyzers.IR;

namespace Optimal.Analyzers.CodeGen
{
    public sealed class ReverseModeCodeGenerator
    {
        private readonly Dictionary<IRNode, int> _nodeToIndex = new();
        private readonly Dictionary<string, int> _parameterNameToIndex = new();
        private readonly HashSet<string> _differentiableParamNames = new();
        private readonly List<object> _operations = new();
        private int _nodeCounter;

        public ReverseModeCodeGenerator(ITypeSymbol doubleType)
        {
        }

        public string GenerateMethod(MethodToTransform method, ITypeSymbol doubleType)
        {
            _nodeToIndex.Clear();
            _parameterNameToIndex.Clear();
            _differentiableParamNames.Clear();
            _operations.Clear();
            _nodeCounter = 0;

            var converter = new ASTToIRConverter(method.SemanticModel);
            var methodBody = converter.ConvertMethod(method.MethodSyntax, method.Parameters);

            var sb = new StringBuilder();

            var returnTypeName = method.ReturnType.ToDisplayString();
            var differentiableParams = method.Parameters.Where(p => p.IsDifferentiable).ToList();

            foreach (var param in differentiableParams)
            {
                _ = _differentiableParamNames.Add(param.Name);
            }

            sb.AppendLine($"        public static ({returnTypeName} value, double[] gradients) {method.MethodName}Reverse(");
            sb.Append("            ");
            sb.Append(string.Join(", ", method.Parameters.Select(p => $"{p.Type.ToDisplayString()} {p.Name}")));
            sb.AppendLine(")");
            sb.AppendLine("        {");

            var forwardCode = GenerateForwardPass(methodBody, sb, differentiableParams);

            // Only generate final backward pass and return if there's no early return in conditionals
            if (forwardCode.resultIndex >= 0)
            {
                sb.AppendLine();
                GenerateBackwardPass(sb, differentiableParams, forwardCode.resultIndex);

                sb.AppendLine($"            return (node{forwardCode.resultIndex}, new double[] {{ {string.Join(", ", differentiableParams.Select(p => $"adj[{_parameterNameToIndex[p.Name]}]"))} }});");
            }

            sb.AppendLine("        }");

            return sb.ToString();
        }

        private (int resultIndex, string resultVar) GenerateForwardPass(MethodBodyNode methodBody, StringBuilder sb, List<ParameterInfo> differentiableParams)
        {
            int resultIndex = -1;
            string resultVar = "";

            foreach (var param in differentiableParams)
            {
                var paramIdx = _nodeCounter++;
                sb.AppendLine($"            var node{paramIdx} = {param.Name};");
                _parameterNameToIndex[param.Name] = paramIdx;
            }

            foreach (var stmt in methodBody.Statements)
            {
                if (stmt is ReturnNode returnNode)
                {
                    resultIndex = GenerateNodeCode(returnNode.Value, sb);
                    resultVar = $"node{resultIndex}";
                }
                else if (stmt is AssignmentNode assignment)
                {
                    var valueIdx = GenerateNodeCode(assignment.Value, sb);
                    sb.AppendLine($"            var {assignment.TargetVariable} = node{valueIdx};");
                }
                else if (stmt is ConditionalNode conditional)
                {
                    GenerateConditionalStatement(conditional, sb);
                }
                else if (stmt is LoopNode loop)
                {
                    GenerateLoopStatement(loop, sb);
                }
            }

            return (resultIndex, resultVar);
        }

        private int GenerateNodeCode(IRNode node, StringBuilder sb)
        {
            if (_nodeToIndex.TryGetValue(node, out var existingIdx))
            {
                return existingIdx;
            }

            if (node is VariableNode varNode && _parameterNameToIndex.TryGetValue(varNode.Name, out var paramIdx))
            {
                _nodeToIndex[node] = paramIdx;
                return paramIdx;
            }

            var nodeIdx = _nodeCounter++;

            var code = node switch
            {
                ConstantNode constant => GenerateConstant(constant),
                VariableNode variable => GenerateVariable(variable),
                BinaryOpNode binary => GenerateBinaryOp(binary, sb, nodeIdx),
                UnaryOpNode unary => GenerateUnaryOp(unary, sb, nodeIdx),
                MethodCallNode methodCall => GenerateMethodCall(methodCall, sb, nodeIdx),
                ConditionalExpressionNode conditional => GenerateConditionalExpression(conditional, sb, nodeIdx),
                _ => throw new NotSupportedException($"Node type not supported: {node.GetType().Name}")
            };

            sb.AppendLine($"            var node{nodeIdx} = {code};");
            _nodeToIndex[node] = nodeIdx;

            return nodeIdx;
        }

        private string GenerateConstant(ConstantNode constant)
        {
            return constant.Value switch
            {
                double d => FormatDouble(d),
                float f => f.ToString("G9", System.Globalization.CultureInfo.InvariantCulture) + "f",
                int i => i.ToString(System.Globalization.CultureInfo.InvariantCulture),
                _ => constant.Value.ToString()!
            };
        }

        private string FormatDouble(double value)
        {
            var str = value.ToString("G17", System.Globalization.CultureInfo.InvariantCulture);
            if (!str.Contains('.') && !str.Contains('E') && !str.Contains('e'))
            {
                str += ".0";
            }
            return str;
        }

        private string GenerateVariable(VariableNode variable)
        {
            return variable.Name;
        }

        private string GenerateBinaryOp(BinaryOpNode binary, StringBuilder sb, int outputIdx)
        {
            var leftIdx = GenerateNodeCode(binary.Left, sb);
            var rightIdx = GenerateNodeCode(binary.Right, sb);

            if (IsArithmeticOp(binary.Op))
            {
                _operations.Add(("BinaryOp", outputIdx, binary.Op, leftIdx, rightIdx));
            }

            var opSymbol = binary.Op switch
            {
                BinaryOperator.Add => "+",
                BinaryOperator.Subtract => "-",
                BinaryOperator.Multiply => "*",
                BinaryOperator.Divide => "/",
                BinaryOperator.GreaterThan => ">",
                BinaryOperator.LessThan => "<",
                BinaryOperator.GreaterThanOrEqual => ">=",
                BinaryOperator.LessThanOrEqual => "<=",
                BinaryOperator.Equal => "==",
                BinaryOperator.NotEqual => "!=",
                _ => throw new NotSupportedException()
            };

            return $"node{leftIdx} {opSymbol} node{rightIdx}";
        }

        private bool IsArithmeticOp(BinaryOperator op)
        {
            return op is BinaryOperator.Add or BinaryOperator.Subtract or BinaryOperator.Multiply or BinaryOperator.Divide;
        }

        private string GenerateUnaryOp(UnaryOpNode unary, StringBuilder sb, int outputIdx)
        {
            var operandIdx = GenerateNodeCode(unary.Operand, sb);
            _operations.Add(("UnaryOp", outputIdx, unary.Op, operandIdx));

            return unary.Op switch
            {
                UnaryOperator.Negate => $"-node{operandIdx}",
                UnaryOperator.Plus or UnaryOperator.Identity => $"node{operandIdx}",
                _ => throw new NotSupportedException()
            };
        }

        private string GenerateMethodCall(MethodCallNode methodCall, StringBuilder sb, int outputIdx)
        {
            var argIndices = methodCall.Arguments.Select(arg => GenerateNodeCode(arg, sb)).ToList();
            _operations.Add(("MethodCall", outputIdx, methodCall.MethodName, argIndices));

            var args = string.Join(", ", argIndices.Select(idx => $"node{idx}"));
            var containingType = methodCall.MethodSymbol?.ContainingType?.ToDisplayString();

            if (containingType == "System.Math")
            {
                return $"Math.{methodCall.MethodName}({args})";
            }

            return $"{methodCall.MethodName}({args})";
        }

        private string GenerateConditionalExpression(ConditionalExpressionNode conditional, StringBuilder sb, int outputIdx)
        {
            var conditionIdx = GenerateNodeCode(conditional.Condition, sb);
            var trueIdx = GenerateNodeCode(conditional.TrueExpression, sb);
            var falseIdx = GenerateNodeCode(conditional.FalseExpression, sb);

            _operations.Add(("ConditionalExpression", outputIdx, conditionIdx, trueIdx, falseIdx));

            return $"node{conditionIdx} ? node{trueIdx} : node{falseIdx}";
        }

        private bool HasReturnStatement(ImmutableArray<IRNode> statements)
        {
            foreach (var stmt in statements)
            {
                if (stmt is ReturnNode)
                {
                    return true;
                }
                if (stmt is ConditionalNode conditional)
                {
                    if (HasReturnStatement(conditional.TrueBranch) || HasReturnStatement(conditional.FalseBranch))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        private void GenerateConditionalStatement(ConditionalNode conditional, StringBuilder sb)
        {
            GenerateNestedConditionalStatement(conditional, sb, "            ");
        }

        private void GenerateNestedConditionalStatement(ConditionalNode conditional, StringBuilder sb, string indent)
        {
            var conditionIdx = GenerateNodeCode(conditional.Condition, sb);

            var hasReturnInTrue = HasReturnStatement(conditional.TrueBranch);
            var hasReturnInFalse = HasReturnStatement(conditional.FalseBranch);

            sb.AppendLine($"{indent}if (node{conditionIdx})");
            sb.AppendLine($"{indent}{{");

            if (hasReturnInTrue)
            {
                var branchOpsStart = _operations.Count;
                foreach (var stmt in conditional.TrueBranch)
                {
                    if (stmt is ReturnNode returnNode)
                    {
                        var resultIdx = GenerateNodeCode(returnNode.Value, sb);
                        GenerateInlineBranchBackwardPass(sb, branchOpsStart, resultIdx, indent + "    ");
                        sb.AppendLine($"{indent}    return (node{resultIdx}, new double[] {{ {string.Join(", ", _parameterNameToIndex.Keys.OrderBy(k => _parameterNameToIndex[k]).Select(k => $"adj[{_parameterNameToIndex[k]}]"))} }});");
                    }
                    else if (stmt is AssignmentNode assignment)
                    {
                        var valueIdx = GenerateNodeCode(assignment.Value, sb);
                        sb.AppendLine($"{indent}    {assignment.TargetVariable} = node{valueIdx};");
                    }
                    else if (stmt is ConditionalNode nested)
                    {
                        GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                    }
                }
            }
            else
            {
                foreach (var stmt in conditional.TrueBranch)
                {
                    if (stmt is AssignmentNode assignment)
                    {
                        var valueIdx = GenerateNodeCode(assignment.Value, sb);
                        sb.AppendLine($"{indent}    {assignment.TargetVariable} = node{valueIdx};");
                    }
                    else if (stmt is ConditionalNode nested)
                    {
                        GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                    }
                }
            }

            sb.AppendLine($"{indent}}}");

            if (conditional.FalseBranch.Length > 0)
            {
                sb.AppendLine($"{indent}else");
                sb.AppendLine($"{indent}{{");

                if (hasReturnInFalse)
                {
                    var branchOpsStart = _operations.Count;
                    foreach (var stmt in conditional.FalseBranch)
                    {
                        if (stmt is ReturnNode returnNode)
                        {
                            var resultIdx = GenerateNodeCode(returnNode.Value, sb);
                            GenerateInlineBranchBackwardPass(sb, branchOpsStart, resultIdx, indent + "    ");
                            sb.AppendLine($"{indent}    return (node{resultIdx}, new double[] {{ {string.Join(", ", _parameterNameToIndex.Keys.OrderBy(k => _parameterNameToIndex[k]).Select(k => $"adj[{_parameterNameToIndex[k]}]"))} }});");
                        }
                        else if (stmt is AssignmentNode assignment)
                        {
                            var valueIdx = GenerateNodeCode(assignment.Value, sb);
                            sb.AppendLine($"{indent}    {assignment.TargetVariable} = node{valueIdx};");
                        }
                        else if (stmt is ConditionalNode nested)
                        {
                            GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                        }
                    }
                }
                else
                {
                    foreach (var stmt in conditional.FalseBranch)
                    {
                        if (stmt is AssignmentNode assignment)
                        {
                            var valueIdx = GenerateNodeCode(assignment.Value, sb);
                            sb.AppendLine($"{indent}    {assignment.TargetVariable} = node{valueIdx};");
                        }
                        else if (stmt is ConditionalNode nested)
                        {
                            GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                        }
                    }
                }

                sb.AppendLine($"{indent}}}");
            }
        }

        private void GenerateInlineBranchBackwardPass(StringBuilder sb, int branchOpsStart, int resultIdx, string indent)
        {
            sb.AppendLine($"{indent}var adj = new double[{_nodeCounter}];");
            sb.AppendLine($"{indent}adj[{resultIdx}] = 1.0;");
            sb.AppendLine();

            for (int i = _operations.Count - 1; i >= branchOpsStart; i--)
            {
                var operation = _operations[i];
                if (operation is ValueTuple<string, int, BinaryOperator, int, int> binaryOp && binaryOp.Item1 == "BinaryOp")
                {
                    GenerateBinaryOpBackpropWithIndent(sb, binaryOp.Item2, binaryOp.Item3, binaryOp.Item4, binaryOp.Item5, indent);
                }
                else if (operation is ValueTuple<string, int, UnaryOperator, int> unaryOp && unaryOp.Item1 == "UnaryOp")
                {
                    GenerateUnaryOpBackpropWithIndent(sb, unaryOp.Item2, unaryOp.Item3, unaryOp.Item4, indent);
                }
                else if (operation is ValueTuple<string, int, string, List<int>> methodCall && methodCall.Item1 == "MethodCall")
                {
                    GenerateMethodCallBackpropWithIndent(sb, methodCall.Item2, methodCall.Item3, methodCall.Item4, indent);
                }
                else if (operation is ValueTuple<string, int, int, int, int> conditionalExpr && conditionalExpr.Item1 == "ConditionalExpression")
                {
                    GenerateConditionalExpressionBackpropWithIndent(sb, conditionalExpr.Item2, conditionalExpr.Item3, conditionalExpr.Item4, conditionalExpr.Item5, indent);
                }
            }
        }

        private void GenerateLoopStatement(LoopNode loop, StringBuilder sb)
        {
            sb.AppendLine($"            for ({loop.IteratorVariable} = {GenerateInlineExpression(loop.Initializer, sb)}; {GenerateInlineExpression(loop.Condition, sb)}; {GenerateInlineExpression(loop.Increment, sb)})");
            sb.AppendLine("            {");

            foreach (var stmt in loop.Body)
            {
                if (stmt is AssignmentNode assignment)
                {
                    var valueIdx = GenerateNodeCode(assignment.Value, sb);
                    sb.AppendLine($"                {assignment.TargetVariable} = node{valueIdx};");
                }
            }

            sb.AppendLine("            }");
        }

        private string GenerateInlineExpression(IRNode? node, StringBuilder sb)
        {
            if (node == null)
            {
                return "";
            }

            return node switch
            {
                ConstantNode constant => GenerateConstant(constant),
                VariableNode variable => variable.Name,
                BinaryOpNode binary when binary.Op == BinaryOperator.Add && binary.Left is VariableNode v1 && binary.Right is ConstantNode c && c.Value.Equals(1) => $"{v1.Name}++",
                BinaryOpNode binary when binary.Op == BinaryOperator.Subtract && binary.Left is VariableNode v2 && binary.Right is ConstantNode c2 && c2.Value.Equals(1) => $"{v2.Name}--",
                BinaryOpNode binary => $"{GenerateInlineExpression(binary.Left, sb)} {GetOperatorSymbol(binary.Op)} {GenerateInlineExpression(binary.Right, sb)}",
                _ => throw new NotSupportedException($"Inline expression not supported: {node.GetType().Name}")
            };
        }

        private string GetOperatorSymbol(BinaryOperator op)
        {
            return op switch
            {
                BinaryOperator.Add => "+",
                BinaryOperator.Subtract => "-",
                BinaryOperator.Multiply => "*",
                BinaryOperator.Divide => "/",
                BinaryOperator.LessThan => "<",
                BinaryOperator.GreaterThan => ">",
                BinaryOperator.LessThanOrEqual => "<=",
                BinaryOperator.GreaterThanOrEqual => ">=",
                BinaryOperator.Equal => "==",
                BinaryOperator.NotEqual => "!=",
                _ => throw new NotSupportedException($"Operator not supported: {op}")
            };
        }

        private void GenerateBackwardPass(StringBuilder sb, List<ParameterInfo> differentiableParams, int resultIndex)
        {
            sb.AppendLine($"            var adj = new double[{_nodeCounter}];");
            sb.AppendLine($"            adj[{resultIndex}] = 1.0;");
            sb.AppendLine();

            for (int i = _operations.Count - 1; i >= 0; i--)
            {
                var operation = _operations[i];
                if (operation is ValueTuple<string, int, BinaryOperator, int, int> binaryOp && binaryOp.Item1 == "BinaryOp")
                {
                    GenerateBinaryOpBackprop(sb, binaryOp.Item2, binaryOp.Item3, binaryOp.Item4, binaryOp.Item5);
                }
                else if (operation is ValueTuple<string, int, UnaryOperator, int> unaryOp && unaryOp.Item1 == "UnaryOp")
                {
                    GenerateUnaryOpBackprop(sb, unaryOp.Item2, unaryOp.Item3, unaryOp.Item4);
                }
                else if (operation is ValueTuple<string, int, string, List<int>> methodCall && methodCall.Item1 == "MethodCall")
                {
                    GenerateMethodCallBackprop(sb, methodCall.Item2, methodCall.Item3, methodCall.Item4);
                }
                else if (operation is ValueTuple<string, int, int, int, int> conditionalExpr && conditionalExpr.Item1 == "ConditionalExpression")
                {
                    GenerateConditionalExpressionBackprop(sb, conditionalExpr.Item2, conditionalExpr.Item3, conditionalExpr.Item4, conditionalExpr.Item5);
                }
            }
        }

        private void GenerateBinaryOpBackprop(StringBuilder sb, int outputIdx, BinaryOperator op, int leftIdx, int rightIdx)
        {
            switch (op)
            {
                case BinaryOperator.Add:
                    sb.AppendLine($"            adj[{leftIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"            adj[{rightIdx}] += adj[{outputIdx}];");
                    break;

                case BinaryOperator.Subtract:
                    sb.AppendLine($"            adj[{leftIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"            adj[{rightIdx}] -= adj[{outputIdx}];");
                    break;

                case BinaryOperator.Multiply:
                    sb.AppendLine($"            adj[{leftIdx}] += adj[{outputIdx}] * node{rightIdx};");
                    sb.AppendLine($"            adj[{rightIdx}] += adj[{outputIdx}] * node{leftIdx};");
                    break;

                case BinaryOperator.Divide:
                    sb.AppendLine($"            adj[{leftIdx}] += adj[{outputIdx}] / node{rightIdx};");
                    sb.AppendLine($"            adj[{rightIdx}] -= adj[{outputIdx}] * node{leftIdx} / (node{rightIdx} * node{rightIdx});");
                    break;
            }
        }

        private void GenerateUnaryOpBackprop(StringBuilder sb, int outputIdx, UnaryOperator op, int operandIdx)
        {
            switch (op)
            {
                case UnaryOperator.Negate:
                    sb.AppendLine($"            adj[{operandIdx}] -= adj[{outputIdx}];");
                    break;

                case UnaryOperator.Plus:
                case UnaryOperator.Identity:
                    sb.AppendLine($"            adj[{operandIdx}] += adj[{outputIdx}];");
                    break;
            }
        }

        private void GenerateMethodCallBackprop(StringBuilder sb, int outputIdx, string methodName, List<int> argIndices)
        {
            if (argIndices.Count == 1)
            {
                var argIdx = argIndices[0];

                switch (methodName)
                {
                    case "Sqrt":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] / (2.0 * node{outputIdx});");
                        break;

                    case "Sin":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] * Math.Cos(node{argIdx});");
                        break;

                    case "Cos":
                        sb.AppendLine($"            adj[{argIdx}] -= adj[{outputIdx}] * Math.Sin(node{argIdx});");
                        break;

                    case "Tan":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] / (Math.Cos(node{argIdx}) * Math.Cos(node{argIdx}));");
                        break;

                    case "Exp":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] * node{outputIdx};");
                        break;

                    case "Log":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] / node{argIdx};");
                        break;

                    case "Abs":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] * (node{argIdx} / node{outputIdx});");
                        break;

                    default:
                        throw new NotSupportedException($"Math function not supported: {methodName}");
                }
            }
            else if (argIndices.Count == 2 && methodName == "Pow")
            {
                var baseIdx = argIndices[0];
                var expIdx = argIndices[1];

                sb.AppendLine($"            adj[{baseIdx}] += adj[{outputIdx}] * node{expIdx} * Math.Pow(node{baseIdx}, node{expIdx} - 1.0);");
                sb.AppendLine($"            adj[{expIdx}] += adj[{outputIdx}] * node{outputIdx} * Math.Log(node{baseIdx});");
            }
            else
            {
                throw new NotSupportedException($"Math function not supported: {methodName}");
            }
        }

        private void GenerateConditionalExpressionBackprop(StringBuilder sb, int outputIdx, int conditionIdx, int trueIdx, int falseIdx)
        {
            sb.AppendLine($"            if (node{conditionIdx})");
            sb.AppendLine("            {");
            sb.AppendLine($"                adj[{trueIdx}] += adj[{outputIdx}];");
            sb.AppendLine("            }");
            sb.AppendLine("            else");
            sb.AppendLine("            {");
            sb.AppendLine($"                adj[{falseIdx}] += adj[{outputIdx}];");
            sb.AppendLine("            }");
        }

        private void GenerateBinaryOpBackpropWithIndent(StringBuilder sb, int outputIdx, BinaryOperator op, int leftIdx, int rightIdx, string indent)
        {
            switch (op)
            {
                case BinaryOperator.Add:
                    sb.AppendLine($"{indent}adj[{leftIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"{indent}adj[{rightIdx}] += adj[{outputIdx}];");
                    break;

                case BinaryOperator.Subtract:
                    sb.AppendLine($"{indent}adj[{leftIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"{indent}adj[{rightIdx}] -= adj[{outputIdx}];");
                    break;

                case BinaryOperator.Multiply:
                    sb.AppendLine($"{indent}adj[{leftIdx}] += adj[{outputIdx}] * node{rightIdx};");
                    sb.AppendLine($"{indent}adj[{rightIdx}] += adj[{outputIdx}] * node{leftIdx};");
                    break;

                case BinaryOperator.Divide:
                    sb.AppendLine($"{indent}adj[{leftIdx}] += adj[{outputIdx}] / node{rightIdx};");
                    sb.AppendLine($"{indent}adj[{rightIdx}] -= adj[{outputIdx}] * node{leftIdx} / (node{rightIdx} * node{rightIdx});");
                    break;
            }
        }

        private void GenerateUnaryOpBackpropWithIndent(StringBuilder sb, int outputIdx, UnaryOperator op, int operandIdx, string indent)
        {
            switch (op)
            {
                case UnaryOperator.Negate:
                    sb.AppendLine($"{indent}adj[{operandIdx}] -= adj[{outputIdx}];");
                    break;

                case UnaryOperator.Plus:
                case UnaryOperator.Identity:
                    sb.AppendLine($"{indent}adj[{operandIdx}] += adj[{outputIdx}];");
                    break;
            }
        }

        private void GenerateMethodCallBackpropWithIndent(StringBuilder sb, int outputIdx, string methodName, List<int> argIndices, string indent)
        {
            if (argIndices.Count == 1)
            {
                var argIdx = argIndices[0];

                switch (methodName)
                {
                    case "Sqrt":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] / (2.0 * node{outputIdx});");
                        break;

                    case "Sin":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] * Math.Cos(node{argIdx});");
                        break;

                    case "Cos":
                        sb.AppendLine($"{indent}adj[{argIdx}] -= adj[{outputIdx}] * Math.Sin(node{argIdx});");
                        break;

                    case "Tan":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] / (Math.Cos(node{argIdx}) * Math.Cos(node{argIdx}));");
                        break;

                    case "Exp":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] * node{outputIdx};");
                        break;

                    case "Log":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] / node{argIdx};");
                        break;

                    case "Abs":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] * (node{argIdx} / node{outputIdx});");
                        break;

                    default:
                        throw new NotSupportedException($"Math function not supported: {methodName}");
                }
            }
            else if (argIndices.Count == 2 && methodName == "Pow")
            {
                var baseIdx = argIndices[0];
                var expIdx = argIndices[1];

                sb.AppendLine($"{indent}adj[{baseIdx}] += adj[{outputIdx}] * node{expIdx} * Math.Pow(node{baseIdx}, node{expIdx} - 1.0);");
                sb.AppendLine($"{indent}adj[{expIdx}] += adj[{outputIdx}] * node{outputIdx} * Math.Log(node{baseIdx});");
            }
            else
            {
                throw new NotSupportedException($"Math function not supported: {methodName}");
            }
        }

        private void GenerateConditionalExpressionBackpropWithIndent(StringBuilder sb, int outputIdx, int conditionIdx, int trueIdx, int falseIdx, string indent)
        {
            sb.AppendLine($"{indent}if (node{conditionIdx})");
            sb.AppendLine($"{indent}{{");
            sb.AppendLine($"{indent}    adj[{trueIdx}] += adj[{outputIdx}];");
            sb.AppendLine($"{indent}}}");
            sb.AppendLine($"{indent}else");
            sb.AppendLine($"{indent}{{");
            sb.AppendLine($"{indent}    adj[{falseIdx}] += adj[{outputIdx}];");
            sb.AppendLine($"{indent}}}");
        }
    }
}
