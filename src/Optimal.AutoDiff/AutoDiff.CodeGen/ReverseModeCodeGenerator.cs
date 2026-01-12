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
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.CodeGen
{
    public sealed class ReverseModeCodeGenerator
    {
        private readonly Dictionary<IRNode, int> _nodeToIndex = new();
        private readonly HashSet<IRNode> _generatedNodes = new();
        private readonly Dictionary<string, int> _parameterNameToIndex = new();
        private readonly HashSet<string> _differentiableParamNames = new();
        private readonly List<object> _operations = new();
        private readonly HashSet<string> _declaredVariables = new();
        private readonly Dictionary<string, int> _intermediateVariableToNodeIndex = new(); // Track intermediate variables
        // Track variable assignments in branches: (varName, conditionIdx, isTrueBranch, nodeIdx)
        private readonly List<(string varName, int conditionIdx, bool isTrueBranch, int nodeIdx)> _branchVariableAssignments = new();
        private int _nodeCounter;
        private string _currentIndent = "            "; // 12 spaces = 3 levels

        public ReverseModeCodeGenerator(ITypeSymbol doubleType)
        {
        }

        public string GenerateMethod(MethodToTransform method, ITypeSymbol doubleType)
        {
            _nodeToIndex.Clear();
            _generatedNodes.Clear();
            _parameterNameToIndex.Clear();
            _differentiableParamNames.Clear();
            _operations.Clear();
            _declaredVariables.Clear();
            _intermediateVariableToNodeIndex.Clear();
            _branchVariableAssignments.Clear();
            _nodeCounter = 0;

            // Mark parameters as declared
            foreach (var param in method.Parameters)
            {
                _declaredVariables.Add(param.Name);
            }

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

                sb.AppendLine($"            return (nodes[{forwardCode.resultIndex}], new double[] {{ {string.Join(", ", differentiableParams.Select(p => $"adj[{_parameterNameToIndex[p.Name]}]"))} }});");
            }

            sb.AppendLine("        }");

            return sb.ToString();
        }

        private (int resultIndex, string resultVar) GenerateForwardPass(MethodBodyNode methodBody, StringBuilder sb, List<ParameterInfo> differentiableParams)
        {
            int resultIndex = -1;
            string resultVar = "";

            // Assign parameter indices first
            foreach (var param in differentiableParams)
            {
                var paramIdx = _nodeCounter++;
                _parameterNameToIndex[param.Name] = paramIdx;
            }

            // Pre-scan to count total nodes needed
            CountNodes(methodBody);

            // Declare nodes array at method scope
            sb.AppendLine($"            var nodes = new double[{_nodeCounter}];");
            sb.AppendLine();

            // Initialize parameter nodes
            foreach (var param in differentiableParams)
            {
                var paramIdx = _parameterNameToIndex[param.Name];
                sb.AppendLine($"{_currentIndent}nodes[{paramIdx}] = {param.Name};");
            }

            foreach (var stmt in methodBody.Statements)
            {
                if (stmt is ReturnNode returnNode)
                {
                    resultIndex = GenerateNodeCode(returnNode.Value, sb);
                    resultVar = $"nodes[resultIndex]";
                }
                else if (stmt is AssignmentNode assignment)
                {
                    var valueIdx = GenerateNodeCode(assignment.Value, sb);

                    // NEW: Track intermediate variable -> node index mapping for gradient propagation
                    _intermediateVariableToNodeIndex[assignment.TargetVariable] = valueIdx;

                    if (!_declaredVariables.Contains(assignment.TargetVariable))
                    {
                        sb.AppendLine($"            var {assignment.TargetVariable} = nodes[{valueIdx}];");
                        _declaredVariables.Add(assignment.TargetVariable);
                    }
                    else
                    {
                        sb.AppendLine($"            {assignment.TargetVariable} = nodes[{valueIdx}];");
                    }
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
            // Check if already generated
            if (_generatedNodes.Contains(node))
            {
                return _nodeToIndex[node];
            }

            // Get the node index (assigned during counting)
            if (!_nodeToIndex.TryGetValue(node, out var nodeIdx))
            {
                throw new InvalidOperationException($"Node not found in index map. This should not happen.");
            }

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

            // For variable nodes that were assigned in branches, add a merge operation
            if (node is VariableNode varNode)
            {
                AddVariableReferenceOperation(varNode.Name, nodeIdx);
            }

            // Assign to nodes array (accessible from anywhere in method)
            sb.AppendLine($"{_currentIndent}nodes[{nodeIdx}] = {code};");
            _generatedNodes.Add(node);

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
            // Check if this variable was assigned in conditional branches
            // If so, we'll use the variable name (for correct runtime value)
            // but we DON'T use nodes[idx] because the branch assignments 
            // will be handled by VariableMerge operations
            if (_intermediateVariableToNodeIndex.TryGetValue(variable.Name, out var _))
            {
                // Variable was assigned somewhere - use the C# variable directly
                // This ensures we get the correct value from whichever branch was taken
                return variable.Name;
            }

            return variable.Name;
        }

        // Called during GenerateNodeCode to add operations for variable references
        private void AddVariableReferenceOperation(string varName, int outputNodeIdx)
        {
            // Check if this variable was assigned in conditional branches
            var branchAssignments = _branchVariableAssignments.Where(a => a.varName == varName).ToList();
            if (branchAssignments.Count > 0)
            {
                // Add a VariableMerge operation for adjoint propagation
                _operations.Add(("VariableMerge", outputNodeIdx, branchAssignments));
            }
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

            var expr = $"nodes[{leftIdx}] {opSymbol} nodes[{rightIdx}]";

            // Convert boolean comparisons to double (0.0 or 1.0)
            if (IsComparisonOp(binary.Op))
            {
                return $"({expr}) ? 1.0 : 0.0";
            }

            return expr;
        }

        private bool IsArithmeticOp(BinaryOperator op)
        {
            return op is BinaryOperator.Add or BinaryOperator.Subtract or BinaryOperator.Multiply or BinaryOperator.Divide;
        }

        private bool IsComparisonOp(BinaryOperator op)
        {
            return op is BinaryOperator.GreaterThan or BinaryOperator.LessThan or
                   BinaryOperator.GreaterThanOrEqual or BinaryOperator.LessThanOrEqual or
                   BinaryOperator.Equal or BinaryOperator.NotEqual;
        }

        private string GenerateUnaryOp(UnaryOpNode unary, StringBuilder sb, int outputIdx)
        {
            var operandIdx = GenerateNodeCode(unary.Operand, sb);
            _operations.Add(("UnaryOp", outputIdx, unary.Op, operandIdx));

            return unary.Op switch
            {
                UnaryOperator.Negate => $"-nodes[{operandIdx}]",
                UnaryOperator.Plus or UnaryOperator.Identity => $"nodes[{operandIdx}]",
                _ => throw new NotSupportedException()
            };
        }

        private string GenerateMethodCall(MethodCallNode methodCall, StringBuilder sb, int outputIdx)
        {
            var argIndices = methodCall.Arguments.Select(arg => GenerateNodeCode(arg, sb)).ToList();
            _operations.Add(("MethodCall", outputIdx, methodCall.MethodName, argIndices));

            var args = string.Join(", ", argIndices.Select(idx => $"nodes[{idx}]"));
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

            return $"(nodes[{conditionIdx}] != 0) ? nodes[{trueIdx}] : nodes[{falseIdx}]";
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
            GenerateNestedConditionalStatement(conditional, sb, _currentIndent);
        }

        private void GenerateNestedConditionalStatement(ConditionalNode conditional, StringBuilder sb, string indent)
        {
            var previousIndent = _currentIndent;
            _currentIndent = indent;

            // Pre-declare any variables that are assigned in branches but not yet declared
            PreDeclareConditionalVariables(conditional, sb, indent);

            var conditionIdx = GenerateNodeCode(conditional.Condition, sb);

            var hasReturnInTrue = HasReturnStatement(conditional.TrueBranch);
            var hasReturnInFalse = HasReturnStatement(conditional.FalseBranch);

            sb.AppendLine($"{indent}if (nodes[{conditionIdx}] != 0)");
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
                        sb.AppendLine($"{indent}    return (nodes[{resultIdx}], new double[] {{ {string.Join(", ", _parameterNameToIndex.Keys.OrderBy(k => _parameterNameToIndex[k]).Select(k => $"adj[{_parameterNameToIndex[k]}]"))} }});");
                    }
                    else if (stmt is AssignmentNode assignment)
                    {
                        var valueIdx = GenerateNodeCode(assignment.Value, sb);

                        if (!_declaredVariables.Contains(assignment.TargetVariable))
                        {
                            sb.AppendLine($"{indent}    var {assignment.TargetVariable} = nodes[{valueIdx}];");
                            _declaredVariables.Add(assignment.TargetVariable);
                        }
                        else
                        {
                            sb.AppendLine($"{indent}    {assignment.TargetVariable} = nodes[{valueIdx}];");
                        }
                    }
                    else if (stmt is ConditionalNode nested)
                    {
                        GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                    }
                }
            }
            else
            {
                // Add branch start marker for backward pass
                _operations.Add(("BranchStart", conditionIdx, true)); // true branch
                foreach (var stmt in conditional.TrueBranch)
                {
                    if (stmt is AssignmentNode assignment)
                    {
                        var valueIdx = GenerateNodeCode(assignment.Value, sb);

                        // Track branch variable assignment for adjoint propagation
                        _branchVariableAssignments.Add((assignment.TargetVariable, conditionIdx, true, valueIdx));

                        if (!_declaredVariables.Contains(assignment.TargetVariable))
                        {
                            sb.AppendLine($"{indent}    var {assignment.TargetVariable} = nodes[{valueIdx}];");
                            _declaredVariables.Add(assignment.TargetVariable);
                        }
                        else
                        {
                            sb.AppendLine($"{indent}    {assignment.TargetVariable} = nodes[{valueIdx}];");
                        }
                    }
                    else if (stmt is ConditionalNode nested)
                    {
                        GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                    }
                }
                // Add branch end marker
                _operations.Add(("BranchEnd", conditionIdx, true));
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
                            sb.AppendLine($"{indent}    return (nodes[{resultIdx}], new double[] {{ {string.Join(", ", _parameterNameToIndex.Keys.OrderBy(k => _parameterNameToIndex[k]).Select(k => $"adj[{_parameterNameToIndex[k]}]"))} }});");
                        }
                        else if (stmt is AssignmentNode assignment)
                        {
                            var valueIdx = GenerateNodeCode(assignment.Value, sb);

                            if (!_declaredVariables.Contains(assignment.TargetVariable))
                            {
                                sb.AppendLine($"{indent}    var {assignment.TargetVariable} = nodes[{valueIdx}];");
                                _declaredVariables.Add(assignment.TargetVariable);
                            }
                            else
                            {
                                sb.AppendLine($"{indent}    {assignment.TargetVariable} = nodes[{valueIdx}];");
                            }
                        }
                        else if (stmt is ConditionalNode nested)
                        {
                            GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                        }
                    }
                }
                else
                {
                    // Add branch start marker for backward pass (false branch)
                    _operations.Add(("BranchStart", conditionIdx, false)); // false branch
                    foreach (var stmt in conditional.FalseBranch)
                    {
                        if (stmt is AssignmentNode assignment)
                        {
                            var valueIdx = GenerateNodeCode(assignment.Value, sb);

                            // Track branch variable assignment for adjoint propagation
                            _branchVariableAssignments.Add((assignment.TargetVariable, conditionIdx, false, valueIdx));

                            if (!_declaredVariables.Contains(assignment.TargetVariable))
                            {
                                sb.AppendLine($"{indent}    var {assignment.TargetVariable} = nodes[{valueIdx}];");
                                _declaredVariables.Add(assignment.TargetVariable);
                            }
                            else
                            {
                                sb.AppendLine($"{indent}    {assignment.TargetVariable} = nodes[{valueIdx}];");
                            }
                        }
                        else if (stmt is ConditionalNode nested)
                        {
                            GenerateNestedConditionalStatement(nested, sb, indent + "    ");
                        }
                    }
                    // Add branch end marker
                    _operations.Add(("BranchEnd", conditionIdx, false));
                }

                sb.AppendLine($"{indent}}}");
            }

            _currentIndent = previousIndent;
        }

        /// <summary>
        /// Pre-declares variables that are assigned inside conditional branches.
        /// This ensures they are in scope after the if-else block.
        /// </summary>
        private void PreDeclareConditionalVariables(ConditionalNode conditional, StringBuilder sb, string indent)
        {
            var varsInBranches = new HashSet<string>();
            CollectAssignedVariables(conditional.TrueBranch, varsInBranches);
            CollectAssignedVariables(conditional.FalseBranch, varsInBranches);

            foreach (var varName in varsInBranches)
            {
                if (!_declaredVariables.Contains(varName))
                {
                    sb.AppendLine($"{indent}double {varName} = default;");
                    _declaredVariables.Add(varName);
                }
            }
        }

        /// <summary>
        /// Recursively collects all variable names that are assigned in a list of statements.
        /// </summary>
        private static void CollectAssignedVariables(ImmutableArray<IRNode> statements, HashSet<string> variables)
        {
            foreach (var stmt in statements)
            {
                if (stmt is AssignmentNode assignment)
                {
                    variables.Add(assignment.TargetVariable);
                }
                else if (stmt is ConditionalNode nested)
                {
                    CollectAssignedVariables(nested.TrueBranch, variables);
                    CollectAssignedVariables(nested.FalseBranch, variables);
                }
                else if (stmt is LoopNode loop)
                {
                    CollectAssignedVariables(loop.Body, variables);
                }
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
            var initializerCode = "";
            if (loop.Initializer != null)
            {
                initializerCode = $"var {loop.IteratorVariable} = {GenerateInlineExpression(loop.Initializer, sb)}";
            }

            var conditionCode = "";
            if (loop.Condition != null)
            {
                conditionCode = GenerateInlineExpression(loop.Condition, sb);
            }

            var incrementCode = "";
            if (loop.Increment != null)
            {
                incrementCode = $"{loop.IteratorVariable} = {GenerateInlineExpression(loop.Increment, sb)}";
            }

            sb.AppendLine($"{_currentIndent}for ({initializerCode}; {conditionCode}; {incrementCode})");
            sb.AppendLine($"{_currentIndent}{{");

            var previousIndent = _currentIndent;
            _currentIndent = _currentIndent + "    "; // Add 4 spaces for next level

            foreach (var stmt in loop.Body)
            {
                if (stmt is AssignmentNode assignment)
                {
                    var valueIdx = GenerateNodeCode(assignment.Value, sb);

                    if (!_declaredVariables.Contains(assignment.TargetVariable))
                    {
                        sb.AppendLine($"{_currentIndent}var {assignment.TargetVariable} = nodes[{valueIdx}];");
                        _declaredVariables.Add(assignment.TargetVariable);
                    }
                    else
                    {
                        sb.AppendLine($"{_currentIndent}{assignment.TargetVariable} = nodes[{valueIdx}];");
                    }
                }
                else if (stmt is ConditionalNode conditional)
                {
                    GenerateNestedConditionalStatement(conditional, sb, _currentIndent);
                }
            }

            _currentIndent = previousIndent;

            sb.AppendLine($"{_currentIndent}}}");
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

            // Track current indentation for branch-conditional wrapping
            var currentIndent = "            ";
            // Stack tracks active branches (condition index, is true branch)
            var branchStack = new Stack<(int conditionIdx, bool isTrueBranch)>();

            for (int i = _operations.Count - 1; i >= 0; i--)
            {
                var operation = _operations[i];

                // Handle branch markers - when iterating backward, we encounter End before Start
                if (operation is ValueTuple<string, int, bool> branchMarker)
                {
                    if (branchMarker.Item1 == "BranchEnd")
                    {
                        // Starting a new branch (end marker comes first when iterating backward)
                        var conditionIdx = branchMarker.Item2;
                        var isTrueBranch = branchMarker.Item3;
                        branchStack.Push((conditionIdx, isTrueBranch));

                        // Generate conditional start
                        var condition = isTrueBranch ? $"nodes[{conditionIdx}] != 0" : $"nodes[{conditionIdx}] == 0";
                        sb.AppendLine($"{currentIndent}if ({condition})");
                        sb.AppendLine($"{currentIndent}{{");
                        currentIndent += "    ";
                    }
                    else if (branchMarker.Item1 == "BranchStart")
                    {
                        // Exiting a branch (start marker comes last when iterating backward)
                        if (branchStack.Count > 0)
                        {
                            branchStack.Pop();
                            currentIndent = currentIndent.Substring(0, currentIndent.Length - 4);
                            sb.AppendLine($"{currentIndent}}}");
                        }
                    }
                    continue;
                }

                if (operation is ValueTuple<string, int, BinaryOperator, int, int> binaryOp && binaryOp.Item1 == "BinaryOp")
                {
                    GenerateBinaryOpBackpropWithIndent(sb, binaryOp.Item2, binaryOp.Item3, binaryOp.Item4, binaryOp.Item5, currentIndent);
                }
                else if (operation is ValueTuple<string, int, UnaryOperator, int> unaryOp && unaryOp.Item1 == "UnaryOp")
                {
                    GenerateUnaryOpBackpropWithIndent(sb, unaryOp.Item2, unaryOp.Item3, unaryOp.Item4, currentIndent);
                }
                else if (operation is ValueTuple<string, int, string, List<int>> methodCall && methodCall.Item1 == "MethodCall")
                {
                    GenerateMethodCallBackpropWithIndent(sb, methodCall.Item2, methodCall.Item3, methodCall.Item4, currentIndent);
                }
                else if (operation is ValueTuple<string, int, int, int, int> conditionalExpr && conditionalExpr.Item1 == "ConditionalExpression")
                {
                    GenerateConditionalExpressionBackpropWithIndent(sb, conditionalExpr.Item2, conditionalExpr.Item3, conditionalExpr.Item4, conditionalExpr.Item5, currentIndent);
                }
                else if (operation is ValueTuple<string, int, List<(string, int, bool, int)>> varMerge && varMerge.Item1 == "VariableMerge")
                {
                    GenerateVariableMergeBackprop(sb, varMerge.Item2, varMerge.Item3, currentIndent);
                }
            }
        }

        private void GenerateVariableMergeBackprop(StringBuilder sb, int outputIdx, List<(string varName, int conditionIdx, bool isTrueBranch, int nodeIdx)> branchAssignments, string indent)
        {
            // Generate conditional adjoint propagation for each branch assignment
            // Group by condition index to handle if-else pairs together
            var groupedByCondition = branchAssignments.GroupBy(a => a.conditionIdx).ToList();

            foreach (var group in groupedByCondition)
            {
                var conditionIdx = group.Key;
                var trueAssign = group.FirstOrDefault(a => a.isTrueBranch);
                var falseAssign = group.FirstOrDefault(a => !a.isTrueBranch);

                if (trueAssign.varName != null && falseAssign.varName != null)
                {
                    // Both branches exist - generate if-else
                    sb.AppendLine($"{indent}if (nodes[{conditionIdx}] != 0)");
                    sb.AppendLine($"{indent}{{");
                    sb.AppendLine($"{indent}    adj[{trueAssign.nodeIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"{indent}}}");
                    sb.AppendLine($"{indent}else");
                    sb.AppendLine($"{indent}{{");
                    sb.AppendLine($"{indent}    adj[{falseAssign.nodeIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"{indent}}}");
                }
                else if (trueAssign.varName != null)
                {
                    // Only true branch
                    sb.AppendLine($"{indent}if (nodes[{conditionIdx}] != 0)");
                    sb.AppendLine($"{indent}{{");
                    sb.AppendLine($"{indent}    adj[{trueAssign.nodeIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"{indent}}}");
                }
                else if (falseAssign.varName != null)
                {
                    // Only false branch
                    sb.AppendLine($"{indent}if (nodes[{conditionIdx}] == 0)");
                    sb.AppendLine($"{indent}{{");
                    sb.AppendLine($"{indent}    adj[{falseAssign.nodeIdx}] += adj[{outputIdx}];");
                    sb.AppendLine($"{indent}}}");
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
                    sb.AppendLine($"            adj[{leftIdx}] += adj[{outputIdx}] * nodes[{rightIdx}];");
                    sb.AppendLine($"            adj[{rightIdx}] += adj[{outputIdx}] * nodes[{leftIdx}];");
                    break;

                case BinaryOperator.Divide:
                    sb.AppendLine($"            adj[{leftIdx}] += adj[{outputIdx}] / nodes[{rightIdx}];");
                    sb.AppendLine($"            adj[{rightIdx}] -= adj[{outputIdx}] * nodes[{leftIdx}] / (nodes[{rightIdx}] * nodes[{rightIdx}]);");
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
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] / (2.0 * nodes[{outputIdx}]);");
                        break;

                    case "Sin":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] * Math.Cos(nodes[{argIdx}]);");
                        break;

                    case "Cos":
                        sb.AppendLine($"            adj[{argIdx}] -= adj[{outputIdx}] * Math.Sin(nodes[{argIdx}]);");
                        break;

                    case "Tan":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] / (Math.Cos(nodes[{argIdx}]) * Math.Cos(nodes[{argIdx}]));");
                        break;

                    case "Exp":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] * nodes[{outputIdx}];");
                        break;

                    case "Log":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] / nodes[{argIdx}];");
                        break;

                    case "Abs":
                        sb.AppendLine($"            adj[{argIdx}] += adj[{outputIdx}] * (nodes[{argIdx}] / nodes[{outputIdx}]);");
                        break;

                    default:
                        throw new NotSupportedException($"Math function not supported: {methodName}");
                }
            }
            else if (argIndices.Count == 2 && methodName == "Pow")
            {
                var baseIdx = argIndices[0];
                var expIdx = argIndices[1];

                sb.AppendLine($"            adj[{baseIdx}] += adj[{outputIdx}] * nodes[{expIdx}] * Math.Pow(nodes[{baseIdx}], nodes[{expIdx}] - 1.0);");
                sb.AppendLine($"            adj[{expIdx}] += adj[{outputIdx}] * nodes[{outputIdx}] * Math.Log(nodes[{baseIdx}]);");
            }
            else if (argIndices.Count == 2 && methodName == "Atan2")
            {
                var yIdx = argIndices[0];
                var xIdx = argIndices[1];

                sb.AppendLine($"            var denom{outputIdx} = nodes[{xIdx}] * nodes[{xIdx}] + nodes[{yIdx}] * nodes[{yIdx}];");
                sb.AppendLine($"            adj[{yIdx}] += adj[{outputIdx}] * nodes[{xIdx}] / denom{outputIdx};");
                sb.AppendLine($"            adj[{xIdx}] += adj[{outputIdx}] * (-nodes[{yIdx}]) / denom{outputIdx};");
            }
            else
            {
                throw new NotSupportedException($"Math function not supported: {methodName}");
            }
        }

        private void GenerateConditionalExpressionBackprop(StringBuilder sb, int outputIdx, int conditionIdx, int trueIdx, int falseIdx)
        {
            sb.AppendLine($"            if (nodes[{conditionIdx}] != 0)");
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
                    sb.AppendLine($"{indent}adj[{leftIdx}] += adj[{outputIdx}] * nodes[{rightIdx}];");
                    sb.AppendLine($"{indent}adj[{rightIdx}] += adj[{outputIdx}] * nodes[{leftIdx}];");
                    break;

                case BinaryOperator.Divide:
                    sb.AppendLine($"{indent}adj[{leftIdx}] += adj[{outputIdx}] / nodes[{rightIdx}];");
                    sb.AppendLine($"{indent}adj[{rightIdx}] -= adj[{outputIdx}] * nodes[{leftIdx}] / (nodes[{rightIdx}] * nodes[{rightIdx}]);");
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
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] / (2.0 * nodes[{outputIdx}]);");
                        break;

                    case "Sin":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] * Math.Cos(nodes[{argIdx}]);");
                        break;

                    case "Cos":
                        sb.AppendLine($"{indent}adj[{argIdx}] -= adj[{outputIdx}] * Math.Sin(nodes[{argIdx}]);");
                        break;

                    case "Tan":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] / (Math.Cos(nodes[{argIdx}]) * Math.Cos(nodes[{argIdx}]));");
                        break;

                    case "Exp":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] * nodes[{outputIdx}];");
                        break;

                    case "Log":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] / nodes[{argIdx}];");
                        break;

                    case "Abs":
                        sb.AppendLine($"{indent}adj[{argIdx}] += adj[{outputIdx}] * (nodes[{argIdx}] / nodes[{outputIdx}]);");
                        break;

                    default:
                        throw new NotSupportedException($"Math function not supported: {methodName}");
                }
            }
            else if (argIndices.Count == 2 && methodName == "Pow")
            {
                var baseIdx = argIndices[0];
                var expIdx = argIndices[1];

                sb.AppendLine($"{indent}adj[{baseIdx}] += adj[{outputIdx}] * nodes[{expIdx}] * Math.Pow(nodes[{baseIdx}], nodes[{expIdx}] - 1.0);");
                sb.AppendLine($"{indent}adj[{expIdx}] += adj[{outputIdx}] * nodes[{outputIdx}] * Math.Log(nodes[{baseIdx}]);");
            }
            else if (argIndices.Count == 2 && methodName == "Atan2")
            {
                var yIdx = argIndices[0];
                var xIdx = argIndices[1];

                sb.AppendLine($"{indent}var denom{outputIdx} = nodes[{xIdx}] * nodes[{xIdx}] + nodes[{yIdx}] * nodes[{yIdx}];");
                sb.AppendLine($"{indent}adj[{yIdx}] += adj[{outputIdx}] * nodes[{xIdx}] / denom{outputIdx};");
                sb.AppendLine($"{indent}adj[{xIdx}] += adj[{outputIdx}] * (-nodes[{yIdx}]) / denom{outputIdx};");
            }
            else
            {
                throw new NotSupportedException($"Math function not supported: {methodName}");
            }
        }

        private void GenerateConditionalExpressionBackpropWithIndent(StringBuilder sb, int outputIdx, int conditionIdx, int trueIdx, int falseIdx, string indent)
        {
            sb.AppendLine($"{indent}if (nodes[{conditionIdx}] != 0)");
            sb.AppendLine($"{indent}{{");
            sb.AppendLine($"{indent}    adj[{trueIdx}] += adj[{outputIdx}];");
            sb.AppendLine($"{indent}}}");
            sb.AppendLine($"{indent}else");
            sb.AppendLine($"{indent}{{");
            sb.AppendLine($"{indent}    adj[{falseIdx}] += adj[{outputIdx}];");
            sb.AppendLine($"{indent}}}");
        }

        private void CountNodes(MethodBodyNode methodBody)
        {
            foreach (var stmt in methodBody.Statements)
            {
                CountNodesInStatement(stmt);
            }
        }

        private void CountNodesInStatement(IRNode stmt)
        {
            switch (stmt)
            {
                case ReturnNode returnNode:
                    CountNodesInExpression(returnNode.Value);
                    break;
                case AssignmentNode assignment:
                    CountNodesInExpression(assignment.Value);
                    break;
                case ConditionalNode conditional:
                    CountNodesInExpression(conditional.Condition);
                    foreach (var s in conditional.TrueBranch)
                        CountNodesInStatement(s);
                    foreach (var s in conditional.FalseBranch)
                        CountNodesInStatement(s);
                    break;
                case LoopNode loop:
                    foreach (var s in loop.Body)
                        CountNodesInStatement(s);
                    break;
            }
        }

        private void CountNodesInExpression(IRNode node)
        {
            if (_nodeToIndex.ContainsKey(node))
                return;

            switch (node)
            {
                case ConstantNode:
                    _nodeToIndex[node] = _nodeCounter++;
                    break;
                case VariableNode varNode:
                    // Check if it's a parameter
                    if (_parameterNameToIndex.TryGetValue(varNode.Name, out var paramIdx))
                    {
                        _nodeToIndex[node] = paramIdx;
                    }
                    else
                    {
                        _nodeToIndex[node] = _nodeCounter++;
                    }
                    break;
                case BinaryOpNode binary:
                    CountNodesInExpression(binary.Left);
                    CountNodesInExpression(binary.Right);
                    _nodeToIndex[node] = _nodeCounter++;
                    break;
                case UnaryOpNode unary:
                    CountNodesInExpression(unary.Operand);
                    _nodeToIndex[node] = _nodeCounter++;
                    break;
                case MethodCallNode methodCall:
                    foreach (var arg in methodCall.Arguments)
                        CountNodesInExpression(arg);
                    _nodeToIndex[node] = _nodeCounter++;
                    break;
                case ConditionalExpressionNode conditional:
                    CountNodesInExpression(conditional.Condition);
                    CountNodesInExpression(conditional.TrueExpression);
                    CountNodesInExpression(conditional.FalseExpression);
                    _nodeToIndex[node] = _nodeCounter++;
                    break;
            }
        }
    }
}
