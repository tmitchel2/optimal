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
using Optimal.AutoDiff.Analyzers.Differentiation;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.CodeGen
{
    public sealed class ForwardModeCodeGenerator
    {
        private readonly Dictionary<IRNode, string> _nodeToVariable = new();
        private readonly Dictionary<IRNode, string> _nodeToTangentVariable = new();
        private readonly HashSet<string> _declaredVariables = new();
        private int _tempVarCounter;
        private string _currentIndent = "            "; // 12 spaces = 3 levels
        private readonly OptimalTransform? _transform;
        private readonly ITypeSymbol _doubleType;

        public ForwardModeCodeGenerator(ITypeSymbol doubleType, OptimalTransform? transform = null)
        {
            _doubleType = doubleType;
            _transform = transform;
        }

        public string GenerateMethod(MethodToTransform method, ParameterInfo wrtParam, ITypeSymbol doubleType)
        {
            _nodeToVariable.Clear();
            _nodeToTangentVariable.Clear();
            _declaredVariables.Clear();
            _tempVarCounter = 0;

            // Mark parameters as declared
            foreach (var param in method.Parameters)
            {
                _declaredVariables.Add(param.Name);
            }

            var converter = new ASTToIRConverter(method.SemanticModel);
            var methodBody = converter.ConvertMethod(method.MethodSyntax, method.Parameters);

            var sb = new StringBuilder();

            var returnTypeName = method.ReturnType.ToDisplayString();

            sb.AppendLine($"        public static ({returnTypeName} value, double gradient) {method.MethodName}Forward_{wrtParam.Name}(");
            sb.Append("            ");
            sb.Append(string.Join(", ", method.Parameters.Select(p => $"{p.Type.ToDisplayString()} {p.Name}")));
            sb.AppendLine(")");
            sb.AppendLine("        {");

            foreach (var param in method.Parameters)
            {
                if (param.IsDifferentiable)
                {
                    if (param.Name == wrtParam.Name)
                    {
                        sb.AppendLine($"            var {param.Name}_tan = 1.0;");
                    }
                    else
                    {
                        sb.AppendLine($"            var {param.Name}_tan = 0.0;");
                    }
                }
            }

            if (method.Parameters.Any(p => p.IsDifferentiable))
            {
                sb.AppendLine();
            }

            var resultVarName = GeneratePrimalCode(methodBody, sb, wrtParam.Name, doubleType, method.ReturnType);

            var gradientVarName = _nodeToTangentVariable.ContainsKey(methodBody.Statements.Last())
                ? _nodeToTangentVariable[methodBody.Statements.Last()]
                : IsConstantLiteral(resultVarName) ? "0.0" : $"{resultVarName}_tan";

            sb.AppendLine();
            sb.AppendLine($"            return ({resultVarName}, {gradientVarName});");
            sb.AppendLine("        }");

            return sb.ToString();
        }

        private bool IsConstantLiteral(string expression)
        {
            // Check if the expression is a numeric literal (constant)
            // Matches: 1.0, 0.0, 3.14159, etc.
            if (string.IsNullOrWhiteSpace(expression))
            {
                return false;
            }

            return double.TryParse(expression, System.Globalization.NumberStyles.Float, 
                System.Globalization.CultureInfo.InvariantCulture, out _);
        }

        private string GeneratePrimalCode(MethodBodyNode methodBody, StringBuilder sb, string wrtParam, ITypeSymbol doubleType, ITypeSymbol returnType)
        {
            string? resultVarName = null;
            var needsResultVariable = HasConditionalWithReturns(methodBody);

            if (needsResultVariable)
            {
                var defaultValue = GetDefaultValue(returnType);
                sb.AppendLine($"            var result = {defaultValue};");
                sb.AppendLine("            var result_tan = 0.0;");
                sb.AppendLine();
                resultVarName = "result";
            }

            foreach (var stmt in methodBody.Statements)
            {
                if (stmt is ReturnNode returnNode)
                {
                    var (primalVar, tangentVar) = GenerateExpressionCode(returnNode.Value, sb, wrtParam, doubleType);
                    resultVarName = primalVar;
                }
                else if (stmt is AssignmentNode assignment)
                {
                    var (primalVar, tangentVar) = GenerateExpressionCode(assignment.Value, sb, wrtParam, doubleType);

                    if (!_declaredVariables.Contains(assignment.TargetVariable))
                    {
                        sb.AppendLine($"            var {assignment.TargetVariable} = {primalVar};");
                        sb.AppendLine($"            var {assignment.TargetVariable}_tan = {tangentVar};");
                        _declaredVariables.Add(assignment.TargetVariable);
                    }
                    else
                    {
                        sb.AppendLine($"            {assignment.TargetVariable} = {primalVar};");
                        sb.AppendLine($"            {assignment.TargetVariable}_tan = {tangentVar};");
                    }

                    _nodeToVariable[assignment] = assignment.TargetVariable;
                    _nodeToTangentVariable[assignment] = $"{assignment.TargetVariable}_tan";
                }
                else if (stmt is ConditionalNode conditional)
                {
                    GenerateConditionalStatement(conditional, sb, wrtParam, doubleType);
                }
                else if (stmt is LoopNode loop)
                {
                    GenerateLoopStatement(loop, sb, wrtParam, doubleType);
                }
            }

            return resultVarName ?? "result";
        }

        private bool HasConditionalWithReturns(MethodBodyNode methodBody)
        {
            foreach (var stmt in methodBody.Statements)
            {
                if (stmt is ConditionalNode conditional)
                {
                    if (HasReturnInBranch(conditional.TrueBranch) || HasReturnInBranch(conditional.FalseBranch))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        private bool HasReturnInBranch(ImmutableArray<IRNode> branch)
        {
            foreach (var stmt in branch)
            {
                if (stmt is ReturnNode)
                {
                    return true;
                }
                if (stmt is ConditionalNode nested)
                {
                    if (HasReturnInBranch(nested.TrueBranch) || HasReturnInBranch(nested.FalseBranch))
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        private (string primalVar, string tangentVar) GenerateExpressionCode(
            IRNode expression,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            if (_nodeToVariable.ContainsKey(expression))
            {
                return (_nodeToVariable[expression], _nodeToTangentVariable[expression]);
            }

            var (primalCode, tangentCode) = expression switch
            {
                ConstantNode constant => GenerateConstantCode(constant),
                VariableNode variable => GenerateVariableCode(variable, wrtParam),
                BinaryOpNode binary => GenerateBinaryOpCode(binary, sb, wrtParam, doubleType),
                UnaryOpNode unary => GenerateUnaryOpCode(unary, sb, wrtParam, doubleType),
                MethodCallNode methodCall => GenerateMethodCallCode(methodCall, sb, wrtParam, doubleType),
                ConditionalExpressionNode conditional => GenerateConditionalExpressionCode(conditional, sb, wrtParam, doubleType),
                _ => throw new NotSupportedException($"Expression type not supported: {expression.GetType().Name}")
            };

            if (NeedsTemporaryVariable(expression))
            {
                var tempVarName = $"node{_tempVarCounter++}";
                var tempTangentVarName = $"{tempVarName}_tan";

                sb.AppendLine($"{_currentIndent}var {tempVarName} = {primalCode};");
                sb.AppendLine($"{_currentIndent}var {tempTangentVarName} = {tangentCode};");

                _nodeToVariable[expression] = tempVarName;
                _nodeToTangentVariable[expression] = tempTangentVarName;

                return (tempVarName, tempTangentVarName);
            }

            return (primalCode, tangentCode);
        }

        private (string primalCode, string tangentCode) GenerateConstantCode(ConstantNode constant)
        {
            var valueStr = constant.Value switch
            {
                double d => FormatDouble(d),
                float f => f.ToString("G9", System.Globalization.CultureInfo.InvariantCulture) + "f",
                int i => i.ToString(System.Globalization.CultureInfo.InvariantCulture),
                _ => constant.Value.ToString()
            };

            return (valueStr!, "0.0");
        }

        private string FormatDouble(double value)
        {
            var str = value.ToString("G17", System.Globalization.CultureInfo.InvariantCulture);
            // Ensure double literals always have a decimal point
            if (!str.Contains('.') && !str.Contains('E') && !str.Contains('e'))
            {
                str += ".0";
            }
            return str;
        }

        private (string primalCode, string tangentCode) GenerateVariableCode(VariableNode variable, string wrtParam)
        {
            if (variable.Name == wrtParam)
            {
                return (variable.Name, $"{variable.Name}_tan");
            }

            var tangentExists = _nodeToTangentVariable.Values.Any(v => v == $"{variable.Name}_tan");
            if (tangentExists)
            {
                return (variable.Name, $"{variable.Name}_tan");
            }

            return (variable.Name, "0.0");
        }

        private (string primalCode, string tangentCode) GenerateBinaryOpCode(
            BinaryOpNode binary,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            var (leftPrimal, leftTangent) = GenerateExpressionCode(binary.Left, sb, wrtParam, doubleType);
            var (rightPrimal, rightTangent) = GenerateExpressionCode(binary.Right, sb, wrtParam, doubleType);

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

            var primalCode = $"{leftPrimal} {opSymbol} {rightPrimal}";

            var tangentCode = binary.Op switch
            {
                BinaryOperator.Add => $"{leftTangent} + {rightTangent}",
                BinaryOperator.Subtract => $"{leftTangent} - {rightTangent}",
                BinaryOperator.Multiply => $"{leftPrimal} * {rightTangent} + {rightPrimal} * {leftTangent}",
                BinaryOperator.Divide => $"({rightPrimal} * {leftTangent} - {leftPrimal} * {rightTangent}) / ({rightPrimal} * {rightPrimal})",
                BinaryOperator.GreaterThan or
                BinaryOperator.LessThan or
                BinaryOperator.GreaterThanOrEqual or
                BinaryOperator.LessThanOrEqual or
                BinaryOperator.Equal or
                BinaryOperator.NotEqual => "0.0",
                _ => throw new NotSupportedException()
            };

            return (primalCode, tangentCode);
        }

        private (string primalCode, string tangentCode) GenerateUnaryOpCode(
            UnaryOpNode unary,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            var (operandPrimal, operandTangent) = GenerateExpressionCode(unary.Operand, sb, wrtParam, doubleType);

            var (primalCode, tangentCode) = unary.Op switch
            {
                UnaryOperator.Negate => ($"-{operandPrimal}", $"-{operandTangent}"),
                UnaryOperator.Plus or UnaryOperator.Identity => (operandPrimal, operandTangent),
                _ => throw new NotSupportedException()
            };

            return (primalCode, tangentCode);
        }

        private (string primalCode, string tangentCode) GenerateMethodCallCode(
            MethodCallNode methodCall,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            var containingType = methodCall.MethodSymbol?.ContainingType?.ToDisplayString();
            var methodName = methodCall.MethodName;

            var argumentPrimals = new List<string>();
            var argumentTangents = new List<string>();

            foreach (var arg in methodCall.Arguments)
            {
                var (primal, tangent) = GenerateExpressionCode(arg, sb, wrtParam, doubleType);
                argumentPrimals.Add(primal);
                argumentTangents.Add(tangent);
            }

            // Check if this is a user-defined function from the same class
            var isUserFunction = _transform != null &&
                methodCall.MethodSymbol != null &&
                SymbolEqualityComparer.Default.Equals(methodCall.MethodSymbol.ContainingType, _transform.ClassSymbol);

            string primalCode;
            if (containingType == "System.Math")
            {
                primalCode = $"Math.{methodName}({string.Join(", ", argumentPrimals)})";
            }
            else if (isUserFunction)
            {
                // For user-defined functions, we need to inline them
                primalCode = GenerateUserFunctionInline(methodCall, argumentPrimals, sb, wrtParam, doubleType);
            }
            else
            {
                primalCode = $"{methodName}({string.Join(", ", argumentPrimals)})";
            }

            string tangentCode;
            if (isUserFunction)
            {
                tangentCode = GenerateUserFunctionTangent(methodCall, argumentPrimals, argumentTangents, sb, wrtParam, doubleType);
            }
            else
            {
                tangentCode = GenerateMethodCallTangent(
                    methodCall,
                    methodName,
                    argumentPrimals,
                    argumentTangents);
            }

            return (primalCode, tangentCode);
        }

        private string GenerateMethodCallTangent(
            MethodCallNode methodCall,
            string methodName,
            List<string> argumentPrimals,
            List<string> argumentTangents)
        {
            var primalCall = $"Math.{methodName}({string.Join(", ", argumentPrimals)})";

            if (argumentPrimals.Count == 1)
            {
                var u = argumentPrimals[0];
                var du = argumentTangents[0];

                return methodName switch
                {
                    "Sqrt" => $"1.0 / (2.0 * {primalCall}) * {du}",
                    "Sin" => $"Math.Cos({u}) * {du}",
                    "Cos" => $"-Math.Sin({u}) * {du}",
                    "Tan" => $"1.0 / (Math.Cos({u}) * Math.Cos({u})) * {du}",
                    "Exp" => $"{primalCall} * {du}",
                    "Log" => $"1.0 / {u} * {du}",
                    "Abs" => $"{u} / {primalCall} * {du}",
                    _ => throw new NotSupportedException($"Math function not supported: {methodName}")
                };
            }
            else if (argumentPrimals.Count == 2 && methodName == "Pow")
            {
                var baseArg = argumentPrimals[0];
                var exponentArg = argumentPrimals[1];
                var dBase = argumentTangents[0];
                var dExponent = argumentTangents[1];

                if (dBase == "0.0" && dExponent == "0.0")
                {
                    return "0.0";
                }
                else if (dExponent == "0.0")
                {
                    return $"{exponentArg} * Math.Pow({baseArg}, {exponentArg} - 1.0) * {dBase}";
                }
                else if (dBase == "0.0")
                {
                    return $"{primalCall} * Math.Log({baseArg}) * {dExponent}";
                }
                else
                {
                    return $"({exponentArg} * Math.Pow({baseArg}, {exponentArg} - 1.0) * {dBase}) + ({primalCall} * Math.Log({baseArg}) * {dExponent})";
                }
            }
            else if (argumentPrimals.Count == 2 && methodName == "Atan2")
            {
                var y = argumentPrimals[0];
                var x = argumentPrimals[1];
                var dy = argumentTangents[0];
                var dx = argumentTangents[1];

                return $"(({x} * {dy}) - ({y} * {dx})) / (({x} * {x}) + ({y} * {y}))";
            }

            throw new NotSupportedException($"Math function not supported: {methodName}");
        }

        private string GenerateUserFunctionInline(
            MethodCallNode methodCall,
            List<string> argumentPrimals,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            // Find the method definition
            var method = _transform!.Methods.FirstOrDefault(m => m.MethodName == methodCall.MethodName);
            if (method == null)
            {
                throw new InvalidOperationException($"Method not found: {methodCall.MethodName}");
            }

            // Convert method to IR
            var converter = new ASTToIRConverter(method.SemanticModel);
            var methodBody = converter.ConvertMethod(method.MethodSyntax, method.Parameters);

            // Create substitution map for parameters
            var substitutions = new Dictionary<string, string>();
            for (var i = 0; i < method.Parameters.Length; i++)
            {
                substitutions[method.Parameters[i].Name] = argumentPrimals[i];
            }

            // If it's an expression body (single return), inline directly
            if (methodBody.Statements.Length == 1 && methodBody.Statements[0] is ReturnNode returnNode)
            {
                return GenerateInlineExpressionWithSubstitutions(returnNode.Value, substitutions);
            }

            // Otherwise, we need a more complex inlining (for now, throw)
            throw new NotSupportedException($"Complex function bodies not yet supported for inlining: {methodCall.MethodName}");
        }

        private string GenerateUserFunctionTangent(
            MethodCallNode methodCall,
            List<string> argumentPrimals,
            List<string> argumentTangents,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            // Find the method definition
            var method = _transform!.Methods.FirstOrDefault(m => m.MethodName == methodCall.MethodName);
            if (method == null)
            {
                throw new InvalidOperationException($"Method not found: {methodCall.MethodName}");
            }

            // Convert method to IR
            var converter = new ASTToIRConverter(method.SemanticModel);
            var methodBody = converter.ConvertMethod(method.MethodSyntax, method.Parameters);

            // Differentiate the inlined function using chain rule
            // df/dx = df/da * da/dx + df/db * db/dx + ...
            // where a, b, ... are the arguments

            if (methodBody.Statements.Length == 1 && methodBody.Statements[0] is ReturnNode returnNode)
            {
                // Create a mini-differentiator context for this function
                var tangentTerms = new List<string>();

                for (var i = 0; i < method.Parameters.Length; i++)
                {
                    var param = method.Parameters[i];
                    var argTangent = argumentTangents[i];

                    // Skip if argument has zero derivative
                    if (argTangent == "0.0")
                    {
                        continue;
                    }

                    // Differentiate the function body w.r.t. this parameter
                    var partialDerivative = ComputePartialDerivative(returnNode.Value, param.Name, method, argumentPrimals);

                    if (partialDerivative == "0.0")
                    {
                        continue;
                    }

                    tangentTerms.Add($"({partialDerivative}) * {argTangent}");
                }

                if (tangentTerms.Count == 0)
                {
                    return "0.0";
                }

                return string.Join(" + ", tangentTerms);
            }

            throw new NotSupportedException($"Complex function bodies not yet supported: {methodCall.MethodName}");
        }

        private string ComputePartialDerivative(
            IRNode expression,
            string parameterName,
            MethodToTransform method,
            List<string> argumentPrimals)
        {
            // Use the differentiator to compute the derivative
            var differentiator = new ForwardModeDifferentiator(_doubleType, _transform);
            var context = new ForwardModeContext(0, parameterName);

            // Set tangents for all parameters (1.0 for our parameter, 0.0 for others)
            for (var i = 0; i < method.Parameters.Length; i++)
            {
                var param = method.Parameters[i];
                if (param.Name == parameterName)
                {
                    context.SetTangent(param.Name, new ConstantNode(context.NewNodeId(), 1.0, _doubleType));
                }
                else
                {
                    context.SetTangent(param.Name, new ConstantNode(context.NewNodeId(), 0.0, _doubleType));
                }
            }

            // Differentiate the expression
            var derivativeNode = differentiator.Differentiate(expression, context);

            // Create substitution map
            var substitutions = new Dictionary<string, string>();
            for (var i = 0; i < method.Parameters.Length; i++)
            {
                substitutions[method.Parameters[i].Name] = argumentPrimals[i];
            }

            // Convert the derivative IR back to code
            return GenerateInlineExpressionWithSubstitutions(derivativeNode, substitutions);
        }

        private string GenerateInlineExpressionWithSubstitutions(IRNode expression, Dictionary<string, string> substitutions)
        {
            return expression switch
            {
                ConstantNode constant => constant.Value switch
                {
                    double d => FormatDouble(d),
                    float f => f.ToString("G9", System.Globalization.CultureInfo.InvariantCulture) + "f",
                    int i => i.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    _ => constant.Value.ToString()!
                },
                VariableNode variable => substitutions.TryGetValue(variable.Name, out var replacement)
                    ? replacement
                    : variable.Name,
                BinaryOpNode binary => GenerateBinaryOpInline(binary, substitutions),
                UnaryOpNode unary => GenerateUnaryOpInline(unary, substitutions),
                MethodCallNode methodCall => GenerateMethodCallInline(methodCall, substitutions),
                _ => throw new NotSupportedException($"Cannot inline expression type: {expression.GetType().Name}")
            };
        }

        private string GenerateBinaryOpInline(BinaryOpNode binary, Dictionary<string, string> substitutions)
        {
            var left = GenerateInlineExpressionWithSubstitutions(binary.Left, substitutions);
            var right = GenerateInlineExpressionWithSubstitutions(binary.Right, substitutions);

            var opSymbol = binary.Op switch
            {
                BinaryOperator.Add => "+",
                BinaryOperator.Subtract => "-",
                BinaryOperator.Multiply => "*",
                BinaryOperator.Divide => "/",
                _ => throw new NotSupportedException()
            };

            return $"({left} {opSymbol} {right})";
        }

        private string GenerateUnaryOpInline(UnaryOpNode unary, Dictionary<string, string> substitutions)
        {
            var operand = GenerateInlineExpressionWithSubstitutions(unary.Operand, substitutions);

            return unary.Op switch
            {
                UnaryOperator.Negate => $"(-{operand})",
                UnaryOperator.Plus or UnaryOperator.Identity => operand,
                _ => throw new NotSupportedException()
            };
        }

        private string GenerateMethodCallInline(MethodCallNode methodCall, Dictionary<string, string> substitutions)
        {
            var args = string.Join(", ", methodCall.Arguments.Select(arg => GenerateInlineExpressionWithSubstitutions(arg, substitutions)));

            var containingType = methodCall.MethodSymbol?.ContainingType?.ToDisplayString();
            if (containingType == "System.Math")
            {
                return $"Math.{methodCall.MethodName}({args})";
            }

            // Check if this is a user-defined function
            var isUserFunction = _transform != null &&
                methodCall.MethodSymbol != null &&
                SymbolEqualityComparer.Default.Equals(methodCall.MethodSymbol.ContainingType, _transform.ClassSymbol);

            if (isUserFunction)
            {
                // Recursively inline user functions
                var method = _transform!.Methods.FirstOrDefault(m => m.MethodName == methodCall.MethodName);
                if (method != null)
                {
                    var converter = new ASTToIRConverter(method.SemanticModel);
                    var methodBody = converter.ConvertMethod(method.MethodSyntax, method.Parameters);

                    if (methodBody.Statements.Length == 1 && methodBody.Statements[0] is ReturnNode returnNode)
                    {
                        // Create new substitutions for the nested call
                        var nestedSubstitutions = new Dictionary<string, string>();
                        var argStrings = methodCall.Arguments.Select(arg => GenerateInlineExpressionWithSubstitutions(arg, substitutions)).ToList();
                        for (var i = 0; i < method.Parameters.Length; i++)
                        {
                            nestedSubstitutions[method.Parameters[i].Name] = argStrings[i];
                        }

                        return GenerateInlineExpressionWithSubstitutions(returnNode.Value, nestedSubstitutions);
                    }
                }
            }

            return $"{methodCall.MethodName}({args})";
        }

        private (string primalCode, string tangentCode) GenerateConditionalExpressionCode(
            ConditionalExpressionNode conditional,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            var (conditionPrimal, _) = GenerateExpressionCode(conditional.Condition, sb, wrtParam, doubleType);
            var (truePrimal, trueTangent) = GenerateExpressionCode(conditional.TrueExpression, sb, wrtParam, doubleType);
            var (falsePrimal, falseTangent) = GenerateExpressionCode(conditional.FalseExpression, sb, wrtParam, doubleType);

            var primalCode = $"({conditionPrimal} ? {truePrimal} : {falsePrimal})";
            var tangentCode = $"({conditionPrimal} ? {trueTangent} : {falseTangent})";

            return (primalCode, tangentCode);
        }

        private void GenerateConditionalStatement(
            ConditionalNode conditional,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            // Pre-declare any variables that are assigned in branches but not yet declared
            // This ensures variables are in scope after the if-else block
            PreDeclareConditionalVariables(conditional, sb);
            
            var (conditionPrimal, _) = GenerateExpressionCode(conditional.Condition, sb, wrtParam, doubleType);

            sb.AppendLine($"            if ({conditionPrimal})");
            sb.AppendLine("            {");

            var previousIndent = _currentIndent;
            _currentIndent = "                "; // 16 spaces = 4 levels

            foreach (var stmt in conditional.TrueBranch)
            {
                GenerateStatementInBranch(stmt, sb, wrtParam, doubleType);
            }

            _currentIndent = previousIndent;

            sb.AppendLine("            }");

            if (conditional.FalseBranch.Length > 0)
            {
                sb.AppendLine("            else");
                sb.AppendLine("            {");

                _currentIndent = "                "; // 16 spaces = 4 levels

                foreach (var stmt in conditional.FalseBranch)
                {
                    GenerateStatementInBranch(stmt, sb, wrtParam, doubleType);
                }

                _currentIndent = previousIndent;

                sb.AppendLine("            }");
            }
        }

        /// <summary>
        /// Pre-declares variables that are assigned inside conditional branches.
        /// This ensures they are in scope after the if-else block.
        /// </summary>
        private void PreDeclareConditionalVariables(ConditionalNode conditional, StringBuilder sb)
        {
            var varsInBranches = new HashSet<string>();
            CollectAssignedVariables(conditional.TrueBranch, varsInBranches);
            CollectAssignedVariables(conditional.FalseBranch, varsInBranches);

            foreach (var varName in varsInBranches)
            {
                if (!_declaredVariables.Contains(varName))
                {
                    sb.AppendLine($"            double {varName} = default;");
                    sb.AppendLine($"            double {varName}_tan = default;");
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

        private void GenerateLoopStatement(
            LoopNode loop,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            var initializerCode = "";
            if (loop.Initializer != null)
            {
                initializerCode = $"var {loop.IteratorVariable} = {GenerateInlineExpression(loop.Initializer)}";
            }

            var conditionCode = "";
            if (loop.Condition != null)
            {
                conditionCode = GenerateInlineExpression(loop.Condition);
            }

            var incrementCode = "";
            if (loop.Increment != null)
            {
                incrementCode = $"{loop.IteratorVariable} = {GenerateInlineExpression(loop.Increment)}";
            }

            sb.AppendLine($"            for ({initializerCode}; {conditionCode}; {incrementCode})");
            sb.AppendLine("            {");

            var previousIndent = _currentIndent;
            _currentIndent = "                "; // 16 spaces = 4 levels

            foreach (var stmt in loop.Body)
            {
                GenerateStatementInBranch(stmt, sb, wrtParam, doubleType);
            }

            _currentIndent = previousIndent;

            sb.AppendLine("            }");
        }

        private string GenerateInlineExpression(IRNode expression)
        {
            return expression switch
            {
                ConstantNode constant => constant.Value switch
                {
                    double d => FormatDouble(d),
                    float f => f.ToString("G9", System.Globalization.CultureInfo.InvariantCulture) + "f",
                    int i => i.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    _ => constant.Value.ToString()!
                },
                VariableNode variable => variable.Name,
                BinaryOpNode binary => GenerateInlineBinaryOp(binary),
                _ => throw new NotSupportedException($"Inline expression not supported: {expression.GetType().Name}")
            };
        }

        private string GenerateInlineBinaryOp(BinaryOpNode binary)
        {
            var left = GenerateInlineExpression(binary.Left);
            var right = GenerateInlineExpression(binary.Right);

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

            return $"{left} {opSymbol} {right}";
        }

        private void GenerateStatementInBranch(
            IRNode stmt,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType)
        {
            if (stmt is AssignmentNode assignment)
            {
                var (primalVar, tangentVar) = GenerateExpressionCode(assignment.Value, sb, wrtParam, doubleType);

                // Check if this is a new variable declaration
                if (!_declaredVariables.Contains(assignment.TargetVariable))
                {
                    sb.AppendLine($"{_currentIndent}var {assignment.TargetVariable} = {primalVar};");
                    sb.AppendLine($"{_currentIndent}var {assignment.TargetVariable}_tan = {tangentVar};");
                    _declaredVariables.Add(assignment.TargetVariable);
                }
                else
                {
                    sb.AppendLine($"{_currentIndent}{assignment.TargetVariable} = {primalVar};");
                    sb.AppendLine($"{_currentIndent}{assignment.TargetVariable}_tan = {tangentVar};");
                }

                _nodeToVariable[assignment] = assignment.TargetVariable;
                _nodeToTangentVariable[assignment] = $"{assignment.TargetVariable}_tan";
            }
            else if (stmt is ReturnNode returnNode)
            {
                var (primalVar, tangentVar) = GenerateExpressionCode(returnNode.Value, sb, wrtParam, doubleType);

                sb.AppendLine($"{_currentIndent}result = {primalVar};");
                sb.AppendLine($"{_currentIndent}result_tan = {tangentVar};");
            }
            else if (stmt is ConditionalNode nested)
            {
                // Pre-declare any variables that are assigned in nested branches
                PreDeclareConditionalVariablesWithIndent(nested, sb, _currentIndent);
                
                var (conditionPrimal, _) = GenerateExpressionCode(nested.Condition, sb, wrtParam, doubleType);

                sb.AppendLine($"{_currentIndent}if ({conditionPrimal})");
                sb.AppendLine($"{_currentIndent}{{");

                var previousIndent = _currentIndent;
                _currentIndent = _currentIndent + "    "; // Add 4 spaces for next level

                foreach (var nestedStmt in nested.TrueBranch)
                {
                    GenerateStatementInBranch(nestedStmt, sb, wrtParam, doubleType);
                }

                _currentIndent = previousIndent;

                sb.AppendLine($"{_currentIndent}}}");

                if (nested.FalseBranch.Length > 0)
                {
                    sb.AppendLine($"{_currentIndent}else");
                    sb.AppendLine($"{_currentIndent}{{");

                    _currentIndent = _currentIndent + "    "; // Add 4 spaces for next level

                    foreach (var nestedStmt in nested.FalseBranch)
                    {
                        GenerateStatementInBranch(nestedStmt, sb, wrtParam, doubleType);
                    }

                    _currentIndent = previousIndent;

                    sb.AppendLine($"{_currentIndent}}}");
                }
            }
        }

        /// <summary>
        /// Pre-declares variables that are assigned inside conditional branches, using specified indent.
        /// </summary>
        private void PreDeclareConditionalVariablesWithIndent(ConditionalNode conditional, StringBuilder sb, string indent)
        {
            var varsInBranches = new HashSet<string>();
            CollectAssignedVariables(conditional.TrueBranch, varsInBranches);
            CollectAssignedVariables(conditional.FalseBranch, varsInBranches);

            foreach (var varName in varsInBranches)
            {
                if (!_declaredVariables.Contains(varName))
                {
                    sb.AppendLine($"{indent}double {varName} = default;");
                    sb.AppendLine($"{indent}double {varName}_tan = default;");
                    _declaredVariables.Add(varName);
                }
            }
        }

        private void GenerateStatementInNestedBranch(
            IRNode stmt,
            StringBuilder sb,
            string wrtParam,
            ITypeSymbol doubleType,
            string indent)
        {
            if (stmt is AssignmentNode assignment)
            {
                var (primalVar, tangentVar) = GenerateExpressionCode(assignment.Value, sb, wrtParam, doubleType);

                sb.AppendLine($"{indent}{assignment.TargetVariable} = {primalVar};");
                sb.AppendLine($"{indent}{assignment.TargetVariable}_tan = {tangentVar};");

                _nodeToVariable[assignment] = assignment.TargetVariable;
                _nodeToTangentVariable[assignment] = $"{assignment.TargetVariable}_tan";
            }
            else if (stmt is ReturnNode returnNode)
            {
                var (primalVar, tangentVar) = GenerateExpressionCode(returnNode.Value, sb, wrtParam, doubleType);

                sb.AppendLine($"{indent}result = {primalVar};");
                sb.AppendLine($"{indent}result_tan = {tangentVar};");
            }
        }

        private bool NeedsTemporaryVariable(IRNode node)
        {
            return node is BinaryOpNode or UnaryOpNode or MethodCallNode or ConditionalExpressionNode;
        }

        private string GetDefaultValue(ITypeSymbol type)
        {
            if (type.SpecialType == SpecialType.System_Double)
            {
                return "0.0";
            }

            var typeName = type.ToDisplayString();
            return typeName switch
            {
                "double" or "System.Double" => "0.0",
                "float" or "System.Single" => "0.0f",
                "int" or "System.Int32" => "0",
                "long" or "System.Int64" => "0L",
                _ => "0.0"  // Default to double for safety
            };
        }
    }
}
