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
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.CSharp.Syntax;

namespace Optimal.AutoDiff.Analyzers.IR
{
    public sealed class ASTToIRConverter
    {
        private readonly SemanticModel _semanticModel;
        private readonly Dictionary<string, VariableNode> _variables = new();
        private readonly Dictionary<string, IRNode> _intermediateValueExpressions = new(); // Track intermediate variable value expressions
        private readonly HashSet<string> _reassignedVariables = new(); // NEW: Track variables that are reassigned
        private readonly List<IRNode> _statements = new();
        private int _nodeIdCounter;

        public ASTToIRConverter(SemanticModel semanticModel)
        {
            _semanticModel = semanticModel;
        }

        public MethodBodyNode ConvertMethod(MethodDeclarationSyntax method, ImmutableArray<ParameterInfo> parameters)
        {
            _statements.Clear();
            _variables.Clear();
            _intermediateValueExpressions.Clear();
            _reassignedVariables.Clear();
            _nodeIdCounter = 0;

            // Mark parameters in variables dictionary but NOT in intermediate expressions
            // Parameters should never be inlined
            foreach (var param in parameters)
            {
                var paramNode = new VariableNode(NewNodeId(), param.Name, param.Type);
                _variables[param.Name] = paramNode;
                // NEW: Explicitly mark parameters as "non-inlineable" by adding to reassigned set
                _reassignedVariables.Add(param.Name);
            }

            if (method.ExpressionBody != null)
            {
                var expression = ConvertExpression(method.ExpressionBody.Expression);
                _statements.Add(new ReturnNode(NewNodeId(), expression));
            }
            else if (method.Body != null)
            {
                foreach (var statement in method.Body.Statements)
                {
                    var irStatement = ConvertStatement(statement);
                    if (irStatement != null)
                    {
                        _statements.Add(irStatement);
                    }
                }
            }

            return new MethodBodyNode(NewNodeId(), _statements.ToImmutableArray());
        }

        private IRNode? ConvertStatement(StatementSyntax statement)
        {
            return statement switch
            {
                ReturnStatementSyntax returnStmt => ConvertReturnStatement(returnStmt),
                LocalDeclarationStatementSyntax localDecl => ConvertLocalDeclaration(localDecl),
                ExpressionStatementSyntax exprStmt => ConvertExpressionStatement(exprStmt),
                IfStatementSyntax ifStmt => ConvertIfStatement(ifStmt),
                ForStatementSyntax forStmt => ConvertForStatement(forStmt),
                _ => throw new NotSupportedException($"Statement type not supported: {statement.GetType().Name}")
            };
        }

        private IRNode ConvertReturnStatement(ReturnStatementSyntax returnStmt)
        {
            var expression = ConvertExpression(returnStmt.Expression!);
            return new ReturnNode(NewNodeId(), expression);
        }

        private IRNode? ConvertLocalDeclaration(LocalDeclarationStatementSyntax localDecl)
        {
            IRNode? lastAssignment = null;

            foreach (var variable in localDecl.Declaration.Variables)
            {
                var variableName = variable.Identifier.Text;
                var typeInfo = _semanticModel.GetTypeInfo(localDecl.Declaration.Type);
                var type = typeInfo.Type ?? typeInfo.ConvertedType;

                var varNode = new VariableNode(NewNodeId(), variableName, type!);
                _variables[variableName] = varNode;

                if (variable.Initializer != null)
                {
                    var value = ConvertExpression(variable.Initializer.Value);
                    // NEW: Track the value expression for intermediate variables
                    _intermediateValueExpressions[variableName] = value;
                    lastAssignment = new AssignmentNode(NewNodeId(), variableName, value);
                }
            }

            return lastAssignment;
        }

        private IRNode? ConvertExpressionStatement(ExpressionStatementSyntax exprStmt)
        {
            if (exprStmt.Expression is AssignmentExpressionSyntax assignment)
            {
                var target = assignment.Left;
                IRNode value;

                if (target is IdentifierNameSyntax identifier)
                {
                    var targetName = identifier.Identifier.Text;

                    // NEW: Mark this variable as reassigned (can't be inlined)
                    if (_variables.ContainsKey(targetName))
                    {
                        _reassignedVariables.Add(targetName);
                        _intermediateValueExpressions.Remove(targetName); // Remove from inlineable expressions
                    }

                    if (assignment.Kind() == SyntaxKind.SimpleAssignmentExpression)
                    {
                        value = ConvertExpression(assignment.Right);
                    }
                    else
                    {
                        var targetVar = _variables.TryGetValue(targetName, out var varNode)
                            ? varNode
                            : new VariableNode(NewNodeId(), targetName, _semanticModel.GetTypeInfo(target).Type!);

                        var rightValue = ConvertExpression(assignment.Right);

                        value = assignment.Kind() switch
                        {
                            SyntaxKind.AddAssignmentExpression => new BinaryOpNode(NewNodeId(), BinaryOperator.Add, targetVar, rightValue, targetVar.Type),
                            SyntaxKind.SubtractAssignmentExpression => new BinaryOpNode(NewNodeId(), BinaryOperator.Subtract, targetVar, rightValue, targetVar.Type),
                            SyntaxKind.MultiplyAssignmentExpression => new BinaryOpNode(NewNodeId(), BinaryOperator.Multiply, targetVar, rightValue, targetVar.Type),
                            SyntaxKind.DivideAssignmentExpression => new BinaryOpNode(NewNodeId(), BinaryOperator.Divide, targetVar, rightValue, targetVar.Type),
                            _ => throw new NotSupportedException($"Assignment kind not supported: {assignment.Kind()}")
                        };
                    }

                    return new AssignmentNode(NewNodeId(), targetName, value);
                }
            }

            return null;
        }

        private IRNode ConvertExpression(ExpressionSyntax expression)
        {
            return expression switch
            {
                LiteralExpressionSyntax literal => ConvertLiteral(literal),
                IdentifierNameSyntax identifier => ConvertIdentifier(identifier),
                BinaryExpressionSyntax binary => ConvertBinaryExpression(binary),
                ParenthesizedExpressionSyntax paren => ConvertExpression(paren.Expression),
                PrefixUnaryExpressionSyntax prefixUnary => ConvertPrefixUnaryExpression(prefixUnary),
                PostfixUnaryExpressionSyntax postfixUnary => ConvertPostfixUnaryExpression(postfixUnary),
                InvocationExpressionSyntax invocation => ConvertInvocation(invocation),
                ConditionalExpressionSyntax conditional => ConvertConditionalExpression(conditional),
                CastExpressionSyntax cast => ConvertCastExpression(cast),
                MemberAccessExpressionSyntax memberAccess => ConvertMemberAccess(memberAccess),
                _ => throw new NotSupportedException($"Expression type not supported: {expression.GetType().Name}")
            };
        }

        private IRNode ConvertCastExpression(CastExpressionSyntax cast)
        {
            // For differentiation purposes, a cast is an identity operation
            // Casts like (double)x don't affect gradients - d/dx[(double)x] = d/dx[x]
            var operand = ConvertExpression(cast.Expression);
            var typeInfo = _semanticModel.GetTypeInfo(cast);
            var targetType = typeInfo.Type ?? typeInfo.ConvertedType;

            // Wrap in an identity operation to preserve the type information
            return new UnaryOpNode(NewNodeId(), UnaryOperator.Identity, operand, targetType!);
        }

        private IRNode ConvertLiteral(LiteralExpressionSyntax literal)
        {
            var typeInfo = _semanticModel.GetTypeInfo(literal);
            var type = typeInfo.Type ?? typeInfo.ConvertedType;
            return new ConstantNode(NewNodeId(), literal.Token.Value!, type!);
        }

        private IRNode ConvertIdentifier(IdentifierNameSyntax identifier)
        {
            var name = identifier.Identifier.Text;

            // NEW: Only inline if variable is an intermediate that hasn't been reassigned
            if (_intermediateValueExpressions.TryGetValue(name, out var valueExpr) && !_reassignedVariables.Contains(name))
            {
                return valueExpr;
            }

            if (_variables.TryGetValue(name, out var varNode))
            {
                return varNode;
            }

            var symbolInfo = _semanticModel.GetSymbolInfo(identifier);
            var symbol = symbolInfo.Symbol;

            if (symbol is ILocalSymbol localSymbol)
            {
                var varNode2 = new VariableNode(NewNodeId(), name, localSymbol.Type);
                _variables[name] = varNode2;
                return varNode2;
            }
            else if (symbol is IParameterSymbol paramSymbol)
            {
                var varNode2 = new VariableNode(NewNodeId(), name, paramSymbol.Type);
                _variables[name] = varNode2;
                return varNode2;
            }
            else if (symbol is IFieldSymbol fieldSymbol && fieldSymbol.IsConst)
            {
                var value = fieldSymbol.ConstantValue;
                return new ConstantNode(NewNodeId(), value!, fieldSymbol.Type);
            }
            else if (symbol is IPropertySymbol propertySymbol)
            {
                // Handle property access by inlining the getter expression
                return ConvertPropertyAccess(propertySymbol);
            }

            throw new InvalidOperationException($"Unknown identifier: {name}");
        }

        private IRNode ConvertPropertyAccess(IPropertySymbol propertySymbol)
        {
            // Get the property declaration syntax from the symbol
            var syntaxReference = propertySymbol.DeclaringSyntaxReferences.FirstOrDefault();
            if (syntaxReference == null)
            {
                throw new InvalidOperationException($"Property '{propertySymbol.Name}' has no syntax reference - cannot inline external properties");
            }

            var propertySyntax = syntaxReference.GetSyntax() as PropertyDeclarationSyntax;
            if (propertySyntax == null)
            {
                throw new InvalidOperationException($"Property '{propertySymbol.Name}' is not a property declaration");
            }

            // Handle expression-bodied property: public static double Prop => expression;
            if (propertySyntax.ExpressionBody != null)
            {
                return ConvertExpression(propertySyntax.ExpressionBody.Expression);
            }

            // Handle get accessor with expression body: public static double Prop { get => expression; }
            var getter = propertySyntax.AccessorList?.Accessors
                .FirstOrDefault(a => a.IsKind(SyntaxKind.GetAccessorDeclaration));

            if (getter?.ExpressionBody != null)
            {
                return ConvertExpression(getter.ExpressionBody.Expression);
            }

            // Handle get accessor with block body: public static double Prop { get { return expression; } }
            if (getter?.Body != null)
            {
                var returnStatement = getter.Body.Statements.OfType<ReturnStatementSyntax>().FirstOrDefault();
                if (returnStatement?.Expression != null)
                {
                    return ConvertExpression(returnStatement.Expression);
                }
            }

            throw new InvalidOperationException($"Property '{propertySymbol.Name}' does not have a simple expression body that can be inlined");
        }

        private IRNode ConvertBinaryExpression(BinaryExpressionSyntax binary)
        {
            var left = ConvertExpression(binary.Left);
            var right = ConvertExpression(binary.Right);
            var op = ConvertBinaryOperator(binary.Kind());
            var typeInfo = _semanticModel.GetTypeInfo(binary);
            var type = typeInfo.Type ?? typeInfo.ConvertedType;

            return new BinaryOpNode(NewNodeId(), op, left, right, type!);
        }

        private IRNode ConvertPrefixUnaryExpression(PrefixUnaryExpressionSyntax prefixUnary)
        {
            var operand = ConvertExpression(prefixUnary.Operand);
            var op = ConvertUnaryOperator(prefixUnary.Kind());
            var typeInfo = _semanticModel.GetTypeInfo(prefixUnary);
            var type = typeInfo.Type ?? typeInfo.ConvertedType;

            return new UnaryOpNode(NewNodeId(), op, operand, type!);
        }

        private IRNode ConvertPostfixUnaryExpression(PostfixUnaryExpressionSyntax postfixUnary)
        {
            var operand = ConvertExpression(postfixUnary.Operand);
            var typeInfo = _semanticModel.GetTypeInfo(postfixUnary);
            var type = typeInfo.Type ?? typeInfo.ConvertedType;

            if (postfixUnary.Kind() == SyntaxKind.PostIncrementExpression)
            {
                var one = new ConstantNode(NewNodeId(), 1, type!);
                return new BinaryOpNode(NewNodeId(), BinaryOperator.Add, operand, one, type!);
            }
            else if (postfixUnary.Kind() == SyntaxKind.PostDecrementExpression)
            {
                var one = new ConstantNode(NewNodeId(), 1, type!);
                return new BinaryOpNode(NewNodeId(), BinaryOperator.Subtract, operand, one, type!);
            }

            throw new NotSupportedException($"Postfix unary operator not supported: {postfixUnary.Kind()}");
        }

        private IRNode ConvertMemberAccess(MemberAccessExpressionSyntax memberAccess)
        {
            var symbolInfo = _semanticModel.GetSymbolInfo(memberAccess);
            var symbol = symbolInfo.Symbol;

            if (symbol is IFieldSymbol fieldSymbol && fieldSymbol.IsConst)
            {
                var value = fieldSymbol.ConstantValue;
                return new ConstantNode(NewNodeId(), value!, fieldSymbol.Type);
            }

            throw new NotSupportedException($"Member access not supported: {memberAccess}");
        }

        private IRNode ConvertInvocation(InvocationExpressionSyntax invocation)
        {
            var symbolInfo = _semanticModel.GetSymbolInfo(invocation);
            var methodSymbol = symbolInfo.Symbol as IMethodSymbol;

            if (methodSymbol == null)
            {
                throw new InvalidOperationException($"Could not resolve method symbol for invocation: {invocation}");
            }

            var arguments = invocation.ArgumentList.Arguments
                .Select(arg => ConvertExpression(arg.Expression))
                .ToImmutableArray();

            var typeInfo = _semanticModel.GetTypeInfo(invocation);
            var returnType = typeInfo.Type ?? methodSymbol.ReturnType;

            var methodName = methodSymbol.Name;

            return new MethodCallNode(NewNodeId(), methodName, methodSymbol, arguments, returnType);
        }

        private IRNode ConvertIfStatement(IfStatementSyntax ifStmt)
        {
            var condition = ConvertExpression(ifStmt.Condition);

            var trueBranch = new List<IRNode>();
            if (ifStmt.Statement is BlockSyntax trueBlock)
            {
                foreach (var stmt in trueBlock.Statements)
                {
                    var converted = ConvertStatement(stmt);
                    if (converted != null)
                    {
                        trueBranch.Add(converted);
                    }
                }
            }
            else
            {
                var converted = ConvertStatement(ifStmt.Statement);
                if (converted != null)
                {
                    trueBranch.Add(converted);
                }
            }

            var falseBranch = new List<IRNode>();
            if (ifStmt.Else != null)
            {
                if (ifStmt.Else.Statement is BlockSyntax falseBlock)
                {
                    foreach (var stmt in falseBlock.Statements)
                    {
                        var converted = ConvertStatement(stmt);
                        if (converted != null)
                        {
                            falseBranch.Add(converted);
                        }
                    }
                }
                else
                {
                    var converted = ConvertStatement(ifStmt.Else.Statement);
                    if (converted != null)
                    {
                        falseBranch.Add(converted);
                    }
                }
            }

            var typeInfo = _semanticModel.GetTypeInfo(ifStmt);
            var type = typeInfo.Type ?? _semanticModel.Compilation.GetSpecialType(SpecialType.System_Void);

            return new ConditionalNode(
                NewNodeId(),
                condition,
                trueBranch.ToImmutableArray(),
                falseBranch.ToImmutableArray(),
                type!);
        }

        private IRNode ConvertForStatement(ForStatementSyntax forStmt)
        {
            string? iteratorVariable = null;
            IRNode? initializer = null;

            if (forStmt.Declaration != null && forStmt.Declaration.Variables.Count > 0)
            {
                var variable = forStmt.Declaration.Variables[0];
                iteratorVariable = variable.Identifier.Text;

                var typeInfo = _semanticModel.GetTypeInfo(forStmt.Declaration.Type);
                var type = typeInfo.Type ?? typeInfo.ConvertedType;

                var varNode = new VariableNode(NewNodeId(), iteratorVariable, type!);
                _variables[iteratorVariable] = varNode;

                if (variable.Initializer != null)
                {
                    initializer = ConvertExpression(variable.Initializer.Value);
                }
            }

            IRNode? condition = null;
            if (forStmt.Condition != null)
            {
                condition = ConvertExpression(forStmt.Condition);
            }

            IRNode? increment = null;
            if (forStmt.Incrementors.Count > 0)
            {
                increment = ConvertExpression(forStmt.Incrementors[0]);
            }

            var body = new List<IRNode>();
            if (forStmt.Statement is BlockSyntax block)
            {
                foreach (var stmt in block.Statements)
                {
                    var converted = ConvertStatement(stmt);
                    if (converted != null)
                    {
                        body.Add(converted);
                    }
                }
            }
            else
            {
                var converted = ConvertStatement(forStmt.Statement);
                if (converted != null)
                {
                    body.Add(converted);
                }
            }

            var loopTypeInfo = _semanticModel.GetTypeInfo(forStmt);
            var loopType = loopTypeInfo.Type ?? _semanticModel.Compilation.GetSpecialType(SpecialType.System_Void);

            return new LoopNode(
                NewNodeId(),
                iteratorVariable ?? "i",
                initializer,
                condition,
                increment,
                body.ToImmutableArray(),
                loopType!);
        }

        private IRNode ConvertConditionalExpression(ConditionalExpressionSyntax conditional)
        {
            var condition = ConvertExpression(conditional.Condition);
            var trueExpr = ConvertExpression(conditional.WhenTrue);
            var falseExpr = ConvertExpression(conditional.WhenFalse);

            var typeInfo = _semanticModel.GetTypeInfo(conditional);
            var type = typeInfo.Type ?? typeInfo.ConvertedType;

            return new ConditionalExpressionNode(
                NewNodeId(),
                condition,
                trueExpr,
                falseExpr,
                type!);
        }

        private BinaryOperator ConvertBinaryOperator(SyntaxKind kind)
        {
            return kind switch
            {
                SyntaxKind.AddExpression => BinaryOperator.Add,
                SyntaxKind.SubtractExpression => BinaryOperator.Subtract,
                SyntaxKind.MultiplyExpression => BinaryOperator.Multiply,
                SyntaxKind.DivideExpression => BinaryOperator.Divide,
                SyntaxKind.GreaterThanExpression => BinaryOperator.GreaterThan,
                SyntaxKind.LessThanExpression => BinaryOperator.LessThan,
                SyntaxKind.GreaterThanOrEqualExpression => BinaryOperator.GreaterThanOrEqual,
                SyntaxKind.LessThanOrEqualExpression => BinaryOperator.LessThanOrEqual,
                SyntaxKind.EqualsExpression => BinaryOperator.Equal,
                SyntaxKind.NotEqualsExpression => BinaryOperator.NotEqual,
                _ => throw new NotSupportedException($"Binary operator not supported: {kind}")
            };
        }

        private UnaryOperator ConvertUnaryOperator(SyntaxKind kind)
        {
            return kind switch
            {
                SyntaxKind.UnaryMinusExpression => UnaryOperator.Negate,
                SyntaxKind.UnaryPlusExpression => UnaryOperator.Plus,
                _ => throw new NotSupportedException($"Unary operator not supported: {kind}")
            };
        }

        private int NewNodeId() => _nodeIdCounter++;
    }
}
