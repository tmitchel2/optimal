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
using Microsoft.CodeAnalysis.Operations;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Converts a kernel or a helper it calls to statement IR: scalar and span parameters, locals, arrays and static
    /// tables, element reads and writes, loops, branches and calls. Unlike <see cref="ASTToIRConverter"/> it never
    /// inlines locals or evaluates anything: the <see cref="LoopUnroller"/> interprets the statements and builds SSA.
    /// Syntax outside the kernel language is a <see cref="KernelException"/> at the offending syntax.
    /// </summary>
    public sealed class KernelASTToIRConverter
    {
        private static readonly char[] LineBreaks = { '\r', '\n' };

        private readonly Compilation _compilation;
        private readonly Dictionary<SyntaxTree, SemanticModel> _models = new();
        private readonly Dictionary<int, Location> _locations = new();
        private readonly Dictionary<ISymbol, IRNode> _staticMembers = new(SymbolEqualityComparer.Default);
        private readonly ITypeSymbol _int;
        private readonly ITypeSymbol _void;
        private SemanticModel _model;
        private int _nextNodeId;

        public KernelASTToIRConverter(Compilation compilation)
        {
            _compilation = compilation;
            _int = compilation.GetSpecialType(SpecialType.System_Int32);
            _void = compilation.GetSpecialType(SpecialType.System_Void);
            _model = null!;
        }

        /// <summary>The source location an IR node was converted from.</summary>
        public Location? LocationOf(IRNode node)
        {
            return _locations.TryGetValue(node.NodeId, out var location) ? location : null;
        }

        /// <summary>
        /// Converts a method with a body in this compilation: a method, constructor or operator declaration, a property's
        /// accessor, or the expression body of a get-only property.
        /// </summary>
        public KernelFunctionNode ConvertFunction(IMethodSymbol method, SyntaxNode declaration)
        {
            var previous = _model;
            _model = ModelFor(declaration.SyntaxTree);
            try
            {
                var parameterList = (declaration as BaseMethodDeclarationSyntax)?.ParameterList;
                var parameters = method.Parameters
                    .Select(p => ConvertParameter(p, parameterList?.Parameters[p.Ordinal] ?? declaration))
                    .ToImmutableArray();
                var body = new MethodBodyNode(NewId(), ConvertBody(method, declaration));
                var node = new KernelFunctionNode(NewId(), method, parameters, body);
                return declaration switch
                {
                    MethodDeclarationSyntax m => At(node, m.Identifier),
                    ConstructorDeclarationSyntax c => At(node, c.Identifier),
                    OperatorDeclarationSyntax o => At(node, o.OperatorToken),
                    _ => At(node, declaration)
                };
            }
            finally
            {
                _model = previous;
            }
        }

        /// <summary>Whether a declaration is one <see cref="ConvertFunction"/> can convert.</summary>
        public static bool IsFunctionDeclaration(SyntaxNode syntax)
        {
            return syntax is MethodDeclarationSyntax or ConstructorDeclarationSyntax or OperatorDeclarationSyntax
                or AccessorDeclarationSyntax or ArrowExpressionClauseSyntax;
        }

        /// <summary>Whether <paramref name="method"/> is <c>KernelMath.ImplicitSolve</c>.</summary>
        public static bool IsImplicitSolve(IMethodSymbol method)
        {
            return method.Name == "ImplicitSolve" && method.ContainingType?.ToDisplayString() == KernelFunctionLibrary.KernelMathTypeName;
        }

        /// <summary>Whether <paramref name="type"/> is <c>Span&lt;T&gt;</c> or <c>ReadOnlySpan&lt;T&gt;</c>, and its element type.</summary>
        public static bool IsSpan(ITypeSymbol type, out ITypeSymbol elementType, out bool isWritable)
        {
            var named = type as INamedTypeSymbol;
            var name = named?.OriginalDefinition.ToDisplayString();
            isWritable = name == "System.Span<T>";
            elementType = named != null && named.TypeArguments.Length == 1 ? named.TypeArguments[0] : type;
            return isWritable || name == "System.ReadOnlySpan<T>";
        }

        /// <summary>Whether <paramref name="type"/> is a scalar a kernel can hold: <c>double</c>, <c>int</c> or <c>bool</c>.</summary>
        public static bool IsScalar(ITypeSymbol type)
        {
            return type.SpecialType is SpecialType.System_Double or SpecialType.System_Int32 or SpecialType.System_Boolean;
        }

        // A struct the kernel flattens, or an error saying why it cannot.
        private static INamedTypeSymbol FlattenableStruct(ITypeSymbol type, SyntaxNode at)
        {
            var problem = KernelStructs.Problem(type);
            return problem == null
                ? (INamedTypeSymbol)type
                : throw Unsupported(at, $"{problem}, so a kernel cannot hold it as a value");
        }

        private static bool IsArrayElement(ITypeSymbol type)
        {
            return type.SpecialType is SpecialType.System_Double or SpecialType.System_Int32;
        }

        private IRNode ConvertParameter(IParameterSymbol parameter, SyntaxNode syntax)
        {
            var isStruct = KernelStructs.IsStruct(parameter.Type);
            if (parameter.RefKind != RefKind.None && !(isStruct && IsReadOnlyReference(parameter.RefKind)))
            {
                throw Unsupported(syntax, $"parameter '{parameter.Name}' is passed by {parameter.RefKind.ToString().ToLowerInvariant()}; kernels and helpers take doubles, ints, bools and spans by value, and structs by value or by in");
            }

            if (IsScalar(parameter.Type))
            {
                return At(new VariableNode(NewId(), parameter.Name, parameter.Type), syntax);
            }

            if (isStruct)
            {
                return At(new VariableNode(NewId(), parameter.Name, FlattenableStruct(parameter.Type, syntax)), syntax);
            }

            if (IsSpan(parameter.Type, out var element, out var writable) && IsArrayElement(element))
            {
                var fixedLength = parameter.GetAttributes()
                    .Where(a => a.AttributeClass?.ToDisplayString() == "Optimal.FixedLengthAttribute")
                    .Select(a => (int?)a.ConstructorArguments[0].Value)
                    .FirstOrDefault();
                return At(new ArrayParameterNode(NewId(), parameter.Name, element, fixedLength, writable), syntax);
            }

            throw Unsupported(syntax, $"parameter '{parameter.Name}' has type {parameter.Type.ToDisplayString()}; kernels and helpers take double, int, bool, Span<double>, ReadOnlySpan<double>, Span<int>, ReadOnlySpan<int> and structs of doubles");
        }

        // in, or ref readonly (RefKind.RefReadOnlyParameter, which netstandard's Roslyn reference names only by value).
        private static bool IsReadOnlyReference(RefKind kind)
        {
            return kind == RefKind.In || (int)kind == 4;
        }

        private ImmutableArray<IRNode> ConvertBody(IMethodSymbol method, SyntaxNode declaration)
        {
            if (declaration is ConstructorDeclarationSyntax { Initializer: not null } constructor)
            {
                throw Unsupported(constructor.Initializer, $"the constructor initialiser '{constructor.Initializer}' is not supported; assign the fields in the constructor's body");
            }

            var (body, expressionBody) = declaration switch
            {
                BaseMethodDeclarationSyntax m => (m.Body, m.ExpressionBody?.Expression),
                AccessorDeclarationSyntax a => (a.Body, a.ExpressionBody?.Expression),
                ArrowExpressionClauseSyntax arrow => ((BlockSyntax?)null, arrow.Expression),
                _ => (null, null)
            };
            if (body != null)
            {
                return ConvertStatements(body.Statements);
            }

            if (expressionBody != null)
            {
                return method.ReturnsVoid
                    ? ImmutableArray.Create(ConvertExpressionStatement(expressionBody))
                    : ImmutableArray.Create<IRNode>(At(new ReturnNode(NewId(), ConvertExpression(expressionBody)), expressionBody));
            }

            throw new KernelException(KernelDiagnostics.UnsupportedCall, declaration.GetLocation(), $"'{method.Name}' has no body to inline");
        }

        private ImmutableArray<IRNode> ConvertStatements(IEnumerable<StatementSyntax> statements)
        {
            return statements.SelectMany(ConvertStatement).ToImmutableArray();
        }

        private IEnumerable<IRNode> ConvertStatement(StatementSyntax statement)
        {
            return statement switch
            {
                LocalDeclarationStatementSyntax declaration => ConvertLocalDeclaration(declaration),
                ExpressionStatementSyntax expression => new[] { ConvertExpressionStatement(expression.Expression) },
                ForStatementSyntax loop => new[] { ConvertFor(loop) },
                WhileStatementSyntax loop => new[] { ConvertWhile(loop) },
                IfStatementSyntax branch => new[] { ConvertIf(branch) },
                BlockSyntax block => ConvertStatements(block.Statements),
                ReturnStatementSyntax { Expression: null } => new[] { At(new VoidReturnNode(NewId()), statement) },
                ReturnStatementSyntax ret => new[] { At(new ReturnNode(NewId(), ConvertExpression(ret.Expression!)), statement) },
                BreakStatementSyntax => new[] { At(new BreakNode(NewId()), statement) },
                ContinueStatementSyntax => new[] { At(new ContinueNode(NewId()), statement) },
                EmptyStatementSyntax => Array.Empty<IRNode>(),
                LocalFunctionStatementSyntax => throw Unsupported(statement, "local functions are not supported; declare the helper as a static method of an [OptimalCode] class"),
                _ => throw Unsupported(statement, $"the statement '{Describe(statement)}' is not supported; kernels use declarations, assignments, for and while loops with compile-time trip counts, if, calls, break, continue and return")
            };
        }

        private IEnumerable<IRNode> ConvertLocalDeclaration(LocalDeclarationStatementSyntax statement)
        {
            if (statement.UsingKeyword != default || statement.AwaitKeyword != default)
            {
                throw Unsupported(statement, "'using' declarations are not supported");
            }

            var type = _model.GetTypeInfo(statement.Declaration.Type).Type
                ?? throw Unsupported(statement, $"the type of '{statement.Declaration}' could not be resolved");
            return statement.Declaration.Variables
                .Select(v => At(new LocalDeclarationNode(NewId(), v.Identifier.Text, type, v.Initializer == null ? null : ConvertExpression(v.Initializer.Value)), v))
                .ToList();
        }

        private IRNode ConvertExpressionStatement(ExpressionSyntax expression)
        {
            return expression switch
            {
                InvocationExpressionSyntax invocation => ConvertInvocation(invocation, isStatement: true),
                AssignmentExpressionSyntax assignment => ConvertAssignment(assignment),
                PostfixUnaryExpressionSyntax postfix when postfix.IsKind(SyntaxKind.PostIncrementExpression) || postfix.IsKind(SyntaxKind.PostDecrementExpression)
                    => ConvertIncrement(postfix, postfix.Operand, postfix.IsKind(SyntaxKind.PostIncrementExpression)),
                PrefixUnaryExpressionSyntax prefix when prefix.IsKind(SyntaxKind.PreIncrementExpression) || prefix.IsKind(SyntaxKind.PreDecrementExpression)
                    => ConvertIncrement(prefix, prefix.Operand, prefix.IsKind(SyntaxKind.PreIncrementExpression)),
                _ => throw Unsupported(expression, $"the expression statement '{Describe(expression)}' is not supported")
            };
        }

        private IRNode ConvertIncrement(ExpressionSyntax syntax, ExpressionSyntax operand, bool isIncrement)
        {
            var type = TypeOf(operand);
            var one = At(new ConstantNode(NewId(), 1, _int), syntax);
            var op = isIncrement ? BinaryOperator.Add : BinaryOperator.Subtract;
            var value = At(new BinaryOpNode(NewId(), op, ConvertExpression(operand), Widen(one, type, syntax), type), syntax);
            return MakeAssignment(operand, value, syntax);
        }

        private IRNode ConvertAssignment(AssignmentExpressionSyntax assignment)
        {
            var right = ConvertExpression(assignment.Right);
            if (assignment.IsKind(SyntaxKind.SimpleAssignmentExpression))
            {
                return MakeAssignment(assignment.Left, right, assignment);
            }

            var op = assignment.Kind() switch
            {
                SyntaxKind.AddAssignmentExpression => BinaryOperator.Add,
                SyntaxKind.SubtractAssignmentExpression => BinaryOperator.Subtract,
                SyntaxKind.MultiplyAssignmentExpression => BinaryOperator.Multiply,
                SyntaxKind.DivideAssignmentExpression => BinaryOperator.Divide,
                SyntaxKind.ModuloAssignmentExpression => BinaryOperator.Modulo,
                _ => throw Unsupported(assignment, $"the assignment '{Describe(assignment)}' is not supported; kernels use =, +=, -=, *=, /= and %=")
            };
            var type = TypeOf(assignment.Left);
            var userOperator = (_model.GetOperation(assignment) as ICompoundAssignmentOperation)?.OperatorMethod;
            var value = userOperator != null
                ? (IRNode)At(new MethodCallNode(NewId(), userOperator.Name, userOperator, ImmutableArray.Create(ConvertExpression(assignment.Left), right), userOperator.ReturnType), assignment)
                : At(new BinaryOpNode(NewId(), op, ConvertExpression(assignment.Left), Widen(right, type, assignment.Right), type), assignment);
            return MakeAssignment(assignment.Left, value, assignment);
        }

        // The right side of a compound assignment is converted on its own, so an int widened to the target's double
        // type is widened here.
        private IRNode Widen(IRNode value, ITypeSymbol targetType, SyntaxNode syntax)
        {
            return targetType.SpecialType == SpecialType.System_Double
                ? At(new UnaryOpNode(NewId(), UnaryOperator.Identity, value, targetType), syntax)
                : value;
        }

        private IRNode MakeAssignment(ExpressionSyntax target, IRNode value, SyntaxNode syntax)
        {
            var symbol = _model.GetSymbolInfo(target).Symbol;
            switch (target)
            {
                case IdentifierNameSyntax identifier when symbol is ILocalSymbol or IParameterSymbol:
                    return At(new AssignmentNode(NewId(), identifier.Identifier.Text, value), syntax);
                case IdentifierNameSyntax when symbol is IFieldSymbol { IsStatic: false } field && KernelStructs.IsStruct(field.ContainingType):
                    return At(new FieldAssignmentNode(NewId(), This(field.ContainingType, target), field, value), syntax);
                case MemberAccessExpressionSyntax member when symbol is IFieldSymbol { IsStatic: false } field && KernelStructs.IsStruct(field.ContainingType):
                    return At(new FieldAssignmentNode(NewId(), ConvertExpression(member.Expression), field, value), syntax);
                case ElementAccessExpressionSyntax element:
                    return At(new ElementAssignmentNode(NewId(), ConvertExpression(element.Expression), ConvertIndices(element), value), syntax);
                default:
                    throw Unsupported(target, $"cannot assign to '{Describe(target)}'; a kernel assigns only its locals, parameters and span elements");
            }
        }

        private IRNode ConvertFor(ForStatementSyntax loop)
        {
            var initializers = new List<IRNode>();
            if (loop.Declaration != null)
            {
                var type = _model.GetTypeInfo(loop.Declaration.Type).Type
                    ?? throw Unsupported(loop, $"the type of '{loop.Declaration}' could not be resolved");
                initializers.AddRange(loop.Declaration.Variables.Select(v =>
                    At(new LocalDeclarationNode(NewId(), v.Identifier.Text, type, v.Initializer == null ? null : ConvertExpression(v.Initializer.Value)), v)));
            }

            initializers.AddRange(loop.Initializers.Select(ConvertExpressionStatement));
            var condition = loop.Condition == null ? null : ConvertExpression(loop.Condition);
            var increments = loop.Incrementors.Select(ConvertExpressionStatement).ToImmutableArray();
            var iterators = string.Join(",", initializers.OfType<LocalDeclarationNode>().Select(d => d.Name));
            return At(
                new LoopNode(
                    NewId(),
                    iterators,
                    new MethodBodyNode(NewId(), initializers.ToImmutableArray()),
                    condition,
                    new MethodBodyNode(NewId(), increments),
                    ConvertStatementBlock(loop.Statement),
                    _void),
                loop.ForKeyword);
        }

        private IRNode ConvertWhile(WhileStatementSyntax loop)
        {
            return At(
                new LoopNode(NewId(), string.Empty, null, ConvertExpression(loop.Condition), null, ConvertStatementBlock(loop.Statement), _void),
                loop.WhileKeyword);
        }

        private IRNode ConvertIf(IfStatementSyntax branch)
        {
            var whenFalse = branch.Else == null ? ImmutableArray<IRNode>.Empty : ConvertStatementBlock(branch.Else.Statement);
            return At(new ConditionalNode(NewId(), ConvertExpression(branch.Condition), ConvertStatementBlock(branch.Statement), whenFalse, _void), branch.Condition);
        }

        private ImmutableArray<IRNode> ConvertStatementBlock(StatementSyntax statement)
        {
            return ConvertStatement(statement).ToImmutableArray();
        }

        private IRNode ConvertExpression(ExpressionSyntax expression)
        {
            var info = _model.GetTypeInfo(expression);
            var converted = ConvertExpressionCore(expression);
            var widened = info.Type?.SpecialType == SpecialType.System_Int32 && info.ConvertedType?.SpecialType == SpecialType.System_Double;
            return widened ? At(new UnaryOpNode(NewId(), UnaryOperator.Identity, converted, info.ConvertedType!), expression) : converted;
        }

        private IRNode ConvertExpressionCore(ExpressionSyntax expression)
        {
            var constant = _model.GetConstantValue(expression);
            if (constant.HasValue)
            {
                return constant.Value is int or double or bool
                    ? At(new ConstantNode(NewId(), constant.Value, TypeOf(expression)), expression)
                    : throw Unsupported(expression, $"the constant '{Describe(expression)}' has type {TypeOf(expression).ToDisplayString()}; kernels compute with double, int and bool");
            }

            var userOperator = expression is BinaryExpressionSyntax or PrefixUnaryExpressionSyntax
                ? _model.GetOperation(expression) switch
                {
                    IBinaryOperation binaryOperation => binaryOperation.OperatorMethod,
                    IUnaryOperation unaryOperation => unaryOperation.OperatorMethod,
                    _ => null
                }
                : null;
            if (userOperator != null)
            {
                return ConvertUserOperator(expression, userOperator);
            }

            return expression switch
            {
                ParenthesizedExpressionSyntax parenthesized => ConvertExpression(parenthesized.Expression),
                IdentifierNameSyntax identifier => ConvertName(identifier),
                ThisExpressionSyntax => This(TypeOf(expression), expression),
                MemberAccessExpressionSyntax member => ConvertMemberAccess(member),
                ElementAccessExpressionSyntax element => ConvertElementAccess(element),
                BaseObjectCreationExpressionSyntax creation => ConvertObjectCreation(creation),
                WithExpressionSyntax with => ConvertWith(with),
                DefaultExpressionSyntax or LiteralExpressionSyntax when KernelStructs.IsStruct(ConvertedTypeOf(expression)) => ConvertDefault(expression),
                BinaryExpressionSyntax binary => At(new BinaryOpNode(NewId(), ConvertBinaryOperator(binary), ConvertExpression(binary.Left), ConvertExpression(binary.Right), TypeOf(binary)), binary),
                PrefixUnaryExpressionSyntax prefix => At(new UnaryOpNode(NewId(), ConvertUnaryOperator(prefix), ConvertExpression(prefix.Operand), TypeOf(prefix)), prefix),
                CastExpressionSyntax cast => At(new UnaryOpNode(NewId(), UnaryOperator.Identity, ConvertExpression(cast.Expression), TypeOf(cast)), cast),
                ConditionalExpressionSyntax conditional => At(
                    new ConditionalExpressionNode(NewId(), ConvertExpression(conditional.Condition), ConvertExpression(conditional.WhenTrue), ConvertExpression(conditional.WhenFalse), TypeOf(conditional)),
                    conditional),
                InvocationExpressionSyntax invocation => ConvertInvocation(invocation, isStatement: false),
                StackAllocArrayCreationExpressionSyntax or ImplicitStackAllocArrayCreationExpressionSyntax or ArrayCreationExpressionSyntax
                    or ImplicitArrayCreationExpressionSyntax or CollectionExpressionSyntax
                    => ConvertArrayCreation(expression, "array", _model.GetTypeInfo(expression).ConvertedType ?? TypeOf(expression), isReadOnly: false),
                _ => throw Unsupported(expression, $"the expression '{Describe(expression)}' ({expression.Kind()}) is not supported in a kernel")
            };
        }

        private IRNode ConvertName(IdentifierNameSyntax identifier)
        {
            var symbol = _model.GetSymbolInfo(identifier).Symbol;
            return symbol switch
            {
                ILocalSymbol local => At(new VariableNode(NewId(), local.Name, local.Type), identifier),
                IParameterSymbol parameter => At(new VariableNode(NewId(), parameter.Name, parameter.Type), identifier),
                IFieldSymbol { IsStatic: false } field when KernelStructs.IsStruct(field.ContainingType)
                    => At(new FieldAccessNode(NewId(), This(field.ContainingType, identifier), field), identifier),
                IPropertySymbol { IsStatic: false } property when KernelStructs.IsStruct(property.ContainingType)
                    => Getter(property, This(property.ContainingType, identifier), identifier),
                IFieldSymbol or IPropertySymbol => ConvertStaticMember(symbol, identifier),
                _ => throw Unsupported(identifier, $"'{identifier}' is not a local, a parameter or a static table")
            };
        }

        private IRNode ConvertMemberAccess(MemberAccessExpressionSyntax member)
        {
            var symbol = _model.GetSymbolInfo(member).Symbol;
            if (symbol is IPropertySymbol { Name: "Length", IsStatic: false } length && IsSpan(length.ContainingType, out _, out _))
            {
                return At(new ArrayLengthNode(NewId(), ConvertExpression(member.Expression)), member);
            }

            switch (symbol)
            {
                case IFieldSymbol { IsStatic: false } field when KernelStructs.IsStruct(field.ContainingType):
                    return At(new FieldAccessNode(NewId(), ConvertExpression(member.Expression), field), member);
                case IPropertySymbol { IsStatic: false } property when KernelStructs.IsStruct(property.ContainingType):
                    return Getter(property, ConvertExpression(member.Expression), member);
                default:
                    break;
            }

            return symbol is IFieldSymbol or IPropertySymbol
                ? ConvertStaticMember(symbol, member)
                : throw Unsupported(member, $"the member access '{member}' is not supported; kernels read span lengths, constants and static tables");
        }

        // A static readonly field or a static expression-bodied property: a table (an array) or a scalar computed from
        // its initialiser. Each is converted once, so every reference to a table shares one array.
        private IRNode ConvertStaticMember(ISymbol symbol, ExpressionSyntax reference)
        {
            if (_staticMembers.TryGetValue(symbol, out var converted))
            {
                return converted;
            }

            var (type, initializer) = StaticInitializer(symbol, reference);
            var previous = _model;
            _model = ModelFor(initializer.SyntaxTree);
            try
            {
                converted = type is IArrayTypeSymbol || IsSpan(type, out _, out _)
                    ? ConvertArrayCreation(initializer, symbol.Name, type, isReadOnly: true)
                    : ConvertExpression(initializer);
            }
            finally
            {
                _model = previous;
            }

            _staticMembers[symbol] = converted;
            return converted;
        }

        private (ITypeSymbol Type, ExpressionSyntax Initializer) StaticInitializer(ISymbol symbol, ExpressionSyntax reference)
        {
            var declaration = symbol.DeclaringSyntaxReferences.FirstOrDefault()?.GetSyntax();
            switch (symbol)
            {
                case IFieldSymbol { IsStatic: true, IsReadOnly: true } field when declaration is VariableDeclaratorSyntax { Initializer: not null } declarator:
                    return (field.Type, declarator.Initializer.Value);
                case IPropertySymbol { IsStatic: true } property when declaration is PropertyDeclarationSyntax { ExpressionBody: not null } body:
                    return (property.Type, body.ExpressionBody.Expression);
                default:
                    throw Unsupported(reference, $"'{symbol.Name}' is not a static readonly field or a static expression-bodied property with source in this compilation; a kernel reads only its locals, parameters, constants and static tables");
            }
        }

        private IRNode ConvertElementAccess(ElementAccessExpressionSyntax element)
        {
            var arguments = element.ArgumentList.Arguments;
            if (arguments.Count == 1 && arguments[0].Expression is RangeExpressionSyntax range)
            {
                return ConvertRange(element, range);
            }

            return At(new ElementAccessNode(NewId(), ConvertExpression(element.Expression), ConvertIndices(element), TypeOf(element)), element);
        }

        private ImmutableArray<IRNode> ConvertIndices(ElementAccessExpressionSyntax element)
        {
            return element.ArgumentList.Arguments
                .Select(a => a.Expression is PrefixUnaryExpressionSyntax hat && hat.IsKind(SyntaxKind.IndexExpression)
                    ? throw Unsupported(a, $"the from-end index '{a}' is not supported; index from the start")
                    : ConvertExpression(a.Expression))
                .ToImmutableArray();
        }

        private IRNode ConvertRange(ElementAccessExpressionSyntax element, RangeExpressionSyntax range)
        {
            if (range.LeftOperand is PrefixUnaryExpressionSyntax || range.RightOperand is PrefixUnaryExpressionSyntax)
            {
                throw Unsupported(range, $"the range '{range}' indexes from the end; index from the start");
            }

            var start = range.LeftOperand == null ? At(new ConstantNode(NewId(), 0, _int), range) : ConvertExpression(range.LeftOperand);
            var length = range.RightOperand == null
                ? null
                : At(new BinaryOpNode(NewId(), BinaryOperator.Subtract, ConvertExpression(range.RightOperand), start, _int), range);
            return At(new ArraySliceNode(NewId(), ConvertExpression(element.Expression), start, length), element);
        }

        private IRNode ConvertInvocation(InvocationExpressionSyntax invocation, bool isStatement)
        {
            var method = _model.GetSymbolInfo(invocation).Symbol as IMethodSymbol
                ?? throw new KernelException(KernelDiagnostics.UnsupportedCall, invocation.GetLocation(), $"the call '{Describe(invocation)}' could not be resolved");
            if (IsImplicitSolve(method))
            {
                return ConvertImplicitSolve(invocation, isStatement);
            }

            if (!method.IsStatic && IsSpan(method.ContainingType, out _, out _) && invocation.Expression is MemberAccessExpressionSyntax receiver)
            {
                return ConvertSpanMethod(invocation, method, ConvertExpression(receiver.Expression), isStatement);
            }

            if (!method.IsStatic && KernelStructs.IsStruct(method.ContainingType))
            {
                var target = invocation.Expression is MemberAccessExpressionSyntax access
                    ? ConvertExpression(access.Expression)
                    : This(method.ContainingType, invocation);
                return At(new InstanceCallNode(NewId(), target, method, ConvertArguments(invocation), method.ReturnType), invocation);
            }

            if (!method.IsStatic)
            {
                throw new KernelException(KernelDiagnostics.UnsupportedCall, invocation.GetLocation(), $"the instance method '{method.ToDisplayString()}' is not supported; a kernel calls System.Math functions, span methods and static helpers");
            }

            return At(new MethodCallNode(NewId(), method.Name, method, ConvertArguments(invocation), method.ReturnType), invocation);
        }

        private IRNode ConvertSpanMethod(InvocationExpressionSyntax invocation, IMethodSymbol method, IRNode span, bool isStatement)
        {
            var arguments = ConvertArguments(invocation);
            return method.Name switch
            {
                "Slice" => At(new ArraySliceNode(NewId(), span, arguments[0], arguments.Length > 1 ? arguments[1] : null), invocation),
                "Clear" when isStatement => At(new ArrayOperationNode(NewId(), ArrayOperation.Clear, span, arguments), invocation),
                "Fill" when isStatement => At(new ArrayOperationNode(NewId(), ArrayOperation.Fill, span, arguments), invocation),
                "CopyTo" when isStatement => At(new ArrayOperationNode(NewId(), ArrayOperation.CopyTo, span, arguments), invocation),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedCall, invocation.GetLocation(), $"the span method '{method.Name}' is not supported here; kernels use Slice, Length and indexing, and Clear, Fill and CopyTo as statements")
            };
        }

        // Arguments in parameter order, with omitted optional arguments filled from their defaults.
        private ImmutableArray<IRNode> ConvertArguments(InvocationExpressionSyntax invocation)
        {
            var operation = _model.GetOperation(invocation) as IInvocationOperation
                ?? throw new KernelException(KernelDiagnostics.UnsupportedCall, invocation.GetLocation(), $"the call '{Describe(invocation)}' could not be resolved");
            return ConvertArguments(operation.Arguments, invocation);
        }

        private ImmutableArray<IRNode> ConvertArguments(ImmutableArray<IArgumentOperation> arguments, SyntaxNode invocation)
        {
            return arguments
                .OrderBy(a => a.Parameter!.Ordinal)
                .Select(a => ConvertArgument(a, invocation))
                .ToImmutableArray();
        }

        private IRNode ConvertArgument(IArgumentOperation argument, SyntaxNode invocation)
        {
            var parameter = argument.Parameter!;
            var byReadOnlyReference = IsReadOnlyReference(parameter.RefKind) && KernelStructs.IsStruct(parameter.Type);
            if (argument.ArgumentKind == ArgumentKind.ParamArray || (parameter.RefKind != RefKind.None && !byReadOnlyReference))
            {
                throw new KernelException(KernelDiagnostics.UnsupportedCall, invocation.GetLocation(), $"the call '{Describe(invocation)}' passes '{parameter.Name}' by reference or as a params array; kernels pass values and spans");
            }

            if (argument.ArgumentKind == ArgumentKind.DefaultValue)
            {
                var value = argument.Value.ConstantValue;
                return value.HasValue && value.Value is int or double or bool
                    ? At(new ConstantNode(NewId(), value.Value, parameter.Type), invocation)
                    : throw new KernelException(KernelDiagnostics.UnsupportedCall, invocation.GetLocation(), $"the default value of '{parameter.Name}' is not a double, int or bool constant");
            }

            return ConvertExpression(((ArgumentSyntax)argument.Syntax).Expression);
        }

        // KernelMath.ImplicitSolve(residual, p, y, iterations): the residual must name a method, not a lambda or a
        // delegate value, so the generator can inline it.
        private IRNode ConvertImplicitSolve(InvocationExpressionSyntax invocation, bool isStatement)
        {
            if (!isStatement)
            {
                throw new KernelException(KernelDiagnostics.InvalidImplicitSolve, invocation.GetLocation(), "KernelMath.ImplicitSolve writes its solution to y and returns nothing; call it as a statement");
            }

            var operation = (IInvocationOperation)_model.GetOperation(invocation)!;
            var arguments = operation.Arguments.OrderBy(a => a.Parameter!.Ordinal).ToList();
            var residual = Unwrap(arguments[0].Value) is IDelegateCreationOperation { Target: IMethodReferenceOperation reference }
                ? reference.Method
                : throw new KernelException(KernelDiagnostics.InvalidImplicitSolve, arguments[0].Syntax.GetLocation(), "the residual must name a static method, as ImplicitSolve(Residual, …), not a lambda or a delegate variable");
            return At(
                new ImplicitSolveNode(
                    NewId(),
                    residual,
                    ConvertArgument(arguments[1], invocation),
                    ConvertArgument(arguments[2], invocation),
                    ConvertArgument(arguments[3], invocation)),
                invocation);
        }

        private static IOperation Unwrap(IOperation operation)
        {
            return operation is IConversionOperation conversion ? Unwrap(conversion.Operand) : operation;
        }

        // A user-defined operator of a struct, inlined like a static helper.
        private IRNode ConvertUserOperator(ExpressionSyntax expression, IMethodSymbol method)
        {
            var operands = expression switch
            {
                BinaryExpressionSyntax binary => ImmutableArray.Create(ConvertExpression(binary.Left), ConvertExpression(binary.Right)),
                PrefixUnaryExpressionSyntax prefix => ImmutableArray.Create(ConvertExpression(prefix.Operand)),
                _ => throw Unsupported(expression, $"the operator in '{Describe(expression)}' is not supported")
            };
            return At(new MethodCallNode(NewId(), method.Name, method, operands, method.ReturnType), expression);
        }

        // `this` inside a struct's constructor, method or property.
        private IRNode This(ITypeSymbol type, SyntaxNode at)
        {
            return At(new VariableNode(NewId(), KernelFrame.This, FlattenableStruct(type, at)), at);
        }

        private IRNode Getter(IPropertySymbol property, IRNode receiver, SyntaxNode at)
        {
            var getter = property.GetMethod
                ?? throw Unsupported(at, $"the property '{property.Name}' has no getter");
            return At(new InstanceCallNode(NewId(), receiver, getter, ImmutableArray<IRNode>.Empty, property.Type), at);
        }

        // new T(…), new(…) or new T { … } of a struct. A struct's implicit parameterless constructor zeroes it.
        private IRNode ConvertObjectCreation(BaseObjectCreationExpressionSyntax creation)
        {
            var type = FlattenableStruct(ConvertedTypeOf(creation), creation);
            var operation = _model.GetOperation(creation) as IObjectCreationOperation
                ?? throw Unsupported(creation, $"the creation '{Describe(creation)}' could not be resolved");
            var constructor = operation.Constructor is { IsImplicitlyDeclared: false } declared ? declared : null;
            var arguments = constructor == null ? ImmutableArray<IRNode>.Empty : ConvertArguments(operation.Arguments, creation);
            return At(new StructCreationNode(NewId(), type, null, constructor, arguments, ConvertInitializers(creation.Initializer)), creation);
        }

        private IRNode ConvertWith(WithExpressionSyntax with)
        {
            var type = FlattenableStruct(TypeOf(with), with);
            return At(new StructCreationNode(NewId(), type, ConvertExpression(with.Expression), null, ImmutableArray<IRNode>.Empty, ConvertInitializers(with.Initializer)), with);
        }

        private IRNode ConvertDefault(ExpressionSyntax expression)
        {
            var type = FlattenableStruct(ConvertedTypeOf(expression), expression);
            return At(new StructCreationNode(NewId(), type, null, null, ImmutableArray<IRNode>.Empty, ImmutableArray<FieldInitializer>.Empty), expression);
        }

        private ImmutableArray<FieldInitializer> ConvertInitializers(InitializerExpressionSyntax? initializer)
        {
            return initializer == null
                ? ImmutableArray<FieldInitializer>.Empty
                : initializer.Expressions
                    .Select(e => e is AssignmentExpressionSyntax { Left: IdentifierNameSyntax name } assignment && _model.GetSymbolInfo(name).Symbol is IFieldSymbol { IsStatic: false } field
                        ? new FieldInitializer(field, ConvertExpression(assignment.Right))
                        : throw Unsupported(e, $"the initialiser '{Describe(e)}' is not supported; a struct initialiser assigns fields, as Field = value"))
                    .ToImmutableArray();
        }

        private ITypeSymbol ConvertedTypeOf(ExpressionSyntax expression)
        {
            var info = _model.GetTypeInfo(expression);
            return info.ConvertedType ?? info.Type
                ?? throw Unsupported(expression, $"the type of '{Describe(expression)}' could not be resolved");
        }

        private IRNode ConvertArrayCreation(ExpressionSyntax expression, string name, ITypeSymbol arrayType, bool isReadOnly)
        {
            var (elementType, rank) = arrayType switch
            {
                IArrayTypeSymbol array => (array.ElementType, array.Rank),
                _ when IsSpan(arrayType, out var spanElement, out _) => (spanElement, 1),
                _ => throw Unsupported(expression, $"'{Describe(expression)}' is not an array; a kernel table is an array or a span")
            };
            if (!IsArrayElement(elementType) || rank > 2)
            {
                throw Unsupported(expression, $"'{Describe(expression)}' is a {rank}-dimensional array of {elementType.ToDisplayString()}; kernel arrays hold double or int in one or two dimensions");
            }

            var (sizes, initializer) = ArrayParts(expression);
            var elements = Flatten(initializer, rank).Select(ConvertExpression).ToImmutableArray();
            var dimensions = sizes.Any(s => s is not OmittedArraySizeExpressionSyntax)
                ? sizes.Select(ConvertExpression).ToImmutableArray()
                : InitializerShape(initializer, rank, expression);
            return At(new ArrayCreationNode(NewId(), name, elementType, dimensions, elements, isReadOnly), expression);
        }

        private static (IReadOnlyList<ExpressionSyntax> Sizes, SyntaxNode? Initializer) ArrayParts(ExpressionSyntax expression)
        {
            return expression switch
            {
                StackAllocArrayCreationExpressionSyntax stackAlloc => (RankSizes(stackAlloc.Type), stackAlloc.Initializer),
                ImplicitStackAllocArrayCreationExpressionSyntax implicitStackAlloc => (Array.Empty<ExpressionSyntax>(), implicitStackAlloc.Initializer),
                ArrayCreationExpressionSyntax creation => (RankSizes(creation.Type), creation.Initializer),
                ImplicitArrayCreationExpressionSyntax implicitCreation => (Array.Empty<ExpressionSyntax>(), implicitCreation.Initializer),
                CollectionExpressionSyntax collection => (Array.Empty<ExpressionSyntax>(), collection),
                InitializerExpressionSyntax initializer => (Array.Empty<ExpressionSyntax>(), initializer),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, expression.GetLocation(), $"'{Describe(expression)}' is not an array creation; a static table's initialiser must create an array")
            };
        }

        private static IReadOnlyList<ExpressionSyntax> RankSizes(TypeSyntax type)
        {
            return type is ArrayTypeSyntax array ? array.RankSpecifiers[0].Sizes : Array.Empty<ExpressionSyntax>();
        }

        private static IEnumerable<ExpressionSyntax> Flatten(SyntaxNode? initializer, int rank)
        {
            return initializer switch
            {
                null => Enumerable.Empty<ExpressionSyntax>(),
                CollectionExpressionSyntax collection => collection.Elements.Select(e => e is ExpressionElementSyntax element
                    ? element.Expression
                    : throw new KernelException(KernelDiagnostics.UnsupportedSyntax, e.GetLocation(), $"the collection element '{e}' is not supported; list each element")),
                InitializerExpressionSyntax list when rank == 2 => list.Expressions.SelectMany(row => Flatten(row, 1)),
                InitializerExpressionSyntax list => list.Expressions,
                ExpressionSyntax row => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, row.GetLocation(), $"'{row}' is not a row of a rectangular table"),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, initializer.GetLocation(), $"'{initializer}' is not an array initialiser")
            };
        }

        private ImmutableArray<IRNode> InitializerShape(SyntaxNode? initializer, int rank, SyntaxNode syntax)
        {
            var rows = initializer switch
            {
                CollectionExpressionSyntax collection => collection.Elements.Count,
                InitializerExpressionSyntax list => list.Expressions.Count,
                _ => throw Unsupported(syntax, $"'{Describe(syntax)}' gives neither a length nor elements")
            };
            if (rank == 1)
            {
                return ImmutableArray.Create<IRNode>(At(new ConstantNode(NewId(), rows, _int), syntax));
            }

            var columns = rows == 0 ? 0 : ((InitializerExpressionSyntax)((InitializerExpressionSyntax)initializer!).Expressions[0]).Expressions.Count;
            return ImmutableArray.Create<IRNode>(At(new ConstantNode(NewId(), rows, _int), syntax), At(new ConstantNode(NewId(), columns, _int), syntax));
        }

        private BinaryOperator ConvertBinaryOperator(BinaryExpressionSyntax binary)
        {
            return binary.Kind() switch
            {
                SyntaxKind.AddExpression => BinaryOperator.Add,
                SyntaxKind.SubtractExpression => BinaryOperator.Subtract,
                SyntaxKind.MultiplyExpression => BinaryOperator.Multiply,
                SyntaxKind.DivideExpression => BinaryOperator.Divide,
                SyntaxKind.ModuloExpression => BinaryOperator.Modulo,
                SyntaxKind.GreaterThanExpression => BinaryOperator.GreaterThan,
                SyntaxKind.LessThanExpression => BinaryOperator.LessThan,
                SyntaxKind.GreaterThanOrEqualExpression => BinaryOperator.GreaterThanOrEqual,
                SyntaxKind.LessThanOrEqualExpression => BinaryOperator.LessThanOrEqual,
                SyntaxKind.EqualsExpression => BinaryOperator.Equal,
                SyntaxKind.NotEqualsExpression => BinaryOperator.NotEqual,
                SyntaxKind.LogicalAndExpression => BinaryOperator.LogicalAnd,
                SyntaxKind.LogicalOrExpression => BinaryOperator.LogicalOr,
                _ => throw Unsupported(binary, $"the operator '{binary.OperatorToken.Text}' is not supported; kernels use + - * / %, comparisons, && and ||")
            };
        }

        private UnaryOperator ConvertUnaryOperator(PrefixUnaryExpressionSyntax prefix)
        {
            return prefix.Kind() switch
            {
                SyntaxKind.UnaryMinusExpression => UnaryOperator.Negate,
                SyntaxKind.UnaryPlusExpression => UnaryOperator.Plus,
                SyntaxKind.LogicalNotExpression => UnaryOperator.LogicalNot,
                _ => throw Unsupported(prefix, $"the operator '{prefix.OperatorToken.Text}' is not supported inside an expression; increment and decrement are statements in a kernel")
            };
        }

        private ITypeSymbol TypeOf(ExpressionSyntax expression)
        {
            return _model.GetTypeInfo(expression).Type
                ?? throw Unsupported(expression, $"the type of '{Describe(expression)}' could not be resolved");
        }

        private SemanticModel ModelFor(SyntaxTree tree)
        {
            if (!_models.TryGetValue(tree, out var model))
            {
                model = _compilation.GetSemanticModel(tree);
                _models[tree] = model;
            }

            return model;
        }

        private T At<T>(T node, SyntaxNode syntax)
            where T : IRNode
        {
            _locations[node.NodeId] = syntax.GetLocation();
            return node;
        }

        private T At<T>(T node, SyntaxToken token)
            where T : IRNode
        {
            _locations[node.NodeId] = token.GetLocation();
            return node;
        }

        private int NewId()
        {
            return _nextNodeId++;
        }

        private static KernelException Unsupported(SyntaxNode syntax, string detail)
        {
            return new KernelException(KernelDiagnostics.UnsupportedSyntax, syntax.GetLocation(), detail);
        }

        // The first line of the syntax, for messages.
        private static string Describe(SyntaxNode syntax)
        {
            var text = syntax.ToString();
            var newline = text.IndexOfAny(LineBreaks);
            return newline < 0 ? text : text.Substring(0, newline) + " …";
        }
    }
}
