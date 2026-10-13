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
using System.Globalization;
using System.Linq;
using System.Threading;
using Microsoft.CodeAnalysis;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Flattens a kernel to SSA by interpreting its statement IR. Integers and booleans are compile-time values, so every
    /// loop is unrolled, every index is a constant, every <c>if</c> on a compile-time condition takes one branch, and
    /// every helper call is inlined. Doubles become nodes of a <see cref="ScalarGraph"/>, and structs of doubles are
    /// flattened to their fields. A loop bound, index or length that depends on a double is a generator error
    /// (<c>OPTAD100</c>), never a silent skip.
    /// </summary>
    /// <remarks>
    /// A comparison of doubles is a run-time condition. A conditional expression on one becomes a select; an <c>if</c>
    /// on one is if-converted: both branches are interpreted from the same state, and every variable, field and array
    /// element either branch assigned becomes a select of the two branches' values. The value is the one C# computes,
    /// and the derivative is the derivative of the branch taken. A branch may not <c>return</c>, <c>break</c> or
    /// <c>continue</c>, and the branches may not leave an int, bool or span different, because those are compile-time.
    /// </remarks>
    public sealed class LoopUnroller
    {
        /// <summary>The most loop iterations one kernel may unroll.</summary>
        public const int MaxIterations = 1_000_000;

        /// <summary>The most graph nodes one kernel may build.</summary>
        public const int MaxNodes = 4_000_000;

        /// <summary>The deepest helper calls may nest.</summary>
        public const int MaxCallDepth = 64;

        private const string BranchExitMessage =
            "a return, break or continue is inside an if whose condition depends on run-time values; both branches of such an if run to their end and are merged, so move the rest of the method into an else branch";

        private const string LoopConditionMessage =
            "a loop condition depends on run-time values; a kernel's loops must have compile-time trip counts";

        private static readonly char[] IteratorSeparator = { ',' };

        private readonly KernelFunctionLibrary _library;
        private readonly CancellationToken _cancellationToken;
        private readonly int _derivativeOrder;
        private readonly ScalarGraph _graph = new();
        private readonly Dictionary<int, ArrayValue> _tables = new();
        private readonly WriteJournal _journal = new();
        private int _iterations;
        private int _depth;
        private int _implicitSolves;

        /// <summary>
        /// An unroller for one kernel. <paramref name="derivativeOrder"/> is the highest derivative any request of the
        /// kernel takes (2 for a Hessian), so an implicit solve can be built exact to that order.
        /// </summary>
        public LoopUnroller(KernelFunctionLibrary library, CancellationToken cancellationToken, int derivativeOrder = 1)
        {
            _library = library;
            _cancellationToken = cancellationToken;
            _derivativeOrder = derivativeOrder is 1 or 2
                ? derivativeOrder
                : throw new ArgumentOutOfRangeException(nameof(derivativeOrder), derivativeOrder, "Kernels are differentiated once or twice.");
        }

        /// <summary>
        /// Interprets <paramref name="kernel"/> with its inputs bound to graph inputs, and returns the node written to each
        /// element of its output. Each instance unrolls one kernel.
        /// </summary>
        public UnrolledKernel Unroll(KernelFunctionNode kernel, ImmutableArray<KernelParameter> parameters)
        {
            if (_graph.Count > 0)
            {
                throw new InvalidOperationException("A LoopUnroller unrolls one kernel.");
            }

            var frame = new KernelFrame(kernel.Method.ReturnType);
            var bound = parameters.Select(Bind).ToList();
            for (var i = 0; i < parameters.Length; i++)
            {
                frame.Declare(parameters[i].Name, kernel.Method.Parameters[i].Type, bound[i]);
            }

            _ = Execute(kernel.Body.Statements, frame);

            var output = parameters.Single(p => p.IsOutput);
            var storage = ((ArrayValue)bound[output.Group]).Storage;
            var unwritten = storage.FirstUnwritten();
            if (unwritten >= 0)
            {
                throw new KernelException(KernelDiagnostics.UnassignedValue, kernel, $"the output '{output.Name}[{unwritten}]' is never written; a kernel must write every element of its output");
            }

            var outputIds = Enumerable.Range(0, storage.Length).Select(i => ((ScalarValue)storage[i]!).Id).ToImmutableArray();
            return new UnrolledKernel(_graph, parameters, output, outputIds, _graph.Count);
        }

        private KernelValue Bind(KernelParameter parameter)
        {
            if (!parameter.IsArray)
            {
                return new ScalarValue(_graph.Input(parameter.Group, 0));
            }

            var elements = parameter.IsOutput
                ? Enumerable.Repeat<KernelValue?>(null, parameter.Length)
                : Enumerable.Range(0, parameter.Length).Select(i => (KernelValue?)new ScalarValue(_graph.Input(parameter.Group, i)));
            return new ArrayValue(new KernelArray(parameter.Name, false, 0, !parameter.IsOutput, elements), 0, parameter.Length);
        }

        private Completion Execute(ImmutableArray<IRNode> statements, KernelFrame frame)
        {
            foreach (var statement in statements)
            {
                var completion = Execute(statement, frame);
                if (completion != Completion.Normal)
                {
                    return completion;
                }
            }

            return Completion.Normal;
        }

        private Completion Execute(IRNode statement, KernelFrame frame)
        {
            switch (statement)
            {
                case LocalDeclarationNode { Value: ArrayCreationNode { IsReadOnly: false } creation } declaration:
                    frame.Declare(declaration.Name, declaration.Type, CreateArray(creation, frame, declaration.Name));
                    return Completion.Normal;
                case LocalDeclarationNode declaration:
                    frame.Declare(declaration.Name, declaration.Type, declaration.Value == null ? null : Convert(declaration.Type, Evaluate(declaration.Value, frame), declaration));
                    return Completion.Normal;
                case AssignmentNode assignment:
                    Assign(assignment, frame);
                    return Completion.Normal;
                case FieldAssignmentNode write:
                    AssignField(write.Target, write.Field, Evaluate(write.Value, frame), frame, write);
                    return Completion.Normal;
                case ElementAssignmentNode write:
                    WriteElement(write, frame);
                    return Completion.Normal;
                case ArrayOperationNode operation:
                    ExecuteArrayOperation(operation, frame);
                    return Completion.Normal;
                case MethodCallNode call:
                    _ = Call(call, frame);
                    return Completion.Normal;
                case InstanceCallNode call:
                    _ = InstanceCall(call, frame);
                    return Completion.Normal;
                case ImplicitSolveNode solve:
                    ExecuteImplicitSolve(solve, frame);
                    return Completion.Normal;
                case LoopNode loop:
                    return ExecuteLoop(loop, frame);
                case ConditionalNode branch:
                    return ExecuteBranch(branch, frame);
                case ReturnNode ret:
                    frame.ReturnValue = Convert(frame.ReturnType, Evaluate(ret.Value, frame), ret);
                    return Completion.Return;
                case VoidReturnNode:
                    return Completion.Return;
                case BreakNode:
                    return Completion.Break;
                case ContinueNode:
                    return Completion.Continue;
                default:
                    throw new KernelException(KernelDiagnostics.UnsupportedSyntax, statement, $"the statement {statement.GetType().Name} is not supported in a kernel");
            }
        }

        private void Assign(AssignmentNode assignment, KernelFrame frame)
        {
            if (!frame.TryGet(assignment.TargetVariable, out var type, out _))
            {
                throw new KernelException(KernelDiagnostics.UnsupportedSyntax, assignment, $"'{assignment.TargetVariable}' is not a local or parameter of this kernel or helper");
            }

            frame.Assign(assignment.TargetVariable, Convert(type, Evaluate(assignment.Value, frame), assignment));
        }

        private void WriteElement(ElementAssignmentNode write, KernelFrame frame)
        {
            var array = EvaluateArray(write.Array, frame, write);
            var index = ElementIndex(array, write.Indices, frame, write);
            if (array.Storage.IsReadOnly)
            {
                throw new KernelException(KernelDiagnostics.ReadOnlyWrite, write, $"'{array.Storage.Name}' is read-only to a kernel; copy a static table into a local array before changing it");
            }

            _journal.Store(array.Storage, index, Element(array.Storage, Evaluate(write.Value, frame), write));
        }

        private void ExecuteArrayOperation(ArrayOperationNode operation, KernelFrame frame)
        {
            var array = EvaluateArray(operation.Array, frame, operation);
            var (target, source) = operation.Operation == ArrayOperation.CopyTo
                ? (EvaluateArray(operation.Arguments[0], frame, operation), array)
                : (array, null);
            if (target.Storage.IsReadOnly)
            {
                throw new KernelException(KernelDiagnostics.ReadOnlyWrite, operation, $"'{target.Storage.Name}' is read-only to a kernel");
            }

            if (source != null && source.Length > target.Length)
            {
                throw new KernelException(KernelDiagnostics.IndexOutOfRange, operation, $"CopyTo copies {source.Length} elements of '{source.Storage.Name}' into {target.Length} of '{target.Storage.Name}'");
            }

            var fill = operation.Operation switch
            {
                ArrayOperation.Clear => Element(target.Storage, new IntValue(0), operation),
                ArrayOperation.Fill => Element(target.Storage, Evaluate(operation.Arguments[0], frame), operation),
                _ => null
            };
            var count = source?.Length ?? target.Length;
            var values = Enumerable.Range(0, count)
                .Select(i => fill ?? Read(source!.Storage, source.Offset + i, operation))
                .ToList();
            for (var i = 0; i < count; i++)
            {
                _journal.Store(target.Storage, target.Offset + i, values[i]);
            }
        }

        private Completion ExecuteLoop(LoopNode loop, KernelFrame frame)
        {
            if (loop.Initializer is MethodBodyNode initializer)
            {
                _ = Execute(initializer.Statements, frame);
            }

            var completion = Completion.Normal;
            while (loop.Condition == null || EvaluateCondition(loop.Condition, frame, LoopConditionMessage))
            {
                CountIteration(loop);
                completion = Execute(loop.Body, frame);
                if (completion is Completion.Break or Completion.Return)
                {
                    break;
                }

                if (loop.Increment is MethodBodyNode increment)
                {
                    _ = Execute(increment.Statements, frame);
                }
            }

            foreach (var name in loop.IteratorVariable.Split(IteratorSeparator, StringSplitOptions.RemoveEmptyEntries))
            {
                frame.Remove(name);
            }

            return completion == Completion.Return ? Completion.Return : Completion.Normal;
        }

        private Completion ExecuteBranch(ConditionalNode branch, KernelFrame frame)
        {
            return Evaluate(branch.Condition, frame) switch
            {
                BoolValue b => Execute(b.Value ? branch.TrueBranch : branch.FalseBranch, frame),
                ConditionValue c => IfConvert(branch, c, frame),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, branch.Condition, "a condition must be a boolean")
            };
        }

        // Both branches from the same state, then every variable and array element either assigned becomes a select.
        private Completion IfConvert(ConditionalNode branch, ConditionValue condition, KernelFrame frame)
        {
            var before = frame.Snapshot();
            var whenTrue = RunBranch(branch.TrueBranch, branch, frame, before);
            var whenFalse = RunBranch(branch.FalseBranch, branch, frame, before);
            var trueValues = whenTrue.Variables.ToDictionary(v => v.Key, v => v.Value.Value);
            var falseValues = whenFalse.Variables.ToDictionary(v => v.Key, v => v.Value.Value);
            foreach (var variable in before)
            {
                var name = variable.Key;
                if (whenTrue.Declared.Contains(name) || whenFalse.Declared.Contains(name)
                    || !trueValues.TryGetValue(name, out var t) || !falseValues.TryGetValue(name, out var f))
                {
                    continue;
                }

                var merged = ValueMerge.Merge(_graph, condition, t, f, $"'{name}'", branch, requireBoth: false);
                if (!ValueMerge.Same(merged, variable.Value.Value))
                {
                    frame.Assign(name, merged);
                }
            }

            var trueWrites = whenTrue.Writes.ToDictionary(w => (w.Storage, w.Index), w => w.Value);
            var falseWrites = whenFalse.Writes.ToDictionary(w => (w.Storage, w.Index), w => w.Value);
            var written = whenTrue.Writes.Select(w => (w.Storage, w.Index))
                .Concat(whenFalse.Writes.Select(w => (w.Storage, w.Index)).Where(key => !trueWrites.ContainsKey(key)))
                .ToList();
            foreach (var (storage, index) in written)
            {
                var original = storage[index];
                var t = trueWrites.TryGetValue((storage, index), out var tv) ? tv : original;
                var f = falseWrites.TryGetValue((storage, index), out var fv) ? fv : original;
                _journal.Store(storage, index, ValueMerge.Merge(_graph, condition, t, f, $"'{storage.Name}[{index}]'", branch, requireBoth: true));
            }

            return Completion.Normal;
        }

        private BranchState RunBranch(ImmutableArray<IRNode> statements, ConditionalNode branch, KernelFrame frame, IReadOnlyList<KeyValuePair<string, (ITypeSymbol Type, KernelValue? Value)>> before)
        {
            frame.BeginBranch();
            _journal.Begin();
            var completion = Execute(statements, frame);
            var writes = _journal.EndAndUndo();
            var declared = frame.EndBranch();
            var variables = frame.Snapshot();
            frame.Restore(before);
            return completion == Completion.Normal
                ? new BranchState(variables, new HashSet<string>(declared), writes)
                : throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, branch.Condition, BranchExitMessage);
        }

        private void CountIteration(IRNode loop)
        {
            _cancellationToken.ThrowIfCancellationRequested();
            if (++_iterations > MaxIterations)
            {
                throw new KernelException(KernelDiagnostics.UnrollLimit, loop, $"the kernel unrolls more than {MaxIterations.ToString("N0", CultureInfo.InvariantCulture)} loop iterations");
            }

            if (_graph.Count > MaxNodes)
            {
                throw new KernelException(KernelDiagnostics.UnrollLimit, loop, $"the kernel builds more than {MaxNodes.ToString("N0", CultureInfo.InvariantCulture)} graph nodes");
            }
        }

        private KernelValue Evaluate(IRNode node, KernelFrame frame)
        {
            return node switch
            {
                ConstantNode constant => Constant(constant),
                VariableNode variable => Variable(variable, frame),
                ElementAccessNode element => ReadElement(element, frame),
                ArraySliceNode slice => Slice(slice, frame),
                ArrayLengthNode length => new IntValue(EvaluateArray(length.Array, frame, length).Length),
                ArrayCreationNode creation => CreateArray(creation, frame, creation.Name),
                UnaryOpNode unary => EvaluateUnary(unary, frame),
                BinaryOpNode binary => EvaluateBinary(binary, frame),
                ConditionalExpressionNode conditional => EvaluateConditional(conditional, frame),
                FieldAccessNode field => ReadField(field, frame),
                StructCreationNode creation => CreateStruct(creation, frame),
                MethodCallNode call => Call(call, frame)
                    ?? throw new KernelException(KernelDiagnostics.UnsupportedSyntax, call, $"'{call.MethodName}' returns no value"),
                InstanceCallNode call => InstanceCall(call, frame)
                    ?? throw new KernelException(KernelDiagnostics.UnsupportedSyntax, call, $"'{call.Method.Name}' returns no value"),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, node, $"the expression {node.GetType().Name} is not supported in a kernel")
            };
        }

        private KernelValue Constant(ConstantNode constant)
        {
            return constant.Value switch
            {
                int i => new IntValue(i),
                double d => new ScalarValue(_graph.Const(d)),
                bool b => new BoolValue(b),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, constant, $"a constant of type {constant.Value.GetType().Name} is not supported; kernels compute with double, int and bool")
            };
        }

        private static KernelValue Variable(VariableNode variable, KernelFrame frame)
        {
            if (!frame.TryGet(variable.Name, out _, out var value))
            {
                throw new KernelException(KernelDiagnostics.UnsupportedSyntax, variable, $"'{variable.Name}' is not in scope");
            }

            return value ?? throw new KernelException(KernelDiagnostics.UnassignedValue, variable, $"'{variable.Name}' is read before it is assigned");
        }

        private KernelValue ReadElement(ElementAccessNode element, KernelFrame frame)
        {
            var array = EvaluateArray(element.Array, frame, element);
            return Read(array.Storage, ElementIndex(array, element.Indices, frame, element), element);
        }

        private static KernelValue Read(KernelArray storage, int index, IRNode at)
        {
            return storage[index]
                ?? throw new KernelException(KernelDiagnostics.UnassignedValue, at, $"'{storage.Name}[{index}]' is read before it is written");
        }

        // The storage index of a[i] or a[i, j].
        private int ElementIndex(ArrayValue array, ImmutableArray<IRNode> indices, KernelFrame frame, IRNode at)
        {
            var values = indices.Select(i => EvaluateInt(i, frame, "an index")).ToArray();
            var storage = array.Storage;
            if (values.Length == 1 && storage.Columns == 0)
            {
                return array.Offset + CheckIndex(values[0], array.Length, storage.Name, at);
            }

            if (values.Length == 2 && storage.Columns > 0)
            {
                var rows = storage.Length / storage.Columns;
                return (CheckIndex(values[0], rows, storage.Name, at) * storage.Columns) + CheckIndex(values[1], storage.Columns, storage.Name, at);
            }

            throw new KernelException(KernelDiagnostics.UnsupportedSyntax, at, $"'{storage.Name}' is indexed with {values.Length} indices but has {(storage.Columns > 0 ? 2 : 1)} dimensions");
        }

        private static int CheckIndex(int index, int length, string name, IRNode at)
        {
            return index >= 0 && index < length
                ? index
                : throw new KernelException(KernelDiagnostics.IndexOutOfRange, at, $"the index {index} is outside '{name}', whose length is {length}");
        }

        private KernelValue Slice(ArraySliceNode slice, KernelFrame frame)
        {
            var array = EvaluateArray(slice.Array, frame, slice);
            var start = EvaluateInt(slice.Start, frame, "a slice's start");
            var length = slice.Length == null ? array.Length - start : EvaluateInt(slice.Length, frame, "a slice's length");
            if (array.Storage.Columns > 0)
            {
                throw new KernelException(KernelDiagnostics.UnsupportedSyntax, slice, $"'{array.Storage.Name}' is a rectangular table and cannot be sliced");
            }

            if (start < 0 || length < 0 || start + length > array.Length)
            {
                throw new KernelException(KernelDiagnostics.IndexOutOfRange, slice, $"the slice ({start}, {length}) is outside '{array.Storage.Name}', whose length is {array.Length}");
            }

            return new ArrayValue(array.Storage, array.Offset + start, length);
        }

        private KernelValue CreateArray(ArrayCreationNode creation, KernelFrame frame, string name)
        {
            if (creation.IsReadOnly && _tables.TryGetValue(creation.NodeId, out var table))
            {
                return table;
            }

            var dimensions = creation.Dimensions.Select(d => EvaluateInt(d, frame, "an array length")).ToArray();
            var length = dimensions.Aggregate(1, (a, b) => a * b);
            if (dimensions.Any(d => d < 0) || (creation.Elements.Length > 0 && creation.Elements.Length != length))
            {
                throw new KernelException(KernelDiagnostics.IndexOutOfRange, creation, $"the array's {creation.Elements.Length} elements do not fill its {string.Join(" × ", dimensions)} shape");
            }

            var isIntegral = creation.ElementType.SpecialType == SpecialType.System_Int32;
            var zero = isIntegral ? (KernelValue)new IntValue(0) : new ScalarValue(_graph.Const(0));
            var elements = creation.Elements.IsEmpty
                ? Enumerable.Repeat<KernelValue?>(zero, length)
                : creation.Elements.Select(e => (KernelValue?)Evaluate(e, frame));
            var storage = new KernelArray(name, isIntegral, dimensions.Length == 2 ? dimensions[1] : 0, creation.IsReadOnly, elements.Select(e => (KernelValue?)ElementValue(isIntegral, e!, creation)));
            var array = new ArrayValue(storage, 0, length);
            if (creation.IsReadOnly)
            {
                _tables[creation.NodeId] = array;
            }
            else
            {
                _journal.Created(storage);
            }

            return array;
        }

        private KernelValue EvaluateUnary(UnaryOpNode unary, KernelFrame frame)
        {
            var operand = Evaluate(unary.Operand, frame);
            return (unary.Op, operand) switch
            {
                (UnaryOperator.Negate, IntValue i) => new IntValue(unchecked(-i.Value)),
                (UnaryOperator.Negate, _) => new ScalarValue(_graph.Neg(ToScalar(operand, unary))),
                (UnaryOperator.Plus, _) => operand,
                (UnaryOperator.LogicalNot, BoolValue b) => new BoolValue(!b.Value),
                (UnaryOperator.LogicalNot, ConditionValue c) => c with { Negated = !c.Negated },
                (UnaryOperator.Identity, _) => Convert(unary.Type, operand, unary),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, unary, $"the operator {unary.Op} cannot be applied to this value")
            };
        }

        private KernelValue EvaluateBinary(BinaryOpNode binary, KernelFrame frame)
        {
            if (binary.Op is BinaryOperator.LogicalAnd or BinaryOperator.LogicalOr)
            {
                var left = EvaluateLogical(binary.Left, frame);
                var decided = binary.Op == BinaryOperator.LogicalAnd ? !left : left;
                return new BoolValue(decided ? left : EvaluateLogical(binary.Right, frame));
            }

            return (Evaluate(binary.Left, frame), Evaluate(binary.Right, frame)) switch
            {
                (IntValue l, IntValue r) => IntBinary(binary, l.Value, r.Value),
                (BoolValue l, BoolValue r) when binary.Op is BinaryOperator.Equal or BinaryOperator.NotEqual
                    => new BoolValue((l.Value == r.Value) == (binary.Op == BinaryOperator.Equal)),
                var (l, r) => ScalarBinary(binary, ToScalar(l, binary), ToScalar(r, binary))
            };
        }

        private bool EvaluateLogical(IRNode operand, KernelFrame frame)
        {
            return Evaluate(operand, frame) switch
            {
                BoolValue b => b.Value,
                ConditionValue => throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, operand, "&& and || need compile-time operands; combine run-time comparisons by nesting if statements or conditional expressions"),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, operand, "&& and || need boolean operands")
            };
        }

        private static KernelValue IntBinary(BinaryOpNode binary, int l, int r)
        {
            if (binary.Op is BinaryOperator.Divide or BinaryOperator.Modulo && r == 0)
            {
                throw new KernelException(KernelDiagnostics.UnsupportedSyntax, binary, "an integer division by zero");
            }

            return binary.Op switch
            {
                BinaryOperator.Add => new IntValue(unchecked(l + r)),
                BinaryOperator.Subtract => new IntValue(unchecked(l - r)),
                BinaryOperator.Multiply => new IntValue(unchecked(l * r)),
                BinaryOperator.Divide => new IntValue(l / r),
                BinaryOperator.Modulo => new IntValue(l % r),
                BinaryOperator.LessThan => new BoolValue(l < r),
                BinaryOperator.LessThanOrEqual => new BoolValue(l <= r),
                BinaryOperator.GreaterThan => new BoolValue(l > r),
                BinaryOperator.GreaterThanOrEqual => new BoolValue(l >= r),
                BinaryOperator.Equal => new BoolValue(l == r),
                BinaryOperator.NotEqual => new BoolValue(l != r),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, binary, $"the operator {binary.Op} is not supported on int")
            };
        }

        private KernelValue ScalarBinary(BinaryOpNode binary, int l, int r)
        {
            return binary.Op switch
            {
                BinaryOperator.Add => new ScalarValue(_graph.Add(l, r)),
                BinaryOperator.Subtract => new ScalarValue(_graph.Sub(l, r)),
                BinaryOperator.Multiply => new ScalarValue(_graph.Mul(l, r)),
                BinaryOperator.Divide => new ScalarValue(_graph.Div(l, r)),
                BinaryOperator.LessThan => Compare(ScalarOp.SelectLess, l, r, false),
                BinaryOperator.GreaterThan => Compare(ScalarOp.SelectLess, r, l, false),
                BinaryOperator.LessThanOrEqual => Compare(ScalarOp.SelectLessEqual, l, r, false),
                BinaryOperator.GreaterThanOrEqual => Compare(ScalarOp.SelectLessEqual, r, l, false),
                BinaryOperator.Equal => Compare(ScalarOp.SelectEqual, l, r, false),
                BinaryOperator.NotEqual => Compare(ScalarOp.SelectEqual, l, r, true),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, binary, $"the operator {binary.Op} is not supported on double")
            };
        }

        // A comparison of constants is decided now; any other is a condition for a select.
        private KernelValue Compare(ScalarOp op, int left, int right, bool negated)
        {
            return _graph.IsConst(left, out var l) && _graph.IsConst(right, out var r)
                ? new BoolValue(ScalarGraph.Compare(op, l, r) != negated)
                : new ConditionValue(op, left, right, negated);
        }

        private KernelValue EvaluateConditional(ConditionalExpressionNode conditional, KernelFrame frame)
        {
            switch (Evaluate(conditional.Condition, frame))
            {
                case BoolValue b:
                    return Convert(conditional.Type, Evaluate(b.Value ? conditional.TrueExpression : conditional.FalseExpression, frame), conditional);
                case ConditionValue c when conditional.Type.SpecialType == SpecialType.System_Double || KernelStructs.IsStruct(conditional.Type):
                    var whenTrue = Convert(conditional.Type, Evaluate(conditional.TrueExpression, frame), conditional.TrueExpression);
                    var whenFalse = Convert(conditional.Type, Evaluate(conditional.FalseExpression, frame), conditional.FalseExpression);
                    return ValueMerge.Merge(_graph, c, whenTrue, whenFalse, "the conditional expression", conditional, requireBoth: true)!;
                case ConditionValue:
                    throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, conditional, $"a run-time condition chooses a value of type {conditional.Type.ToDisplayString()}; only a double or a struct of doubles can be chosen at run time");
                default:
                    throw new KernelException(KernelDiagnostics.UnsupportedSyntax, conditional.Condition, "a condition must be a boolean");
            }
        }

        private bool EvaluateCondition(IRNode condition, KernelFrame frame, string runTimeMessage)
        {
            return Evaluate(condition, frame) switch
            {
                BoolValue b => b.Value,
                ConditionValue => throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, condition, runTimeMessage),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, condition, "a condition must be a boolean")
            };
        }

        private KernelValue ReadField(FieldAccessNode access, KernelFrame frame)
        {
            var target = Evaluate(access.Target, frame) as StructValue
                ?? throw new KernelException(KernelDiagnostics.UnsupportedSyntax, access, $"'{access.Field.Name}' is read from a value that is not a struct");
            return target.Fields[KernelStructs.IndexOf(target.Type, access.Field)]
                ?? throw new KernelException(KernelDiagnostics.UnassignedValue, access, $"the field '{access.Field.Name}' is read before it is assigned");
        }

        // target.Field = value, where target is a variable or a field of one: the variable gets a new struct value.
        private void AssignField(IRNode target, IFieldSymbol field, KernelValue value, KernelFrame frame, IRNode at)
        {
            var current = ReadForWrite(target, frame, at);
            StoreInto(target, current.With(KernelStructs.IndexOf(current.Type, field), Convert(field.Type, value, at)), frame, at);
        }

        // The struct value at target, or a value with every field unassigned if the variable is unassigned.
        private StructValue ReadForWrite(IRNode target, KernelFrame frame, IRNode at)
        {
            switch (target)
            {
                case VariableNode variable when frame.TryGet(variable.Name, out var type, out var value):
                    return value as StructValue ?? KernelStructs.Unassigned((INamedTypeSymbol)type);
                case FieldAccessNode access:
                    var parent = ReadForWrite(access.Target, frame, at);
                    return parent.Fields[KernelStructs.IndexOf(parent.Type, access.Field)] as StructValue
                        ?? KernelStructs.Unassigned((INamedTypeSymbol)access.Field.Type);
                default:
                    throw new KernelException(KernelDiagnostics.UnsupportedSyntax, at, "a field is assigned on a value that is not a variable; assign the struct to a local first");
            }
        }

        private void StoreInto(IRNode target, KernelValue value, KernelFrame frame, IRNode at)
        {
            switch (target)
            {
                case VariableNode variable:
                    frame.Assign(variable.Name, value);
                    break;
                case FieldAccessNode access:
                    AssignField(access.Target, access.Field, value, frame, at);
                    break;
                default:
                    throw new KernelException(KernelDiagnostics.UnsupportedSyntax, at, "a field is assigned on a value that is not a variable; assign the struct to a local first");
            }
        }

        private static bool IsAssignable(IRNode target)
        {
            return target is VariableNode || (target is FieldAccessNode access && IsAssignable(access.Target));
        }

        private StructValue CreateStruct(StructCreationNode creation, KernelFrame frame)
        {
            var value = creation switch
            {
                { Source: not null } => Evaluate(creation.Source, frame) as StructValue
                    ?? throw new KernelException(KernelDiagnostics.UnsupportedSyntax, creation, "the source of a with expression is not a struct"),
                { Constructor: not null } => Construct(creation.Constructor, creation.Arguments.Select(a => Evaluate(a, frame)).ToList(), creation),
                _ => KernelStructs.Zero(creation.Type, _graph)
            };
            foreach (var initializer in creation.Initializers)
            {
                value = value.With(KernelStructs.IndexOf(value.Type, initializer.Field), Convert(initializer.Field.Type, Evaluate(initializer.Value, frame), initializer.Value));
            }

            return value;
        }

        // A constructor runs on a zeroed value, as C# 11 does for fields the constructor leaves unassigned.
        private StructValue Construct(IMethodSymbol constructor, IReadOnlyList<KernelValue> args, IRNode at)
        {
            var self = KernelStructs.Zero(constructor.ContainingType, _graph);
            _ = Invoke(constructor, args, self, at, out var constructed);
            return constructed!;
        }

        private KernelValue? Call(MethodCallNode call, KernelFrame frame)
        {
            var method = call.MethodSymbol
                ?? throw new KernelException(KernelDiagnostics.UnsupportedCall, call, $"the call to '{call.MethodName}' could not be resolved");
            var args = call.Arguments.Select(a => Evaluate(a, frame)).ToList();
            return KernelMathCalls.IsMath(method)
                ? MathCall(method, call, args)
                : Invoke(method, args, null, call, out _);
        }

        // receiver.Method(args): the callee sees the receiver as `this`, and a change it makes is written back when the
        // receiver is a variable or a field of one.
        private KernelValue? InstanceCall(InstanceCallNode call, KernelFrame frame)
        {
            var receiver = Evaluate(call.Receiver, frame) as StructValue
                ?? throw new KernelException(KernelDiagnostics.UnsupportedSyntax, call, $"'{call.Method.Name}' is called on a value that is not a struct");
            var args = call.Arguments.Select(a => Evaluate(a, frame)).ToList();
            var result = Invoke(call.Method, args, receiver, call, out var after);
            if (after != null && !ValueMerge.Same(after, receiver) && IsAssignable(call.Receiver))
            {
                StoreInto(call.Receiver, after, frame, call);
            }

            return result;
        }

        // Inlines a helper, constructor or struct member; self is `this` for a struct member, and selfAfter its value at
        // the end of the call.
        private KernelValue? Invoke(IMethodSymbol method, IReadOnlyList<KernelValue> args, StructValue? self, IRNode at, out StructValue? selfAfter)
        {
            var helper = _library.Helper(method, at);
            if (++_depth > MaxCallDepth)
            {
                throw new KernelException(KernelDiagnostics.UnrollLimit, at, $"helper calls nest more than {MaxCallDepth} deep");
            }

            var callee = new KernelFrame(method.ReturnType);
            if (self != null)
            {
                callee.Declare(KernelFrame.This, self.Type, self);
            }

            foreach (var (parameter, arg) in method.Parameters.Zip(args, (p, a) => (p, a)))
            {
                callee.Declare(parameter.Name, parameter.Type, Convert(parameter.Type, arg, at));
            }

            _ = Execute(helper.Body.Statements, callee);
            _depth--;
            selfAfter = self == null ? null : CurrentThis(callee, method, at);
            return method.ReturnsVoid || method.MethodKind == MethodKind.Constructor
                ? null
                : callee.ReturnValue ?? throw new KernelException(KernelDiagnostics.UnassignedValue, at, $"'{method.Name}' returned no value");
        }

        private static StructValue CurrentThis(KernelFrame callee, IMethodSymbol method, IRNode at)
        {
            _ = callee.TryGet(KernelFrame.This, out _, out var value);
            var self = (StructValue)value!;
            var unassigned = self.Fields.IndexOf(null);
            return unassigned < 0
                ? self
                : throw new KernelException(KernelDiagnostics.UnassignedValue, at, $"the field '{KernelStructs.Fields(self.Type)[unassigned].Name}' of {self.Type.Name} is unassigned at the end of '{method.ToDisplayString()}'");
        }

        private KernelValue MathCall(IMethodSymbol method, MethodCallNode call, IReadOnlyList<KernelValue> args)
        {
            if (method.Parameters.All(p => p.Type.SpecialType == SpecialType.System_Int32))
            {
                return KernelMathCalls.ApplyInt(args.Select(a => a is IntValue i ? i.Value : throw new KernelException(KernelDiagnostics.UnsupportedCall, call, "an int Math function needs compile-time int arguments")).ToList(), call);
            }

            if (method.Parameters.All(p => p.Type.SpecialType == SpecialType.System_Double))
            {
                return KernelMathCalls.ApplyDouble(_graph, args.Select(a => ToScalar(a, call)).ToList(), call);
            }

            throw new KernelException(KernelDiagnostics.UnsupportedCall, call, $"the overload '{method.ToDisplayString()}' is not supported; kernels call Math functions on double, and Abs, Min, Max, Sign and Clamp on int");
        }

        // KernelMath.ImplicitSolve(Residual, p, y, iterations): the residual is inlined once at placeholder unknowns, and
        // ImplicitSolver unrolls Newton's method and the implicit-function step from it.
        private void ExecuteImplicitSolve(ImplicitSolveNode solve, KernelFrame frame)
        {
            var residual = solve.Residual;
            if (!IsResidualSignature(residual))
            {
                throw new KernelException(KernelDiagnostics.InvalidImplicitSolve, solve, $"the residual '{residual.Name}' must be a static void method (ReadOnlySpan<double> y, ReadOnlySpan<double> p, Span<double> g)");
            }

            var p = EvaluateArray(solve.Parameters, frame, solve);
            var y = EvaluateArray(solve.Unknowns, frame, solve);
            var m = y.Length;
            if (m < 1 || m > ImplicitSolver.MaxUnknowns)
            {
                throw new KernelException(KernelDiagnostics.InvalidImplicitSolve, solve, $"the system has {m} unknowns; an implicit solve has 1 to {ImplicitSolver.MaxUnknowns}");
            }

            if (y.Storage.IsReadOnly)
            {
                throw new KernelException(KernelDiagnostics.ReadOnlyWrite, solve, $"'{y.Storage.Name}' is read-only to a kernel; ImplicitSolve writes its solution to y");
            }

            var iterations = EvaluateInt(solve.Iterations, frame, "ImplicitSolve's iteration count");
            if (iterations < 1)
            {
                throw new KernelException(KernelDiagnostics.InvalidImplicitSolve, solve, $"the iteration count is {iterations}; an implicit solve takes at least one Newton step");
            }

            var guess = Enumerable.Range(0, m).Select(i => ToScalar(Read(y.Storage, y.Offset + i, solve), solve)).ToList();
            var group = -1 - _implicitSolves++;
            var placeholders = Enumerable.Range(0, m).Select(i => _graph.Input(group, i)).ToList();
            var unknowns = new KernelArray("y", false, 0, true, placeholders.Select(id => (KernelValue?)new ScalarValue(id)));
            var g = new KernelArray("g", false, 0, false, Enumerable.Repeat<KernelValue?>(null, m));
            _journal.Created(g);
            _ = Invoke(residual, new KernelValue[] { new ArrayValue(unknowns, 0, m), p, new ArrayValue(g, 0, m) }, null, solve, out _);
            var unwritten = g.FirstUnwritten();
            if (unwritten >= 0)
            {
                throw new KernelException(KernelDiagnostics.UnassignedValue, solve, $"the residual '{residual.Name}' never writes g[{unwritten}]; it must write one equation per unknown");
            }

            var residualIds = Enumerable.Range(0, m).Select(i => ((ScalarValue)g[i]!).Id).ToList();
            var solution = ImplicitSolver.Solve(_graph, placeholders, residualIds, guess, iterations, _derivativeOrder, () => CountIteration(solve));
            for (var i = 0; i < m; i++)
            {
                _journal.Store(y.Storage, y.Offset + i, new ScalarValue(solution[i]));
            }
        }

        private static bool IsResidualSignature(IMethodSymbol method)
        {
            var parameters = method.Parameters;
            return method.IsStatic
                && method.ReturnsVoid
                && parameters.Length == 3
                && IsDoubleSpan(parameters[0].Type, writable: false)
                && IsDoubleSpan(parameters[1].Type, writable: false)
                && IsDoubleSpan(parameters[2].Type, writable: true);
        }

        private static bool IsDoubleSpan(ITypeSymbol type, bool writable)
        {
            return KernelASTToIRConverter.IsSpan(type, out var element, out var isWritable)
                && isWritable == writable
                && element.SpecialType == SpecialType.System_Double;
        }

        // Converts a value to a declared or cast type: double, int, bool, a span or a struct.
        private KernelValue Convert(ITypeSymbol type, KernelValue value, IRNode at)
        {
            return (type.SpecialType, value) switch
            {
                (SpecialType.System_Double, _) => new ScalarValue(ToScalar(value, at)),
                (SpecialType.System_Int32, IntValue) => value,
                (SpecialType.System_Int32, ScalarValue s) when _graph.IsConst(s.Id, out var c) => new IntValue(unchecked((int)c)),
                (SpecialType.System_Int32, ScalarValue) => throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, at, "a run-time double is converted to int; a kernel's ints must be compile-time values"),
                (SpecialType.System_Boolean, BoolValue or ConditionValue) => value,
                (_, ArrayValue) when KernelASTToIRConverter.IsSpan(type, out _, out _) || type is IArrayTypeSymbol => value,
                (_, StructValue s) when SymbolEqualityComparer.Default.Equals(type, s.Type) => value,
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, at, $"a value cannot be converted to {type.ToDisplayString()} in a kernel")
            };
        }

        private KernelValue ElementValue(bool isIntegral, KernelValue value, IRNode at)
        {
            return isIntegral
                ? value as IntValue ?? throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, at, "an int array element is given a run-time value; a kernel's ints must be compile-time values")
                : new ScalarValue(ToScalar(value, at));
        }

        private KernelValue Element(KernelArray storage, KernelValue value, IRNode at)
        {
            return ElementValue(storage.IsIntegral, value, at);
        }

        private int ToScalar(KernelValue value, IRNode at)
        {
            return value switch
            {
                ScalarValue s => s.Id,
                IntValue i => _graph.Const(i.Value),
                _ => throw new KernelException(KernelDiagnostics.UnsupportedSyntax, at, "a boolean, span or struct is used as a number")
            };
        }

        private int EvaluateInt(IRNode node, KernelFrame frame, string what)
        {
            return Evaluate(node, frame) switch
            {
                IntValue i => i.Value,
                _ => throw new KernelException(KernelDiagnostics.ControlFlowNotConstant, node, $"{what} is not a compile-time int; a kernel's indices, lengths and loop bounds must not depend on run-time values")
            };
        }

        private ArrayValue EvaluateArray(IRNode node, KernelFrame frame, IRNode at)
        {
            return Evaluate(node, frame) as ArrayValue
                ?? throw new KernelException(KernelDiagnostics.UnsupportedSyntax, at, "the value indexed is not a span or array");
        }

        private sealed record BranchState(
            IReadOnlyList<KeyValuePair<string, (ITypeSymbol Type, KernelValue? Value)>> Variables,
            HashSet<string> Declared,
            IReadOnlyList<(KernelArray Storage, int Index, KernelValue? Value)> Writes);
    }
}
