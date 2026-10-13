/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Immutable;
using System.Linq;
using Microsoft.CodeAnalysis;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// If-conversion's merge: after both branches of an <c>if</c> on a run-time condition have run, a variable or array
    /// element is the value its branch left, chosen by a select. A double becomes a select node; a struct merges field by
    /// field. Compile-time values (ints, bools, spans) cannot be chosen at run time, so they must agree.
    /// </summary>
    public static class ValueMerge
    {
        /// <summary>
        /// <paramref name="condition"/> ? <paramref name="whenTrue"/> : <paramref name="whenFalse"/>. An unassigned side
        /// (null) leaves the result unassigned, unless <paramref name="requireBoth"/>, when it is an error: an array
        /// element written in one branch only would keep, in the other, a value the kernel cannot see.
        /// </summary>
        public static KernelValue? Merge(ScalarGraph g, ConditionValue condition, KernelValue? whenTrue, KernelValue? whenFalse, string what, IRNode at, bool requireBoth)
        {
            if (Same(whenTrue, whenFalse))
            {
                return whenTrue;
            }

            if (whenTrue == null || whenFalse == null)
            {
                return requireBoth
                    ? throw new KernelException(KernelDiagnostics.UnassignedValue, at, $"{what} is written in only one branch of an if whose condition depends on run-time values; write it in both, or before the if")
                    : null;
            }

            switch (whenTrue, whenFalse)
            {
                case (ScalarValue t, ScalarValue f):
                    return new ScalarValue(condition.Negated
                        ? g.Select(condition.Op, condition.Left, condition.Right, f.Id, t.Id)
                        : g.Select(condition.Op, condition.Left, condition.Right, t.Id, f.Id));
                case (StructValue t, StructValue f) when SymbolEqualityComparer.Default.Equals(t.Type, f.Type):
                    var fields = KernelStructs.Fields(t.Type);
                    return new StructValue(t.Type, t.Fields
                        .Select((field, i) => Merge(g, condition, field, f.Fields[i], $"{what}.{fields[i].Name}", at, requireBoth))
                        .ToImmutableArray());
                default:
                    throw new KernelException(
                        KernelDiagnostics.ControlFlowNotConstant,
                        at,
                        $"{what} is given different {Kind(whenTrue)} values in the branches of an if whose condition depends on run-time values; only doubles can be chosen at run time");
            }
        }

        /// <summary>Whether two values are the same: the same node, compile-time value, span or struct of the same fields.</summary>
        public static bool Same(KernelValue? a, KernelValue? b)
        {
            return (a, b) switch
            {
                (null, null) => true,
                (ScalarValue x, ScalarValue y) => x.Id == y.Id,
                (StructValue x, StructValue y) => SymbolEqualityComparer.Default.Equals(x.Type, y.Type) && x.Fields.Zip(y.Fields, Same).All(s => s),
                (null, _) or (_, null) => false,
                _ => a.Equals(b)
            };
        }

        private static string Kind(KernelValue value)
        {
            return value switch
            {
                IntValue => "int",
                BoolValue or ConditionValue => "bool",
                ArrayValue => "span",
                _ => "incompatible"
            };
        }
    }
}
