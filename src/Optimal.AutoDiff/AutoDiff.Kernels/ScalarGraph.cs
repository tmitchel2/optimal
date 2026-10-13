/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// A straight-line SSA graph of scalar operations. Every builder folds constants, applies the algebraic identities
    /// (x + 0, x − x, x × 1, x × 0, −(−x), and negations moved outward through products) and hash-conses the result, so
    /// a value is built once however many times it is asked for: common subexpressions, and the zeros of sparse
    /// tangents, vanish as the graph grows. Nodes are never removed; the emitter keeps only those an output reaches.
    /// </summary>
    /// <remarks>
    /// Two identities are not exact in IEEE arithmetic and are taken deliberately, as every AD tool takes them:
    /// x × 0 = 0 (wrong for an infinite or NaN x) and x + 0 = x (wrong in the sign of a negative zero). Everything
    /// else, including the reordering of commutative operands, leaves every result bit unchanged.
    /// </remarks>
    public sealed class ScalarGraph
    {
        private readonly List<ScalarNode> _nodes = new();
        private readonly Dictionary<(ScalarOp Op, int A, int B, int C, int D, long Bits), int> _interned = new();

        /// <summary>The number of nodes; ids run from 0 to <c>Count − 1</c>.</summary>
        public int Count => _nodes.Count;

        public ScalarNode this[int id] => _nodes[id];

        /// <summary>Whether node <paramref name="id"/> is a constant, and its value.</summary>
        public bool IsConst(int id, out double value)
        {
            var node = _nodes[id];
            value = node.Value;
            return node.Op == ScalarOp.Const;
        }

        /// <summary>Whether node <paramref name="id"/> is the constant <paramref name="value"/> (bitwise, so −0 is not 0).</summary>
        public bool IsConst(int id, double value)
        {
            return IsConst(id, out var actual) && BitConverter.DoubleToInt64Bits(actual) == BitConverter.DoubleToInt64Bits(value);
        }

        public int Const(double value)
        {
            return Intern(ScalarOp.Const, -1, -1, -1, -1, value);
        }

        public int Input(int group, int index)
        {
            return Intern(ScalarOp.Input, group, index, -1, -1, 0);
        }

        /// <summary>Finds an input node without creating it.</summary>
        public bool TryGetInput(int group, int index, out int id)
        {
            return _interned.TryGetValue((ScalarOp.Input, group, index, -1, -1, 0), out id);
        }

        public int Add(int a, int b)
        {
            if (IsConst(a, out var ca) && IsConst(b, out var cb))
            {
                return Const(ca + cb);
            }

            if (IsZero(a))
            {
                return b;
            }

            if (IsZero(b))
            {
                return a;
            }

            if (_nodes[b].Op == ScalarOp.Neg)
            {
                return Sub(a, _nodes[b].A);
            }

            return _nodes[a].Op == ScalarOp.Neg ? Sub(b, _nodes[a].A) : Binary(ScalarOp.Add, a, b);
        }

        public int Sub(int a, int b)
        {
            if (IsConst(a, out var ca) && IsConst(b, out var cb))
            {
                return Const(ca - cb);
            }

            if (IsZero(b))
            {
                return a;
            }

            if (IsZero(a))
            {
                return Neg(b);
            }

            if (a == b)
            {
                return Const(0);
            }

            return _nodes[b].Op == ScalarOp.Neg ? Add(a, _nodes[b].A) : Binary(ScalarOp.Sub, a, b);
        }

        public int Mul(int a, int b)
        {
            if (IsConst(a, out var ca) && IsConst(b, out var cb))
            {
                return Const(ca * cb);
            }

            if (IsZero(a) || IsZero(b))
            {
                return Const(0);
            }

            if (IsConst(a, 1))
            {
                return b;
            }

            if (IsConst(b, 1))
            {
                return a;
            }

            if (IsConst(a, -1))
            {
                return Neg(b);
            }

            if (IsConst(b, -1))
            {
                return Neg(a);
            }

            if (_nodes[a].Op == ScalarOp.Neg)
            {
                return Neg(Mul(_nodes[a].A, b));
            }

            return _nodes[b].Op == ScalarOp.Neg ? Neg(Mul(a, _nodes[b].A)) : Binary(ScalarOp.Mul, a, b);
        }

        public int Div(int a, int b)
        {
            if (IsConst(a, out var ca) && IsConst(b, out var cb))
            {
                return Const(ca / cb);
            }

            if (IsZero(a))
            {
                return Const(0);
            }

            if (IsConst(b, 1))
            {
                return a;
            }

            if (IsConst(b, -1))
            {
                return Neg(a);
            }

            if (_nodes[a].Op == ScalarOp.Neg)
            {
                return Neg(Div(_nodes[a].A, b));
            }

            return _nodes[b].Op == ScalarOp.Neg ? Neg(Div(a, _nodes[b].A)) : Binary(ScalarOp.Div, a, b);
        }

        public int Neg(int a)
        {
            if (IsConst(a, out var ca))
            {
                return Const(-ca);
            }

            return _nodes[a].Op == ScalarOp.Neg ? _nodes[a].A : Intern(ScalarOp.Neg, a, -1, -1, -1, 0);
        }

        /// <summary>A one-operand function: Sqrt, Cbrt, Abs, Sign, Exp, Log, Sin, Cos, Tan, Atan or Tanh.</summary>
        public int Unary(ScalarOp op, int a)
        {
            if (ScalarOps.Arity(op) != 1 || op is ScalarOp.Neg or ScalarOp.PowConst)
            {
                throw new ArgumentException($"{op} is not a one-operand function.", nameof(op));
            }

            // netstandard2.0 has no Math.Cbrt to fold with, so a constant cube root is left to the run time.
            if (op != ScalarOp.Cbrt && IsConst(a, out var ca))
            {
                return Const(Evaluate(op, ca));
            }

            var operand = _nodes[a];
            return op switch
            {
                ScalarOp.Abs when operand.Op is ScalarOp.Abs => a,
                ScalarOp.Abs when operand.Op is ScalarOp.Neg => Unary(ScalarOp.Abs, operand.A),
                _ => Intern(op, a, -1, -1, -1, 0)
            };
        }

        /// <summary>A raised to the constant <paramref name="exponent"/>, with the exact small cases rewritten.</summary>
        public int PowConst(int a, double exponent)
        {
            if (IsConst(a, out var ca))
            {
                return Const(Math.Pow(ca, exponent));
            }

            return exponent switch
            {
                0 => Const(1),
                1 => a,
                2 => Mul(a, a),
                -1 => Div(Const(1), a),
                _ => Intern(ScalarOp.PowConst, a, -1, -1, -1, exponent)
            };
        }

        public int Atan2(int y, int x)
        {
            return IsConst(y, out var cy) && IsConst(x, out var cx)
                ? Const(Math.Atan2(cy, cx))
                : Intern(ScalarOp.Atan2, y, x, -1, -1, 0);
        }

        public int Min(int a, int b)
        {
            if (IsConst(a, out var ca) && IsConst(b, out var cb))
            {
                return Const(Math.Min(ca, cb));
            }

            return a == b ? a : Binary(ScalarOp.Min, a, b);
        }

        public int Max(int a, int b)
        {
            if (IsConst(a, out var ca) && IsConst(b, out var cb))
            {
                return Const(Math.Max(ca, cb));
            }

            return a == b ? a : Binary(ScalarOp.Max, a, b);
        }

        /// <summary><paramref name="left"/> ⋚ <paramref name="right"/> ? <paramref name="whenTrue"/> : <paramref name="whenFalse"/>, for a select <paramref name="op"/>.</summary>
        public int Select(ScalarOp op, int left, int right, int whenTrue, int whenFalse)
        {
            if (!ScalarOps.IsSelect(op))
            {
                throw new ArgumentException($"{op} is not a select.", nameof(op));
            }

            if (whenTrue == whenFalse)
            {
                return whenTrue;
            }

            if (IsConst(left, out var cl) && IsConst(right, out var cr))
            {
                return Compare(op, cl, cr) ? whenTrue : whenFalse;
            }

            return Intern(op, left, right, whenTrue, whenFalse, 0);
        }

        /// <summary>Evaluates a select's comparison on constants.</summary>
        public static bool Compare(ScalarOp op, double left, double right)
        {
            return op switch
            {
                ScalarOp.SelectLess => left < right,
                ScalarOp.SelectLessEqual => left <= right,
                ScalarOp.SelectEqual => left == right,
                _ => throw new ArgumentException($"{op} is not a select.", nameof(op))
            };
        }

        /// <summary>Evaluates a one-operand function other than Cbrt on a constant, exactly as the emitted code does at run time.</summary>
        public static double Evaluate(ScalarOp op, double x)
        {
            return op switch
            {
                ScalarOp.Sqrt => Math.Sqrt(x),
                ScalarOp.Abs => Math.Abs(x),
                ScalarOp.Sign => x > 0 ? 1 : (x < 0 ? -1 : 0),
                ScalarOp.Exp => Math.Exp(x),
                ScalarOp.Log => Math.Log(x),
                ScalarOp.Sin => Math.Sin(x),
                ScalarOp.Cos => Math.Cos(x),
                ScalarOp.Tan => Math.Tan(x),
                ScalarOp.Atan => Math.Atan(x),
                ScalarOp.Tanh => Math.Tanh(x),
                _ => throw new ArgumentException($"{op} is not a one-operand function.", nameof(op))
            };
        }

        private bool IsZero(int id)
        {
            return IsConst(id, out var value) && value == 0;
        }

        private int Binary(ScalarOp op, int a, int b)
        {
            return ScalarOps.IsCommutative(op) && b < a ? Intern(op, b, a, -1, -1, 0) : Intern(op, a, b, -1, -1, 0);
        }

        private int Intern(ScalarOp op, int a, int b, int c, int d, double value)
        {
            var key = (op, a, b, c, d, BitConverter.DoubleToInt64Bits(value));
            if (_interned.TryGetValue(key, out var existing))
            {
                return existing;
            }

            var id = _nodes.Count;
            _nodes.Add(new ScalarNode(op, a, b, c, d, value));
            _interned[key] = id;
            return id;
        }
    }
}
