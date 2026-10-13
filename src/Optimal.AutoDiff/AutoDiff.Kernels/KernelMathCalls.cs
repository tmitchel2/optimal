/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Microsoft.CodeAnalysis;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The <c>System.Math</c> functions a kernel may call. On doubles each becomes a graph node with its own derivative
    /// rule; on compile-time ints Abs, Min, Max, Sign and Clamp are evaluated by the generator.
    /// </summary>
    public static class KernelMathCalls
    {
        /// <summary>The supported functions, for diagnostics.</summary>
        public const string SupportedList = "Abs, Atan, Atan2, Cbrt, Clamp, Cos, Exp, Log, Max, Min, Pow (constant exponent), Sign, Sin, Sqrt, Tan and Tanh";

        public static bool IsMath(IMethodSymbol method)
        {
            return method.ContainingType?.ToDisplayString() == "System.Math";
        }

        /// <summary>A Math function on compile-time ints.</summary>
        public static KernelValue ApplyInt(IReadOnlyList<int> args, MethodCallNode call)
        {
            return call.MethodName switch
            {
                "Abs" => new IntValue(Math.Abs(args[0])),
                "Min" => new IntValue(Math.Min(args[0], args[1])),
                "Max" => new IntValue(Math.Max(args[0], args[1])),
                "Sign" => new IntValue(Math.Sign(args[0])),
                "Clamp" when args[1] <= args[2] => new IntValue(Math.Min(Math.Max(args[0], args[1]), args[2])),
                "Clamp" => throw new KernelException(KernelDiagnostics.UnsupportedCall, call, $"Math.Clamp's minimum {args[1]} exceeds its maximum {args[2]}"),
                _ => throw Unsupported(call, "on int")
            };
        }

        /// <summary>A Math function on doubles, as graph nodes; <paramref name="args"/> are node ids.</summary>
        public static KernelValue ApplyDouble(ScalarGraph g, IReadOnlyList<int> args, MethodCallNode call)
        {
            return call.MethodName switch
            {
                "Sqrt" => Unary(g, ScalarOp.Sqrt, args),
                "Cbrt" => Unary(g, ScalarOp.Cbrt, args),
                "Abs" => Unary(g, ScalarOp.Abs, args),
                "Exp" => Unary(g, ScalarOp.Exp, args),
                "Log" when args.Count == 1 => Unary(g, ScalarOp.Log, args),
                "Sin" => Unary(g, ScalarOp.Sin, args),
                "Cos" => Unary(g, ScalarOp.Cos, args),
                "Tan" => Unary(g, ScalarOp.Tan, args),
                "Atan" => Unary(g, ScalarOp.Atan, args),
                "Tanh" => Unary(g, ScalarOp.Tanh, args),
                "Sign" => Sign(g, args[0]),
                "Pow" => Pow(g, args[0], args[1], call),
                "Atan2" => new ScalarValue(g.Atan2(args[0], args[1])),
                "Min" => new ScalarValue(g.Min(args[0], args[1])),
                "Max" => new ScalarValue(g.Max(args[0], args[1])),
                "Clamp" => new ScalarValue(g.Min(g.Max(args[0], args[1]), args[2])),
                _ => throw Unsupported(call, "on double")
            };
        }

        private static KernelValue Unary(ScalarGraph g, ScalarOp op, IReadOnlyList<int> args)
        {
            return new ScalarValue(g.Unary(op, args[0]));
        }

        // Math.Sign returns an int: a compile-time int when its argument is constant, otherwise a double node that can
        // only be used as a number.
        private static KernelValue Sign(ScalarGraph g, int x)
        {
            var sign = g.Unary(ScalarOp.Sign, x);
            return g.IsConst(sign, out var value) ? new IntValue((int)value) : new ScalarValue(sign);
        }

        private static KernelValue Pow(ScalarGraph g, int x, int exponent, MethodCallNode call)
        {
            return g.IsConst(exponent, out var c)
                ? new ScalarValue(g.PowConst(x, c))
                : throw new KernelException(
                    KernelDiagnostics.NonConstantExponent,
                    call,
                    "Math.Pow's exponent depends on run-time values; use a constant exponent, or Math.Exp(y * Math.Log(x)) for x > 0");
        }

        private static KernelException Unsupported(MethodCallNode call, string overload)
        {
            return new KernelException(
                KernelDiagnostics.UnsupportedCall,
                call,
                $"Math.{call.MethodName} {overload} is not supported in a kernel; the supported functions are {SupportedList}");
        }
    }
}
