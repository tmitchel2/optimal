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
using System.Text;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// Emits graph nodes as C# locals: no tape and no heap, so a generated method allocates nothing. Only nodes an output
    /// reaches are emitted (dead-code elimination), in id order, which is a topological order. A method of more than
    /// <see cref="MaxNodesPerMethod"/> nodes is split into chunk methods, because the JIT compiles a method with
    /// thousands of locals without optimisation; a value one chunk defines and a later chunk reads passes through a
    /// <c>stackalloc</c> spill buffer. Span lengths are checked once, on entry.
    /// </summary>
    public sealed class KernelCodeEmitter
    {
        private const string Indent = "            ";

        public KernelCodeEmitter(int maxNodesPerMethod)
        {
            MaxNodesPerMethod = maxNodesPerMethod > 0
                ? maxNodesPerMethod
                : throw new ArgumentOutOfRangeException(nameof(maxNodesPerMethod), maxNodesPerMethod, "A chunk holds at least one node.");
        }

        public int MaxNodesPerMethod { get; }

        /// <summary>
        /// Emits a public static method <paramref name="methodName"/> (and its chunks) taking <paramref name="inputs"/>
        /// then one <c>Span&lt;double&gt;</c> per output, documented by <paramref name="summary"/>.
        /// </summary>
        public string Emit(string methodName, string summary, ScalarGraph g, IReadOnlyList<KernelParameter> inputs, IReadOnlyList<KernelOutput> outputs)
        {
            var order = LiveOperations(g, outputs);
            var chunks = Enumerable.Range(0, Math.Max(1, (order.Count + MaxNodesPerMethod - 1) / MaxNodesPerMethod))
                .Select(c => order.Skip(c * MaxNodesPerMethod).Take(MaxNodesPerMethod).ToList())
                .ToList();
            var chunkOf = chunks.SelectMany((ids, c) => ids.Select(id => (id, c))).ToDictionary(x => x.id, x => x.c);
            var spillSlots = chunks
                .SelectMany((ids, c) => ids.SelectMany(id => g[id].Operands).Where(o => chunkOf.TryGetValue(o, out var defined) && defined != c))
                .Distinct()
                .OrderBy(id => id)
                .Select((id, slot) => (id, slot))
                .ToDictionary(x => x.id, x => x.slot);
            var groups = inputs.ToDictionary(p => p.Group);
            var signature = string.Join(", ", inputs.Select(p => $"{p.TypeName} {p.Name}").Concat(outputs.Select(o => $"Span<double> {o.Name}")));

            var sb = new StringBuilder();
            sb.AppendLine($"        /// <summary>{summary}</summary>");
            sb.AppendLine("        [MethodImpl(MethodImplOptions.AggressiveOptimization)]");
            sb.AppendLine($"        public static void {methodName}({signature})");
            sb.AppendLine("        {");
            foreach (var p in inputs.Where(p => p.IsArray))
            {
                sb.AppendLine($"{Indent}if ({p.Name}.Length != {p.Length}) ThrowLength(nameof({p.Name}), {p.Length});");
            }

            foreach (var o in outputs)
            {
                sb.AppendLine($"{Indent}if ({o.Name}.Length != {o.Ids.Length}) ThrowLength(nameof({o.Name}), {o.Ids.Length});");
            }

            if (chunks.Count == 1)
            {
                EmitBody(sb, g, chunks[0], groups, outputs, chunkOf, 0, spillSlots, spilled: false);
            }
            else
            {
                var arguments = string.Join(", ", inputs.Select(p => p.Name).Concat(outputs.Select(o => o.Name)));
                sb.AppendLine($"{Indent}// {order.Count} operations in {chunks.Count} chunks; {spillSlots.Count} values cross a chunk boundary.");
                sb.AppendLine($"{Indent}Span<double> spill = stackalloc double[{Math.Max(1, spillSlots.Count)}];");
                for (var c = 0; c < chunks.Count; c++)
                {
                    sb.AppendLine($"{Indent}{methodName}Chunk{c}({arguments}, spill);");
                }
            }

            // Outputs that are constants or inputs are written directly.
            foreach (var (name, index, id) in OutputWrites(outputs).Where(w => !chunkOf.ContainsKey(w.Id)))
            {
                var node = g[id];
                var value = node.Op == ScalarOp.Input && groups[node.A].IsArray ? $"{groups[node.A].Name}[{node.B}]" : Operand(g, id, groups);
                sb.AppendLine($"{Indent}{name}[{index}] = {value};");
            }

            sb.AppendLine("        }");
            sb.AppendLine();

            for (var c = 0; chunks.Count > 1 && c < chunks.Count; c++)
            {
                sb.AppendLine("        [MethodImpl(MethodImplOptions.NoInlining | MethodImplOptions.AggressiveOptimization)]");
                sb.AppendLine($"        private static void {methodName}Chunk{c}({signature}, Span<double> spill)");
                sb.AppendLine("        {");
                EmitBody(sb, g, chunks[c], groups, outputs, chunkOf, c, spillSlots, spilled: true);
                sb.AppendLine("        }");
                sb.AppendLine();
            }

            return sb.ToString();
        }

        /// <summary>A C# literal for <paramref name="value"/> that round-trips every bit, negative zero and non-finite values included.</summary>
        public static string Literal(double value)
        {
            if (double.IsNaN(value))
            {
                return "double.NaN";
            }

            if (double.IsInfinity(value))
            {
                return value > 0 ? "double.PositiveInfinity" : "double.NegativeInfinity";
            }

            var text = value.ToString("R", CultureInfo.InvariantCulture) + "d";
            var isNegative = value < 0 || (value == 0 && BitConverter.DoubleToInt64Bits(value) < 0);
            return isNegative ? $"({(value == 0 ? "-0d" : text)})" : text;
        }

        // The operations an output reaches, in id order. Constants and inputs are written in place, not as locals.
        private static List<int> LiveOperations(ScalarGraph g, IReadOnlyList<KernelOutput> outputs)
        {
            var live = new bool[g.Count];
            foreach (var id in outputs.SelectMany(o => o.Ids))
            {
                live[id] = true;
            }

            for (var id = g.Count - 1; id >= 0; id--)
            {
                if (live[id])
                {
                    foreach (var operand in g[id].Operands)
                    {
                        live[operand] = true;
                    }
                }
            }

            return Enumerable.Range(0, g.Count).Where(id => live[id] && ScalarOps.Arity(g[id].Op) > 0).ToList();
        }

        private static IEnumerable<(string Name, int Index, int Id)> OutputWrites(IReadOnlyList<KernelOutput> outputs)
        {
            return outputs.SelectMany(o => o.Ids.Select((id, k) => (o.Name, k, id)));
        }

        private static void EmitBody(
            StringBuilder sb,
            ScalarGraph g,
            List<int> chunk,
            Dictionary<int, KernelParameter> groups,
            IReadOnlyList<KernelOutput> outputs,
            Dictionary<int, int> chunkOf,
            int c,
            Dictionary<int, int> spillSlots,
            bool spilled)
        {
            var operands = chunk.SelectMany(id => g[id].Operands).Distinct().OrderBy(id => id).ToList();
            foreach (var id in operands.Where(id => g[id].Op == ScalarOp.Input && groups[g[id].A].IsArray))
            {
                var input = g[id];
                sb.AppendLine($"{Indent}var {InputLocal(input)} = {groups[input.A].Name}[{input.B}];");
            }

            foreach (var id in operands.Where(id => chunkOf.TryGetValue(id, out var defined) && defined != c))
            {
                sb.AppendLine($"{Indent}var v{id} = spill[{spillSlots[id]}];");
            }

            foreach (var id in chunk)
            {
                sb.AppendLine($"{Indent}var v{id} = {Expression(g, id, groups)};");
                if (spilled && spillSlots.TryGetValue(id, out var slot))
                {
                    sb.AppendLine($"{Indent}spill[{slot}] = v{id};");
                }
            }

            var defined = new HashSet<int>(chunk);
            foreach (var (name, index, id) in OutputWrites(outputs).Where(w => defined.Contains(w.Id)))
            {
                sb.AppendLine($"{Indent}{name}[{index}] = v{id};");
            }
        }

        private static string Expression(ScalarGraph g, int id, Dictionary<int, KernelParameter> groups)
        {
            var node = g[id];
            string O(int operand) => Operand(g, operand, groups);
            return node.Op switch
            {
                ScalarOp.Add => $"{O(node.A)} + {O(node.B)}",
                ScalarOp.Sub => $"{O(node.A)} - {O(node.B)}",
                ScalarOp.Mul => $"{O(node.A)} * {O(node.B)}",
                ScalarOp.Div => $"{O(node.A)} / {O(node.B)}",
                ScalarOp.Neg => $"-{O(node.A)}",
                ScalarOp.Sqrt => $"Math.Sqrt({O(node.A)})",
                ScalarOp.Cbrt => $"Math.Cbrt({O(node.A)})",
                ScalarOp.Abs => $"Math.Abs({O(node.A)})",
                ScalarOp.Sign => $"{O(node.A)} > 0d ? 1d : ({O(node.A)} < 0d ? -1d : 0d)",
                ScalarOp.PowConst => $"Math.Pow({O(node.A)}, {Literal(node.Value)})",
                ScalarOp.Exp => $"Math.Exp({O(node.A)})",
                ScalarOp.Log => $"Math.Log({O(node.A)})",
                ScalarOp.Sin => $"Math.Sin({O(node.A)})",
                ScalarOp.Cos => $"Math.Cos({O(node.A)})",
                ScalarOp.Tan => $"Math.Tan({O(node.A)})",
                ScalarOp.Atan => $"Math.Atan({O(node.A)})",
                ScalarOp.Tanh => $"Math.Tanh({O(node.A)})",
                ScalarOp.Atan2 => $"Math.Atan2({O(node.A)}, {O(node.B)})",
                ScalarOp.Min => $"Math.Min({O(node.A)}, {O(node.B)})",
                ScalarOp.Max => $"Math.Max({O(node.A)}, {O(node.B)})",
                ScalarOp.SelectLess => $"{O(node.A)} < {O(node.B)} ? {O(node.C)} : {O(node.D)}",
                ScalarOp.SelectLessEqual => $"{O(node.A)} <= {O(node.B)} ? {O(node.C)} : {O(node.D)}",
                ScalarOp.SelectEqual => $"{O(node.A)} == {O(node.B)} ? {O(node.C)} : {O(node.D)}",
                ScalarOp.Detach => O(node.A),
                _ => throw new ArgumentOutOfRangeException(nameof(id), node.Op, "Not an operation.")
            };
        }

        private static string Operand(ScalarGraph g, int id, Dictionary<int, KernelParameter> groups)
        {
            var node = g[id];
            return node.Op switch
            {
                ScalarOp.Const => Literal(node.Value),
                ScalarOp.Input => groups[node.A].IsArray ? InputLocal(node) : groups[node.A].Name,
                _ => $"v{id}"
            };
        }

        private static string InputLocal(ScalarNode input)
        {
            return $"x{input.A}_{input.B}";
        }
    }
}
