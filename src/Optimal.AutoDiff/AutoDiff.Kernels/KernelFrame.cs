/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Generic;
using Microsoft.CodeAnalysis;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The variables of one kernel or inlined helper call while the unroller interprets it. C# forbids a name from
    /// shadowing another in scope, so one flat map per call is enough: a sibling scope's redeclaration rebinds the name.
    /// Inside a struct's constructor, method or property, the struct's value is the variable <c>this</c>.
    /// </summary>
    public sealed class KernelFrame
    {
        /// <summary>The name of the variable that holds a struct member's receiver.</summary>
        public const string This = "this";

        private readonly Dictionary<string, (ITypeSymbol Type, KernelValue? Value)> _variables = new();
        private readonly Stack<HashSet<string>> _branches = new();

        public KernelFrame(ITypeSymbol returnType)
        {
            ReturnType = returnType;
        }

        public ITypeSymbol ReturnType { get; }

        /// <summary>The value of the <c>return</c> statement that ended the call, if it returned one.</summary>
        public KernelValue? ReturnValue { get; set; }

        public void Declare(string name, ITypeSymbol type, KernelValue? value)
        {
            _variables[name] = (type, value);
            if (_branches.Count > 0)
            {
                _ = _branches.Peek().Add(name);
            }
        }

        public bool TryGet(string name, out ITypeSymbol type, out KernelValue? value)
        {
            var found = _variables.TryGetValue(name, out var variable);
            type = variable.Type;
            value = variable.Value;
            return found;
        }

        /// <summary>Gives a declared variable a new value; null leaves it unassigned.</summary>
        public void Assign(string name, KernelValue? value)
        {
            _variables[name] = (_variables[name].Type, value);
        }

        public void Remove(string name)
        {
            _ = _variables.Remove(name);
        }

        /// <summary>A copy of every variable and its value, in an order that depends only on the kernel's source.</summary>
        public IReadOnlyList<KeyValuePair<string, (ITypeSymbol Type, KernelValue? Value)>> Snapshot()
        {
            return new List<KeyValuePair<string, (ITypeSymbol Type, KernelValue? Value)>>(_variables);
        }

        /// <summary>Puts every variable back as <paramref name="snapshot"/> recorded it.</summary>
        public void Restore(IReadOnlyList<KeyValuePair<string, (ITypeSymbol Type, KernelValue? Value)>> snapshot)
        {
            _variables.Clear();
            foreach (var variable in snapshot)
            {
                _variables[variable.Key] = variable.Value;
            }
        }

        /// <summary>Starts recording the names declared in a branch, which are local to it.</summary>
        public void BeginBranch()
        {
            _branches.Push(new HashSet<string>());
        }

        /// <summary>Stops recording, and returns the names the branch declared (an enclosing branch declared them too).</summary>
        public IReadOnlyCollection<string> EndBranch()
        {
            var declared = _branches.Pop();
            if (_branches.Count > 0)
            {
                _branches.Peek().UnionWith(declared);
            }

            return declared;
        }
    }
}
