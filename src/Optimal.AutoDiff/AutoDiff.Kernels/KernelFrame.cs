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
    /// </summary>
    public sealed class KernelFrame
    {
        private readonly Dictionary<string, (ITypeSymbol Type, KernelValue? Value)> _variables = new();

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
        }

        public bool TryGet(string name, out ITypeSymbol type, out KernelValue? value)
        {
            var found = _variables.TryGetValue(name, out var variable);
            type = variable.Type;
            value = variable.Value;
            return found;
        }

        public void Assign(string name, KernelValue value)
        {
            _variables[name] = (_variables[name].Type, value);
        }

        public void Remove(string name)
        {
            _ = _variables.Remove(name);
        }
    }
}
