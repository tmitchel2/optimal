/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System.Collections.Generic;
using System.Linq;

namespace Optimal.AutoDiff.Analyzers.Kernels
{
    /// <summary>
    /// The array writes of the branches of an <c>if</c> on a run-time condition. Both branches are interpreted from the
    /// same state: the journal records each element a branch writes, with its value before the branch, so the writes can
    /// be undone before the other branch runs and merged afterwards. Branches nest, and arrays a branch creates are local
    /// to it, so their writes are not merged.
    /// </summary>
    public sealed class WriteJournal
    {
        private readonly Stack<Recording> _recordings = new();

        /// <summary>Starts recording a branch.</summary>
        public void Begin()
        {
            _recordings.Push(new Recording());
        }

        /// <summary>Notes an array created while recording; it lives only as long as the branch.</summary>
        public void Created(KernelArray array)
        {
            if (_recordings.Count > 0)
            {
                _ = _recordings.Peek().Created.Add(array);
            }
        }

        /// <summary>Writes an element, recording its old value if a branch is being recorded.</summary>
        public void Store(KernelArray storage, int index, KernelValue? value)
        {
            if (_recordings.Count > 0)
            {
                var recording = _recordings.Peek();
                var key = (storage, index);
                if (!recording.Before.ContainsKey(key))
                {
                    recording.Before[key] = storage[index];
                    recording.Order.Add(key);
                }
            }

            storage[index] = value;
        }

        /// <summary>
        /// Stops recording a branch and undoes its writes, returning each element it wrote (outside the arrays it created)
        /// with the value it left there.
        /// </summary>
        public IReadOnlyList<(KernelArray Storage, int Index, KernelValue? Value)> EndAndUndo()
        {
            var recording = _recordings.Pop();
            if (_recordings.Count > 0)
            {
                _recordings.Peek().Created.UnionWith(recording.Created);
            }

            var written = recording.Order
                .Where(key => !recording.Created.Contains(key.Storage))
                .Select(key => (key.Storage, key.Index, key.Storage[key.Index]))
                .ToList();
            foreach (var key in recording.Order)
            {
                key.Storage[key.Index] = recording.Before[key];
            }

            return written;
        }

        private sealed class Recording
        {
            public Dictionary<(KernelArray Storage, int Index), KernelValue?> Before { get; } = new();

            public List<(KernelArray Storage, int Index)> Order { get; } = new();

            public HashSet<KernelArray> Created { get; } = new();
        }
    }
}
