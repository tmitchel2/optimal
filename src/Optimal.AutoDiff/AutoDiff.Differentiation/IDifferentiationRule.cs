/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Optimal.AutoDiff.Analyzers.IR;

namespace Optimal.AutoDiff.Analyzers.Differentiation
{
    public interface IDifferentiationRule
    {
        bool CanDifferentiate(IRNode node);
        IRNode DifferentiateForward(IRNode node, ForwardModeContext context);
    }

    public sealed class ForwardModeContext
    {
        private readonly Dictionary<string, IRNode> _tangents = new();
        private int _nodeIdCounter;

        public ForwardModeContext(int startNodeId, string wrtParameter)
        {
            _nodeIdCounter = startNodeId;
            WithRespectTo = wrtParameter;
        }

        public string WithRespectTo { get; }

        public void SetTangent(string variable, IRNode tangent)
        {
            _tangents[variable] = tangent;
        }

        public IRNode GetTangent(string variable)
        {
            if (_tangents.TryGetValue(variable, out var tangent))
            {
                return tangent;
            }

            throw new InvalidOperationException($"No tangent defined for variable: {variable}");
        }

        public bool HasTangent(string variable)
        {
            return _tangents.ContainsKey(variable);
        }

        public int NewNodeId() => _nodeIdCounter++;

        /// <summary>
        /// Snapshot the current tangent bindings. Used by the IR expander to
        /// implement SSA-style scoping around if/else branches: take a
        /// snapshot before expanding each branch, restore between branches so
        /// they start from the same pre-branch state, then merge after.
        /// </summary>
        public Dictionary<string, IRNode> SnapshotTangents()
        {
            return new Dictionary<string, IRNode>(_tangents);
        }

        /// <summary>
        /// Restore tangent bindings from a previous snapshot. Used between
        /// branches in <c>ForwardModeIRExpander</c>'s ConditionalNode arm.
        /// </summary>
        public void RestoreTangents(Dictionary<string, IRNode> snapshot)
        {
            _tangents.Clear();
            foreach (var kv in snapshot)
            {
                _tangents[kv.Key] = kv.Value;
            }
        }
    }
}
