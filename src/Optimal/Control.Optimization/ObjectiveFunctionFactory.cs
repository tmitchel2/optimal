/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using Optimal.Control.Collocation;
using Optimal.Control.Core;

namespace Optimal.Control.Optimization
{
    /// <summary>
    /// Factory for creating objective functions for collocation-based optimal control solvers.
    /// </summary>
    internal static class ObjectiveFunctionFactory
    {
        /// <summary>
        /// Computes the total cost (running + terminal) for the given decision vector.
        /// </summary>
        public static double ComputeTotalCost(ControlProblem problem, ParallelHermiteSimpsonTranscription transcription, double[] z)
        {
            var cost = 0.0;

            if (problem.RunningCost != null)
            {
                double RunningCostValue(double[] x, double[] u, double t) => problem.RunningCost(x, u, t).value;
                cost += transcription.ComputeRunningCost(z, RunningCostValue);
            }

            if (problem.TerminalCost != null)
            {
                double TerminalCostValue(double[] x, double t) => problem.TerminalCost(new TerminalCostInput(x, t)).Value;
                cost += transcription.ComputeTerminalCost(z, TerminalCostValue);
            }

            return cost;
        }

        /// <summary>
        /// Computes the gradient of the objective function.
        /// </summary>
        public static double[] ComputeObjectiveGradient(
            ControlProblem problem,
            CollocationGrid grid,
            ParallelHermiteSimpsonTranscription transcription,
            double[] z)
        {
            if (!CanUseAnalyticalGradientsForCosts(problem))
            {
                return NumericalGradients.ComputeGradient(zz => ComputeTotalCost(problem, transcription, zz), z);
            }

            var gradient = new double[z.Length];

            if (problem.RunningCost != null)
            {
                var runningGrad = AutoDiffGradientHelper.ComputeRunningCostGradient(
                    problem, grid, z, transcription.GetState, transcription.GetControl,
                    (x, u, t) =>
                    {
                        var res = problem.RunningCost!(x, u, t);
                        return (res.value, res.gradients!);
                    });

                for (var i = 0; i < gradient.Length; i++)
                {
                    gradient[i] += runningGrad[i];
                }
            }

            if (problem.TerminalCost != null)
            {
                var terminalGrad = AutoDiffGradientHelper.ComputeTerminalCostGradient(
                    problem, grid, z, transcription.GetState,
                    (x, t) =>
                    {
                        var res = problem.TerminalCost!(new TerminalCostInput(x, t));
                        return (res.Value, res.Gradients!);
                    });

                for (var i = 0; i < gradient.Length; i++)
                {
                    gradient[i] += terminalGrad[i];
                }
            }

            return gradient;
        }

        /// <summary>
        /// Checks if analytical gradients can be used for cost computations.
        /// </summary>
        public static bool CanUseAnalyticalGradientsForCosts(ControlProblem problem)
        {
            var useAnalytical = true;

            if (problem.RunningCost != null)
            {
                try
                {
                    var testResult = problem.RunningCost(new double[problem.StateDim], new double[problem.ControlDim], 0.0);
                    var expectedSize = problem.StateDim + problem.ControlDim + 1;
                    useAnalytical = useAnalytical && testResult.gradients != null && testResult.gradients.Length >= expectedSize;
                }
                catch
                {
                    useAnalytical = false;
                }
            }

            if (problem.TerminalCost != null)
            {
                try
                {
                    var testResult = problem.TerminalCost(new TerminalCostInput(new double[problem.StateDim], 0.0));
                    var expectedSize = problem.StateDim + 1;
                    useAnalytical = useAnalytical && testResult.Gradients != null && testResult.Gradients.Length >= expectedSize;
                }
                catch
                {
                    useAnalytical = false;
                }
            }

            return useAnalytical;
        }
    }
}
