/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control
{
    /// <summary>
    /// Detects whether a control problem provides analytical gradients for dynamics and costs.
    /// </summary>
    internal static class GradientCapabilityDetector
    {
        /// <summary>
        /// Checks if the dynamics function provides valid analytical gradients.
        /// </summary>
        /// <param name="dynamics">The dynamics function to test.</param>
        /// <param name="stateDim">Expected state dimension.</param>
        /// <param name="controlDim">Expected control dimension.</param>
        /// <returns>True if analytical gradients are properly provided, false otherwise.</returns>
        public static bool HasAnalyticalDynamicsGradients(
            Func<double[], double[], double, (double[] value, double[][]? gradients)> dynamics,
            int stateDim,
            int controlDim)
        {
            if (dynamics == null)
            {
                return false;
            }

            try
            {
                var testX = new double[stateDim];
                var testU = new double[controlDim];
                var testResult = dynamics(testX, testU, 0.0);

                // Check if gradients are not only present but also properly populated
                if (testResult.gradients == null || testResult.gradients.Length < 2)
                {
                    return false;
                }

                var gradWrtState = testResult.gradients[0];
                var gradWrtControl = testResult.gradients[1];

                // Verify gradients are the right size
                var expectedStateGradSize = stateDim * stateDim;
                var expectedControlGradSize = stateDim * controlDim;

                return gradWrtState != null && gradWrtState.Length == expectedStateGradSize &&
                       gradWrtControl != null && gradWrtControl.Length == expectedControlGradSize;
            }
            catch
            {
                // If evaluation fails, fall back to numerical gradients
                return false;
            }
        }

        /// <summary>
        /// Checks if the running cost function provides valid analytical gradients.
        /// </summary>
        /// <param name="runningCost">The running cost function to test.</param>
        /// <param name="stateDim">Expected state dimension.</param>
        /// <param name="controlDim">Expected control dimension.</param>
        /// <returns>True if analytical gradients are properly provided, false otherwise.</returns>
        public static bool HasAnalyticalRunningCostGradients(
            Func<double[], double[], double, (double value, double[] gradients)>? runningCost,
            int stateDim,
            int controlDim)
        {
            if (runningCost == null)
            {
                return true; // No cost means no gradients needed
            }

            try
            {
                var testX = new double[stateDim];
                var testU = new double[controlDim];
                var testResult = runningCost(testX, testU, 0.0);

                // gradients array for running cost should be: [∂L/∂x (StateDim), ∂L/∂u (ControlDim), ∂L/∂t]
                var expectedSize = stateDim + controlDim + 1;
                return testResult.gradients.Length >= expectedSize;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Checks if the terminal cost function provides valid analytical gradients.
        /// </summary>
        /// <param name="terminalCost">The terminal cost function to test.</param>
        /// <param name="stateDim">Expected state dimension.</param>
        /// <returns>True if analytical gradients are properly provided, false otherwise.</returns>
        public static bool HasAnalyticalTerminalCostGradients(
            Func<double[], double, (double value, double[] gradients)>? terminalCost,
            int stateDim)
        {
            if (terminalCost == null)
            {
                return true; // No cost means no gradients needed
            }

            try
            {
                var testX = new double[stateDim];
                var testResult = terminalCost(testX, 0.0);

                // gradients array for terminal cost should be: [∂Φ/∂x (StateDim), ∂Φ/∂t]
                var expectedSize = stateDim + 1;
                return testResult.gradients.Length >= expectedSize;
            }
            catch
            {
                return false;
            }
        }

        /// <summary>
        /// Checks if both running and terminal cost functions provide valid analytical gradients.
        /// </summary>
        /// <param name="runningCost">The running cost function to test.</param>
        /// <param name="terminalCost">The terminal cost function to test.</param>
        /// <param name="stateDim">Expected state dimension.</param>
        /// <param name="controlDim">Expected control dimension.</param>
        /// <returns>True if all applicable cost gradients are properly provided, false otherwise.</returns>
        public static bool HasAnalyticalCostGradients(
            Func<double[], double[], double, (double value, double[] gradients)>? runningCost,
            Func<double[], double, (double value, double[] gradients)>? terminalCost,
            int stateDim,
            int controlDim)
        {
            var hasRunningCostGradients = HasAnalyticalRunningCostGradients(runningCost, stateDim, controlDim);
            var hasTerminalCostGradients = HasAnalyticalTerminalCostGradients(terminalCost, stateDim);

            return hasRunningCostGradients && hasTerminalCostGradients;
        }
    }
}
