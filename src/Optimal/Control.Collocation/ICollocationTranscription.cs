/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.Control.Collocation
{
    /// <summary>
    /// Interface for collocation transcription implementations.
    /// Allows different transcription methods (LGL, Hermite-Simpson, etc.) to be used interchangeably.
    /// </summary>
    public interface ICollocationTranscription
    {
        /// <summary>
        /// Gets the total size of the decision vector.
        /// </summary>
        int DecisionVectorSize { get; }

        /// <summary>
        /// Gets the total number of unique collocation points.
        /// </summary>
        int TotalPoints { get; }

        /// <summary>
        /// Gets the number of collocation segments.
        /// </summary>
        int Segments { get; }

        /// <summary>
        /// Gets the LGL order (number of collocation points per segment).
        /// </summary>
        int Order { get; }

        /// <summary>
        /// Extracts state vector at a given global point index from the decision vector.
        /// </summary>
        double[] GetState(double[] z, int globalPointIndex);

        /// <summary>
        /// Extracts control vector at a given global point index from the decision vector.
        /// </summary>
        double[] GetControl(double[] z, int globalPointIndex);

        /// <summary>
        /// Sets state vector at a given global point index in the decision vector.
        /// </summary>
        void SetState(double[] z, int globalPointIndex, double[] state);

        /// <summary>
        /// Sets control vector at a given global point index in the decision vector.
        /// </summary>
        void SetControl(double[] z, int globalPointIndex, double[] control);

        /// <summary>
        /// Creates an initial guess for the decision vector.
        /// </summary>
        double[] CreateInitialGuess(double[] initialState, double[] finalState, double[] constantControl);

        /// <summary>
        /// Computes all defect constraints for the entire trajectory.
        /// </summary>
        double[] ComputeAllDefects(double[] z, Func<double[], double[], double, double[]> dynamicsEvaluator);

        /// <summary>
        /// Computes the running cost integrated over the trajectory.
        /// </summary>
        double ComputeRunningCost(double[] z, Func<double[], double[], double, double> runningCostEvaluator);

        /// <summary>
        /// Computes the terminal cost at the final time.
        /// </summary>
        double ComputeTerminalCost(double[] z, Func<double[], double, double> terminalCostEvaluator);

        /// <summary>
        /// Computes the total objective function value.
        /// </summary>
        double ComputeTotalCost(
            double[] z,
            Func<double[], double[], double, double>? runningCostEvaluator,
            Func<double[], double, double>? terminalCostEvaluator);
    }
}
