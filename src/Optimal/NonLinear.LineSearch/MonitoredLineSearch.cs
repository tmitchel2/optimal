/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using Optimal.NonLinear.Monitoring;

namespace Optimal.NonLinear.LineSearch
{
    /// <summary>
    /// Decorator that wraps a line search to capture trial points for smoothness monitoring.
    /// </summary>
    public sealed class MonitoredLineSearch : ILineSearch
    {
        private readonly ILineSearch _inner;
        private readonly OptimisationMonitor _monitor;

        /// <summary>
        /// Initializes a new instance of the <see cref="MonitoredLineSearch"/> class.
        /// </summary>
        /// <param name="inner">The inner line search to wrap.</param>
        /// <param name="monitor">The optimization monitor to report to.</param>
        public MonitoredLineSearch(ILineSearch inner, OptimisationMonitor monitor)
        {
            _inner = inner ?? throw new ArgumentNullException(nameof(inner));
            _monitor = monitor ?? throw new ArgumentNullException(nameof(monitor));
        }

        /// <inheritdoc/>
        public double FindStepSize(
            Func<double[], (double value, double[] gradient)> objective,
            double[] x,
            double fx,
            double[] gradient,
            double[] direction,
            double initialStepSize)
        {
            // Notify monitor that line search is starting
            _monitor.OnLineSearchStart(x, direction);

            // Record the initial point (alpha = 0)
            _monitor.OnLineSearchStep(new LineSearchStepInfo
            {
                Alpha = 0.0,
                FunctionValue = fx,
                Gradient = (double[])gradient.Clone()
            });

            // Wrap the objective to intercept each evaluation
            (double value, double[] grad) MonitoredObjective(double[] xNew)
            {
                var result = objective(xNew);

                // Calculate alpha from xNew = x + alpha * direction
                var alpha = ComputeAlpha(x, xNew, direction);

                _monitor.OnLineSearchStep(new LineSearchStepInfo
                {
                    Alpha = alpha,
                    FunctionValue = result.value,
                    Gradient = (double[])result.gradient.Clone()
                });

                return result;
            }

            // Delegate to inner line search
            var stepSize = _inner.FindStepSize(
                MonitoredObjective,
                x,
                fx,
                gradient,
                direction,
                initialStepSize);

            // Notify monitor that line search has ended
            _monitor.OnLineSearchEnd();

            return stepSize;
        }

        /// <summary>
        /// Computes the step size alpha given x, xNew, and direction where xNew = x + alpha * direction.
        /// </summary>
        private static double ComputeAlpha(double[] x, double[] xNew, double[] direction)
        {
            // Find alpha from first non-zero direction component
            for (var i = 0; i < direction.Length; i++)
            {
                if (Math.Abs(direction[i]) > 1e-15)
                {
                    return (xNew[i] - x[i]) / direction[i];
                }
            }
            return 0.0;
        }
    }
}
