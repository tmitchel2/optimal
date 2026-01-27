/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Collections.Generic;
using Optimal.NonLinear.Constraints;

namespace Optimal.NonLinear.Monitoring
{
    /// <summary>
    /// Verifies analytic gradients against numerical differentiation using 4-point central difference.
    /// </summary>
    internal sealed class GradientVerifier
    {
        private readonly double _stepSize;
        private const double RelativeTolerance = 1e-4;
        private const double AbsoluteTolerance = 1e-8;

        private int _functionEvaluations;

        /// <summary>
        /// Initializes a new instance of the <see cref="GradientVerifier"/> class.
        /// </summary>
        /// <param name="stepSize">Step size for finite differences.</param>
        public GradientVerifier(double stepSize)
        {
            _stepSize = stepSize;
            _functionEvaluations = 0;
        }

        /// <summary>
        /// Gets the total number of function evaluations performed.
        /// </summary>
        public int FunctionEvaluations => _functionEvaluations;

        /// <summary>
        /// Verifies the gradient of an objective function.
        /// </summary>
        /// <param name="objective">Objective function returning (value, gradient).</param>
        /// <param name="testPoint">Point at which to verify the gradient.</param>
        /// <returns>Gradient verification result.</returns>
        public GradientVerificationResult VerifyObjectiveGradient(
            Func<double[], (double value, double[] gradient)> objective,
            double[] testPoint)
        {
            var (_, analyticGradient) = objective(testPoint);
            _functionEvaluations++;

            double ValueOnly(double[] x)
            {
                var (value, _) = objective(x);
                _functionEvaluations++;
                return value;
            }

            var numericalGradient = ComputeFourPointGradient(ValueOnly, testPoint);

            return CompareGradients(
                analyticGradient,
                numericalGradient,
                testPoint,
                functionIndex: -1);
        }

        /// <summary>
        /// Verifies gradients of all constraints.
        /// </summary>
        /// <param name="constraints">List of constraints to verify.</param>
        /// <param name="testPoint">Point at which to verify gradients.</param>
        /// <returns>List of gradient verification results, one per constraint.</returns>
        public IReadOnlyList<GradientVerificationResult> VerifyConstraintGradients(
            IReadOnlyList<IConstraint> constraints,
            double[] testPoint)
        {
            var results = new List<GradientVerificationResult>();

            for (var i = 0; i < constraints.Count; i++)
            {
                var constraint = constraints[i];
                var (_, analyticGradient) = constraint.Evaluate(testPoint);
                _functionEvaluations++;

                double ValueOnly(double[] x)
                {
                    var (value, _) = constraint.Evaluate(x);
                    _functionEvaluations++;
                    return value;
                }

                var numericalGradient = ComputeFourPointGradient(ValueOnly, testPoint);

                results.Add(CompareGradients(
                    analyticGradient,
                    numericalGradient,
                    testPoint,
                    functionIndex: i));
            }

            return results;
        }

        /// <summary>
        /// Computes gradient using 4-point central difference formula.
        /// f'(x) = (-f(x+2h) + 8*f(x+h) - 8*f(x-h) + f(x-2h)) / (12h)
        /// This formula has error O(h^4) compared to O(h^2) for standard central difference.
        /// </summary>
        private double[] ComputeFourPointGradient(Func<double[], double> f, double[] x)
        {
            var n = x.Length;
            var gradient = new double[n];
            var xWork = (double[])x.Clone();
            var h = _stepSize;

            for (var i = 0; i < n; i++)
            {
                var original = xWork[i];

                // Scale step by magnitude of x[i] for better numerical conditioning
                var scaledH = h * Math.Max(1.0, Math.Abs(original));

                // f(x + 2h)
                xWork[i] = original + 2 * scaledH;
                var f2Plus = f(xWork);

                // f(x + h)
                xWork[i] = original + scaledH;
                var fPlus = f(xWork);

                // f(x - h)
                xWork[i] = original - scaledH;
                var fMinus = f(xWork);

                // f(x - 2h)
                xWork[i] = original - 2 * scaledH;
                var f2Minus = f(xWork);

                // 4-point central difference formula
                gradient[i] = (-f2Plus + 8 * fPlus - 8 * fMinus + f2Minus) / (12 * scaledH);

                // Restore original value
                xWork[i] = original;
            }

            return gradient;
        }

        private static GradientVerificationResult CompareGradients(
            double[] analytic,
            double[] numerical,
            double[] testPoint,
            int functionIndex)
        {
            var n = analytic.Length;
            var maxRelativeError = 0.0;
            var worstIndex = -1;
            var errors = new double[n];
            var isSuspicious = false;

            for (var i = 0; i < n; i++)
            {
                var absError = Math.Abs(analytic[i] - numerical[i]);
                var scale = Math.Max(Math.Abs(analytic[i]), Math.Abs(numerical[i]));
                var relError = scale > AbsoluteTolerance
                    ? absError / scale
                    : absError;

                errors[i] = relError;

                if (relError > maxRelativeError)
                {
                    maxRelativeError = relError;
                    worstIndex = i;
                }

                // Mark suspicious if relative error exceeds tolerance
                if (relError > RelativeTolerance && absError > AbsoluteTolerance)
                {
                    isSuspicious = true;
                }
            }

            return new GradientVerificationResult
            {
                IsSuspicious = isSuspicious,
                FunctionIndex = functionIndex,
                WorstVariableIndex = worstIndex,
                TestPoint = (double[])testPoint.Clone(),
                AnalyticGradient = (double[])analytic.Clone(),
                NumericalGradient = (double[])numerical.Clone(),
                RelativeErrors = errors,
                MaxRelativeError = maxRelativeError
            };
        }
    }
}
