/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;
using System.Numerics.Tensors;

namespace Optimal.NonLinear.Mma
{
    /// <summary>
    /// Svanberg's primal–dual interior-point method for the MMA subproblem (subsolv, 2007):
    /// minimize f̃_0(x) + a0 z + Σ (c_i y_i + ½ d_i y_i²) subject to f̃_i(x) - a_i z - y_i &lt;= 0, α &lt;= x &lt;= β,
    /// y, z &gt;= 0. The barrier ε runs from 1 down to the tolerance by factors of ten; at each level damped Newton steps,
    /// with a backtracking line search on the residual norm, run until the largest residual is below 0.9ε. Each step's
    /// reduced system is m×m when m &lt; n and n×n otherwise. All storage is allocated once.
    /// </summary>
    internal sealed class MmaSubproblemSolver
    {
        private const int MaxLineSearchSteps = 50;

        private readonly ConstraintMatrix _matrix;
        private readonly SubproblemSettings _settings;
        private readonly ReducedSystemSolver _reduced;
        private readonly bool _hasA;

        // Variable-space work (n).
        private readonly double[] _uxInverse;
        private readonly double[] _xlInverse;
        private readonly double[] _uxInverse2;
        private readonly double[] _xlInverse2;
        private readonly double[] _pLambda;
        private readonly double[] _qLambda;
        private readonly double[] _delX;
        private readonly double[] _diagX;
        private readonly double[] _diagXInverse;
        private readonly double[] _dx;
        private readonly double[] _dXsi;
        private readonly double[] _dEta;
        private readonly double[] _xOld;
        private readonly double[] _xsiOld;
        private readonly double[] _etaOld;
        private readonly double[] _workN;
        private readonly double[] _workN2;

        // Constraint-space work (m).
        private readonly double[] _gVector;
        private readonly double[] _delY;
        private readonly double[] _delLambda;
        private readonly double[] _diagY;
        private readonly double[] _diagLambdaY;
        private readonly double[] _dLambda;
        private readonly double[] _dy;
        private readonly double[] _dMu;
        private readonly double[] _ds;
        private readonly double[] _yOld;
        private readonly double[] _lambdaOld;
        private readonly double[] _muOld;
        private readonly double[] _sOld;
        private readonly double[] _workM;
        private readonly double[] _workM2;

        // Stored like the Jacobian.
        private readonly double[] _g;

        // The problem of the current solve.
        private MmaApproximation? _approximation;
        private double[] _low = [];
        private double[] _upp = [];
        private double[] _alpha = [];
        private double[] _beta = [];
        private double _dz;
        private double _dZeta;

        public MmaSubproblemSolver(ConstraintMatrix matrix, SubproblemSettings settings)
        {
            _matrix = matrix;
            _settings = settings;
            _reduced = new ReducedSystemSolver(
                matrix,
                settings.LinearSolver,
                settings.DirectSolveLimit,
                settings.ConjugateGradientTolerance,
                settings.MaxConjugateGradientIterations);
            _hasA = Array.Exists(settings.A, value => value != 0.0);
            var n = matrix.ColumnCount;
            var m = matrix.RowCount;
            X = new double[n];
            Xsi = new double[n];
            Eta = new double[n];
            Y = new double[m];
            Lambda = new double[m];
            Mu = new double[m];
            S = new double[m];
            _uxInverse = new double[n];
            _xlInverse = new double[n];
            _uxInverse2 = new double[n];
            _xlInverse2 = new double[n];
            _pLambda = new double[n];
            _qLambda = new double[n];
            _delX = new double[n];
            _diagX = new double[n];
            _diagXInverse = new double[n];
            _dx = new double[n];
            _dXsi = new double[n];
            _dEta = new double[n];
            _xOld = new double[n];
            _xsiOld = new double[n];
            _etaOld = new double[n];
            _workN = new double[n];
            _workN2 = new double[n];
            _gVector = new double[m];
            _delY = new double[m];
            _delLambda = new double[m];
            _diagY = new double[m];
            _diagLambdaY = new double[m];
            _dLambda = new double[m];
            _dy = new double[m];
            _dMu = new double[m];
            _ds = new double[m];
            _yOld = new double[m];
            _lambdaOld = new double[m];
            _muOld = new double[m];
            _sOld = new double[m];
            _workM = new double[m];
            _workM2 = new double[m];
            _g = new double[matrix.StoredCount];
        }

        /// <summary>Gets x of the last solve.</summary>
        public double[] X { get; }

        /// <summary>Gets y of the last solve.</summary>
        public double[] Y { get; }

        /// <summary>Gets z of the last solve.</summary>
        public double Z { get; private set; }

        /// <summary>Gets λ of the last solve.</summary>
        public double[] Lambda { get; }

        /// <summary>Gets ξ (multipliers of x &gt;= α) of the last solve.</summary>
        public double[] Xsi { get; }

        /// <summary>Gets η (multipliers of x &lt;= β) of the last solve.</summary>
        public double[] Eta { get; }

        /// <summary>Gets μ (multipliers of y &gt;= 0) of the last solve.</summary>
        public double[] Mu { get; }

        /// <summary>Gets ζ (the multiplier of z &gt;= 0) of the last solve.</summary>
        public double Zeta { get; private set; }

        /// <summary>Gets s (the slacks) of the last solve.</summary>
        public double[] S { get; }

        /// <summary>Gets the Newton steps of the last solve.</summary>
        public int NewtonIterations { get; private set; }

        /// <summary>Gets the residual evaluations of the last solve (one per line-search trial, plus one per barrier level).</summary>
        public int ResidualEvaluations { get; private set; }

        /// <summary>Gets the conjugate-gradient iterations of the last solve.</summary>
        public int LinearIterations => _reduced.LinearIterations;

        /// <summary>Gets the conjugate-gradient solves of the last solve that hit their cap.</summary>
        public int UnconvergedLinearSolves => _reduced.UnconvergedSolves;

        /// <summary>Gets whether the reduced system is m×m.</summary>
        public bool ConstraintSpace => _reduced.ConstraintSpace;

        /// <summary>Gets whether the reduced system is solved directly.</summary>
        public bool DirectLinearSolve => _reduced.IsDirect;

        /// <summary>
        /// Solves the subproblem for the approximation on the box [α, β] between the asymptotes. Returns false on a
        /// numerical failure (a non-finite residual or a reduced matrix that is not positive definite).
        /// </summary>
        public bool Solve(MmaApproximation approximation, double[] low, double[] upp, double[] alpha, double[] beta)
        {
            _approximation = approximation;
            _low = low;
            _upp = upp;
            _alpha = alpha;
            _beta = beta;
            NewtonIterations = 0;
            ResidualEvaluations = 0;
            _reduced.ResetStatistics();
            Initialise();
            var epsilon = 1.0;
            while (epsilon > _settings.Tolerance)
            {
                var norm = Residual(epsilon, out var max);
                if (!double.IsFinite(norm))
                {
                    return false;
                }

                for (var iteration = 0; max > 0.9 * epsilon && iteration < _settings.MaxNewtonIterations; iteration++)
                {
                    NewtonIterations++;
                    if (!NewtonDirection(epsilon))
                    {
                        return false;
                    }

                    norm = LineSearch(epsilon, norm, out max);
                    if (!double.IsFinite(norm))
                    {
                        return false;
                    }
                }

                epsilon *= 0.1;
            }

            return true;
        }

        private MmaApproximation Approximation => _approximation!;

        private void Initialise()
        {
            var c = _settings.C;
            for (var j = 0; j < X.Length; j++)
            {
                X[j] = 0.5 * (_alpha[j] + _beta[j]);
                Xsi[j] = Math.Max(1.0 / (X[j] - _alpha[j]), 1.0);
                Eta[j] = Math.Max(1.0 / (_beta[j] - X[j]), 1.0);
            }

            for (var i = 0; i < Y.Length; i++)
            {
                Y[i] = 1.0;
                Lambda[i] = 1.0;
                Mu[i] = Math.Max(1.0, 0.5 * c[i]);
                S[i] = 1.0;
            }

            Z = 1.0;
            Zeta = 1.0;
        }

        /// <summary>
        /// Computes p_λ = p0 + Pᵀλ, q_λ = q0 + Qᵀλ, 1/(U - x), 1/(x - L) and g(x) = P/(U - x) + Q/(x - L) at the current x and λ.
        /// </summary>
        private void Evaluate()
        {
            var approximation = Approximation;
            for (var j = 0; j < X.Length; j++)
            {
                _uxInverse[j] = 1.0 / (_upp[j] - X[j]);
                _xlInverse[j] = 1.0 / (X[j] - _low[j]);
            }

            approximation.P0.CopyTo(_pLambda, 0);
            approximation.Q0.CopyTo(_qLambda, 0);
            _matrix.TransposeAddAndMultiplyPair(
                approximation.P, approximation.Q, Lambda, _pLambda, _qLambda, _uxInverse, _xlInverse, _gVector);
        }

        /// <summary>
        /// The residual of the perturbed KKT conditions at barrier ε: its Euclidean norm, and its largest component.
        /// </summary>
        private double Residual(double epsilon, out double max)
        {
            ResidualEvaluations++;
            Evaluate();
            var a = _settings.A;
            var c = _settings.C;
            var d = _settings.D;
            var b = Approximation.B;
            var sum = 0.0;
            max = 0.0;
            for (var j = 0; j < X.Length; j++)
            {
                var rex = (_pLambda[j] * _uxInverse[j] * _uxInverse[j]) - (_qLambda[j] * _xlInverse[j] * _xlInverse[j]) - Xsi[j] + Eta[j];
                var rexsi = (Xsi[j] * (X[j] - _alpha[j])) - epsilon;
                var reeta = (Eta[j] * (_beta[j] - X[j])) - epsilon;
                sum += (rex * rex) + (rexsi * rexsi) + (reeta * reeta);
                max = Math.Max(max, Math.Max(Math.Abs(rex), Math.Max(Math.Abs(rexsi), Math.Abs(reeta))));
            }

            var aLambda = 0.0;
            for (var i = 0; i < Y.Length; i++)
            {
                var rey = c[i] + (d[i] * Y[i]) - Mu[i] - Lambda[i];
                var relam = _gVector[i] - (a[i] * Z) - Y[i] + S[i] - b[i];
                var remu = (Mu[i] * Y[i]) - epsilon;
                var res = (Lambda[i] * S[i]) - epsilon;
                aLambda += a[i] * Lambda[i];
                sum += (rey * rey) + (relam * relam) + (remu * remu) + (res * res);
                max = Math.Max(max, Math.Max(Math.Max(Math.Abs(rey), Math.Abs(relam)), Math.Max(Math.Abs(remu), Math.Abs(res))));
            }

            var rez = _settings.A0 - Zeta - aLambda;
            var rezet = (Zeta * Z) - epsilon;
            sum += (rez * rez) + (rezet * rezet);
            max = Math.Max(max, Math.Max(Math.Abs(rez), Math.Abs(rezet)));
            if (!double.IsFinite(sum))
            {
                max = double.NaN;
                return double.NaN;
            }

            return Math.Sqrt(sum);
        }

        /// <summary>
        /// The Newton direction of the perturbed KKT system at barrier ε, with the slacks and bound multipliers
        /// eliminated and the remaining system reduced to λ (m &lt; n) or x (otherwise).
        /// </summary>
        private bool NewtonDirection(double epsilon)
        {
            // p_λ, q_λ, g(x) and the reciprocals are current: the last Residual call evaluated them at this point.
            var approximation = Approximation;
            var a = _settings.A;
            var c = _settings.C;
            var d = _settings.D;
            for (var j = 0; j < X.Length; j++)
            {
                var uxInverse = _uxInverse[j];
                var xlInverse = _xlInverse[j];
                _uxInverse2[j] = uxInverse * uxInverse;
                _xlInverse2[j] = xlInverse * xlInverse;
                var xAlpha = X[j] - _alpha[j];
                var betaX = _beta[j] - X[j];
                var dpsidx = (_pLambda[j] * _uxInverse2[j]) - (_qLambda[j] * _xlInverse2[j]);
                _delX[j] = dpsidx - (epsilon / xAlpha) + (epsilon / betaX);
                var curvature = (_pLambda[j] * _uxInverse2[j] * uxInverse) + (_qLambda[j] * _xlInverse2[j] * xlInverse);
                _diagX[j] = (2.0 * curvature) + (Xsi[j] / xAlpha) + (Eta[j] / betaX);
                _diagXInverse[j] = 1.0 / _diagX[j];
            }

            FormG(approximation);
            var aLambda = 0.0;
            for (var i = 0; i < Y.Length; i++)
            {
                _delY[i] = c[i] + (d[i] * Y[i]) - Lambda[i] - (epsilon / Y[i]);
                _delLambda[i] = _gVector[i] - (a[i] * Z) - Y[i] - approximation.B[i] + (epsilon / Lambda[i]);
                _diagY[i] = d[i] + (Mu[i] / Y[i]);
                _diagLambdaY[i] = (S[i] / Lambda[i]) + (1.0 / _diagY[i]);
                aLambda += a[i] * Lambda[i];
            }

            var delZ = _settings.A0 - aLambda - (epsilon / Z);
            var solved = _reduced.ConstraintSpace ? ConstraintSpaceStep(delZ) : VariableSpaceStep(delZ);
            if (!solved)
            {
                return false;
            }

            for (var i = 0; i < Y.Length; i++)
            {
                _dy[i] = (_dLambda[i] - _delY[i]) / _diagY[i];
                _dMu[i] = -Mu[i] + (epsilon / Y[i]) - (Mu[i] * _dy[i] / Y[i]);
                _ds[i] = -S[i] + (epsilon / Lambda[i]) - (S[i] * _dLambda[i] / Lambda[i]);
            }

            for (var j = 0; j < X.Length; j++)
            {
                var xAlpha = X[j] - _alpha[j];
                var betaX = _beta[j] - X[j];
                _dXsi[j] = -Xsi[j] + (epsilon / xAlpha) - (Xsi[j] * _dx[j] / xAlpha);
                _dEta[j] = -Eta[j] + (epsilon / betaX) + (Eta[j] * _dx[j] / betaX);
            }

            _dZeta = -Zeta + (epsilon / Z) - (Zeta * _dz / Z);
            return true;
        }

        /// <summary>
        /// G = P diag(1/(U - x)²) - Q diag(1/(x - L)²), the Jacobian of the approximated constraints.
        /// </summary>
        private void FormG(MmaApproximation approximation)
        {
            var p = approximation.P;
            var q = approximation.Q;
            for (var i = 0; i < _matrix.RowCount; i++)
            {
                var offset = _matrix.RowOffset(i);
                var length = _matrix.RowLength(i);
                if (_matrix.IsDense(i))
                {
                    var row = _g.AsSpan(offset, length);
                    TensorPrimitives.Multiply(p.AsSpan(offset, length), _uxInverse2, row);
                    TensorPrimitives.Multiply(q.AsSpan(offset, length), _xlInverse2, _workN);
                    TensorPrimitives.Subtract(row, _workN, row);
                    continue;
                }

                var columns = _matrix.RowColumns(i);
                for (var t = 0; t < length; t++)
                {
                    var j = columns[t];
                    _g[offset + t] = (p[offset + t] * _uxInverse2[j]) - (q[offset + t] * _xlInverse2[j]);
                }
            }
        }

        /// <summary>
        /// m &lt; n: [Alam a; aᵀ -ζ/z] [Δλ; Δz] = [b_λ; δz], with Alam = diag(D_λy) + G diag(1/D_x) Gᵀ, by block
        /// elimination; then Δx = -(δx + GᵀΔλ)/D_x.
        /// </summary>
        private bool ConstraintSpaceStep(double delZ)
        {
            var a = _settings.A;
            var rhs = _workM;
            TensorPrimitives.Multiply(_delX, _diagXInverse, _workN);
            _matrix.Multiply(_g, _workN, rhs);
            for (var i = 0; i < rhs.Length; i++)
            {
                rhs[i] = _delLambda[i] + (_delY[i] / _diagY[i]) - rhs[i];
            }

            if (!_reduced.Prepare(_g, _diagLambdaY, _diagXInverse))
            {
                return false;
            }

            _reduced.Solve(rhs, _dLambda);
            if (_hasA)
            {
                _reduced.Solve(a, _workM2);
                var aP = TensorPrimitives.Dot(a, _dLambda);
                var aQ = TensorPrimitives.Dot(a, _workM2);
                _dz = (aP - delZ) / (aQ + (Zeta / Z));
                TensorPrimitives.MultiplyAdd(_workM2, -_dz, _dLambda, _dLambda);
            }
            else
            {
                _dz = -delZ * Z / Zeta;
            }

            Array.Clear(_workN);
            _matrix.MultiplyTransposeAdd(_g, _dLambda, _workN);
            for (var j = 0; j < X.Length; j++)
            {
                _dx[j] = -(_delX[j] + _workN[j]) * _diagXInverse[j];
            }

            return true;
        }

        /// <summary>
        /// m &gt;= n: [Axx axz; axzᵀ azz] [Δx; Δz] = [-b_x; -b_z], with Axx = diag(D_x) + Gᵀ diag(1/D_λy) G, by block
        /// elimination; then Δλ = (GΔx - Δz a + δ_λy)/D_λy.
        /// </summary>
        private bool VariableSpaceStep(double delZ)
        {
            var a = _settings.A;
            var weights = _workM;
            var scaled = _workM2;
            var aScaled = 0.0;
            var bz = delZ;
            for (var i = 0; i < Y.Length; i++)
            {
                weights[i] = 1.0 / _diagLambdaY[i];
                var delLambdaY = _delLambda[i] + (_delY[i] / _diagY[i]);
                _dLambda[i] = delLambdaY;
                scaled[i] = delLambdaY * weights[i];
                aScaled += a[i] * a[i] * weights[i];
                bz -= a[i] * scaled[i];
            }

            // -b_x = -(δx + Gᵀ(δ_λy/D_λy)), into _workN.
            Array.Clear(_workN);
            _matrix.MultiplyTransposeAdd(_g, scaled, _workN);
            for (var j = 0; j < X.Length; j++)
            {
                _workN[j] = -(_delX[j] + _workN[j]);
            }

            if (!_reduced.Prepare(_g, _diagX, weights))
            {
                return false;
            }

            _reduced.Solve(_workN, _dx);
            var azz = (Zeta / Z) + aScaled;
            if (_hasA)
            {
                // axz = -Gᵀ(a/D_λy), into _workN2; then Axx q = axz into _workN.
                for (var i = 0; i < Y.Length; i++)
                {
                    scaled[i] = a[i] * weights[i];
                }

                Array.Clear(_workN2);
                _matrix.MultiplyTransposeAdd(_g, scaled, _workN2);
                TensorPrimitives.Negate(_workN2, _workN2);
                _reduced.Solve(_workN2, _workN);
                var axzP = TensorPrimitives.Dot(_workN2, _dx);
                var axzQ = TensorPrimitives.Dot(_workN2, _workN);
                _dz = (-bz - axzP) / (azz - axzQ);
                TensorPrimitives.MultiplyAdd(_workN, -_dz, _dx, _dx);
            }
            else
            {
                _dz = -bz / azz;
            }

            _matrix.Multiply(_g, _dx, scaled);
            for (var i = 0; i < Y.Length; i++)
            {
                _dLambda[i] = (scaled[i] - (_dz * a[i]) + _dLambda[i]) * weights[i];
            }

            return true;
        }

        /// <summary>
        /// Takes the longest step (to 1/1.01 of the boundary, at most 1) that keeps the iterate interior, then halves it
        /// until the residual norm does not grow (at most 50 times). Returns the new residual norm.
        /// </summary>
        private double LineSearch(double epsilon, double norm, out double max)
        {
            var step = 1.0 / StepBound();
            X.CopyTo(_xOld, 0);
            Xsi.CopyTo(_xsiOld, 0);
            Eta.CopyTo(_etaOld, 0);
            Y.CopyTo(_yOld, 0);
            Lambda.CopyTo(_lambdaOld, 0);
            Mu.CopyTo(_muOld, 0);
            S.CopyTo(_sOld, 0);
            var zOld = Z;
            var zetaOld = Zeta;
            var newNorm = 2.0 * norm;
            max = double.NaN;
            for (var attempt = 0; attempt < MaxLineSearchSteps && !(newNorm <= norm); attempt++)
            {
                TensorPrimitives.MultiplyAdd(_dx, step, _xOld, X);
                TensorPrimitives.MultiplyAdd(_dXsi, step, _xsiOld, Xsi);
                TensorPrimitives.MultiplyAdd(_dEta, step, _etaOld, Eta);
                TensorPrimitives.MultiplyAdd(_dy, step, _yOld, Y);
                TensorPrimitives.MultiplyAdd(_dLambda, step, _lambdaOld, Lambda);
                TensorPrimitives.MultiplyAdd(_dMu, step, _muOld, Mu);
                TensorPrimitives.MultiplyAdd(_ds, step, _sOld, S);
                Z = zOld + (step * _dz);
                Zeta = zetaOld + (step * _dZeta);
                newNorm = Residual(epsilon, out max);
                step *= 0.5;
            }

            return newNorm;
        }

        /// <summary>
        /// The reciprocal of the step to the boundary of the positive variables and of [α, β], scaled by 1.01, and at
        /// least 1.
        /// </summary>
        private double StepBound()
        {
            var bound = 1.0;
            bound = Math.Max(bound, BoundOf(_dy, Y));
            bound = Math.Max(bound, BoundOf(_dLambda, Lambda));
            bound = Math.Max(bound, BoundOf(_dMu, Mu));
            bound = Math.Max(bound, BoundOf(_ds, S));
            bound = Math.Max(bound, BoundOf(_dXsi, Xsi));
            bound = Math.Max(bound, BoundOf(_dEta, Eta));
            bound = Math.Max(bound, -1.01 * _dz / Z);
            bound = Math.Max(bound, -1.01 * _dZeta / Zeta);
            for (var j = 0; j < X.Length; j++)
            {
                bound = Math.Max(bound, -1.01 * _dx[j] / (X[j] - _alpha[j]));
                bound = Math.Max(bound, 1.01 * _dx[j] / (_beta[j] - X[j]));
            }

            return bound;
        }

        private static double BoundOf(ReadOnlySpan<double> direction, ReadOnlySpan<double> value)
        {
            var bound = double.NegativeInfinity;
            for (var k = 0; k < value.Length; k++)
            {
                bound = Math.Max(bound, -1.01 * direction[k] / value[k]);
            }

            return bound;
        }
    }
}
