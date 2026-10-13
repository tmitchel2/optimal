/**
 * Copyright (c) Small Trading Company Ltd (Destash.com).
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 *
 */

using System;

namespace Optimal.AutoDiff.Tests
{
    /// <summary>
    /// A 3 × 3 tensor of nine double fields, row-major, as an array kernel holds one: the generator flattens it to its
    /// fields and inlines its constructor, members and operators. It is not symmetrised, so a kernel differentiates
    /// with respect to all nine components.
    /// </summary>
    internal struct Tensor3
    {
        public double XX;
        public double XY;
        public double XZ;
        public double YX;
        public double YY;
        public double YZ;
        public double ZX;
        public double ZY;
        public double ZZ;

        public Tensor3(double xx, double xy, double xz, double yx, double yy, double yz, double zx, double zy, double zz)
        {
            XX = xx;
            XY = xy;
            XZ = xz;
            YX = yx;
            YY = yy;
            YZ = yz;
            ZX = zx;
            ZY = zy;
            ZZ = zz;
        }

        public static Tensor3 Identity => new(1, 0, 0, 0, 1, 0, 0, 0, 1);

        public readonly double Trace => XX + YY + ZZ;

        public readonly double Determinant => (XX * ((YY * ZZ) - (YZ * ZY))) - (XY * ((YX * ZZ) - (YZ * ZX))) + (XZ * ((YX * ZY) - (YY * ZX)));

        public static Tensor3 operator +(Tensor3 a, Tensor3 b)
        {
            return new(a.XX + b.XX, a.XY + b.XY, a.XZ + b.XZ, a.YX + b.YX, a.YY + b.YY, a.YZ + b.YZ, a.ZX + b.ZX, a.ZY + b.ZY, a.ZZ + b.ZZ);
        }

        public static Tensor3 operator -(Tensor3 a, Tensor3 b)
        {
            return new(a.XX - b.XX, a.XY - b.XY, a.XZ - b.XZ, a.YX - b.YX, a.YY - b.YY, a.YZ - b.YZ, a.ZX - b.ZX, a.ZY - b.ZY, a.ZZ - b.ZZ);
        }

        public static Tensor3 operator *(double s, Tensor3 a)
        {
            return new(s * a.XX, s * a.XY, s * a.XZ, s * a.YX, s * a.YY, s * a.YZ, s * a.ZX, s * a.ZY, s * a.ZZ);
        }

        /// <summary>The tensor in the nine elements of <paramref name="v"/>, row-major.</summary>
        public static Tensor3 From(ReadOnlySpan<double> v)
        {
            return new(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]);
        }

        /// <summary>Writes the nine components to <paramref name="v"/>, row-major.</summary>
        public readonly void CopyTo(Span<double> v)
        {
            v[0] = XX;
            v[1] = XY;
            v[2] = XZ;
            v[3] = YX;
            v[4] = YY;
            v[5] = YZ;
            v[6] = ZX;
            v[7] = ZY;
            v[8] = ZZ;
        }

        /// <summary>A : B = Σ A_ij B_ij.</summary>
        public readonly double DoubleDot(Tensor3 b)
        {
            return (XX * b.XX) + (XY * b.XY) + (XZ * b.XZ) + (YX * b.YX) + (YY * b.YY) + (YZ * b.YZ) + (ZX * b.ZX) + (ZY * b.ZY) + (ZZ * b.ZZ);
        }

        /// <summary>The Frobenius norm √(A : A).</summary>
        public readonly double Norm()
        {
            return Math.Sqrt(DoubleDot(this));
        }

        /// <summary>A − tr(A)/3 I.</summary>
        public readonly Tensor3 Deviator()
        {
            return this - ((Trace / 3) * Identity);
        }
    }
}
