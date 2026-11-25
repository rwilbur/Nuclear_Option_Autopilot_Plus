using System;

namespace AutopilotMod.Fbw.Model
{
    /// <summary>
    /// Minimal 3-vector for the FBW's pure maths (the game ships no System.Numerics.Vectors, and the offline
    /// checks run without Unity). Same formulas as UnityEngine.Vector3, so results match the game's maths.
    /// </summary>
    internal readonly struct V3
    {
        public readonly float X, Y, Z;

        public V3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static readonly V3 Zero = new V3(0f, 0f, 0f);

        public static V3 operator +(V3 a, V3 b) => new V3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static V3 operator -(V3 a, V3 b) => new V3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static V3 operator -(V3 a) => new V3(-a.X, -a.Y, -a.Z);
        public static V3 operator *(V3 a, float s) => new V3(a.X * s, a.Y * s, a.Z * s);
        public static V3 operator *(float s, V3 a) => new V3(a.X * s, a.Y * s, a.Z * s);
        public static V3 operator /(V3 a, float s) => new V3(a.X / s, a.Y / s, a.Z / s);

        public static float Dot(V3 a, V3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static V3 Cross(V3 a, V3 b) => new V3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);

        public float SqrMagnitude => X * X + Y * Y + Z * Z;
        public float Magnitude => (float)Math.Sqrt(SqrMagnitude);

        /// <summary>Unit vector, or zero below 1e-5 like UnityEngine.Vector3.normalized.</summary>
        public V3 Normalized
        {
            get
            {
                float m = Magnitude;
                return m > 1e-5f ? this / m : Zero;
            }
        }

        public override string ToString() => $"({X:G5}, {Y:G5}, {Z:G5})";
    }

    /// <summary>Unit quaternion (x, y, z, w), Unity's convention and rotation formula.</summary>
    internal readonly struct Q4
    {
        public readonly float X, Y, Z, W;

        public Q4(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public static readonly Q4 Identity = new Q4(0f, 0f, 0f, 1f);

        public Q4 Inverse => new Q4(-X, -Y, -Z, W);

        public static Q4 operator *(Q4 a, Q4 b) => new Q4(
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y + a.Y * b.W + a.Z * b.X - a.X * b.Z,
            a.W * b.Z + a.Z * b.W + a.X * b.Y - a.Y * b.X,
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

        /// <summary>Rotate a vector (UnityEngine.Quaternion * Vector3).</summary>
        public static V3 operator *(Q4 q, V3 v)
        {
            float x = q.X * 2f, y = q.Y * 2f, z = q.Z * 2f;
            float xx = q.X * x, yy = q.Y * y, zz = q.Z * z, xy = q.X * y, xz = q.X * z, yz = q.Y * z;
            float wx = q.W * x, wy = q.W * y, wz = q.W * z;
            return new V3(
                (1f - (yy + zz)) * v.X + (xy - wz) * v.Y + (xz + wy) * v.Z,
                (xy + wz) * v.X + (1f - (xx + zz)) * v.Y + (yz - wx) * v.Z,
                (xz - wy) * v.X + (yz + wx) * v.Y + (1f - (xx + yy)) * v.Z);
        }

        /// <summary>Rotation of <paramref name="deg"/> degrees about a unit <paramref name="axis"/> (UnityEngine.Quaternion.AngleAxis).</summary>
        public static Q4 AngleAxis(float deg, V3 axis)
        {
            double h = deg * Math.PI / 360.0;
            float s = (float)Math.Sin(h);
            return new Q4(axis.X * s, axis.Y * s, axis.Z * s, (float)Math.Cos(h));
        }

        public V3 Right => this * new V3(1f, 0f, 0f);
    }
}
