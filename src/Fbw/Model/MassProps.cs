using System;

namespace AutopilotMod.Fbw.Model
{
    /// <summary>
    /// Mass, centre of gravity and inertia tensor of a set of rigid bodies (the aircraft's parts, each its
    /// own Unity rigidbody), and the rigid-body angular acceleration they give. Sums are taken about the
    /// first body's centre in double precision, then moved to the CG, so large world coordinates lose nothing.
    /// </summary>
    internal sealed class MassProps
    {
        private double m, sx, sy, sz;
        private readonly double[] io = new double[9]; // about the reference point
        private V3 reference;
        private bool started;

        public float Mass { get; private set; }
        public V3 Cg { get; private set; }

        /// <summary>Inertia about the CG, world axes, row-major (kg m^2).</summary>
        public readonly double[] Inertia = new double[9];

        public void Clear()
        {
            m = sx = sy = sz = 0;
            Array.Clear(io, 0, 9);
            started = false;
        }

        /// <summary>One body: mass, world centre of mass, principal inertia and the world rotation of its principal axes.</summary>
        public void Add(float mass, V3 com, V3 principal, Q4 axes)
        {
            if (!(mass > 0f)) return;
            if (!started)
            {
                reference = com;
                started = true;
            }
            V3 r = com - reference;
            m += mass;
            sx += mass * r.X;
            sy += mass * r.Y;
            sz += mass * r.Z;
            // R diag(I) R^T, R's columns = the principal axes in world
            V3 a = axes * new V3(1f, 0f, 0f), b = axes * new V3(0f, 1f, 0f), c = axes * new V3(0f, 0f, 1f);
            Accumulate(a, principal.X);
            Accumulate(b, principal.Y);
            Accumulate(c, principal.Z);
            // point mass at r about the reference: m (|r|^2 E - r r^T)
            double rx = r.X, ry = r.Y, rz = r.Z, rr = rx * rx + ry * ry + rz * rz;
            io[0] += mass * (rr - rx * rx); io[1] -= mass * rx * ry; io[2] -= mass * rx * rz;
            io[3] -= mass * ry * rx; io[4] += mass * (rr - ry * ry); io[5] -= mass * ry * rz;
            io[6] -= mass * rz * rx; io[7] -= mass * rz * ry; io[8] += mass * (rr - rz * rz);
        }

        /// <summary>Close the sums: Mass, Cg and Inertia (about the CG).</summary>
        public void Finish()
        {
            if (!(m > 0)) return;
            double cx = sx / m, cy = sy / m, cz = sz / m, cc = cx * cx + cy * cy + cz * cz;
            Mass = (float)m;
            Cg = reference + new V3((float)cx, (float)cy, (float)cz);
            // parallel axis: I_cg = I_ref - m (|c|^2 E - c c^T)
            Inertia[0] = io[0] - m * (cc - cx * cx); Inertia[1] = io[1] + m * cx * cy; Inertia[2] = io[2] + m * cx * cz;
            Inertia[3] = io[3] + m * cy * cx; Inertia[4] = io[4] - m * (cc - cy * cy); Inertia[5] = io[5] + m * cy * cz;
            Inertia[6] = io[6] + m * cz * cx; Inertia[7] = io[7] + m * cz * cy; Inertia[8] = io[8] - m * (cc - cz * cz);
        }

        /// <summary>Euler's equation for the whole body: angular acceleration (rad/s^2, world) from the moment about the CG and the angular velocity.</summary>
        public V3 AngularAccel(V3 moment, V3 omega)
        {
            V3 h = Mul(Inertia, omega);
            return Solve(Inertia, moment - V3.Cross(omega, h));
        }

        private void Accumulate(V3 axis, float value)
        {
            double x = axis.X, y = axis.Y, z = axis.Z;
            io[0] += value * x * x; io[1] += value * x * y; io[2] += value * x * z;
            io[3] += value * y * x; io[4] += value * y * y; io[5] += value * y * z;
            io[6] += value * z * x; io[7] += value * z * y; io[8] += value * z * z;
        }

        public static V3 Mul(double[] a, V3 v) => new V3(
            (float)(a[0] * v.X + a[1] * v.Y + a[2] * v.Z),
            (float)(a[3] * v.X + a[4] * v.Y + a[5] * v.Z),
            (float)(a[6] * v.X + a[7] * v.Y + a[8] * v.Z));

        /// <summary>a x = b for a symmetric positive-definite 3x3 (Cramer's rule in double); zero if singular.</summary>
        public static V3 Solve(double[] a, V3 b)
        {
            double c0 = a[4] * a[8] - a[5] * a[7], c1 = a[5] * a[6] - a[3] * a[8], c2 = a[3] * a[7] - a[4] * a[6];
            double det = a[0] * c0 + a[1] * c1 + a[2] * c2;
            if (Math.Abs(det) < 1e-12) return V3.Zero;
            double bx = b.X, by = b.Y, bz = b.Z;
            double x = (bx * c0 + by * (a[2] * a[7] - a[1] * a[8]) + bz * (a[1] * a[5] - a[2] * a[4])) / det;
            double y = (bx * c1 + by * (a[0] * a[8] - a[2] * a[6]) + bz * (a[2] * a[3] - a[0] * a[5])) / det;
            double z = (bx * c2 + by * (a[1] * a[6] - a[0] * a[7]) + bz * (a[0] * a[4] - a[1] * a[3])) / det;
            return new V3((float)x, (float)y, (float)z);
        }
    }
}
