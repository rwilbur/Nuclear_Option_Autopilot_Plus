using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// The incremental allocation INDI needs: effector changes du that give the wanted change in angular
    /// acceleration dv, within each effector's box (what its servo reaches this tick). Box-constrained
    /// weighted least squares:
    ///
    ///   minimise  Σ_a W_a (Σ_j B_aj du_j − dv_a)²  +  Σ_j C_j (U_j + du_j − Pref_j)²  +  Σ_j D_j du_j²,   Lo_j ≤ du_j ≤ Hi_j
    ///
    /// B: angular acceleration per unit of each effector (rad/s² per degree), W: axis weights, C: how much
    /// each effector is pulled toward its preferred position (Pref, e.g. neutral) with the spare freedom.
    /// Solved exactly by a primal active-set method on the convex box QP (Härkegård's WLS allocation in
    /// box form): Cholesky on the free effectors, then fix the first bound hit or free the bound with the
    /// wrong-signed gradient, until neither happens. Finite; no allocation. (Coordinate descent was tried
    /// first: it crawls when two effectors do nearly the same thing, which left/right pairs always do.)
    /// </summary>
    internal sealed class Allocator
    {
        public const int Axes = 3;

        public readonly int Capacity;
        public int Count;

        /// <summary>Column-major: B[3 j + a] = acceleration on axis a (roll, pitch, yaw) per unit of effector j.</summary>
        public readonly float[] B;
        public readonly float[] Lo, Hi, U, Pref, C, Du;

        /// <summary>Per effector: cost of moving this tick, D du² (damps tick-to-tick trading between effectors).</summary>
        public readonly float[] D;
        public readonly float[] W = { 1f, 1f, 1f };

        /// <summary>Active-set iterations of the last solve (diagnostic).</summary>
        public int Iterations { get; private set; }

        private readonly double[] h, g, x, y, m, rhs;
        private readonly int[] state; // 0 free, -1 at Lo, +1 at Hi
        private readonly int[] free;

        public Allocator(int capacity)
        {
            Capacity = capacity;
            B = new float[Axes * capacity];
            Lo = new float[capacity];
            Hi = new float[capacity];
            U = new float[capacity];
            Pref = new float[capacity];
            C = new float[capacity];
            Du = new float[capacity];
            D = new float[capacity];
            h = new double[capacity * capacity];
            g = new double[capacity];
            x = new double[capacity];
            y = new double[capacity];
            m = new double[capacity * capacity];
            rhs = new double[capacity];
            state = new int[capacity];
            free = new int[capacity];
        }

        /// <summary>Solve for Du. <paramref name="dv"/>: wanted change of angular acceleration (roll, pitch, yaw), rad/s².</summary>
        public void Solve(float dvRoll, float dvPitch, float dvYaw)
        {
            int n = Count;
            // H = B^T W B + diag(C), g = B^T W dv + C (Pref - U)
            for (int i = 0; i < n; i++)
            {
                double bi0 = B[3 * i], bi1 = B[3 * i + 1], bi2 = B[3 * i + 2];
                for (int j = i; j < n; j++)
                {
                    double v = W[0] * bi0 * B[3 * j] + W[1] * bi1 * B[3 * j + 1] + W[2] * bi2 * B[3 * j + 2];
                    h[i * n + j] = v;
                    h[j * n + i] = v;
                }
                h[i * n + i] += C[i] + D[i];
                g[i] = W[0] * bi0 * dvRoll + W[1] * bi1 * dvPitch + W[2] * bi2 * dvYaw + C[i] * (Pref[i] - U[i]);
            }

            // feasible start: du = 0 clamped into each box; a zero-width or useless effector stays bound
            for (int i = 0; i < n; i++)
            {
                double lo = Lo[i], hi = Math.Max(Lo[i], Hi[i]);
                x[i] = Math.Max(lo, Math.Min(hi, 0.0));
                bool stuck = hi - lo <= 0 || !(h[i * n + i] > 1e-15);
                state[i] = stuck ? (x[i] <= lo ? -1 : 1) : 0;
            }

            int it = 0;
            for (; it < 4 * n + 8; it++)
            {
                // minimise over the free set with the bound ones held: H_FF y_F = g_F - H_FB x_B
                int nf = 0;
                for (int i = 0; i < n; i++) if (state[i] == 0) free[nf++] = i;
                for (int a = 0; a < nf; a++)
                {
                    int i = free[a];
                    double r = g[i];
                    for (int j = 0; j < n; j++) if (state[j] != 0) r -= h[i * n + j] * x[j];
                    rhs[a] = r;
                    for (int b = 0; b < nf; b++) m[a * nf + b] = h[i * n + free[b]];
                }
                if (!Cholesky(nf)) break; // cannot happen with C > 0; keep the last feasible x
                // walk from x toward y; stop at the first box edge
                double step = 1.0;
                int block = -1, blockSide = 0;
                for (int a = 0; a < nf; a++)
                {
                    int i = free[a];
                    double yi = rhs[a], d = yi - x[i];
                    y[i] = yi;
                    if (yi < Lo[i] && d < 0) { double t = (Lo[i] - x[i]) / d; if (t < step) { step = t; block = i; blockSide = -1; } }
                    else if (yi > Hi[i] && d > 0) { double t = (Hi[i] - x[i]) / d; if (t < step) { step = t; block = i; blockSide = 1; } }
                }
                for (int a = 0; a < nf; a++) { int i = free[a]; x[i] += step * (y[i] - x[i]); }
                if (block >= 0)
                {
                    x[block] = blockSide < 0 ? Lo[block] : Hi[block];
                    state[block] = blockSide;
                    continue;
                }
                // optimal on this free set: free the bound effector whose gradient points into its box
                int release = -1;
                double worst = 1e-12;
                for (int i = 0; i < n; i++)
                {
                    if (state[i] == 0 || !(Math.Max(Lo[i], Hi[i]) - Lo[i] > 0) || !(h[i * n + i] > 1e-15)) continue;
                    double grad = -g[i];
                    for (int j = 0; j < n; j++) grad += h[i * n + j] * x[j];
                    double pull = state[i] < 0 ? -grad : grad; // > 0: moving off the bound lowers the cost
                    if (pull > worst) { worst = pull; release = i; }
                }
                if (release < 0) break;
                state[release] = 0;
            }
            Iterations = it;
            for (int i = 0; i < n; i++) Du[i] = (float)x[i];
        }

        /// <summary>Solve m (nf x nf, symmetric positive definite) y = rhs in place: rhs becomes y.</summary>
        private bool Cholesky(int nf)
        {
            for (int j = 0; j < nf; j++)
            {
                double d = m[j * nf + j];
                for (int k = 0; k < j; k++) d -= m[j * nf + k] * m[j * nf + k];
                if (!(d > 0)) return false;
                d = Math.Sqrt(d);
                m[j * nf + j] = d;
                for (int i = j + 1; i < nf; i++)
                {
                    double s = m[i * nf + j];
                    for (int k = 0; k < j; k++) s -= m[i * nf + k] * m[j * nf + k];
                    m[i * nf + j] = s / d;
                }
            }
            for (int i = 0; i < nf; i++) // L z = rhs
            {
                double s = rhs[i];
                for (int k = 0; k < i; k++) s -= m[i * nf + k] * rhs[k];
                rhs[i] = s / m[i * nf + i];
            }
            for (int i = nf - 1; i >= 0; i--) // L^T y = z
            {
                double s = rhs[i];
                for (int k = i + 1; k < nf; k++) s -= m[k * nf + i] * rhs[k];
                rhs[i] = s / m[i * nf + i];
            }
            return true;
        }

        /// <summary>The change of angular acceleration the solution gives, B du.</summary>
        public void Achieved(out float roll, out float pitch, out float yaw)
        {
            roll = pitch = yaw = 0f;
            for (int j = 0; j < Count; j++)
            {
                roll += B[3 * j] * Du[j];
                pitch += B[3 * j + 1] * Du[j];
                yaw += B[3 * j + 2] * Du[j];
            }
        }
    }
}
