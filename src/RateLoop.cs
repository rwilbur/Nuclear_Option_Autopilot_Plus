using System;

namespace AutopilotMod
{
    /// <summary>
    /// Online estimate of how the airframe (fly-by-wire included) answers the stick on one axis,
    /// by recursive least squares with forgetting.
    ///
    /// Model, per physics tick, in "model units" (stick x the rate per stick our FBW model predicts):
    ///     dq[k+1] = a * dq[k] + b1 * du[k] + b2 * du[k-1]
    /// where dq is the tick-to-tick change of body rate and du that of the model-scaled stick (u[k] is
    /// the stick applied between ticks k and k+1). Differencing drops any steady trim/gravity bias.
    /// From it:
    ///     Ratio = (b1 + b2) / (1 - a)   steady rate per model unit: 1 = the FBW model is exact
    ///     B     = (b1 + b2) / dt        rate acceleration per model unit (what INDI inverts)
    /// It only updates while the stick moves (pilot or autopilot), so steady flight can't drift it,
    /// and it is blended with the prior until it has seen enough excitation.
    /// </summary>
    internal sealed class EffectivenessRls
    {
        private const int N = 3;
        private const float Forget = 0.995f;    // ~4 s of excited samples at 50 Hz
        private const float P0 = 1f, PMax = 10f;
        private const float ExciteMin = 0.002f;  // rad/s^2: |du|+|du_prev| below this = no information
        private const float WarmUp = 60f;        // excited samples before the estimate is half trusted

        private readonly float tau0;
        private float a, b1, b2, a0, bSum0, dt0 = -1f;
        private readonly float[] P = new float[N * N];
        private readonly float[] phi = new float[N], gain = new float[N], pphi = new float[N];
        private float qPrev = float.NaN, uPrev = float.NaN, dqPrev = float.NaN, duPrev = float.NaN;

        /// <summary>Excited samples seen; 0 = pure prior.</summary>
        public float Samples { get; private set; }

        /// <param name="tau0">Prior time constant (s) of the rate response. Small = conservative for INDI.</param>
        public EffectivenessRls(float tau0)
        {
            this.tau0 = tau0;
            Reset(0.02f);
        }

        public void Reset(float dt)
        {
            dt0 = dt;
            a0 = (float)Math.Exp(-dt / tau0);
            bSum0 = 1f - a0;   // Ratio = 1 at the prior
            a = a0;
            b1 = bSum0;
            b2 = 0f;
            Array.Clear(P, 0, P.Length);
            for (int i = 0; i < N; i++) P[i * N + i] = P0;
            Samples = 0f;
            qPrev = uPrev = dqPrev = duPrev = float.NaN;
        }

        /// <summary>Blend of estimate and prior, by how much excitation has been seen.</summary>
        private float Trust => Samples / (Samples + WarmUp);

        private float ABlend => a0 + (a - a0) * Trust;
        private float BSum => bSum0 + (b1 + b2 - bSum0) * Trust;

        /// <summary>Steady rate per model unit (1 = the FBW model is exact), clamped to [0.4, 2.5].
        /// Computed from the raw estimate and only then blended toward 1: blending a and b first
        /// would mix a fast prior pole with a slow estimated one and miss badly.</summary>
        public float Ratio => 1f + (Clamp((b1 + b2) / Math.Max(1f - a, 1e-3f), 0.4f, 2.5f) - 1f) * Trust;

        /// <summary>Rate acceleration (1/s) per model unit, clamped to [0.3, 3] x the prior.</summary>
        public float B => Clamp(BSum, 0.3f * bSum0, 3f * bSum0) / dt0;

        internal string Raw => $"a {a:F4} b1 {b1:F4} b2 {b2:F4} a0 {a0:F4} b0 {bSum0:F4}";

        /// <summary>Response time constant (s) implied by the estimate.</summary>
        public float Tau => -dt0 / (float)Math.Log(Clamp(ABlend, 0.05f, 0.9995f));

        /// <summary>
        /// One physics tick: q = body rate measured now (rad/s), uModel = stick applied since the last
        /// tick times the FBW model's rate per stick (rad/s).
        /// </summary>
        public void Observe(float q, float uModel, float dt)
        {
            if (Math.Abs(dt - dt0) > 1e-5f) Reset(dt);
            if (!float.IsNaN(qPrev))
            {
                // dq = q[k+1]-q[k]; du = u[k]-u[k-1] (uPrev holds u[k-1])
                float dq = (q - qPrev) / dt;
                float du = float.IsNaN(uPrev) ? float.NaN : (uModel - uPrev) / dt;
                if (!float.IsNaN(dqPrev) && !float.IsNaN(du) && !float.IsNaN(duPrev))
                    Update(dq, dqPrev, du, duPrev);
                dqPrev = dq;
                duPrev = du;
            }
            qPrev = q;
            uPrev = uModel;
        }

        private void Update(float y, float dqk, float duk, float dukm1)
        {
            // Every tick carries information about the response (a settling rate shows a), but only
            // a moving stick shows the gain. Forget old data only while excited, so long steady
            // stretches can't wind the covariance up (the classic RLS blow-up) or erase the estimate.
            bool excited = Math.Abs(duk) + Math.Abs(dukm1) >= ExciteMin / dt0;
            if (!excited && Math.Abs(dqk) < 1e-4f && Math.Abs(y) < 1e-4f) return; // nothing to learn
            float lambda = excited ? Forget : 1f;

            phi[0] = dqk;
            phi[1] = duk;
            phi[2] = dukm1;
            float denom = lambda;
            for (int i = 0; i < N; i++)
            {
                float s = 0f;
                for (int j = 0; j < N; j++) s += P[i * N + j] * phi[j];
                pphi[i] = s;
                denom += phi[i] * s;
            }
            if (!(denom > 1e-9f) || float.IsInfinity(denom)) return;
            for (int i = 0; i < N; i++) gain[i] = pphi[i] / denom;
            float err = y - (a * dqk + b1 * duk + b2 * dukm1);
            a += gain[0] * err;
            b1 += gain[1] * err;
            b2 += gain[2] * err;
            float trace = 0f;
            for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
            {
                P[i * N + j] = (P[i * N + j] - gain[i] * pphi[j]) / lambda;
                if (i == j) trace += P[i * N + j];
            }
            if (trace > PMax)
                for (int i = 0; i < N * N; i++) P[i] *= PMax / trace;

            if (float.IsNaN(a) || float.IsNaN(b1) || float.IsNaN(b2))
            {
                Reset(dt0);
                return;
            }
            // Keep the parameters physical (stable, positive gain) so a bad stretch can't run away.
            a = Clamp(a, 0f, 0.9995f);
            float bs = b1 + b2;
            if (bs < 0.03f * bSum0 || bs > 8f * bSum0)
            {
                b1 = Clamp(bs, 0.03f * bSum0, 8f * bSum0);
                b2 = 0f;
            }
            if (excited) Samples = Math.Min(Samples + 1f, 1e6f);
        }

        internal static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;
    }

    /// <summary>
    /// Incremental nonlinear dynamic inversion (INDI) rate loop for one axis.
    ///
    /// Each tick it asks for a rate acceleration qdotDes = K * (qCmd - q) and moves the stick from
    /// where it (filtered) was by the increment that should produce the missing acceleration:
    ///     u = uF + (qdotDes - qdotF) / (B * model)
    /// qdotF is the measured rate derivative and uF the applied stick, both through the SAME low-pass
    /// filter so their lags match. Trim, gravity and model errors show up in the measurement, so
    /// there is no integrator to wind up; an error in B only changes how fast it converges.
    /// </summary>
    internal sealed class IndiAxis
    {
        public readonly EffectivenessRls Rls;
        private readonly float tauPrior;
        private float qPrev = float.NaN, qdotF, uF;
        private bool warm;
        private readonly float[] uHist = new float[8];

        /// <summary>Fraction of the inverted increment applied each tick (1 = textbook INDI).</summary>
        public float IncrementGain = 1f;

        /// <summary>Ticks the applied stick is delayed before entering uF, to line it up with the
        /// measured acceleration it caused (the airframe/FBW transport delay).</summary>
        public int SyncTicks;

        public IndiAxis(float tauPrior)
        {
            this.tauPrior = tauPrior;
            Rls = new EffectivenessRls(tauPrior);
        }

        /// <summary>Last stick increment (debug line).</summary>
        public float LastIncrement { get; private set; }

        /// <summary>New aircraft: forget filters and the estimate.</summary>
        public void Reset()
        {
            qPrev = float.NaN;
            warm = false;
            LastIncrement = 0f;
            Rls.Reset(0.02f);
        }

        /// <summary>
        /// Call every physics tick (autopilot on or off) with the measured rate, the stick that was
        /// applied since the last tick, and the FBW model's rate per stick.
        /// </summary>
        public void Observe(float q, float uApplied, float model, float dt, float filterHz)
        {
            float alpha = 1f - (float)Math.Exp(-2.0 * Math.PI * filterHz * dt);
            for (int i = uHist.Length - 1; i > 0; i--) uHist[i] = warm ? uHist[i - 1] : uApplied;
            uHist[0] = uApplied;
            float uSync = uHist[Math.Min(Math.Max(SyncTicks, 0), uHist.Length - 1)];
            if (!float.IsNaN(qPrev))
            {
                float qdot = (q - qPrev) / dt;
                if (!warm)
                {
                    qdotF = qdot;
                    uF = uApplied;
                    warm = true;
                }
                qdotF += alpha * (qdot - qdotF);
                uF += alpha * (uSync - uF);
            }
            qPrev = q;
            Rls.Observe(q, uApplied * model, dt);
        }

        /// <summary>Stick (-1..1) for a commanded body rate.</summary>
        /// <param name="bandwidth">Rate error -> rate acceleration gain (1/s).</param>
        /// <param name="adaptive">Use the RLS effectiveness; false = the prior (FBW model x 1/tau).</param>
        /// <param name="uFallback">Stick to return before the filters have a sample.</param>
        public float Command(float qCmd, float q, float model, float bandwidth, bool adaptive, float uFallback)
        {
            if (!warm) return uFallback;
            float b = (adaptive ? Rls.B : 1f / tauPrior) * Math.Max(model, 1e-3f);
            float du = EffectivenessRls.Clamp(IncrementGain * (bandwidth * (qCmd - q) - qdotF) / b, -0.25f, 0.25f);
            LastIncrement = du;
            return EffectivenessRls.Clamp(uF + du, -1f, 1f);
        }
    }
}
