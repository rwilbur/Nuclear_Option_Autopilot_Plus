using System;

namespace AutopilotMod
{
    /// <summary>
    /// Online estimate of how much rate the stick really gives on one axis (fly-by-wire included), so the
    /// rate loops keep their tuning when the airframe changes (damage, stores), plus a guard that backs a
    /// loop off if it starts to chatter.
    ///
    /// Ratio = measured rate / rate the FBW model predicts for the same stick (1 = the model is exact). It
    /// is a least-squares fit of the rate against the model's rate, both band-passed to the slow part of
    /// the motion (swings of about 0.4 s to 2 s, i.e. manoeuvres): faster than that is where the game's
    /// several-tick delay and the airframe's lag live, slower is trim drift. The model's rate is first
    /// lagged by the prior response time so the two line up. An earlier short-window estimate (one tick
    /// of delay assumed) read the game's late answer as a weak stick, pinned at its floor and made both
    /// loops 2.4-3x too aggressive: a 7.5 Hz limit cycle in a recorded flight. This one, replayed on a
    /// recorded flight, read pitch 1.0-1.2 and a steady roll value from 270 to 416 m/s.
    ///
    /// Detune (1 = off, up to 4) multiplies the Ratio the loops use: while the stick reverses faster than
    /// 6 times a second with real amplitude (a limit cycle, from a wrong estimate or anything else) it
    /// grows, softening the loop, and it lets go over about 20 s once the stick is quiet.
    /// </summary>
    internal sealed class EffectivenessRls
    {
        private const float Memory = 10f;     // s of excited flight the estimate remembers (forgets only while learning)
        private const float HighPass = 2f;    // s: slower than this is trim drift, not the stick's doing
        private const float LowPass = 0.4f;   // s: faster than this is the game's delay and the airframe's lag
        private const float ExciteMin = 1e-8f; // (rad/s)^2 of band-passed model rate: below = nothing to learn
        private const float WarmUp = 3f;      // s of excited flight before the estimate is half trusted

        private readonly float tau0;
        private bool started;
        private float uLag, uSlow, xLo, qSlow, yLo, sxx, sxy, excited;
        private float uFastBase, uFastPrev, reversals, chatter;

        /// <summary>Excited seconds seen; 0 = pure prior.</summary>
        public float Samples => excited;

        /// <summary>Loop softening from the chatter guard: 1 = none, up to 4.</summary>
        public float Detune { get; private set; } = 1f;

        /// <param name="tau0">Prior time constant (s) of the rate response.</param>
        public EffectivenessRls(float tau0)
        {
            this.tau0 = tau0;
        }

        public void Reset(float dt)
        {
            started = false;
            sxx = sxy = excited = 0f;
            reversals = chatter = 0f;
            Detune = 1f;
        }

        private float Trust => excited / (excited + WarmUp);

        /// <summary>Not learning this tick (the pilot flying): keep the estimate, restart the filters when it resumes.</summary>
        public void Pause() => started = false;

        /// <summary>Learned rate per model unit (clamped to [0.4, 2.5], blended toward 1 until trusted), times Detune.</summary>
        public float Ratio => (1f + (Clamp(sxx > 0f ? sxy / sxx : 1f, 0.4f, 2.5f) - 1f) * Trust) * Detune;

        /// <summary>Rate acceleration (1/s) per model unit: the ratio over the prior response time (what INDI inverts).</summary>
        public float B => Ratio / tau0;

        internal string Raw => $"sxy/sxx {(sxx > 0f ? sxy / sxx : float.NaN):F3} excited {excited:F1}s detune {Detune:F2}";

        /// <summary>Response time constant (s) the loops assume: the prior.</summary>
        public float Tau => tau0;

        /// <summary>
        /// One physics tick: q = body rate measured now (rad/s), uModel = stick applied since the last
        /// tick times the FBW model's rate per stick (rad/s), stick = that stick (-1..1).
        /// </summary>
        public void Observe(float q, float uModel, float stick, float dt)
        {
            if (!(dt > 0f)) return;
            if (!started)
            {
                started = true;
                uLag = uSlow = uModel;
                qSlow = q;
                xLo = yLo = 0f;
                uFastBase = stick;
                uFastPrev = 0f;
            }
            float up = Lp(ref uLag, uModel, tau0, dt);
            float x = Lp(ref xLo, up - Lp(ref uSlow, up, HighPass, dt), LowPass, dt);
            float y = Lp(ref yLo, q - Lp(ref qSlow, q, HighPass, dt), LowPass, dt);
            if (x * x >= ExciteMin)
            {
                float lambda = (float)Math.Exp(-dt / Memory);
                sxx = lambda * sxx + x * x;
                sxy = lambda * sxy + x * y;
                excited = Math.Min(excited + dt, 1e6f);
            }
            Guard(stick, dt);
        }

        /// <summary>Chatter guard: reversals of the stick's fast part (above ~2 Hz) with amplitude, per second.</summary>
        private void Guard(float stick, float dt)
        {
            float fast = stick - Lp(ref uFastBase, stick, 0.08f, dt);
            bool flip = Math.Abs(fast) > 0.01f && fast * uFastPrev < 0f;
            if (Math.Abs(fast) > 0.01f) uFastPrev = fast;
            reversals += ((flip ? 1f / dt : 0f) - reversals) * Math.Min(1f, dt / 0.5f); // per second, over ~0.5 s
            chatter += (fast * fast - chatter) * Math.Min(1f, dt / 0.5f);
            if (reversals > 6f && chatter > 0.02f * 0.02f)
                Detune = Math.Min(4f, Detune * (1f + dt));   // doubles in ~0.7 s
            else
                Detune = 1f + (Detune - 1f) * (float)Math.Exp(-dt / 20f);
        }

        private static float Lp(ref float s, float v, float tau, float dt)
        {
            s += (v - s) * dt / (tau + dt);
            return s;
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
        /// <param name="learn">false = the pilot is flying: the effectiveness estimate holds (hand-flown stick, through
        /// mouse aim and full-stick rate limits, read the roll as weak in a recorded flight)</param>
        public void Observe(float q, float uApplied, float model, float dt, float filterHz, bool learn = true)
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
            if (learn) Rls.Observe(q, uApplied * model, uApplied, dt);
            else Rls.Pause();
        }

        /// <summary>Stick (-1..1) for a commanded body rate.</summary>
        /// <param name="bandwidth">Rate error -> rate acceleration gain (1/s).</param>
        /// <param name="adaptive">Use the RLS effectiveness; false = the prior (FBW model x 1/tau).</param>
        /// <param name="uFallback">Stick to return before the filters have a sample.</param>
        public float Command(float qCmd, float q, float model, float bandwidth, bool adaptive, float uFallback)
        {
            if (!warm) return uFallback;
            float b = (adaptive ? Rls.B : Rls.Detune / tauPrior) * Math.Max(model, 1e-3f); // the chatter guard softens it either way
            float du = EffectivenessRls.Clamp(IncrementGain * (bandwidth * (qCmd - q) - qdotF) / b, -0.25f, 0.25f);
            LastIncrement = du;
            return EffectivenessRls.Clamp(uF + du, -1f, 1f);
        }
    }
}
