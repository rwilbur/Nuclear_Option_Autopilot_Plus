using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// Engine-out yaw: when the effectors can't hold the yaw of asymmetric thrust with a reserve to spare, the engines on the
    /// strong side are throttled back through their own inputs. Nothing here knows an engine failed: any asymmetry (one engine
    /// dead, one damaged, the pilot's split throttle) is cut back to what the yaw authority holds, and given back as the
    /// authority grows with speed.
    ///
    /// Thrust follows the throttle slowly (spool, about 0.35 thrust ratio per second), so the cut aims at the steady state: the
    /// strong side's uncut moment is estimated from its moment now and the cut the thrust has caught up with so far.
    /// </summary>
    internal sealed class ThrustCut
    {
        /// <summary>Share of the yaw authority kept free for manoeuvring and sideslip control (and of the roll authority, by the caller).</summary>
        public const float Reserve = 0.3f;

        private const float SpoolTau = 1f;    // s: how fast thrust follows the throttle, roughly (Turbojet: rpm at spoolRate)
        private const float CutRate = 2f;     // 1/s: cutting is urgent
        private const float RestoreRate = 0.25f; // 1/s: giving thrust back is not
        private const float AuthorityTau = 2f;   // s: the steady authority, not a manoeuvre's (a full pull leaves the ruddervators no yaw for a moment)

        /// <summary>
        /// An asymmetry under this share of the effectors' plain yaw authority never needs a cut: the airframe's own weathervaning holds it
        /// with little sideslip. (Without it a centreline engine's trace of yaw got the single-engine jets cut in pulls, bench.)
        /// </summary>
        public const float MinShare = 0.1f;

        /// <summary>Per engine, filled by the caller each tick: stability-yaw acceleration from its thrust now (rad/s²).</summary>
        public readonly float[] Yaw;

        /// <summary>Per engine: its throttle as a share of the pilot's, as of the latest <see cref="Update"/>.</summary>
        public readonly float[] Scale;

        /// <summary>Off (the limit override): no cut, and any cut is given back at once.</summary>
        public bool Enabled = true;

        /// <summary>The cut on the strong side now (0 = none), and the yaw authority the thrust may use (rad/s²), for logs.</summary>
        public float Cut, Allowed;

        private float cutLag, loF, hiF;
        private bool seeded;
        private int side; // the sign of the asymmetry being cut

        public ThrustCut(int engines)
        {
            Yaw = new float[engines];
            Scale = new float[engines];
            for (int e = 0; e < engines; e++) Scale[e] = 1f;
        }

        /// <summary>
        /// One tick. <paramref name="authorityLo"/>/<paramref name="authorityHi"/>: the stability-yaw acceleration the effectors give at
        /// the ends of their usable ranges, from neutral (lo ≤ 0 ≤ hi).
        /// </summary>
        public void Update(float authorityLo, float authorityHi, float dt)
        {
            if (!seeded) { loF = authorityLo; hiF = authorityHi; seeded = true; }
            loF += (authorityLo - loF) * dt / (AuthorityTau + dt);
            hiF += (authorityHi - hiF) * dt / (AuthorityTau + dt);
            authorityLo = loF;
            authorityHi = hiF;
            float total = 0f;
            for (int e = 0; e < Yaw.Length; e++) total += Yaw[e];
            int sign = Math.Sign(total);
            if (sign != side && cutLag < 0.05f) side = sign; // pick the side only once the last cut is given back
            float strong = 0f, weak = 0f;
            for (int e = 0; e < Yaw.Length; e++)
            {
                if (Math.Sign(Yaw[e]) == side && side != 0) strong += Math.Abs(Yaw[e]);
                else weak += Yaw[e] * side;
            }
            Allowed = (1f - Reserve) * (side > 0 ? -authorityLo : authorityHi);
            float want = 0f;
            float full = strong / Math.Max(0.05f, 1f - cutLag); // the strong side's moment uncut, at the pilot's throttle
            if (Enabled && side != 0 && full > 0f) want = Clamp01(1f - (Allowed - weak) / full);
            // weak is signed along side: an engine pushing against the asymmetry (weak < 0) adds to what the strong side may give
            float step = (want > Cut ? CutRate : RestoreRate) * dt;
            Cut = Enabled ? Cut + Math.Max(-step, Math.Min(step, want - Cut)) : 0f;
            cutLag += (Cut - cutLag) * dt / (SpoolTau + dt);
            for (int e = 0; e < Yaw.Length; e++)
                Scale[e] = side != 0 && Math.Sign(Yaw[e]) == side ? 1f - Cut : 1f;
        }

        /// <summary>
        /// Anything to cut, or to give back, against the effectors' plain yaw authority from neutral (<paramref name="plainLo"/> ≤ 0 ≤
        /// <paramref name="plainHi"/>): only then is the steady authority (a few small LPs) worked out and <see cref="Update"/> called.
        /// </summary>
        public bool Needs(float plainLo, float plainHi)
        {
            if (Cut > 0f || cutLag > 1e-3f) return true;
            float total = 0f;
            for (int e = 0; e < Yaw.Length; e++) total += Yaw[e];
            bool need = Math.Abs(total) > MinShare * (total > 0f ? -plainLo : plainHi);
            if (!need) seeded = false; // the next asymmetry starts from the authority then, not a stale one
            return need;
        }

        private static float Clamp01(float v) => v < 0f ? 0f : v > 1f ? 1f : v;
    }
}
