using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// The control law: the stick commands body rates (aerospace axes, rad/s).
    /// - Pitch: pitch rate, at most <see cref="QMax"/> and the rate of a steady pull at <see cref="NzMax"/> g. The
    ///   envelope (<see cref="CommandGovernor"/>) then shapes pitch and roll together to keep AoA and g inside their limits.
    /// - Roll: roll about the flight path at up to <see cref="PMax"/>: body roll p cos α with the yaw p sin α
    ///   that keeps the sideslip out.
    /// - Yaw: pedal yaw rate, plus sideslip feedback and the turn-coordination yaw rate g sin φ cos θ / V.
    /// Commands pass a first-order pre-filter; its rate is the feed-forward for the rate loop.
    /// Stick signs are the game's: +pitch = nose down, +roll = right, +yaw = right.
    /// </summary>
    internal sealed class RateLaw
    {
        private const float G = 9.81f;

        public float PMax = 3f;     // rad/s
        public float QMax = 0.9f;   // rad/s
        public float NzMax = 9f;    // g: the jet's own limit
        public float GScale = 1f;   // the limit override's multiplier on it

        /// <summary>The g limit in force: <see cref="NzMax"/> times <see cref="GScale"/>, at most <see cref="PilotG"/>.</summary>
        public float NzLimit => Math.Max(NzMax, Math.Min(NzMax * GScale, PilotG));

        /// <summary>The game hurts the pilot above 20 g (PHASE0_FACTS); a raised limit stays under it.</summary>
        public const float PilotG = 19f;
        public float RMax = 0.35f;  // rad/s at full pedal
        public float KBeta = 1.5f;  // 1/s: yaw rate per radian of sideslip
        public float Tau = 0.08f;   // s: command pre-filter

        public float P, Q, R;          // filtered commands
        public float PDot, QDot, RDot; // their rates (feed-forward)

        public void Reset(float p, float q, float r)
        {
            P = p;
            Q = q;
            R = r;
            PDot = QDot = RDot = 0f;
        }

        /// <summary>
        /// Pseudo-control hedging: the acceleration the effectors could not give this tick (rad/s², per axis) comes
        /// off the command at once, so a command the jet can't follow settles at what it can do instead of running
        /// away (and asking the allocator for the impossible every tick). It only pulls a command toward the
        /// measured rate, never past it. (Applying it a tick later, inside the command model's rate, was tried:
        /// the delayed feedback oscillated every other tick.)
        /// </summary>
        /// Roll and yaw are hedged in stability axes (shortfalls in those axes too), so a roll the yaw can't coordinate
        /// is slowed as a whole, body roll and its yaw together.
        public void Hedge(float pShort, float qShort, float rShort, float p, float q, float r, float alpha, float dt)
        {
            float ca = (float)Math.Cos(alpha), sa = (float)Math.Sin(alpha);
            float ps = Toward(P * ca + R * sa, p * ca + r * sa, pShort * dt);
            float rs = Toward(R * ca - P * sa, r * ca - p * sa, rShort * dt);
            P = ps * ca - rs * sa;
            R = ps * sa + rs * ca;
            Q = Toward(Q, q, qShort * dt);
        }

        // move cmd by -shortfall (the direction the shortfall points), but not past the measured rate
        private static float Toward(float cmd, float measured, float shortfall)
        {
            float next = cmd - shortfall;
            if ((cmd - measured) * (next - measured) < 0f) return measured; // would cross: stop at it
            return Math.Abs(next - measured) < Math.Abs(cmd - measured) ? next : cmd;
        }

        /// <summary>
        /// The full-stick pitch rate (rad/s): the rate of a steady pull at <see cref="NzMax"/> g, at most <see cref="QMax"/>.
        /// With Stability Assist off, <see cref="QMax"/> at every speed (stock does the same below 1.2 x corner dynamic pressure).
        /// </summary>
        public float FullStickPitch(float speed, bool assist) => assist ? Math.Min(QMax, NzLimit * G / Math.Max(speed, 30f)) : QMax;

        /// <summary>
        /// The stick's targets (rad/s), before the envelope: roll about the flight path <paramref name="psT"/>, pitch rate
        /// <paramref name="qT"/>, and the yaw rate of the pedal, sideslip feedback and turn coordination <paramref name="rYaw"/>.
        /// </summary>
        public void Targets(float stickPitch, float stickRoll, float stickYaw, float speed, float beta, float bank, float pitch, bool assist,
                            out float psT, out float qT, out float rYaw)
        {
            float v = Math.Max(speed, 30f);
            qT = -stickPitch * FullStickPitch(speed, assist);
            psT = stickRoll * PMax;
            rYaw = stickYaw * RMax + KBeta * beta + G * (float)(Math.Sin(bank) * Math.Cos(pitch)) / v;
        }

        /// <summary>Body-rate targets from a stability-axis roll <paramref name="ps"/>: p = pₛ cos α, r = pₛ sin α + <paramref name="rYaw"/>.</summary>
        public static void Body(float ps, float rYaw, float alpha, out float pT, out float rT)
        {
            pT = ps * (float)Math.Cos(alpha);
            rT = ps * (float)Math.Sin(alpha) + rYaw;
        }

        /// <summary>The pre-filter: the commands move toward the targets; their rates are the rate loop's feed-forward.</summary>
        public void Filter(float pT, float qT, float rT, float dt)
        {
            float k = dt / (Tau + dt);
            PDot = (pT - P) * k / dt;
            QDot = (qT - Q) * k / dt;
            RDot = (rT - R) * k / dt;
            P += PDot * dt;
            Q += QDot * dt;
            R += RDot * dt;
        }
    }
}
