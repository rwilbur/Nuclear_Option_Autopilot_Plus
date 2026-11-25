using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// Phase-3 control law: the stick commands body rates (aerospace axes, rad/s).
    /// - Pitch: pitch rate, at most <see cref="QMax"/> and the rate of a steady pull at <see cref="NzMax"/> g.
    /// - Roll: roll about the flight path at up to <see cref="PMax"/>: body roll p cos α with the yaw p sin α
    ///   that keeps the sideslip out.
    /// - Yaw: pedal yaw rate, plus sideslip feedback and the turn-coordination yaw rate g sin φ cos θ / V.
    /// Commands pass a first-order pre-filter; its rate is the feed-forward for the rate loop.
    /// No AoA protection yet (phase 4). Stick signs are the game's: +pitch = nose down, +roll = right, +yaw = right.
    /// </summary>
    internal sealed class RateLaw
    {
        private const float G = 9.81f;

        /// <summary>
        /// Phase-3 roll-rate cap (rad/s, 180 deg/s, the Revoker's stock rate). Stock commands half its max roll rate
        /// whatever the jet can coordinate (286 deg/s on the Ifrit, whose yaw can't follow that: 10 deg of sideslip on
        /// the bench). Phase 4 replaces this with the capacity governor.
        /// </summary>
        public const float RollCap = 3.1416f;

        public float PMax = 3f;     // rad/s
        public float QMax = 0.9f;   // rad/s
        public float NzMax = 9f;    // g
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
        public void Hedge(float pShort, float qShort, float rShort, float p, float q, float r, float dt)
        {
            P = Toward(P, p, pShort * dt);
            Q = Toward(Q, q, qShort * dt);
            R = Toward(R, r, rShort * dt);
        }

        // move cmd by -shortfall (the direction the shortfall points), but not past the measured rate
        private static float Toward(float cmd, float measured, float shortfall)
        {
            float next = cmd - shortfall;
            if ((cmd - measured) * (next - measured) < 0f) return measured; // would cross: stop at it
            return Math.Abs(next - measured) < Math.Abs(cmd - measured) ? next : cmd;
        }

        public void Step(float stickPitch, float stickRoll, float stickYaw, float speed, float alpha, float beta, float bank, float pitch, float dt)
        {
            float v = Math.Max(speed, 30f);
            float qLim = Math.Min(QMax, NzMax * G / v);
            float qT = -stickPitch * qLim;
            float ps = stickRoll * PMax;
            float ca = (float)Math.Cos(alpha), sa = (float)Math.Sin(alpha);
            float pT = ps * ca;
            float rT = ps * sa + stickYaw * RMax + KBeta * beta + G * (float)(Math.Sin(bank) * Math.Cos(pitch)) / v;

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
