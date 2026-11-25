using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// The inner rate loops: rate error -> wanted angular acceleration ν -> INDI increment -> allocator.
    ///
    /// Tick-synchronised INDI: the FBW runs before the game's jobs, so the measured acceleration is the one
    /// over the physics step just taken, and the base of the increment is each effector's position during
    /// that same step (its position now). Each effector's box is what it reaches this tick, so the command is
    /// what happens:
    ///   ν = K (ω_cmd − ω) + ω̇_cmd,   Δν = ν − ω̇_f,   u = u_f + Δu  with  B Δu ≈ Δν.
    /// The measured acceleration and the positions pass the same low-pass (the joints between parts make the
    /// measurement vibrate a little), so the increment stays consistent [Smeur 2016].
    ///
    /// The allocation runs in stability axes: after a step, Alloc.B and the unmet acceleration are in (stability roll,
    /// pitch, stability yaw), turned from body axes by the AoA.
    ///
    /// Effectors are fixed slots: an unusable one (shot off, nozzle above its speed limit) gets B = 0 and a
    /// zero-width box, so slots and their filters never shift.
    /// </summary>
    internal sealed class IndiLoop
    {
        public float KRoll = 10f, KPitch = 8f, KYaw = 5f; // 1/s: rate-loop bandwidth
        public float FilterTau = 0.04f;                   // s, on the measured acceleration and on the positions
        public float IncrementGain = 0.7f;                // share of the missing acceleration asked for per tick: < 1 for margin against delay and model error
        public float Reg = 1e-2f;                         // pull toward neutral with spare freedom, relative to each effector's own (weighted) effect
        public float Damp = 1f;                           // cost of moving this tick, likewise: damps tick-to-tick trading between near-twin effectors
                                                          // (bench 25 Nov: 0.05 let the Compass and Ifrit trade surfaces every tick; 1 keeps tracking and stops it)
        public int HomeAfter = 6;                         // ticks: an effector idle longer heads for neutral (bench: 6, 12 and 30 alike)
        public float Idle = 0.1f;                         // an effector under this share of the strongest one's effect, on every axis, holds (or heads back to its unstalled range)

        /// <summary>
        /// Axis priority when the effectors can't give everything: what yields first. Roll gives way to yaw
        /// (sideslip) and both to pitch. With a reachable request the weights change nothing.
        /// </summary>
        public const float RollWeight = 1f, PitchWeight = 100f, YawWeight = 30f;

        public readonly Allocator Alloc;
        public readonly float[] Command; // per slot, the position commanded this tick
        public float NuRoll, NuPitch, NuYaw; // last ν (rad/s²)
        public float RequestYaw; // last increment asked of stability yaw (rad/s²)
        public float GotPitch;   // last increment pitch got (rad/s²)
        public float UnmetRoll, UnmetPitch, UnmetYaw; // Δν the effectors could not give (rad/s², stability axes)

        private readonly float[] uf;
        private readonly int[] idleTicks;
        private float wdP, wdQ, wdR;

        /// <summary>The measured pitch acceleration through the INDI filter (rad/s²), as of the last step.</summary>
        public float FilteredQDot => wdQ;

        /// <summary>The measured roll and yaw accelerations through the INDI filter (rad/s², body), as of the last step.</summary>
        public float FilteredPDot => wdP;
        public float FilteredRDot => wdR;
        private bool started;

        public IndiLoop(int slots)
        {
            Alloc = new Allocator(slots);
            Alloc.W[0] = RollWeight;
            Alloc.W[1] = PitchWeight;
            Alloc.W[2] = YawWeight;
            Command = new float[slots];
            uf = new float[slots];
            idleTicks = new int[slots];
        }

        public void Reset() => started = false;

        /// <summary>
        /// One tick. The caller has filled, per slot j &lt; <paramref name="count"/>: Alloc.B (rad/s² per unit, roll/pitch/yaw),
        /// <paramref name="position"/>[j] (now), <paramref name="reachLo"/>/<paramref name="reachHi"/>[j] (positions reachable this tick),
        /// <paramref name="neutral"/>[j] and <paramref name="costScale"/>[j] (1 = surface, more = prefer neutral more).
        /// </summary>
        public void Step(int count, float p, float q, float r, float pDot, float qDot, float rDot, float alpha,
                         RateLaw law, float[] position, float[] reachLo, float[] reachHi, float[] neutral, float[] costScale, float dt)
        {
            float k = started ? dt / (FilterTau + dt) : 1f;
            wdP += (pDot - wdP) * k;
            wdQ += (qDot - wdQ) * k;
            wdR += (rDot - wdR) * k;
            for (int j = 0; j < count; j++) uf[j] += (position[j] - uf[j]) * k;
            started = true;

            NuRoll = KRoll * (law.P - p) + law.PDot;
            NuPitch = KPitch * (law.Q - q) + law.QDot;
            NuYaw = KYaw * (law.R - r) + law.RDot;

            Allocator a = Alloc;
            a.Count = count;
            // Allocate in stability axes (roll about the flight path, and the yaw across it that is sideslip): the axis
            // weights then say "keep the sideslip, give up roll rate" at every AoA. In body axes at high AoA, yaw first meant
            // a body yaw rate without its matching body roll, which is sideslip (bench rolling pulls: 35 deg on the Ifrit).
            float ca = (float)Math.Cos(alpha), sa = (float)Math.Sin(alpha);
            for (int j = 0; j < count; j++)
            {
                float bp = a.B[3 * j], br = a.B[3 * j + 2];
                a.B[3 * j] = bp * ca + br * sa;
                a.B[3 * j + 2] = br * ca - bp * sa;
            }
            // An effector that barely acts on every axis right now (near stall, or flow it no longer turns) isn't allocated:
            // the linearisation says little about it and its position would follow round-off. It holds (or heads back to its unstalled range). Per axis, against the
            // strongest effector on that axis: a single weighted total let a thrust-vectoring nozzle at low speed (huge
            // pitch, pitch weighted 100) idle every surface, roll included (flight logs, Vortex and Ifrit, 25 Nov).
            float s0 = 0f, s1 = 0f, s2 = 0f;
            for (int j = 0; j < count; j++)
            {
                s0 = Math.Max(s0, Math.Abs(a.B[3 * j]));
                s1 = Math.Max(s1, Math.Abs(a.B[3 * j + 1]));
                s2 = Math.Max(s2, Math.Abs(a.B[3 * j + 2]));
            }
            // costs scale with each effector's own weighted effect, so they mean the same at every dynamic pressure
            for (int j = 0; j < count; j++)
            {
                a.U[j] = uf[j];
                float b2 = Weighted(a, j);
                bool idle = Weak(a.B[3 * j], s0) && Weak(a.B[3 * j + 1], s1) && Weak(a.B[3 * j + 2], s2);
                // an idle one holds, but one idle for longer than HomeAfter heads for neutral as fast as its servo goes: held, a stalled
                // aileron stayed at full deflection and kept a 173 deg/s roll going for another 450 deg after the stick was centred
                // (Revoker log, 25 Nov). (Homing at once traded with using it: Ifrit full-roll chatter 0.1 -> 15.)
                idleTicks[j] = idle ? idleTicks[j] + 1 : 0;
                float home = idleTicks[j] <= HomeAfter ? position[j]
                    : Math.Max(reachLo[j], Math.Min(Math.Max(reachHi[j], reachLo[j]), neutral[j]));
                float lo = idle ? home : reachLo[j], hi = idle ? home : Math.Max(reachHi[j], reachLo[j]);
                a.Lo[j] = lo - uf[j];
                a.Hi[j] = hi - uf[j];
                a.Pref[j] = neutral[j];
                a.C[j] = Reg * costScale[j] * b2 + 1e-12f;
                a.D[j] = Damp * b2;
            }
            float dvB = IncrementGain * (NuRoll - wdP), dvQ = IncrementGain * (NuPitch - wdQ), dvY = IncrementGain * (NuYaw - wdR);
            float dvP = dvB * ca + dvY * sa, dvR = dvY * ca - dvB * sa;
            RequestYaw = dvR;
            a.Solve(dvP, dvQ, dvR);
            a.Achieved(out float gp, out float gq, out float gr);
            UnmetRoll = dvP - gp;
            UnmetPitch = dvQ - gq;
            GotPitch = gq;
            UnmetYaw = dvR - gr;
            for (int j = 0; j < count; j++) Command[j] = uf[j] + a.Du[j];
        }

        private bool Weak(float b, float strongest) => strongest <= 0f || Math.Abs(b) < Idle * strongest;

        private static float Weighted(Allocator a, int j) =>
            a.W[0] * a.B[3 * j] * a.B[3 * j] + a.W[1] * a.B[3 * j + 1] * a.B[3 * j + 1] + a.W[2] * a.B[3 * j + 2] * a.B[3 * j + 2];
    }
}
