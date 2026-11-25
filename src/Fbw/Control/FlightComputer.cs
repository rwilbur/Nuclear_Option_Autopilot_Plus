using System;
using AutopilotMod.Fbw.Model;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>The flight state the control laws read, in aerospace signs (rad, rad/s, rad/s², m/s).</summary>
    internal struct FlightState
    {
        public float P, Q, R;          // body rates
        public float PDot, QDot, RDot; // body angular accelerations over the last physics step
        public float Speed, Alpha, Beta, Bank, Pitch;
        public float Nz;               // load factor along body up, gravity removed (g; 1 in level flight)
    }

    /// <summary>
    /// The fly-by-wire's computer: laws + INDI + allocation, with no game types. The game adapter and the
    /// offline bench both fill the effector slots (position, this tick's reach, effectiveness) and the
    /// state each tick, call <see cref="Step"/>, and turn <see cref="Command"/> into the game's inputs.
    /// </summary>
    internal sealed class FlightComputer
    {
        /// <summary>
        /// Effector boxes span what each servo reaches in this many ticks. One tick makes every command happen
        /// exactly; longer boxes let a surface whose effectiveness changes with its position (near stall) jump
        /// between two regimes every tick. (Bench, 25 Nov: 1 tick best once the boxes keep surfaces on the attached
        /// side of their lift curves.)
        /// </summary>
        public const int ReachTicks = 1;

        /// <summary>The negative load-factor limit, as a share of the positive one (-3 g on a 9 g jet).</summary>
        public const float NegativeG = 1f / 3f;

        public readonly RateLaw Law = new RateLaw();
        public readonly CommandGovernor Governor = new CommandGovernor();
        public readonly IndiLoop Indi;

        /// <summary>
        /// The airframe, filled by the caller each tick: its lift slope (g/rad, from <see cref="LiftCurve"/>; 0 = unknown,
        /// no envelope), AoA limits (rad), and whether Stability Assist is on (off: no AoA limit, as stock).
        /// </summary>
        public float LiftSlope, AlphaMax = 0.35f, AlphaMin = -0.26f;

        /// <summary>The airframe's static pitch stiffness (rad/s² per rad of AoA, + = pitches up as AoA grows), filled by the caller.</summary>
        public float PitchStiffness;

        /// <summary>Inertia coupling gain (I_yaw − I_roll) / I_pitch in body axes, filled by the caller.</summary>
        public float CouplingGain;

        public bool Assist = true;
        public readonly int Slots;

        // per slot, filled by the caller each tick
        public readonly float[] Position, ReachLo, ReachHi, Neutral, CostScale;
        public readonly float[] UsableLo, UsableHi; // the whole range the slot may use (travel, attached flow), set with its box

        public FlightComputer(int slots)
        {
            Slots = slots;
            Indi = new IndiLoop(slots);
            Position = new float[slots];
            ReachLo = new float[slots];
            ReachHi = new float[slots];
            Neutral = new float[slots];
            CostScale = new float[slots];
            UsableLo = new float[slots];
            UsableHi = new float[slots];
            for (int j = 0; j < slots; j++) CostScale[j] = 1f;
        }

        /// <summary>Slot j's angular acceleration per unit (rad/s² per deg) on roll, pitch, yaw.</summary>
        public void SetEffect(int j, V3 aero)
        {
            float[] b = Indi.Alloc.B;
            b[3 * j] = aero.X;
            b[3 * j + 1] = aero.Y;
            b[3 * j + 2] = aero.Z;
        }

        /// <summary>
        /// Slot j's box: what its servo reaches, within the range that keeps it usable (the attached side of a
        /// surface's lift curve, within its travel). Outside that range and too far to get back this tick: as far toward it as it reaches.
        /// </summary>
        public void SetBox(int j, float reachLo, float reachHi, float usableLo, float usableHi)
        {
            UsableLo[j] = usableLo;
            UsableHi[j] = usableHi;
            float lo = Math.Max(reachLo, usableLo), hi = Math.Min(reachHi, usableHi);
            if (lo > hi)
            {
                lo = hi = usableHi < reachLo ? reachLo : reachHi;
            }
            ReachLo[j] = lo;
            ReachHi[j] = hi;
        }

        /// <summary>A slot that can't act this tick: no effect, held where it is.</summary>
        public void Disable(int j, float position)
        {
            SetEffect(j, V3.Zero);
            Position[j] = ReachLo[j] = ReachHi[j] = UsableLo[j] = UsableHi[j] = position;
        }

        /// <summary>Start (or restart) from the current rates, so engaging is bumpless.</summary>
        public void Engage(in FlightState s)
        {
            Law.Reset(s.P, s.Q, s.R);
            Indi.Reset();
        }

        public float[] Command => Indi.Command;

        public void Step(float stickPitch, float stickRoll, float stickYaw, in FlightState s, float dt)
        {
            Law.Targets(stickPitch, stickRoll, stickYaw, s.Speed, s.Beta, s.Bank, s.Pitch, Assist, out float psT, out float qT, out float rYaw);
            CommandGovernor gov = Governor;
            gov.AlphaMax = AlphaMax;
            gov.AlphaMin = AlphaMin;
            gov.AlphaLimited = Assist;
            gov.NzMax = Law.NzMax;
            gov.NzMin = -NegativeG * Law.NzMax;
            if (LiftSlope > 0f)
            {
                float ca = (float)Math.Cos(s.Alpha), sa = (float)Math.Sin(s.Alpha);
                gov.Pitch = Capacity(0f, 1f, 0f, Indi.FilteredQDot, dt);
                gov.Roll = Capacity(ca, 0f, sa, Indi.FilteredPDot * ca + Indi.FilteredRDot * sa, dt);
                gov.Stiffness = PitchStiffness;
                gov.CouplingGain = CouplingGain;
                gov.Tau = Law.Tau;
                gov.KPitch = Indi.KPitch;
                gov.KRoll = Indi.KRoll;
                gov.Govern(new CommandGovernor.Now
                {
                    PitchTarget = qT, RollTarget = psT,
                    PitchFilter = Law.Q, RollFilter = Law.P * ca + Law.R * sa,
                    Q = s.Q, QDot = Indi.FilteredQDot, Roll = s.P * ca + s.R * sa, RollDot = Indi.FilteredPDot * ca + Indi.FilteredRDot * sa,
                    Alpha = s.Alpha, Nz = s.Nz, NzGravity = (float)(Math.Cos(s.Pitch) * Math.Cos(s.Bank)), Speed = s.Speed, LiftSlope = LiftSlope,
                    FullPitch = Law.FullStickPitch(s.Speed, Assist), FullRoll = Law.PMax,
                    PitchLimit = Law.QMax, RollLimit = Law.PMax,
                }, dt);
                qT = gov.Governed;
                psT = gov.GovernedRoll;
            }
            else
            {
                gov.Wanted = gov.Governed = qT;
                gov.WantedRoll = gov.GovernedRoll = psT;
            }
            RateLaw.Body(psT, rYaw, s.Alpha, out float pT, out float rT);
            Law.Filter(pT, qT, rT, dt);
            Indi.Step(Slots, s.P, s.Q, s.R, s.PDot, s.QDot, s.RDot, s.Alpha, Law, Position, ReachLo, ReachHi, Neutral, CostScale, dt);
            // What the effectors couldn't give this tick, as acceleration (the INDI asked for IncrementGain of the gap),
            // comes off every axis's command. (Bench: hedging roll alone left the Ifrit 8 deg of sideslip in a full roll,
            // all three 4.7.)
            float g = 1f / Indi.IncrementGain;
            Law.Hedge(Indi.UnmetRoll * g, Indi.UnmetPitch * g, Indi.UnmetYaw * g, s.P, s.Q, s.R, s.Alpha, dt);
        }

        /// <summary>
        /// The effectors' capacity about one axis (weights on body roll, pitch, yaw; for stability-axis roll, cos α and sin α):
        /// the acceleration they reach at full usable travel from where they are, on top of <paramref name="now"/>, and how fast
        /// that can change (one tick of servo travel). Each axis as if the others needed nothing: the effectors' most, which
        /// hedging trims when they do.
        /// </summary>
        private AxisCapacity Capacity(float wRoll, float wPitch, float wYaw, float now, float dt)
        {
            float[] b = Indi.Alloc.B;
            float up = 0f, down = 0f, jUp = 0f, jDown = 0f;
            for (int j = 0; j < Slots; j++)
            {
                float bj = wRoll * b[3 * j] + wPitch * b[3 * j + 1] + wYaw * b[3 * j + 2], u = Position[j];
                float t1 = bj * (UsableHi[j] - u), t2 = bj * (UsableLo[j] - u);
                up += Math.Max(0f, Math.Max(t1, t2));
                down += Math.Min(0f, Math.Min(t1, t2));
                float r1 = bj * (ReachHi[j] - u), r2 = bj * (ReachLo[j] - u);
                jUp += Math.Max(0f, Math.Max(r1, r2));
                jDown += Math.Min(0f, Math.Min(r1, r2));
            }
            return new AxisCapacity { Lo = now + down, Hi = now + up, JerkLo = jDown / dt, JerkHi = jUp / dt };
        }

        /// <summary>
        /// A world vector (moment, angular acceleration) in aerospace body axes (roll, pitch, yaw), given the
        /// body's world rotation. Unity is left-handed: +x turns the nose down, +z rolls left (see AirData).
        /// </summary>
        public static V3 ToAero(Q4 bodyRotation, V3 world)
        {
            V3 b = bodyRotation.Inverse * world;
            return new V3(-b.Z, -b.X, b.Y);
        }

        /// <summary>Bank and pitch attitude (rad) from the body's world rotation.</summary>
        public static void Attitude(Q4 bodyRotation, out float bank, out float pitch)
        {
            V3 right = bodyRotation * new V3(1f, 0f, 0f), up = bodyRotation * new V3(0f, 1f, 0f), fwd = bodyRotation * new V3(0f, 0f, 1f);
            bank = (float)Math.Atan2(-right.Y, up.Y);
            pitch = (float)Math.Asin(Math.Max(-1f, Math.Min(1f, fwd.Y)));
        }
    }
}
