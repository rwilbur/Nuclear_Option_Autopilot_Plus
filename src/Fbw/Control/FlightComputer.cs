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

        /// <summary>
        /// The airframe's response to stability-axis roll rate, filled by the caller (<see cref="RollResponse"/>): body angular acceleration
        /// (aerospace axes, rad/s² per rad/s) from the aero rate damping with the surfaces where they are, and the gyroscopic coupling.
        /// Zero: no coordinated-roll limit.
        /// </summary>
        public V3 RollAccel;

        /// <summary>The stability-axis roll rates (rad/s) the effectors can hold steady with the yaw coordinated, as of the latest step.</summary>
        public float RollLo = -99f, RollHi = 99f;

        /// <summary>The stability-axis roll accelerations (rad/s²) the effectors give this tick with pitch and yaw served, as of the latest step.</summary>
        public float RollAccelLo, RollAccelHi;
        private const float OnsetTau = 0.03f; // s

        /// <summary>
        /// Sideslip the roll may build before the coordinated-roll limits take over (rad, filled by the caller; 0 = always coordinated):
        /// they fade in as |β| grows to it, so a roll runs at the stick's rate while the sideslip stays small.
        /// </summary>
        public float SlipAllowance = 3f / 57.29578f;

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

        /// <summary>The state of the latest <see cref="Step"/>, for the display.</summary>
        public FlightState Last;

        public void Step(float stickPitch, float stickRoll, float stickYaw, in FlightState s, float dt)
        {
            Last = s;
            Law.Targets(stickPitch, stickRoll, stickYaw, s.Speed, s.Beta, s.Bank, s.Pitch, Assist, out float psT, out float qT, out float rYaw);
            // sideslip protection: no roll faster than the yaw can coordinate (a roll the yaw can't follow turns AoA into sideslip,
            // beta rate = -(stability yaw rate): Revoker log and bench, 25 Nov). It only slows the pilot's roll, never reverses it.
            CoordinatedRoll(s, dt);
            float psFree = psT, free = SlipAllowance > 0f ? Math.Max(0f, 1f - Math.Abs(s.Beta) / SlipAllowance) : 0f;
            psT = psT > 0f ? Math.Max(0f, Math.Min(psT, RollHi)) : Math.Min(0f, Math.Max(psT, RollLo));
            CommandGovernor gov = Governor;
            gov.AlphaMax = AlphaMax;
            gov.AlphaMin = AlphaMin;
            gov.AlphaLimited = Assist;
            gov.NzMax = Law.NzLimit;
            gov.NzMin = -NegativeG * Law.NzLimit;
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
                    Alpha = s.Alpha, Beta = s.Beta, Nz = s.Nz, NzGravity = (float)(Math.Cos(s.Pitch) * Math.Cos(s.Bank)), Speed = s.Speed, LiftSlope = LiftSlope,
                    FullPitch = Law.FullStickPitch(s.Speed, Assist), FullRoll = Law.PMax,
                    PitchLimit = Law.QMax,
                }, dt);
                qT = gov.Governed;
                psT = gov.GovernedRoll;
            }
            else
            {
                gov.Wanted = gov.Governed = qT;
                gov.WantedRoll = gov.GovernedRoll = psT;
            }
            // onset: the roll command speeds up no faster than the coordinated roll acceleration (rolling out is never held back)
            {
                float ca = (float)Math.Cos(s.Alpha), sa = (float)Math.Sin(s.Alpha), k = dt / (Law.Tau + dt);
                float ps = Law.P * ca + Law.R * sa;
                if (psT > ps && ps >= 0f) psT = Math.Min(psT, ps + Math.Max(0f, RollAccelHi) * dt / k);
                else if (psT < ps && ps <= 0f) psT = Math.Max(psT, ps + Math.Min(0f, RollAccelLo) * dt / k);
            }
            psT += free * (psFree - psT); // within the sideslip allowance the stick's roll, fading to the coordinated one
            RateLaw.Body(psT, rYaw, s.Alpha, out float pT, out float rT);
            Law.Filter(pT, qT, rT, dt);
            Indi.Step(Slots, s.P, s.Q, s.R, s.PDot, s.QDot, s.RDot, s.Alpha, Law, Position, ReachLo, ReachHi, Neutral, CostScale, dt);
            // What the effectors couldn't give this tick, as acceleration (the INDI asked for IncrementGain of the gap),
            // comes off every axis's command. (Bench: hedging roll alone left the Ifrit 8 deg of sideslip in a full roll,
            // all three 4.7.)
            float g = 1f / Indi.IncrementGain;
            Law.Hedge(Indi.UnmetRoll * g, Indi.UnmetPitch * g, Indi.UnmetYaw * g, s.P, s.Q, s.R, s.Alpha, dt);
        }

        private float[] lpA, lpB, lpQ, lpObj, lpLo, lpHi, lpKey, lpX;
        private int[] lpOrder;

        /// <summary>
        /// <see cref="RollLo"/>/<see cref="RollHi"/>: the range of stability-axis roll rate ps at which the effectors, from where they are,
        /// can hold roll, pitch and stability yaw steady. Steady at ps means ω̇_now + B·x + (ps − ps_now)·A = 0 (stability axes), A the
        /// roll response: the line c + Δ·d (c = −ω̇_now, d = −A) must stay inside the set of reachable increments B·x. Pitch is held
        /// first (an effector that pitches and yaws, like the Vortex's V-tail, is pitch's before it is yaw's, as in the allocator).
        /// Solved exactly across the line in roll/yaw (<see cref="Sweep"/>) inside a bisection on the price of pitch.
        /// </summary>
        private void CoordinatedRoll(in FlightState s, float dt)
        {
            float ca = (float)Math.Cos(s.Alpha), sa = (float)Math.Sin(s.Alpha);
            float ps = s.P * ca + s.R * sa, psDot = Indi.FilteredPDot * ca + Indi.FilteredRDot * sa;
            // steady: the roll rates the yaw (and pitch) can hold
            V3 c = -new V3(psDot, Indi.FilteredQDot, Indi.FilteredRDot * ca - Indi.FilteredPDot * sa);
            if (LineRange(-new V3(RollAccel.X * ca + RollAccel.Z * sa, RollAccel.Y, RollAccel.Z * ca - RollAccel.X * sa), c, ca, sa, out float lo, out float hi))
            {
                RollLo = ps + lo;
                RollHi = ps + hi;
            }
            else
            {
                RollLo = -99f;
                RollHi = 99f;
            }
            // onset: the roll acceleration the effectors give this tick while pitch keeps what it got and yaw gets what the INDI asked
            // (a roll that outruns the yaw's servos leaves sideslip behind: Vortex V-tail, bench 25 Nov)
            // Over the usable travel, not this tick's servo reach: a one-tick limit starved the roll (the INDI asks little of a held-back
            // command, so the surfaces barely move and the limit stays small; bench 25 Nov, rolling pulls at 90 m/s rolled 1 deg/s).
            LineRange(new V3(1f, 0f, 0f), new V3(0f, Indi.GotPitch, Indi.RequestYaw), ca, sa, out lo, out hi);
            // smoothed: from the measured acceleration it flickers tick to tick, and so did the Revoker's surfaces (bench chatter 3 -> 6)
            float k = dt / (OnsetTau + dt);
            RollAccelLo += (psDot + lo - RollAccelLo) * k;
            RollAccelHi += (psDot + hi - RollAccelHi) * k;
        }

        /// <summary>
        /// The Δ range for which c + Δ·d (stability axes: roll, pitch, yaw; rad/s²) lies in the effectors' reachable increments, from
        /// where they are to their usable range. False if d is ~0.
        /// </summary>
        private bool LineRange(V3 d, V3 c, float ca, float sa, out float lo, out float hi)
        {
            lo = -99f;
            hi = 99f;
            float dn = d.Magnitude;
            if (!(dn > 1e-4f)) return false;
            if (lpA == null)
            {
                lpA = new float[Slots]; lpB = new float[Slots]; lpQ = new float[Slots]; lpObj = new float[Slots];
                lpLo = new float[Slots]; lpHi = new float[Slots]; lpKey = new float[Slots]; lpX = new float[Slots];
                lpOrder = new int[Slots];
            }
            // e along the line; n2 the pitch axis made square to it; n1 = e x n2 across it in roll/yaw
            V3 e = d / dn, pitch = new V3(0f, 1f, 0f) - e * e.Y;
            if (pitch.SqrMagnitude < 1e-6f) pitch = new V3(1f, 0f, 0f) - e * e.X;
            V3 n2 = pitch / pitch.Magnitude, n1 = V3.Cross(e, n2);
            float[] b = Indi.Alloc.B;
            for (int j = 0; j < Slots; j++)
            {
                V3 m = new V3(b[3 * j] * ca + b[3 * j + 2] * sa, b[3 * j + 1], b[3 * j + 2] * ca - b[3 * j] * sa);
                lpA[j] = V3.Dot(e, m);
                lpB[j] = V3.Dot(n1, m);
                lpQ[j] = V3.Dot(n2, m);
                lpLo[j] = UsableLo[j] - Position[j];
                lpHi[j] = UsableHi[j] - Position[j];
            }
            float along = V3.Dot(e, c), across = V3.Dot(n1, c), pitchNeed = V3.Dot(n2, c);
            hi = (Furthest(1f, across, pitchNeed) - along) / dn;
            lo = (Furthest(-1f, across, pitchNeed) - along) / dn; // Furthest gives Σ a x itself, here its least
            return true;
        }

        // Σ a x furthest in direction sign along the line, with Σ b x = across and Σ q x = pitchNeed (or the most pitch there is)
        private float Furthest(float sign, float across, float pitchNeed)
        {
            // objective cosθ·sign·a − sinθ·q: θ up prices pitch more, so Σ q x falls; find the θ where it meets pitchNeed
            float lo = -1.55f, hi = 1.55f, theta = 0f;
            if (PitchAt(lo, sign, across) <= pitchNeed) theta = lo;      // even all-out pitch falls short: pitch first
            else if (PitchAt(hi, sign, across) >= pitchNeed) theta = hi; // even pitch-shy, more than needed
            else
            {
                for (int i = 0; i < 30; i++)
                {
                    theta = 0.5f * (lo + hi);
                    if (PitchAt(theta, sign, across) > pitchNeed) lo = theta; else hi = theta;
                }
            }
            PitchAt(theta, sign, across);
            float f = 0f;
            for (int j = 0; j < Slots; j++) f += lpA[j] * lpX[j];
            return f;
        }

        private float PitchAt(float theta, float sign, float across)
        {
            float cs = (float)Math.Cos(theta) * sign, sn = (float)Math.Sin(theta);
            for (int j = 0; j < Slots; j++) lpObj[j] = cs * lpA[j] - sn * lpQ[j];
            Sweep(Slots, lpObj, lpB, lpLo, lpHi, across, lpKey, lpOrder, lpX);
            float q = 0f;
            for (int j = 0; j < Slots; j++) q += lpQ[j] * lpX[j];
            return q;
        }

        /// <summary>
        /// x maximising Σ a_j x_j subject to Σ b_j x_j = <paramref name="need"/>, lo_j ≤ x_j ≤ hi_j (an exact LP: the Lagrangian sweep over
        /// the breakpoints a_j / b_j); returns that max. If <paramref name="need"/> is out of reach, the max at the nearest reachable Σ b x.
        /// <paramref name="key"/>, <paramref name="order"/> are scratch; <paramref name="x"/> gets the solution; all at least <paramref name="n"/> long.
        /// </summary>
        internal static float Sweep(int n, float[] a, float[] b, float[] lo, float[] hi, float need, float[] key, int[] order, float[] x)
        {
            float g = 0f, f = 0f;
            int m = 0;
            for (int j = 0; j < n; j++)
            {
                x[j] = b[j] > 0f ? hi[j] : b[j] < 0f ? lo[j] : a[j] > 0f ? hi[j] : lo[j];
                g += b[j] * x[j];
                f += a[j] * x[j];
                if (b[j] != 0f)
                {
                    key[m] = a[j] / b[j];
                    order[m++] = j;
                }
            }
            if (need >= g) return f; // at or past the most Σ b x there is
            Array.Sort(key, order, 0, m);
            for (int i = 0; i < m; i++)
            {
                int j = order[i];
                float to = b[j] > 0f ? lo[j] : hi[j], step = to - x[j];
                float dg = b[j] * step;
                if (g + dg <= need)
                {
                    float t = (need - g) / dg;
                    x[j] += step * t;
                    return f + a[j] * step * t;
                }
                g += dg;
                f += a[j] * step;
                x[j] = to;
            }
            return f;
        }

        /// <summary>
        /// The airframe's body angular acceleration per rad/s of stability-axis roll (any frame; <paramref name="inertia"/>, <paramref name="omega"/>
        /// and <paramref name="axis"/> in it, game sign): the rate damping <paramref name="rateMoment"/> (<see cref="LiftCurve.RateMoment"/>) and the
        /// change of the gyroscopic term −I⁻¹(ω × Iω).
        /// </summary>
        public static V3 RollResponse(double[] inertia, V3 rateMoment, V3 omega, V3 axis)
        {
            const float h = 0.05f;
            V3 Gyro(V3 w) => MassProps.Solve(inertia, V3.Cross(w, MassProps.Mul(inertia, w)));
            return MassProps.Solve(inertia, rateMoment) - (Gyro(omega + axis * h) - Gyro(omega - axis * h)) / (2f * h);
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
