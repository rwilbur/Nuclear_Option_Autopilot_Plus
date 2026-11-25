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

        public readonly RateLaw Law = new RateLaw();
        public readonly IndiLoop Indi;
        public readonly int Slots;

        // per slot, filled by the caller each tick
        public readonly float[] Position, ReachLo, ReachHi, Neutral, CostScale;

        public FlightComputer(int slots)
        {
            Slots = slots;
            Indi = new IndiLoop(slots);
            Position = new float[slots];
            ReachLo = new float[slots];
            ReachHi = new float[slots];
            Neutral = new float[slots];
            CostScale = new float[slots];
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
        /// surface's lift curve). Outside that range and too far to get back this tick: as far toward it as it reaches.
        /// </summary>
        public void SetBox(int j, float reachLo, float reachHi, float usableLo, float usableHi)
        {
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
            Position[j] = ReachLo[j] = ReachHi[j] = position;
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
            Law.Step(stickPitch, stickRoll, stickYaw, s.Speed, s.Alpha, s.Beta, s.Bank, s.Pitch, dt);
            Indi.Step(Slots, s.P, s.Q, s.R, s.PDot, s.QDot, s.RDot, Law, Position, ReachLo, ReachHi, Neutral, CostScale, dt);
            // What the effectors couldn't give this tick, as acceleration (the INDI asked for IncrementGain of the gap),
            // comes off every axis's command. (Bench: hedging roll alone left the Ifrit 8 deg of sideslip in a full roll,
            // all three 4.7.)
            float g = 1f / Indi.IncrementGain;
            Law.Hedge(Indi.UnmetRoll * g, Indi.UnmetPitch * g, Indi.UnmetYaw * g, s.P, s.Q, s.R, dt);
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
