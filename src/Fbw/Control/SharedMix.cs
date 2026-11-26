using System;
using AutopilotMod.Fbw.Model;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// Multiplayer mode's last layer: the aircraft's three shared inputs (pitch, roll, yaw; ±1) are the
    /// <see cref="FlightComputer"/>'s slots, seen through the game's own mixing (docs/fbw/PHASE0_FACTS.md 3 and 5).
    /// Every surface and nozzle moves by stock mixing, so other players see exactly what flies.
    ///
    /// Each tick the caller hands in every effector with its effect per degree at the live flow. A slot's effect is the sum,
    /// over effectors, of effect × degrees per unit input, taken where the mixing is now: a channel already clamped, or a
    /// nozzle past its gain's clamp, adds nothing. The box is one tick of servo travel, as an effect-weighted mean over the
    /// effectors. The usable range stops the input where most of its effect would be past its surfaces' stall (see <see cref="Fill"/>).
    /// </summary>
    internal sealed class SharedMix
    {
        public const int Slots = 3; // pitch, roll, yaw: the surfaces' channel order

        /// <summary>An input stops where less than this share of its effect now is still on the attached side.</summary>
        public const float AttachedShare = 0.5f;

        private readonly float[] input = new float[Slots]; // the inputs the mixing follows now (the last written)
        private readonly V3[] b = new V3[Slots];
        private readonly float[] reachSum = new float[Slots], weightSum = new float[Slots];
        // per entry: its input, its effect weight, and the input range it can use (stall)
        private readonly int[] entryInput;
        private readonly float[] entryWeight, entryLo, entryHi, sortKey, sortWeight;
        private int entries;

        public SharedMix(int effectors)
        {
            int n = Slots * Math.Max(1, effectors);
            entryInput = new int[n];
            entryWeight = new float[n];
            entryLo = new float[n];
            entryHi = new float[n];
            sortKey = new float[n];
            sortWeight = new float[n];
        }

        /// <summary>Start a tick from the inputs the mixing follows now.</summary>
        public void Begin(float pitch, float roll, float yaw)
        {
            input[0] = pitch;
            input[1] = roll;
            input[2] = yaw;
            for (int c = 0; c < Slots; c++)
            {
                b[c] = V3.Zero;
                reachSum[c] = weightSum[c] = 0f;
            }
            entries = 0;
        }

        /// <summary>
        /// A control surface: its effect per degree (rad/s², aerospace body axes), its channel ranges (deg; pitch, roll, yaw), servo
        /// (deg/s per channel) and the deflections it may use (its attached range within travel, deg).
        /// </summary>
        public void Surface(V3 perDeg, float[] range, float servo, float usableLo, float usableHi, float dt)
        {
            float steady = 0f;
            for (int c = 0; c < Slots; c++) steady += Clamp(input[c], -1f, 1f) * range[c];
            for (int c = 0; c < Slots; c++)
            {
                float r = range[c];
                if (r == 0f) continue;
                // the deflection with input c at u and the others where they are: steady - input[c] r + u r
                float a = input[c] + (usableLo - steady) / r, z = input[c] + (usableHi - steady) / r;
                Add(c, perDeg * r, servo * dt / Math.Abs(r), Math.Min(a, z), Math.Max(a, z));
            }
        }

        /// <summary>
        /// A vectoring engine's nozzle (Turbojet/Turbofan.FixedUpdate): its effect per degree of nozzle pitch and yaw, whether each
        /// moves now, its vectoring angles (x pitch, y yaw, z roll into pitch) and input gains, and its slew (deg/s).
        /// </summary>
        public void Nozzle(V3 perDegPitch, V3 perDegYaw, bool pitchOn, bool yawOn, float vecX, float vecY, float vecZ, float gainX, float gainZ, float slew, float dt)
        {
            if (pitchOn)
            {
                // pitch angle = -clamp(pitch gx) vx - clamp(roll gz) vz
                if (vecX != 0f && Math.Abs(input[0] * gainX) < 1f) Add(0, perDegPitch * (-gainX * vecX), slew * dt / Math.Abs(gainX * vecX), -1f, 1f);
                if (vecZ != 0f && Math.Abs(input[1] * gainZ) < 1f) Add(1, perDegPitch * (-gainZ * vecZ), slew * dt / Math.Abs(gainZ * vecZ), -1f, 1f);
            }
            // yaw angle = clamp(-yaw) vy
            if (yawOn && vecY != 0f) Add(2, perDegYaw * -vecY, slew * dt / Math.Abs(vecY), -1f, 1f);
        }

        /// <summary>A split surface (drag rudder): effect per degree of split; it opens by yawSplitFactor × yaw × maxSplit (not at idle).</summary>
        public void Split(V3 perDeg, float yawSplitFactor, float maxSplit, bool idle)
        {
            float span = yawSplitFactor * maxSplit, open = span * input[2];
            if (idle || span == 0f || open >= maxSplit) return;
            // shortcut: at a centred pedal it is a kink (one side opens either way); taken as half the slope until the pedal picks a side
            float slope = open > 0f ? span : open < 0f ? 0f : 0.5f * span;
            b[2] += perDeg * slope;
        }

        /// <summary>The three slots into the computer: effect, position (the inputs now), one tick's reach, the usable range.</summary>
        public void Fill(FlightComputer fc)
        {
            for (int c = 0; c < Slots; c++)
            {
                // Stall: each effector's limit counts by its effect now, and the input stops where less than AttachedShare of that would
                // still be attached. Not every carrier's limit: past their knee the Ifrit's tails gave a fifth of the pitch while its
                // nozzles, on the same input, still pushed the nose down; their limit froze the input and the nose ran to 62 deg (bench).
                // Not none: the roll ran on after release, as in Full mode before the knee rule (bench, Compass and Revoker damage cases).
                // And never back toward the attached side: that drags the whole input, every other effector on it with it.
                float u = Clamp(input[c], -1f, 1f);
                float hi = Math.Max(u, Limit(c, +1f)), lo = Math.Min(u, -Limit(c, -1f));
                // shortcut: an effect-weighted mean reach lets the fast effectors set the pace and a slow nozzle lag a little; the
                // slowest one's reach would make a strong nozzle (Ifrit roll, 75 deg per unit) hold the whole input to 1 s end to end
                float reach = weightSum[c] > 0f ? reachSum[c] / weightSum[c] : 1f;
                fc.SetEffect(c, b[c]);
                fc.Position[c] = u;
                fc.SetBox(c, Math.Max(-1f, u - reach), Math.Min(1f, u + reach), lo, hi);
            }
        }

        // the furthest input (times side: +1 up, -1 down) at which AttachedShare of input c's effect is still attached; 1 at most
        private float Limit(int c, float side)
        {
            int m = 0;
            float total = 0f;
            for (int k = 0; k < entries; k++)
            {
                if (entryInput[k] != c) continue;
                sortKey[m] = -side * (side > 0f ? entryHi[k] : entryLo[k]); // ascending = the furthest limit first
                sortWeight[m++] = entryWeight[k];
                total += entryWeight[k];
            }
            if (m == 0 || !(total > 0f)) return 1f;
            Array.Sort(sortKey, sortWeight, 0, m);
            float held = 0f;
            for (int i = 0; i < m; i++)
            {
                held += sortWeight[i];
                if (held >= AttachedShare * total) return Math.Min(1f, -sortKey[i]);
            }
            return 1f;
        }

        private void Add(int c, V3 effect, float reach, float lo, float hi)
        {
            b[c] += effect;
            float w = effect.Magnitude;
            reachSum[c] += w * reach;
            weightSum[c] += w;
            if (entries == entryInput.Length) return; // more effectors than sized for: still in the effect, not in the stall limit
            entryInput[entries] = c;
            entryWeight[entries] = w;
            entryLo[entries] = lo;
            entryHi[entries] = hi;
            entries++;
        }

        private static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;
    }
}
