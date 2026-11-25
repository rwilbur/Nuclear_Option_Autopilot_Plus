using System;
using AutopilotMod.Fbw.Model;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// The whole airframe's lift against angle of attack, from the game's own plates: every aero part's plate
    /// input of this tick with its air turned about the body's lateral axis, so the AoA changes and nothing else
    /// does. Gives the envelope its two airframe numbers:
    /// - the slope of the normal load (along body up, what n_z measures) at the current AoA and flow, in g per
    ///   rad, which links predicted AoA and g;
    /// - the AoA limits: the lowest AoA on each side that gives <see cref="LiftShare"/> of the airframe's peak lift
    ///   (lift across the flight path, not the normal load, which drag keeps raising long after the stall). On
    ///   these jets the lift curve flattens into a long plateau (Ifrit: 1 g/rad from 20 deg on, peak at 42): past
    ///   the limit the lift barely grows while the drag and the departure risk do.
    /// The limits come from the clean airframe (surfaces neutral, the CG's air on every part), so trim, the
    /// manoeuvre's own pitch rate and control inputs don't move them; configuration and damage do.
    /// The caller fills <see cref="Parts"/>, <see cref="Hinge"/>, <see cref="Deflection"/> and <see cref="Count"/> each tick, then calls <see cref="Update"/>.
    /// </summary>
    internal sealed class LiftCurve
    {
        /// <summary>The AoA limit gives this share of the peak lift.</summary>
        public const float LiftShare = 0.97f;

        private const float Deg2Rad = 0.017453292f;
        private const float SlopeStep = 1.5f;              // deg each side: about one point of the game's 2.8-deg chart grid
        private const float SweepLo = -40f, SweepHi = 60f; // deg of AoA searched
        private const int SweepEvery = 30;                 // ticks: the limits move with configuration, not with the manoeuvre
        private const float SweepSlipMax = 0.17f;          // sin of the most sideslip (10 deg) a sweep is taken in

        public readonly Plate.Input[] Parts;  // live plate inputs
        public readonly V3[] Hinge;           // per part: its surface's hinge (same frame), if it is a control surface
        public readonly float[] Deflection;   // per part: its surface's deflection now (deg); 0 for fixed parts
        public readonly V3[] Arm;             // per part: from the CG to where its force acts (its rigidbody's centre of mass)
        public int Count;

        /// <summary>d(n_z)/dα at the current AoA (g per rad).</summary>
        public float Slope { get; private set; }

        /// <summary>d(aero moment about the CG)/dα at the current AoA (N m per rad, the parts' frame): the airframe's static pitch stiffness.</summary>
        public V3 MomentSlope { get; private set; }

        /// <summary>The AoA limits (rad, body AoA).</summary>
        public float AlphaMax { get; private set; } = 15f * Deg2Rad;
        public float AlphaMin { get; private set; } = -10f * Deg2Rad;

        /// <summary>Where the lift peaks (rad), for the display and the logs.</summary>
        public float PeakPos { get; private set; }
        public float PeakNeg { get; private set; }

        /// <summary>The limits have been found at least once.</summary>
        public bool Known { get; private set; }

        /// <summary>The latest sweep's airspeed (m/s) and weight (N): its lift scales with the airspeed squared.</summary>
        public float SweepSpeed { get; private set; }
        public float SweepWeight { get; private set; }

        /// <summary>The latest sweep's lift (N, across the flight path) at <paramref name="alpha"/> (rad), interpolated; 0 before the first sweep.</summary>
        public float LiftAt(float alpha)
        {
            if (!Known) return 0f;
            float x = Math.Max(0f, Math.Min(sweep.Length - 1.001f, alpha / Deg2Rad - SweepLo));
            int k = (int)x;
            return sweep[k] + (x - k) * (sweep[k + 1] - sweep[k]);
        }

        private readonly float[] sweep = new float[(int)(SweepHi - SweepLo) + 1];
        private readonly Plate.Input[] clean;
        private int sinceSweep;
        private float maxTarget, minTarget;   // the limits from the latest sweep (rad)
        private const float LimitRate = 2f;   // deg/s the limits may move: slower than the governor's horizon
        private float[] liftChart, dragChart; // the game's airfoil charts, as of the last update

        public LiftCurve(int capacity)
        {
            Parts = new Plate.Input[capacity];
            clean = new Plate.Input[capacity];
            Hinge = new V3[capacity];
            Deflection = new float[capacity];
            Arm = new V3[capacity];
        }

        /// <summary>
        /// <paramref name="alpha"/>: the body AoA now (rad). <paramref name="flow"/>: the air velocity at the CG; <paramref name="right"/>,
        /// <paramref name="up"/>: the body's lateral and up axes; all in the frame of the parts' inputs. <paramref name="weight"/>: m g (N).
        /// </summary>
        public void Update(float alpha, V3 flow, V3 right, V3 up, float weight, float[] lift, float[] drag, float dt)
        {
            if (Count == 0 || !(weight > 0f) || !(flow.SqrMagnitude > 0f)) return;
            liftChart = lift;
            dragChart = drag;
            float nUp = Load(Parts, SlopeStep, flow, right, up, out _, out V3 mUp);
            float nDown = Load(Parts, -SlopeStep, flow, right, up, out _, out V3 mDown);
            Slope = (nUp - nDown) / (2f * SlopeStep * Deg2Rad * weight);
            MomentSlope = (mUp - mDown) / (2f * SlopeStep * Deg2Rad);
            if (Known)
            {
                // the limits move toward the latest sweep at LimitRate: a step under a jet sitting at its limit made the governor
                // kick the nose down for a tick (Vortex log, 25 Nov: 32 -> 31 deg)
                float step = LimitRate * Deg2Rad * dt;
                AlphaMax += Math.Max(-step, Math.Min(step, maxTarget - AlphaMax));
                AlphaMin += Math.Max(-step, Math.Min(step, minTarget - AlphaMin));
            }
            if (Known && ++sinceSweep < SweepEvery) return;
            // turning sideslipping air about the wing isn't an AoA sweep: the limits fell from 33 to 15 deg in a Revoker tumble (25 Nov).
            // The limits move with configuration only, so a skipped sweep costs nothing; the next straight tick sweeps.
            if (Known && Math.Abs(V3.Dot(flow, right)) > SweepSlipMax * flow.Magnitude) return;
            sinceSweep = 0;

            for (int i = 0; i < Count; i++)
            {
                Plate.Input p = Parts[i];
                p.VAir = flow;
                if (Deflection[i] != 0f) p.LiftRot = Q4.AngleAxis(-Deflection[i], Hinge[i]) * p.LiftRot;
                clean[i] = p;
            }
            float a0 = alpha / Deg2Rad, best = float.MinValue, worst = float.MaxValue;
            int iPos = 0, iNeg = 0, i0 = (int)-SweepLo;
            for (int k = 0; k < sweep.Length; k++)
            {
                Load(clean, SweepLo + k - a0, flow, right, up, out float l, out _);
                sweep[k] = l;
                if (k >= i0 && l > best) { best = l; iPos = k; }
                if (k <= i0 && l < worst) { worst = l; iNeg = k; }
            }
            int hi = iPos, lo = iNeg;
            float top = LiftShare * best, bottom = LiftShare * worst;
            while (hi > i0 && sweep[hi - 1] >= top) hi--;
            while (lo < i0 && sweep[lo + 1] <= bottom) lo++;
            // where the curve crosses the share, between the sweep's 1-deg points
            float aHi = SweepLo + hi, aLo = SweepLo + lo;
            if (hi > i0 && sweep[hi] > sweep[hi - 1]) aHi -= (sweep[hi] - top) / (sweep[hi] - sweep[hi - 1]);
            if (lo < i0 && sweep[lo] < sweep[lo + 1]) aLo += (bottom - sweep[lo]) / (sweep[lo + 1] - sweep[lo]);
            maxTarget = aHi * Deg2Rad;
            minTarget = aLo * Deg2Rad;
            if (!Known)
            {
                AlphaMax = maxTarget;
                AlphaMin = minTarget;
            }
            SweepSpeed = flow.Magnitude;
            SweepWeight = weight;
            PeakPos = (SweepLo + iPos) * Deg2Rad;
            PeakNeg = (SweepLo + iNeg) * Deg2Rad;
            Known = true;
        }

        /// <summary>
        /// The aero moment about the CG (returned) and force of <paramref name="parts"/>, with every part's air turned by <paramref name="turn"/>
        /// and the body turning at <paramref name="omega"/> more (the game's frame and sign).
        /// </summary>
        private V3 Response(Plate.Input[] parts, Q4 turn, V3 omega, out V3 force)
        {
            V3 sum = V3.Zero, m = V3.Zero;
            for (int i = 0; i < Count; i++)
            {
                Plate.Input p = parts[i];
                p.VAir = turn * p.VAir + V3.Cross(omega, Arm[i]);
                V3 f = Plate.Force(p, liftChart, dragChart, out _);
                sum += f;
                m += V3.Cross(Arm[i], f);
                if (p.CenterOfLift.SqrMagnitude > 0f) m += Plate.Torque(p, f);
            }
            force = sum;
            return m;
        }

        /// <summary>
        /// The aero force of <paramref name="parts"/> along body up (N, returned) and across the flight path (<paramref name="liftForce"/>,
        /// N, + toward body up) and moment about the CG (<paramref name="moment"/>) with the air turned to <paramref name="delta"/> deg more AoA.
        /// </summary>
        private float Load(Plate.Input[] parts, float delta, V3 flow, V3 right, V3 up, out float liftForce, out V3 moment)
        {
            // the air turned about +right by +delta: in the game's left-handed frame that raises the AoA by delta (SelfCheck)
            Q4 turn = Q4.AngleAxis(delta, right);
            V3 f = (turn * flow).Normalized;
            V3 across = (up - f * V3.Dot(up, f)).Normalized;
            moment = Response(parts, turn, V3.Zero, out V3 sum);
            liftForce = V3.Dot(sum, across);
            return V3.Dot(sum, up);
        }
    }
}
