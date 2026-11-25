using System;
using AutopilotMod.Fbw.Model;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// Each airfoil's stall angles (deg, the game's local-AoA sign): where its lift chart peaks on either side
    /// of zero, within 60 deg. Found once from the charts the game uses.
    /// </summary>
    internal readonly struct FoilPeaks
    {
        private readonly float[] neg, pos;

        public FoilPeaks(float[] lift)
        {
            int foils = lift.Length / Plate.ChartPoints;
            neg = new float[foils];
            pos = new float[foils];
            for (int f = 0; f < foils; f++)
            {
                int o = f * Plate.ChartPoints;
                pos[f] = Knee(lift, o, +1);
                neg[f] = Knee(lift, o, -1);
            }
        }

        /// <summary>Share of the lift slope at zero below which the curve counts as stalled.</summary>
        public const float KneeShare = 0.25f;

        // Where the lift stops growing usefully on one side (deg): the first chart segment whose slope is under KneeShare of the slope
        // at zero, or the peak if that comes first (within 60 deg). Not the peak alone: the Vortex's surfaces' foil climbs 0.90 -> 1.00
        // from 17 to 45 deg, and surfaces allowed onto that plateau sat at 50-65 deg, stalled, and kept a roll going (log, 25 Nov).
        private static float Knee(float[] lift, int o, int side)
        {
            const int Zero = 64;
            float step = 0.04908734f * Rad2Deg;
            float slope0 = side * (lift[o + Zero + side] - lift[o + Zero]);
            float best = float.MinValue, bestDeg = 0f;
            for (int k = 1; k * step <= 60f && Zero + side * (k + 1) >= 0 && Zero + side * (k + 1) < Plate.ChartPoints; k++)
            {
                float here = side * lift[o + Zero + side * k], next = side * lift[o + Zero + side * (k + 1)];
                if (here > best) { best = here; bestDeg = k * step; }
                if (slope0 > 0f && next - here < KneeShare * slope0) return side * k * step;
            }
            return side * bestDeg;
        }

        /// <summary>Stall angles of a foil (<see cref="Knee"/>); the game's built-in curve (foil −1, CL = 1.8 sin 5α) peaks at ±18 deg, knee about 15.</summary>
        public void Of(int foil, out float stallNeg, out float stallPos)
        {
            if (foil < 0 || pos == null || foil >= pos.Length) { stallNeg = -15f; stallPos = 15f; return; }
            stallNeg = neg[foil];
            stallPos = pos[foil];
        }

        private const float Rad2Deg = 57.29578f;
    }

    /// <summary>
    /// How much moment each effector makes per degree, at this tick's flow: what the allocator's B is
    /// built from (divided by the inertia). Surfaces come from the game's own plate (Model.Plate) with the
    /// lift frame turned about the hinge, so stall and reversal show up as they do in the game.
    /// </summary>
    internal static class Effectiveness
    {
        // deg each side of the current deflection. The game's airfoil charts are 128 points over 360 deg
        // (2.8 deg apart) joined linearly, so the lift slope jumps at every point; a span of about one point
        // keeps the effectiveness continuous in the deflection (0.5 deg each side flipped it from tick to tick
        // near stall, and the allocation with it).
        private const float Step = 1.5f;
        private const float Deg2Rad = 0.017453292f;

        /// <summary>
        /// Moment about the CG per degree of deflection (N m / deg, world). <paramref name="hinge"/>: the surface's
        /// rotation axis (its visible mesh's right, world); <paramref name="arm"/>: from the CG to the surface part's centre of mass.
        /// </summary>
        public static V3 Surface(in Plate.Input p, V3 hinge, V3 arm, float[] lift, float[] drag)
        {
            Plate.Input a = p, b = p;
            a.LiftRot = Q4.AngleAxis(Step, hinge) * p.LiftRot;
            b.LiftRot = Q4.AngleAxis(-Step, hinge) * p.LiftRot;
            return (Moment(a, arm, lift, drag) - Moment(b, arm, lift, drag)) / (2f * Step);
        }

        /// <summary>
        /// Moment about the CG per degree of nozzle rotation about <paramref name="axis"/> (N m / deg, world): the thrust
        /// along <paramref name="forward"/> at <paramref name="arm"/> from the CG turns with the nozzle.
        /// </summary>
        public static V3 Nozzle(V3 axis, V3 forward, float thrust, V3 arm) =>
            V3.Cross(arm, V3.Cross(axis, forward) * (thrust * Deg2Rad));

        /// <summary>
        /// Moment about the CG per degree of split (N m / deg, world). A split surface's drag is
        /// −split × splitDrag × ρ |v|² v̂ on its part (ControlSurface.Aero): linear in the split, so its effect is exact.
        /// </summary>
        public static V3 Split(V3 vAir, float rho, float splitDrag, V3 arm) =>
            V3.Cross(arm, vAir * (-splitDrag * rho * vAir.Magnitude));

        /// <summary>
        /// The deflections (deg, absolute) that keep the surface on the attached side of its lift curve: its local AoA
        /// within <paramref name="margin"/> of the airfoil's stall peaks. Past a peak the lift slope turns over, the
        /// linear allocation stops holding, and the surface would be driven in and out of stall every tick. A
        /// surface already past a peak gets a box that brings it back.
        /// </summary>
        public static void AttachedRange(in Plate.Input p, V3 hinge, float deflection, in FoilPeaks peaks, float margin, out float lo, out float hi)
        {
            Plate.Input a = p;
            float alpha0 = Plate.LocalAoA(p);
            a.LiftRot = Q4.AngleAxis(1f, hinge) * p.LiftRot;
            float slope = (Plate.LocalAoA(a) - alpha0) * Rad2Deg; // deg of local AoA per deg of deflection (about +-1)
            peaks.Of(p.Foil, out float stallNeg, out float stallPos);
            float aLo = stallNeg + margin, aHi = stallPos - margin, a0 = alpha0 * Rad2Deg;
            if (Math.Abs(slope) < 0.2f) { lo = float.NegativeInfinity; hi = float.PositiveInfinity; return; } // hinge along the flow
            float d1 = deflection + (aLo - a0) / slope, d2 = deflection + (aHi - a0) / slope;
            lo = Math.Min(d1, d2);
            hi = Math.Max(d1, d2);
        }

        private const float Rad2Deg = 57.29578f;

        private static V3 Moment(in Plate.Input p, V3 arm, float[] lift, float[] drag)
        {
            V3 f = Plate.Force(p, lift, drag, out _);
            V3 m = V3.Cross(arm, f);
            if (p.CenterOfLift.SqrMagnitude > 0f) m += Plate.Torque(p, f);
            return m;
        }
    }
}
