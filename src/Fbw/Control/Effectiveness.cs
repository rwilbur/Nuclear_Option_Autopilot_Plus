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
                float best = float.MinValue, worst = float.MaxValue;
                for (int i = 0; i < Plate.ChartPoints; i++)
                {
                    float deg = (i - 64) * 0.04908734f * Rad2Deg, cl = lift[f * Plate.ChartPoints + i];
                    if (deg >= 0f && deg <= 60f && cl > best) { best = cl; pos[f] = deg; }
                    if (deg <= 0f && deg >= -60f && cl < worst) { worst = cl; neg[f] = deg; }
                }
            }
        }

        /// <summary>Stall angles of a foil; the game's built-in curve (foil −1, CL = 1.8 sin 5α) peaks at ±18 deg.</summary>
        public void Of(int foil, out float stallNeg, out float stallPos)
        {
            if (foil < 0 || pos == null || foil >= pos.Length) { stallNeg = -18f; stallPos = 18f; return; }
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
