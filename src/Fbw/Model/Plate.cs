using System;

namespace AutopilotMod.Fbw.Model
{
    /// <summary>
    /// The game's aerodynamic plate (NuclearOption.Jobs.AeroJob_Math), in air: lift and drag of one part
    /// from its air velocity, lift-frame rotation and airfoil chart. Mirrors the game's expressions, so
    /// the force matches what the game applies (docs/fbw/PHASE0_FACTS.md section 4). Water, splash and the
    /// below-sea-level push are left out: the FBW does not fly underwater.
    /// </summary>
    internal static class Plate
    {
        public const int ChartPoints = 128;
        private const float ChartScale = 20.37185f; // chart points per radian: 128 over +-pi

        /// <summary>The inputs of one plate evaluation, as the aero job sees them.</summary>
        internal struct Input
        {
            public V3 VAir;            // part velocity minus the wind at the part (m/s, world)
            public Q4 LiftRot;         // the lift transform's world rotation (the surface's deflection is in it)
            public V3 ChannelForward;  // forward of the part's own transform, used when Channeling > 0
            public float Channeling;   // airflowChanneling: max turn of the flow toward ChannelForward (rad)
            public int Foil;           // chart index (airfoilID); -1 = the game's built-in curve
            public float Area, DragArea, Eff; // wingArea, dragArea, wingEffectiveness
            public float Rho, SpeedOfSound;
            public V3 CenterOfLift;    // local to the lift transform; zero = no torque
        }

        /// <summary>The force the game applies to the part's rigidbody (N, world), and the local angle of attack (rad).</summary>
        public static V3 Force(in Input p, float[] lift, float[] drag, out float alpha)
        {
            V3 v = p.VAir;
            float v2 = v.SqrMagnitude;
            alpha = 0f;
            if (!(v2 > 0f)) return V3.Zero; // the game's force is non-finite here and it zeroes it
            if (p.Channeling > 0f) v = RotateTowards(v, p.ChannelForward, p.Channeling);

            V3 vLocal = p.LiftRot.Inverse * v;
            V3 n = V3.Cross(v, p.LiftRot.Right).Normalized;
            alpha = (float)Math.Atan2(vLocal.Y, vLocal.Z);
            Coefficients(lift, drag, p.Foil, alpha, out float cl, out float cd);

            float parasite = (p.DragArea + p.Area * 0.1f) * (1f - p.Eff);
            float dragParasite = 0.5f * p.Rho * v2 * 0.5f * (p.DragArea + parasite);
            float dragFoil = cd * p.Rho * v2 * 0.5f * p.Area * p.Eff;
            V3 dragForce = -(v / v.Magnitude) * (dragFoil + dragParasite);

            float speed = v.Magnitude;
            float a = p.SpeedOfSound;
            if (speed > a * 0.8f && speed < a * 1.2f) // transonic drag rise
            {
                float near = Math.Min(Math.Abs((a - speed) / a), 0.2f);
                float k = (0.2f - near) / 0.2f;
                dragForce *= 1f + k * k * k * 0.15f;
            }
            V3 liftForce = -n * (cl * p.Rho * v2 * 0.5f * p.Area * p.Eff);
            return liftForce + dragForce;
        }

        /// <summary>The torque the game adds with the force: as if applied at the centre of lift (N m, world).</summary>
        public static V3 Torque(in Input p, V3 force) => V3.Cross(p.LiftRot * p.CenterOfLift, force);

        /// <summary>Lift and drag coefficients, read from the charts the way ChartHelper.SafeRead does.</summary>
        public static void Coefficients(float[] lift, float[] drag, int foil, float alpha, out float cl, out float cd)
        {
            if (foil < 0)
            {
                cl = 1.8f * (float)Math.Sin(5f * alpha);
                cd = 1.5f * (1f - (float)Math.Cos(2f * alpha)) + 0.02f;
                return;
            }
            float index = alpha * ChartScale + 64f;
            cl = Read(lift, foil * ChartPoints, index);
            cd = Read(drag, foil * ChartPoints, index);
        }

        /// <summary>ChartHelper.SafeRead: linear between points, clamped at both ends.</summary>
        public static float Read(float[] chart, int offset, float index, int points = ChartPoints)
        {
            if (float.IsNaN(index) || float.IsInfinity(index)) return chart[offset + points - 1];
            if (index <= 0f) return chart[offset];
            if (index >= points - 1) return chart[offset + points - 1];
            int i = (int)Math.Floor(index);
            int j = (int)Math.Ceiling(index);
            float t = index % 1f;
            return chart[offset + i] + (chart[offset + j] - chart[offset + i]) * t;
        }

        /// <summary>Vector3.RotateTowards with a zero magnitude change: turn <paramref name="v"/> toward <paramref name="target"/> by at most <paramref name="maxRad"/>.</summary>
        public static V3 RotateTowards(V3 v, V3 target, float maxRad)
        {
            float vm = v.Magnitude, tm = target.Magnitude;
            if (vm < 1e-5f || tm < 1e-5f) return v;
            V3 a = v / vm, b = target / tm;
            float cos = Math.Max(-1f, Math.Min(1f, V3.Dot(a, b)));
            float angle = (float)Math.Acos(cos);
            if (angle <= maxRad) return b * vm;
            // slerp a toward b by maxRad
            V3 perp = (b - a * cos).Normalized;
            if (perp.SqrMagnitude == 0f) return v; // opposite: Unity picks an arbitrary axis; leave unchanged
            return (a * (float)Math.Cos(maxRad) + perp * (float)Math.Sin(maxRad)) * vm;
        }
    }
}
