using System;

namespace AutopilotMod
{
    /// <summary>
    /// The autopilot's pure arithmetic, free of Unity and game types so tests/SelfCheck can run it
    /// without the game.
    /// </summary>
    internal static class ApMath
    {
        /// <summary>Lowest altitude / radar-altitude target the A/P accepts (m).</summary>
        public const float MinAltitudeTarget = 1f;

        // Helicopter height loop: collective rate (1/s) per m/s^2 of vertical-acceleration error, and
        // acceleration demanded per m/s of climb-rate error.
        private const float CollectiveGain = 0.12f, RotorVsGain = 1f;
        // A collective can't arrest a descent as fast as a jet's pitch loop: approach the floor gently.
        private const float RotorFloorSinkGain = 0.3f;

        /// <summary>Step to the next multiple of <paramref name="step"/> in the given direction.</summary>
        public static float Snap(float value, int dir, float step)
        {
            float n = value / step;
            float snapped = dir > 0 ? (float)Math.Floor(n + 1e-3f) + 1f : (float)Math.Ceiling(n - 1e-3f) - 1f;
            return snapped * step;
        }

        /// <summary>
        /// Climb-rate command for an altitude error. Linear (gain k) near the target; further out a
        /// square-root profile, i.e. the rate from which a constant vertical deceleration <paramref name="a"/>
        /// stops exactly on the target. Big changes go at full rate and only round out at the end,
        /// instead of creeping in exponentially.
        /// </summary>
        public static float CaptureProfile(float err, float k, float a, float maxVs)
        {
            float e0 = 2f * a / (k * k);   // where the linear and sqrt segments meet
            float abs = Math.Abs(err);
            float vs = abs <= e0 ? k * abs : (float)Math.Sqrt(2f * a * abs);
            return (err < 0f ? -1f : 1f) * Math.Min(vs, maxVs);
        }

        /// <summary>
        /// Lowest climb rate a rotorcraft may fly with its lowest point <paramref name="clearance"/> above
        /// the surface: a slow sink toward the floor, a climb at the full <paramref name="gain"/> below it.
        /// <paramref name="airplane"/> (0..1, tiltwing wing forward) blends back to the jets' floor.
        /// </summary>
        public static float RotorFloorVs(float clearance, float floor, float gain, float airplane)
        {
            float room = clearance - floor;
            float sinkGain = Math.Min(gain, RotorFloorSinkGain);
            return -room * (room > 0f ? sinkGain + (gain - sinkGain) * airplane : gain);
        }

        /// <summary>Helicopter-mode collective rate (1/s) for a climb-rate command.</summary>
        public static float CollectiveRate(float vsCmd, float vs, float vsAccel)
        {
            float azErr = Math.Max(-3f, Math.Min(3f, RotorVsGain * (vsCmd - vs))) - vsAccel;
            return CollectiveGain * azErr;
        }

        /// <summary>
        /// 0..1 share of the speed loop's pitch changes allowed while the aircraft is
        /// <paramref name="vsDeficit"/> m/s short of its commanded climb rate and <paramref name="room"/> m
        /// above the floor. Full effect within 5 m of the floor, none from 15 m up (a power-limited
        /// climb at altitude must not freeze the speed hold).
        /// </summary>
        public static float HeightPriority(float vsDeficit, float room)
        {
            float near = Clamp01(1f - (room - 5f) / 10f);
            return 1f - near * Clamp01(vsDeficit / 2f);
        }

        private static float Clamp01(float v) => Math.Max(0f, Math.Min(1f, v));

        /// <summary>
        /// Lowest vertical speed allowed with <paramref name="room"/> m between the airframe's lowest
        /// point and the floor. A barrier on the clearance h: dh/dt >= -min(gain*h, sqrt(2*a*h)).
        /// The square-root part is the sink rate a pull-up of <paramref name="accel"/> m/s^2 can still
        /// stop exactly at the floor (a flight path can't change instantly, so a linear barrier alone
        /// lets a fast jet arrive with more sink than it can arrest). Below the floor: climb at gain.
        /// </summary>
        public static float FloorVs(float room, float gain, float accel)
        {
            if (room <= 0f) return -room * gain;
            return -Math.Min(gain * room, (float)Math.Sqrt(2f * Math.Max(accel, 0.1f) * room));
        }

        /// <summary>
        /// Lowest vertical speed now from which a pull-up still clears terrain <paramref name="t"/> s
        /// ahead that leaves <paramref name="room"/> m (negative = it rises above the floor line).
        /// The pull-up starts after <paramref name="react"/> s (loop and airframe lag), then the climb
        /// rate grows at <paramref name="accel"/> m/s^2 up to <paramref name="vsMax"/> (the steepest
        /// flight path allowed) and stays there. Solved for the smallest vs that still gains -room by t.
        /// Terrain that can't be cleared even from vsMax returns vsMax: climb as hard as allowed.
        /// </summary>
        public static float LookaheadMinVs(float room, float t, float accel, float vsMax, float react = 0f)
        {
            if (t <= 0.05f) return float.NegativeInfinity;
            float need = -room;
            if (t <= react) return Math.Min(vsMax, need / t);
            float tp = t - react, a = Math.Max(accel, 0.1f);
            float vs = (need - 0.5f * a * tp * tp) / t;      // the cap is never reached
            if (vs + a * tp <= vsMax) return vs;
            // capped: with w = vsMax - vs, gain = vsMax*t - w*react - w^2/(2a) >= need
            float slack = vsMax * t - need;
            if (slack <= 0f) return vsMax;
            float w = -a * react + (float)Math.Sqrt(a * a * react * react + 2f * a * slack);
            return Math.Min(vsMax, vsMax - w);
        }

        /// <summary>
        /// Throttle feedforward for coupled A/P + A/T (energy coupling, TECS-style): extra specific
        /// excess power a climb at <paramref name="gammaCmd"/> rad needs, plus the induced-drag rise of a
        /// level turn at <paramref name="bankRad"/> (proportional to n^2 - 1).
        /// </summary>
        public static float EnergyFeedforward(float gammaCmd, float bankRad, float climbGain, float turnGain)
        {
            float c = (float)Math.Cos(Math.Min(Math.Abs(bankRad), 1.2f));
            return climbGain * (float)Math.Sin(gammaCmd) + turnGain * (1f / (c * c) - 1f);
        }

        /// <summary>
        /// Speed protection (speed priority, TECSMOD-style): largest flight-path angle (rad) allowed when
        /// the throttle can give no more and the aircraft is <paramref name="deficit"/> m/s below its
        /// target. <paramref name="energyRate"/> = gamma + Vdot/g at the current (maxed) throttle: what
        /// the engines can sustain; demanding a recovery acceleration leaves this much for climbing.
        /// </summary>
        public static float SpeedProtectGamma(float energyRate, float deficit, float margin)
        {
            float vdotWanted = Math.Min(2f, 0.15f * Math.Max(0f, deficit - margin));
            return energyRate - vdotWanted / 9.81f;
        }

        /// <summary>
        /// Gain factor for a sluggish airframe: 1 when its fly-by-wire rate limit (<paramref name="maxRate"/>,
        /// rad/s) is at least <paramref name="agileRate"/>, falling in proportion below that, never under
        /// <paramref name="min"/>.
        /// </summary>
        public static float Agility(float maxRate, float agileRate, float min) =>
            Math.Max(min, Math.Min(1f, maxRate / agileRate));
    }
}
