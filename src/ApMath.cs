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

        // ---- terrain following ----
        // Share of the planned pull-up g the safety bound counts on; the rest is margin for the loop's lag.
        private const float TfSafetyShare = 0.7f;
        // Flight-path angles are in radians and treated as slopes (fine up to the ~20 deg the A/P flies).

        /// <summary>
        /// Distance of look-ahead sample <paramref name="i"/> of <paramref name="n"/> for a look-ahead of
        /// <paramref name="look"/> metres: packed close to the aircraft, spreading out with range.
        /// </summary>
        public static float TfSampleDistance(int i, int n, float look)
        {
            float f = (i + 1) / (float)n;
            return look * f * (float)Math.Sqrt(f);
        }

        /// <summary>
        /// Ride-quality bound: the flight-path angle to hold now so the aircraft crosses a point
        /// <paramref name="dh"/> m above it and <paramref name="d"/> m ahead LEVEL, pushing over at
        /// curvature <paramref name="kPush"/> (rad/m) on the way - so it doesn't balloon over the crest.
        /// The approach path is a push-over parabola, extended backwards by a straight climb at
        /// <paramref name="gammaMax"/>; a point still outside that shape needs nothing yet (-infinity).
        /// </summary>
        public static float TfComfortSlope(float dh, float d, float gammaMax, float kPush)
        {
            if (dh <= 0f) return float.NegativeInfinity;
            float reach = dh <= gammaMax * gammaMax / (2f * kPush)
                ? (float)Math.Sqrt(2f * dh / kPush)
                : gammaMax / (2f * kPush) + dh / gammaMax;
            if (d > reach) return float.NegativeInfinity;
            return Math.Min(gammaMax, dh / d + kPush * d * 0.5f);
        }

        /// <summary>
        /// Safety bound: the lowest flight-path angle from which pulling up now at curvature
        /// <paramref name="kPull"/> (rad/m), to at most <paramref name="gammaMax"/>, still passes over a point
        /// <paramref name="dh"/> m above the aircraft (negative = below) and <paramref name="d"/> m ahead.
        /// <paramref name="limited"/> is set when even a climb at gammaMax can't make it.
        /// </summary>
        public static float TfHardSlope(float dh, float d, float gammaMax, float kPull, ref bool limited)
        {
            if (dh / d + kPull * d * 0.5f <= gammaMax)   // still pulling when it gets there
                return dh / d - kPull * d * 0.5f;
            float spare = gammaMax * d - dh;
            if (spare <= 0f)
            {
                limited = true;
                return gammaMax;
            }
            return gammaMax - (float)Math.Sqrt(2f * kPull * spare);
        }

        /// <summary>
        /// Extra sink rate (m/s, never positive) for following a downslope: <paramref name="nearSlope"/> is
        /// the steepest line from the surface under the aircraft to the terrain over the next couple of
        /// seconds, so a negative value means everything close ahead falls away at least that fast.
        /// </summary>
        public static float TfDescentFeedforward(float nearSlope, float groundSpeed) =>
            nearSlope < 0f && !float.IsInfinity(nearSlope) ? 0.8f * groundSpeed * nearSlope : 0f;   // -inf = no samples

        /// <summary>
        /// Terrain-following command: the lowest flight-path angle (rad) that clears every look-ahead
        /// sample by <paramref name="clearance"/>, or -infinity when nothing ahead needs a climb.
        /// <paramref name="dist"/>/<paramref name="rel"/> are the samples' distance along the track and
        /// surface height relative to the aircraft when they were taken; since then the aircraft has
        /// moved <paramref name="travelled"/> m and climbed <paramref name="climbed"/> m.
        /// The ride is planned with <paramref name="aPull"/>/<paramref name="aPush"/> (m/s^2) and
        /// <paramref name="planGamma"/>; the safety bound assumes only TfSafetyShare of that pull and may climb
        /// at <paramref name="maxGamma"/>. <paramref name="lag"/> (s) is the flight-path loop's response time.
        /// </summary>
        public static float TfSlope(float[] dist, float[] rel, int n, float travelled, float climbed, float clearance,
            float v, float gamma, float planGamma, float maxGamma, float aPull, float aPush, float lag,
            out bool limited)
        {
            limited = false;
            float v2 = v * v;
            float kPull = aPull / v2, kPush = aPush / v2, kHard = TfSafetyShare * aPull / v2;
            float react = v * lag;
            float best = float.NegativeInfinity;
            for (int i = 0; i < n; i++)
            {
                float d = dist[i] - travelled;
                if (d <= 0f) continue;   // already overhead: the radar altimeter has it
                float dh = rel[i] + clearance - climbed;

                // Safety: from the real distance, less the loop's reaction time.
                float hard = TfHardSlope(dh, Math.Max(d - react, 0.5f * v), maxGamma, kHard, ref limited);
                if (hard > best) best = hard;

                if (dh <= 0f) continue;
                // Ride: start early enough to also cover the pull-up onto the approach path.
                float entry = Math.Min((float)Math.Sqrt(2f * dh * kPush), planGamma);
                float lead = react + Math.Max(0f, entry - gamma) / (2f * kPull);
                float comfort = TfComfortSlope(dh, Math.Max(d - lead, 2f * v), planGamma, kPush);
                if (comfort > best) best = comfort;
            }
            return best;
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
