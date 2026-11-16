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
        public const float MinAltitudeTarget = 2f;

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
        /// <summary>Share of TF's g limits the ride is planned with; the rest is the flight-path loop's margin to catch up.</summary>
        public const float TfPlanG = 0.6f;
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
                // A descent carries on for the reaction time before a pull-up bites: count the height lost in it.
                float sunk = -Math.Min(gamma, 0f) * Math.Min(d, react);
                float hard = TfHardSlope(dh + sunk, Math.Max(d - react, 0.5f * v), maxGamma, kHard, ref limited);
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

        /// <summary>
        /// Steepest bank (rad) at which the lift still gives the vertical acceleration
        /// <paramref name="aVert"/> (m/s^2, on top of holding 1 g against gravity) within
        /// <paramref name="qa"/>, the normal acceleration the A/P may command (pitch rate x speed,
        /// = n*g - g*cos(bank)). The vertical need is met first; the turn gets what is left. Solves
        /// (qa + g*c)*c = g + aVert for c = cos(bank): 0 when the jet needs all of it to climb, 90 deg
        /// when it wants 0 g.
        /// </summary>
        public static float BankLimit(float qa, float aVert, float g)
        {
            float c = (-qa + (float)Math.Sqrt(Math.Max(0f, qa * qa + 4f * g * (g + aVert)))) / (2f * g);
            return (float)Math.Acos(Math.Min(Math.Max(c, 0f), 1f));
        }

        /// <summary>
        /// Lift allocation (m/s^2). <paramref name="lv"/> = vertical lift wanted (gravity plus the climb wanted;
        /// below 0 = push), <paramref name="ah"/> = horizontal (the turn, + right). Both are trimmed to
        /// <paramref name="nMax"/>, climb first; <paramref name="room"/> = the most sideways lift left.
        /// Returns the bank (rad, + right) that points the lift that way, but never steeper than a level
        /// turn for the same sideways pull: pushing over in a turn keeps that bank and keeps pulling round
        /// (no lift, no turn), and a small correction while pushing stays a small bank.
        /// </summary>
        public static float LiftBank(ref float lv, ref float ah, float nMax, float g, out float room)
        {
            lv = Math.Min(lv, nMax);
            float up = Math.Max(lv, 0f);
            room = (float)Math.Sqrt(Math.Max(0f, nMax * nMax - up * up));
            ah = Math.Max(-room, Math.Min(room, ah));
            return (float)Math.Atan2(ah, Math.Max(lv, g));
        }

        /// <summary>Normal acceleration (m/s^2) to pull at the jet's actual bank: the wanted lift projected onto its lift axis.</summary>
        public static float LiftAlong(float lv, float ah, float bank) => lv * (float)Math.Cos(bank) + ah * (float)Math.Sin(bank);

        /// <summary>
        /// The pull (normal acceleration, m/s^2) at the jet's actual bank: the wanted lift projected onto
        /// its lift axis (LiftAlong), but never more lift upward than the climb wanted (at least 1 g), and
        /// never less than it while that is positive. Short of the bank wanted (rolling in, or held shallow
        /// near the water by the wingtip limit) a pull for the whole turn would mostly lift the jet: it
        /// turns slower and holds its height instead. Past it (rolling out) it keeps its height.
        /// </summary>
        public static float LiftPull(float lv, float ah, float bank, float g)
        {
            float c = Math.Max((float)Math.Cos(bank), 0.05f);
            float hi = Math.Max(lv, g) / c, lo = lv > 0f ? lv / c : float.NegativeInfinity;
            return Math.Max(lo, Math.Min(hi, LiftAlong(lv, ah, bank)));
        }

        /// <summary>Distance (m) from point P to the segment A-B.</summary>
        public static float SegmentDistance(float px, float pz, float ax, float az, float bx, float bz)
        {
            float lx = bx - ax, lz = bz - az, l2 = lx * lx + lz * lz;
            float t = l2 > 0f ? Math.Min(Math.Max(((px - ax) * lx + (pz - az) * lz) / l2, 0f), 1f) : 0f;
            float dx = px - (ax + t * lx), dz = pz - (az + t * lz);
            return (float)Math.Sqrt(dx * dx + dz * dz);
        }

        /// <summary>How far along the leg A->B the jet (at 0,0) is, in m: 0 at A, the leg's length at B.</summary>
        public static float AlongTrack(float ax, float az, float bx, float bz)
        {
            float lx = bx - ax, lz = bz - az, len = (float)Math.Sqrt(lx * lx + lz * lz);
            return len < 1f ? 0f : (-ax * lx - az * lz) / len;
        }

        /// <summary>
        /// Distance to fly from the jet (at 0,0) along the path through the <paramref name="n"/> points in
        /// <paramref name="xz"/> (x0,z0,x1,z1,.. relative to the jet): to the path's end, or, with a target T,
        /// along the path to its point nearest T (the first one, on a tie) and then straight to T.
        /// </summary>
        public static float PathDistance(float[] xz, int n, float tx, float tz, bool toTarget)
        {
            float ax = 0f, az = 0f, run = 0f;
            float bestOff = (float)Math.Sqrt(tx * tx + tz * tz), best = bestOff; // break off right away
            for (int i = 0; i < n; i++)
            {
                float bx = xz[2 * i], bz = xz[2 * i + 1], lx = bx - ax, lz = bz - az;
                float len = (float)Math.Sqrt(lx * lx + lz * lz);
                if (toTarget && len > 0f)
                {
                    float t = Math.Min(Math.Max(((tx - ax) * lx + (tz - az) * lz) / (len * len), 0f), 1f);
                    float px = ax + lx * t - tx, pz = az + lz * t - tz, off = (float)Math.Sqrt(px * px + pz * pz);
                    if (off < bestOff - 1f)
                    {
                        bestOff = off;
                        best = run + len * t + off;
                    }
                }
                run += len;
                ax = bx;
                az = bz;
            }
            return toTarget ? best : run;
        }

        /// <summary>
        /// NAV steering point: <paramref name="look"/> m on along the leg A->B past the jet's abeam point,
        /// never past B. Positions relative to the jet (at 0,0). Flying at it joins the drawn line and holds
        /// it; NAV moves on to the next leg at the turn point, so corners are flown from there.
        /// </summary>
        public static void LegCarrot(float ax, float az, float bx, float bz, float look, out float cx, out float cz)
        {
            float lx = bx - ax, lz = bz - az, len = (float)Math.Sqrt(lx * lx + lz * lz);
            float along = len < 1f ? len : Math.Min(Math.Max(AlongTrack(ax, az, bx, bz) + look, 0f), len);
            cx = len < 1f ? bx : ax + lx / len * along;
            cz = len < 1f ? bz : az + lz / len * along;
        }

        /// <summary>
        /// How far ahead (s of flight) NAV steers on a leg, never less than a turn radius. Short look-aheads
        /// bank hard for small errors at jet speeds and swing back and forth after each turn.
        /// </summary>
        public const float NavLookSeconds = 8f;

        /// <summary>
        /// L1 guidance (Park, Deyst and How, 2004): the turn rate (rad/s, + right) that flies the jet,
        /// moving at vx/vz, onto the circle through the steering point cx/cz (relative to the jet) that is
        /// tangent to its track: 2 v sin(eta) / L, eta = angle from the track to the point, L = its
        /// distance. Past 90 deg off the nose it turns at the 90 deg rate.
        /// </summary>
        public static float L1TurnRate(float vx, float vz, float cx, float cz)
        {
            float v = (float)Math.Sqrt(vx * vx + vz * vz), l = (float)Math.Sqrt(cx * cx + cz * cz);
            if (v < 1f || l < 1f) return 0f;
            float cross = vz * cx - vx * cz, dot = vx * cx + vz * cz; // + cross = point to the right
            float sinEta = dot >= 0f ? cross / (v * l) : Math.Sign(cross);
            return 2f * v * sinEta / l;
        }

        public const float MinWaypointLead = 300f;

        /// <summary>Fly-over NAV: how far (m) inside a waypoint the hard turn's roll-in may start.</summary>
        public const float FlyOverMiss = 2f;

        /// <summary>
        /// Fly-over NAV: seconds before the waypoint the hard turn may start and still pass within
        /// FlyOverMiss of it. Rolling in at <paramref name="rollRate"/> (rad/s) the track curves away by g*rollRate*t^3/6.
        /// </summary>
        public static float FlyOverLeadTime(float rollRate, float g) =>
            (float)Math.Pow(6.0 * FlyOverMiss / (g * Math.Max(rollRate, 0.05f)), 1.0 / 3.0);

        /// <summary>
        /// Fly-over hard turn: roll out once the heading still to turn (<paramref name="eta"/>, rad) is what the
        /// roll-out itself turns through: rolling <paramref name="bank"/> (rad) off at <paramref name="rollRate"/> (rad/s)
        /// turns the integral of g tan(bank)/v, = g (-ln cos bank) / (v rollRate).
        /// </summary>
        public static bool HardTurnDone(float eta, float bank, float rollRate, float v, float g) =>
            Math.Abs(eta) <= g * -Math.Log(Math.Max(Math.Cos(bank), 0.05)) / (v * Math.Max(rollRate, 0.05f));
    }
}
