using System;
using System.Collections.Generic;

namespace AutopilotMod
{
    /// <summary>One route point on the map: global x (east) / z (north) in metres, and the height
    /// above ground to fly the leg that ENDS here (NaN = no height; the vertical mode is left alone).</summary>
    internal struct Waypoint
    {
        public float X, Z, Agl;
        public Waypoint(float x, float z, float agl = float.NaN) { X = x; Z = z; Agl = agl; }
    }

    /// <summary>
    /// The route the NAV mode flies, shared by the radar page (which edits and draws it) and the
    /// flight controller (which flies it). Leg i runs from point i-1 to point i; leg 0 runs from where
    /// the jet was when NAV engaged.
    /// </summary>
    internal sealed class NavRoute
    {
        public readonly List<Waypoint> Points = new List<Waypoint>();
        public int Active;                  // index of the point being flown to
        public float StartX, StartZ;        // leg 0's start
        public bool Planned;                // made by the planner (heights per leg, replanned automatically)
        public float DestX = float.NaN, DestZ = float.NaN; // planner destination (the last point clicked)
        public int Version;                 // bumped on every change (redraws, planner hand-off)

        public bool Has => Active < Points.Count;

        public void Clear()
        {
            Points.Clear();
            Active = 0;
            Planned = false;
            DestX = DestZ = float.NaN;
            Version++;
        }

        public void Replace(List<Waypoint> pts, float startX, float startZ, bool planned)
        {
            Points.Clear();
            Points.AddRange(pts);
            Active = 0;
            StartX = startX;
            StartZ = startZ;
            Planned = planned;
            Version++;
        }

        /// <summary>Start of the leg being flown.</summary>
        public void LegStart(out float x, out float z)
        {
            if (Active > 0 && Active <= Points.Count)
            {
                x = Points[Active - 1].X;
                z = Points[Active - 1].Z;
            }
            else
            {
                x = StartX;
                z = StartZ;
            }
        }
    }

    /// <summary>
    /// L1 nonlinear path following (Park, Deyst and How 2004; the lateral law ArduPilot flies). A
    /// reference point L1 metres ahead on the track is chased with a lateral acceleration
    /// 2 V^2/L1 sin(eta), which follows straight legs and circles alike and converges onto the track
    /// from any angle without overshoot. L1 grows with speed, so the response is a fixed period.
    /// </summary>
    internal static class L1Guidance
    {
        /// <summary>
        /// Lateral acceleration (m/s^2, + = turn right) to follow the line A->B, with the aircraft at
        /// (px,pz) moving (vx,vz) over the ground. Also returns the cross-track error (m, + = right of
        /// track) and the along-track distance left to B. <paramref name="maxAccel"/> is the most
        /// lateral acceleration the aircraft may use (g x tan(max bank)): L1 is kept at least 0.6 x the
        /// turn radius that allows, so a jet's wide turns can't saturate the law into circling (the
        /// period-based L1 alone suits slow drones; 0.6 R was the quickest that still settled from a
        /// 180 deg reversal in the self-check model).
        /// </summary>
        public static float LateralAccel(float px, float pz, float vx, float vz, float ax, float az, float bx, float bz,
            float period, float damping, float maxAccel, out float crossTrack, out float toGo)
        {
            float speed = (float)Math.Sqrt(vx * vx + vz * vz);
            float v = Math.Max(speed, 1f);
            float k = 4f * damping * damping;
            float l1 = Math.Max(Math.Max(damping * period * v / (float)Math.PI, 0.6f * v * v / Math.Max(maxAccel, 0.5f)), 100f);

            float tx = bx - ax, tz = bz - az, len = (float)Math.Sqrt(tx * tx + tz * tz);
            if (len < 1f)
            {
                // degenerate leg: fly straight at B
                crossTrack = 0f;
                toGo = (float)Math.Sqrt((bx - px) * (bx - px) + (bz - pz) * (bz - pz));
                return k * v * v / l1 * (float)Math.Sin(AngleRight(vx, vz, bx - px, bz - pz));
            }
            tx /= len;
            tz /= len;
            float rx = px - ax, rz = pz - az;
            float along = rx * tx + rz * tz;
            crossTrack = rx * tz - rz * tx;   // + = right of track (x east, z north)
            toGo = len - along;

            float eta;
            float distA = (float)Math.Sqrt(rx * rx + rz * rz);
            if (distA > l1 && along < 0f && (rx * tx + rz * tz) / Math.Max(distA, 1f) < -0.7f)
            {
                // well behind A, facing away: head for A first
                eta = AngleRight(vx, vz, -rx, -rz);
            }
            else
            {
                float nu1 = AngleRight(vx, vz, tx, tz);   // track direction relative to velocity
                float nu2 = -(float)Math.Asin(Clamp(crossTrack / l1, -1f, 1f));
                eta = Clamp(nu1 + nu2, -(float)Math.PI * 0.5f, (float)Math.PI * 0.5f);
            }
            if (speed < 1f) eta = 0f;
            return k * v * v / l1 * (float)Math.Sin(eta);
        }

        /// <summary>
        /// Distance before a waypoint to start turning onto the next leg, so the turn rolls out on it:
        /// turn radius x tan(half the course change), never more than half the next leg.
        /// </summary>
        public static float TurnLead(float speed, float maxBankDeg, float courseChangeDeg, float nextLegLength)
        {
            float bank = Math.Max(5f, maxBankDeg) * (float)Math.PI / 180f;
            float radius = speed * speed / (9.81f * (float)Math.Tan(bank));
            float half = Math.Min(Math.Abs(courseChangeDeg), 150f) * 0.5f * (float)Math.PI / 180f;
            return Math.Min(radius * (float)Math.Tan(half), nextLegLength * 0.5f);
        }

        /// <summary>Course from (x,z) toward (x2,z2), degrees from north.</summary>
        public static float Course(float x, float z, float x2, float z2) =>
            (float)(Math.Atan2(x2 - x, z2 - z) * 180.0 / Math.PI + 360.0) % 360f;

        /// <summary>Signed angle (rad, + = clockwise/right) from vector (ux,uz) to (wx,wz).</summary>
        private static float AngleRight(float ux, float uz, float wx, float wz) =>
            (float)Math.Atan2(ux * wz - uz * wx, ux * wx + uz * wz) * -1f;

        private static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;
    }
}
