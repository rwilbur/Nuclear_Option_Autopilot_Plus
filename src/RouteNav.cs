using System.Collections.Generic;
using UnityEngine;

namespace AutopilotMod
{
    internal sealed class Waypoint
    {
        public float X, Z;      // global position (m)
        public float Clearance; // RALT / TF target for the leg flown to it (m)
        public float Speed = float.NaN; // A/T target for the leg flown to it (m/s); NaN = leave the speed alone
    }

    /// <summary>
    /// NAV: HDG mode flying the legs of a route placed on the radar map with L1 guidance: the lateral
    /// law gets a turn rate (not a heading) toward a point some seconds on along the leg, which joins
    /// the drawn line and holds it; at each turn point it moves on to the next leg.
    /// Flown waypoints drop off the route. When the jet starts a leg, the waypoint's clearance becomes
    /// the RALT/TF target and its speed (if set) the A/T target; the rockers override them until the next leg.
    /// </summary>
    internal sealed partial class FlightController
    {
        public readonly List<Waypoint> Route = new List<Waypoint>(); // still to fly, next first
        public bool Nav;

        private const float DefaultClearance = 60f; // first waypoint's clearance when not already in RALT/TF (m)
        private Waypoint navLeg;           // waypoint whose clearance was last applied
        private VerticalMode navLegMode;
        private float navLegClearance, navLegSpeed;
        private float navTurnRate;         // rad/s, + right: the L1 guidance's turn command for the lateral law
        private float legX, legZ;          // start of the leg being flown: the last waypoint, or where NAV was selected
        private bool legFromJet;
        private bool navHardTurn;          // fly-over: in the max-g turn after a waypoint
        private const float HardTurnRequest = 10f; // rad/s, more than any jet turns: the lift allocation trims it to the g left after the climb

        public void SelectNav()
        {
            if (Route.Count == 0)
            {
                report("NAV - NO WAYPOINTS", true);
                return;
            }
            Lateral = LateralMode.Hdg;
            Nav = true;
            navLeg = null;
            legFromJet = true;
            navHardTurn = false;
        }

        /// <summary>Insert a waypoint at index <paramref name="at"/> (Route.Count = append), with its neighbour's clearance and speed.</summary>
        public void AddWaypoint(int at, float x, float z)
        {
            at = Mathf.Clamp(at, 0, Route.Count);
            bool ralt = Vertical == VerticalMode.Ralt || Vertical == VerticalMode.Tf;
            Waypoint near = at > 0 ? Route[at - 1] : Route.Count > 0 ? Route[0] : null;
            float c = near != null ? near.Clearance : Mathf.Max(ralt ? TgtRadarAlt : DefaultClearance, TfMinClearance);
            Route.Insert(at, new Waypoint { X = x, Z = z, Clearance = c, Speed = near != null ? near.Speed : float.NaN });
        }

        /// <summary>Replace the whole route (a flight plan from the planner). NAV, if on, starts again from the jet.</summary>
        public void LoadRoute(List<Waypoint> plan)
        {
            Route.Clear();
            foreach (Waypoint w in plan) Route.Add(w);
            for (int i = 0; i < Route.Count; i++) SetClearance(i, Route[i].Clearance);
            navLeg = null;
            legFromJet = true;
            navHardTurn = false;
        }

        /// <summary>Waypoint speed in m/s; NaN or 0 clears it (the leg leaves the speed alone).</summary>
        public void SetSpeed(int i, float ms)
        {
            if (i >= 0 && i < Route.Count) Route[i].Speed = float.IsNaN(ms) || ms <= 0f ? float.NaN : Mathf.Clamp(ms, cfg.MinEngageSpeed.Value, 1000f);
        }

        /// <summary>Drag waypoint i to global x/z. Its height and speed stay; NAV, if flying to it, steers there from the next tick.</summary>
        public void MoveWaypoint(int i, float x, float z)
        {
            if (i < 0 || i >= Route.Count) return;
            Route[i].X = x;
            Route[i].Z = z;
        }

        public void RemoveWaypoint(int i)
        {
            if (i >= 0 && i < Route.Count) Route.RemoveAt(i);
        }

        public void SetClearance(int i, float m)
        {
            if (i >= 0 && i < Route.Count && !float.IsNaN(m)) Route[i].Clearance = Mathf.Clamp(m, TfMinClearance, 10000f);
        }

        public void ClearRoute() => Route.Clear();

        private float[] etaPath = new float[16];

        /// <summary>
        /// Seconds to the first selected target (along the route while in NAV, else straight), or with no
        /// target to the route's last waypoint (NAV only), at the current ground speed. False = nothing to time.
        /// </summary>
        public bool Eta(Aircraft ac, out float seconds, out bool toTarget)
        {
            seconds = 0f;
            List<Unit> targets = TargetPanel.Targets();
            GlobalPosition t = default;
            toTarget = targets != null && targets.Count > 0 && targets[0] != null && TargetPanel.KnownPosition(targets[0], out t);
            if (ac == null || !toTarget && !(Nav && Route.Count > 0)) return false;
            Vector3 v = ac.CockpitRB().velocity;
            float gs = new Vector2(v.x, v.z).magnitude;
            if (gs < 5f) return false; // parked or hovering: no meaningful ETA

            GlobalPosition g = ac.GlobalPosition();
            int n = Nav ? Route.Count : 0;
            if (etaPath.Length < 2 * n) etaPath = new float[4 * n];
            for (int i = 0; i < n; i++)
            {
                etaPath[2 * i] = Route[i].X - g.x;
                etaPath[2 * i + 1] = Route[i].Z - g.z;
            }
            if (!toTarget) t = g;
            seconds = ApMath.PathDistance(etaPath, n, t.x - g.x, t.z - g.z, toTarget) / gs;
            return true;
        }

        /// <summary>"ETA TGT h:mm:ss" / "ETA WPT h:mm:ss" for the MFD and the web page, or null.</summary>
        public string EtaText(Aircraft ac)
        {
            if (!Eta(ac, out float eta, out bool toTgt)) return null;
            int sec = Mathf.RoundToInt(Mathf.Min(eta, 359999f));
            return $"ETA {(toTgt ? "TGT" : "WPT")} {sec / 3600}:{sec / 60 % 60:00}:{sec % 60:00}";
        }

        private static float Bearing(float dx, float dz) => Mathf.Repeat(Mathf.Atan2(dx, dz) * Mathf.Rad2Deg, 360f);

        /// <summary>Physics tick: sequence the route and point the heading bug at the next waypoint.</summary>
        private void FollowRoute(Aircraft ac)
        {
            if (!Nav) return;
            if (Route.Count == 0)
            {
                Nav = false;
                report("NAV - ROUTE END", false);
                return;
            }
            Waypoint w = Route[0];
            GlobalPosition g = ac.GlobalPosition();
            if (legFromJet)
            {
                legX = g.x;
                legZ = g.z;
                legFromJet = false;
            }
            Vector3 v = ac.CockpitRB().velocity;
            float speed = Mathf.Max(new Vector2(v.x, v.z).magnitude, 40f);
            // the turn the jet can keep up level, within its g budget and MaxTurnRate (rotorcraft: 30 deg of bank)
            float omega = Mathf.Min(cfg.MaxTurnRate.Value * Mathf.Deg2Rad, G * Mathf.Tan(Mathf.Clamp(Rotorcraft ? 30f : turnBank, 5f, 85f) * Mathf.Deg2Rad) / speed);
            float radius = speed / omega;

            // leg A -> B (-> N), relative to the jet
            float ax = legX - g.x, az = legZ - g.z, bx = w.X - g.x, bz = w.Z - g.z;
            bool hasNext = Route.Count > 1;
            float nx = hasNext ? Route[1].X - g.x : 0f, nz = hasNext ? Route[1].Z - g.z : 0f;
            float turn = hasNext ? Mathf.DeltaAngle(Bearing(bx - ax, bz - az), Bearing(nx - bx, nz - bz)) * Mathf.Deg2Rad : 0f;

            // Fly-by: next leg once abeam the point where a turn of this radius joins it, or close to the waypoint.
            // Fly-over: next leg once the roll-in of the hard turn can start and still pass over the waypoint.
            bool flyOver = cfg.NavFlyOver.Value;
            float len = new Vector2(bx - ax, bz - az).magnitude;
            float lead = flyOver ? speed * ApMath.FlyOverLeadTime(0.6f * rollLimit * Mathf.Deg2Rad, G)
                                 : radius * Mathf.Tan(Mathf.Min(Mathf.Abs(turn), 2.6f) * 0.5f);
            if (ApMath.AlongTrack(ax, az, bx, bz) >= len - lead || !flyOver && bx * bx + bz * bz < ApMath.MinWaypointLead * ApMath.MinWaypointLead)
            {
                legX = w.X;
                legZ = w.Z;
                Route.RemoveAt(0); // the next tick steers along the next leg, or ends NAV
                navHardTurn = flyOver && hasNext;
                return;
            }
            if (navHardTurn)
            {
                // Fly-over: turn at the g budget's limit straight at the waypoint; roll out pointing at it,
                // and fly the rest of the leg from there (direct, wings level).
                float eta = Mathf.Atan2(v.z * bx - v.x * bz, v.x * bx + v.z * bz); // + = waypoint right of the track
                if (ApMath.HardTurnDone(eta, State.Bank * Mathf.Deg2Rad, 0.6f * rollLimit * Mathf.Deg2Rad, speed, G))
                {
                    navHardTurn = false;
                    legX = g.x;
                    legZ = g.z;
                    ax = az = 0f;
                }
                else
                {
                    navTurnRate = (eta < 0f ? -1f : 1f) * HardTurnRequest;
                    TgtHeading = Bearing(bx, bz);
                }
            }
            if (!navHardTurn)
            {
                // L1 guidance at a point NavLookSeconds of flight on (at least a turn radius)
                ApMath.LegCarrot(ax, az, bx, bz, Mathf.Max(radius, ApMath.NavLookSeconds * speed), out float cx, out float cz);
                navTurnRate = ApMath.L1TurnRate(v.x, v.z, cx, cz);
                TgtHeading = Bearing(cx, cz);
            }

            // applied on every engage too: engaging re-syncs the RALT target to the current height
            if (!ApEngaged) navLeg = null;
            if (!ApEngaged || (w == navLeg && Vertical == navLegMode && w.Clearance == navLegClearance && w.Speed.Equals(navLegSpeed))) return;
            navLeg = w;
            navLegMode = Vertical;
            navLegClearance = w.Clearance;
            navLegSpeed = w.Speed;
            if (!float.IsNaN(w.Speed)) TgtSpeed = w.Speed; // flown by the A/T (rotorcraft: SPD mode)
            if (Vertical == VerticalMode.Tf) TgtRadarAlt = Mathf.Max(w.Clearance, TfMinClearance);
            else if (Vertical == VerticalMode.Ralt) TgtRadarAlt = Mathf.Max(w.Clearance, cfg.GroundFloor.Value, ApMath.MinAltitudeTarget);
        }
    }
}
