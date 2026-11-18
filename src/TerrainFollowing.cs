using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// Terrain following (TF): a vertical mode that holds the RALT target as a set clearance, like
    /// RALT, but looks ahead along the predicted ground track and starts climbing early enough to
    /// cross rising terrain at that clearance - and level, so it doesn't balloon over the crest.
    ///
    /// Look-ahead: a sweep of vertical raycasts (terrain, ships, sea level) at TfSamples distances along
    /// the track, bent by the turn the lateral mode is about to fly, three rays at each across a corridor
    /// wider than the wingspan. Vertical rays are the cheapest a terrain collider can answer, and the
    /// sweep only runs every TfSweepInterval (not every physics tick) and only while TF is flying
    /// the jet: about 1200 rays a second, no allocations. Between sweeps the samples are reused, shifted
    /// by the distance flown and height gained since. The arithmetic is in ApMath (TfSlope).
    /// </summary>
    internal sealed partial class FlightController
    {
        private const int TfSamples = 40;
        private const float TfSweepInterval = 0.1f;                 // s
        private const float TfMinLook = 1000f, TfMaxLook = 8000f;   // m
        private const float TfPlanShare = 0.75f;   // share of MaxFlightPathAngle the ride is planned with
        private const float TfLoopLag = 1f;        // s, flight-path loop response the look-ahead allows for
        internal const float AirframeShare = 0.8f;  // share of the airframe's full-stick g the A/P uses, climb and turn together; the rest is for the rate loop

        /// <summary>Terrain ahead can't be cleared at the set clearance even at the maximum climb angle.</summary>
        public bool TfClimbLimited { get; private set; }

        private readonly float[] tfDist = new float[TfSamples];   // m along the track at sweep time
        private readonly float[] tfRel = new float[TfSamples];    // m, surface above the gear reference at sweep time
        private bool tfValid;
        private float tfSinceSweep, tfTravelled, tfAltAtSweep, tfNearSlope;
        private float tfCmdPrev, tfCmdRate;

        private void TfReset()
        {
            tfValid = false;
            TfClimbLimited = false;
            tfCmdRate = 0f;
            tfCmdPrev = gammaCmdSlewed;
        }

        /// <summary>Lowest clearance TF accepts as a target.</summary>
        private float TfMinClearance => Mathf.Max(cfg.TfMinClearance.Value, ApMath.MinAltitudeTarget);

        /// <summary>
        /// Climb-rate command (m/s) for terrain following. <paramref name="aPull"/>/<paramref name="aPush"/>
        /// are the pull-up and push-over accelerations the ride is planned with (m/s^2).
        /// </summary>
        private float TfClimbRate(Aircraft ac, float v, float k, float aPull, float aPush, float lag, float dt)
        {
            FlightState s = State;
            float groundSpeed = Mathf.Sqrt(s.FwdSpeed * s.FwdSpeed + s.SideSpeed * s.SideSpeed);

            tfSinceSweep += dt;
            tfTravelled += groundSpeed * dt;
            if (!tfValid || tfSinceSweep >= TfSweepInterval)
                TfSweep(ac, groundSpeed);

            float maxGamma = cfg.MaxFlightPathAngle.Value * Mathf.Deg2Rad;
            float planGamma = TfPlanShare * maxGamma;
            float maxVs = v * Mathf.Sin(planGamma);

            // Under the aircraft: RALT's capture onto the set clearance, plus following a downslope.
            // Twice RALT's gain: at RALT's gain the last 100 m of a descent takes about 10 s, so TF rode high.
            float vsCmd = ApMath.CaptureProfile(TgtRadarAlt - raltFiltered, 2f * k, 0.5f * aPull, maxVs)
                          + ApMath.TfDescentFeedforward(tfNearSlope, groundSpeed);
            vsCmd = Mathf.Clamp(vsCmd, -maxVs, maxVs);

            // Ahead: never below the flight path that clears every sample.
            float slope = ApMath.TfSlope(tfDist, tfRel, TfSamples, tfTravelled, s.Altitude - tfAltAtSweep, TgtRadarAlt,
                v, s.Gamma * Mathf.Deg2Rad, planGamma, maxGamma, aPull, aPush, lag, out bool limited);
            TfClimbLimited = limited;
            if (!float.IsNegativeInfinity(slope))
                vsCmd = Mathf.Max(vsCmd, v * Mathf.Sin(slope));
            return vsCmd;
        }

        /// <summary>Raycast the surface height at every look-ahead sample along the predicted track.</summary>
        private void TfSweep(Aircraft ac, float groundSpeed)
        {
            FlightState s = State;
            tfValid = true;
            tfSinceSweep = 0f;
            tfTravelled = 0f;
            tfAltAtSweep = s.Altitude;

            // Ground track: the velocity's direction, or the nose when barely moving over the ground.
            float hdg = s.Heading * Mathf.Deg2Rad;
            Vector3 nose = new Vector3(Mathf.Sin(hdg), 0f, Mathf.Cos(hdg));
            Vector3 noseRight = new Vector3(nose.z, 0f, -nose.x);
            Vector3 track = groundSpeed > 5f ? (nose * s.FwdSpeed + noseRight * s.SideSpeed) / groundSpeed : nose;
            Vector3 right = new Vector3(track.z, 0f, -track.x);
            float vg = Mathf.Max(groundSpeed, 20f);

            // The turn the lateral mode is about to fly (rad/s, + right) and where it ends (rad), at the rate
            // the bank limits actually allow: a tighter predicted arc would scan ground the jet never flies over.
            float turnRate, turnLimit;
            if (Lateral != LateralMode.Att)
            {
                turnRate = HdgTurnRate(s, turnRateMax);
                // HDG turns to the bug; NAV's TgtHeading is the leg's course, which its L1 guidance turns onto
                turnLimit = Mathf.Abs(Mathf.DeltaAngle(s.Heading, TgtHeading)) * Mathf.Deg2Rad;
            }
            else
            {
                turnRate = G * Mathf.Tan(Mathf.Clamp(TgtBank, -bankLimit, bankLimit) * Mathf.Deg2Rad) / vg;
                turnLimit = Mathf.PI;
            }
            bool turning = Mathf.Abs(turnRate) > 1e-3f;
            float radius = turning ? vg / turnRate : 0f;   // signed, + = centre to the right

            Vector3 pos = ac.transform.position;
            float sea = Datum.LocalSeaY;
            float gearY = pos.y - s.GearDrop;
            float top = sea + 10000f;   // start above any terrain: a ray that starts inside a hill misses it
            int mask = (int)PhysicsLayers.StaticsMask | (int)PhysicsLayers.ShipsMask;
            float look = Mathf.Clamp(vg * cfg.TfLookAhead.Value, TfMinLook, TfMaxLook);
            float halfWidth = s.HalfSpan + 10f;

            float nearSlope = float.NegativeInfinity;
            for (int i = 0; i < TfSamples; i++)
            {
                float d = ApMath.TfSampleDistance(i, TfSamples, look);
                float fwd = d, side = 0f, cos = 1f, sin = 0f;
                if (turning)
                {
                    float arc = Mathf.Min(d, turnLimit * Mathf.Abs(radius));
                    float theta = arc / radius;
                    cos = Mathf.Cos(theta);
                    sin = Mathf.Sin(theta);
                    fwd = radius * sin + (d - arc) * cos;
                    side = radius * (1f - cos) + (d - arc) * sin;
                }
                // Highest of the left edge, centre and right edge of a corridor that widens with range (track
                // uncertainty). One ray per sample, alternating sides, missed peaked crests: it crossed them low.
                float surfaceY = sea;
                for (int lane = -1; lane <= 1; lane++)
                {
                    float offset = lane * (halfWidth + 0.05f * d);
                    Vector3 p = pos + track * (fwd - offset * sin) + right * (side + offset * cos);
                    p.y = top;
                    if (Physics.Raycast(p, Vector3.down, out RaycastHit hit, 10500f, mask, QueryTriggerInteraction.Ignore))
                        surfaceY = Mathf.Max(hit.point.y, surfaceY);
                }

                tfDist[i] = d;
                tfRel[i] = surfaceY - gearY;
                if (d >= 0.5f * vg && d <= 2f * vg)
                    nearSlope = Mathf.Max(nearSlope, (tfRel[i] + s.RadarAlt) / d);
            }
            tfNearSlope = nearSlope;
        }
    }
}
