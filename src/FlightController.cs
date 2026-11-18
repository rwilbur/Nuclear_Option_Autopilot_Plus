using System;
using System.Collections.Generic;
using HarmonyLib;
using UnityEngine;

namespace AutopilotMod
{
    internal enum LateralMode { Hdg, Att, Nav }
    internal enum VerticalMode { Balt, Ralt, Vs, Tf }

    /// <summary>Aircraft state in an aviation frame (nose-up / right-wing-down positive), SI units.</summary>
    internal struct FlightState
    {
        public bool Valid;
        public float Speed;          // m/s (same value the HUD shows)
        public float Altitude;       // m above sea level (floating-origin safe)
        public float RadarAlt;       // m above ground/water, gear-bottom reference (matches the HUD "R")
        public float Clearance;      // m from the airframe's lowest point (wingtips/nose/tail/gear) to the surface
        public float HalfSpan;       // m, half the wingspan
        public float GearDrop;       // m from the aircraft origin down to the gear-bottom reference
        public float VerticalSpeed;  // m/s, + up
        public float Heading;        // deg [0,360), same as the HUD compass
        public float Pitch;          // deg, + nose up
        public float Bank;           // deg, + right wing down
        public float Gamma;          // deg, flight path angle, + climbing
        public float PitchRate;      // rad/s body axis, + nose up
        public float RollRate;       // rad/s body axis, + rolling right
        public float Throttle;       // 0..1 as last applied
        public float AirDensity;     // kg/m^3
        public float FwdSpeed;       // m/s, horizontal ground speed along the nose heading
        public float SideSpeed;      // m/s, horizontal ground speed to the right of the nose
        public float PosX, PosZ;     // m, global map position (x east, z north)
        public float VelX, VelZ;     // m/s, horizontal ground velocity (east, north)
        public float LowestDrop;     // m from the aircraft origin down to its lowest point
    }

    /// <summary>
    /// Autopilot control laws. Runs inside PilotPlayerState's physics tick, after the game has
    /// written the player's stick into ControlInputs and before Aircraft.FilterInputs() feeds the
    /// fly-by-wire, so the autopilot "holds the stick" exactly like the pilot does.
    ///
    /// Structure (everything proportional except the innermost rate loops):
    ///   vertical: altitude err -> climb rate -> flight-path angle -> pitch rate -> stick
    ///   lateral:  heading err  -> turn rate  -> bank angle        -> roll rate  -> stick
    ///   speed:    speed err    -> throttle (PI + acceleration damping)
    /// Most of the old script's oscillation came from stacking integrators (vs_pid.ki + pitch_pid.ki
    /// + the FBW's own integrator) on top of a rate-commanded airframe, plus ViGEm/gamepad latency.
    /// Here only the rate loops integrate, and they act on measured body rates. With the INDI law
    /// (default) not even those: the rate loops are incremental, driven by measured rate acceleration.
    ///
    /// Safety layers on top of the modes, in every vertical mode:
    ///   surface floor   sink limited near the surface, and further out to what a pull-up can stop
    ///   look-ahead      terrain along the predicted path (current turn included) climbed over in time
    ///   speed priority  with A/P + A/T, a climb the engines can't sustain is traded for speed
    /// </summary>
    internal sealed partial class FlightController
    {
        private const float G = 9.81f;
        private const float AttHoldSnapDeg = 7f;   // ATT mode snaps to wings-level inside this bank

        private readonly ApSettings cfg;
        private readonly Action<string, bool> report;

        public event Action ApDisconnected;

        public bool ApEngaged { get; private set; }
        public bool AtEngaged { get; private set; }
        public LateralMode Lateral = LateralMode.Hdg;
        public VerticalMode Vertical = VerticalMode.Balt;
        public bool CoarseStep;

        public float TgtHeading, TgtBank, TgtAltitude, TgtRadarAlt, TgtVs, TgtSpeed;

        public FlightState State;
        public bool HasFbw { get; private set; }
        public bool BankLimitedBySurface { get; private set; }
        public bool TerrainAhead { get; private set; }        // the look-ahead is commanding the climb
        public bool SpeedProtectActive { get; private set; }  // speed priority is limiting the climb
        public float NavToGo, NavCrossTrack;                  // m, for the MFD
        private float navTurnRate;                            // rad/s, + right: NAV's L1 turn command for the HDG law
        private int navSpeedLeg = -1;                         // leg whose speed was last applied

        /// <summary>The route NAV flies (the radar page edits it).</summary>
        public readonly NavRoute Nav = new NavRoute();
        /// <summary>RALT target follows the planned route's leg heights.</summary>
        public bool RouteVertical { get; private set; }

        // Rate loops: INDI axes (their RLS also corrects the classic law's FBW model)
        public readonly IndiAxis PitchAxis = new IndiAxis(0.15f), RollAxis = new IndiAxis(0.1f);
        private float lastPitchStick, lastRollStick;   // stick applied last tick (stick-up / roll-right +)
        private float gammaMaxSpd = float.NaN;

        // Exposed for the MFD's data line
        public float CmdVs, CmdGammaDeg, CmdBankDeg, CmdQ, CmdP, OutPitch, OutRoll, OutThrottle;

        // Rate-loop integrators (stick units) and throttle integrator
        private float qI, pI, thrI;
        private float gammaCmdSlewed, bankCmdSlewed;
        private float turnRateMax = 1f;      // rad/s, fastest turn the lift left after the climb gives now
        private float turnBank = 30f;        // deg, steepest level-turn bank within the g budget (NAV plans its turns with it)
        private float liftBank;              // deg, + right: bank that points the lift at the climb and the turn wanted
        private float rollLimit = 25f;       // deg/s, fastest roll the lateral law commands now
        private float bankLimit = 30f;       // deg, steepest bank the lateral law may fly now (all limits)
        private float gammaPrev = float.NaN, gammaRate;
        private float raltFiltered;
        private float prevSpeed = float.NaN, accelFiltered;
        private float playerThrottle = float.NaN, playerThrottleAtEngage = float.NaN;
        private bool suppressStickOverride;
        private float pilotPitch, pilotRoll;   // raw stick from the latest tick, before the A/P overrides it
        // Outer-loop gain factors for sluggish airframes (1 = fighter). The heavy jets' fly-by-wire allows
        // 0.3 rad/s of pitch rate and 3 rad/s of roll rate where the fighters get 0.7-1 and 5-12; the
        // gains tuned on fighters outrun them and they oscillate.
        private float pitchAgility = 1f, rollAgility = 1f;

        // ---- FBW internals (private serialized fields on ControlsFilter.FlyByWire) ----
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> FbwGLimit = TryRef("gLimitPositive");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> FbwCorner = TryRef("cornerSpeed");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> FbwMaxPitchRate = TryRef("maxPitchAngularVel");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> FbwLimitFactor = TryRef("limitFactorSmoothed");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> FbwMaxRollRate = TryRef("maxRollAngularVel");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> FbwMaxRollSpeed = TryRef("maxRollSpeed");
        private static readonly bool FbwReadable = FbwGLimit != null && FbwCorner != null && FbwMaxPitchRate != null
                                                   && FbwLimitFactor != null && FbwMaxRollRate != null && FbwMaxRollSpeed != null;

        private static AccessTools.FieldRef<ControlsFilter.FlyByWire, float> TryRef(string name)
        {
            try { return AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>(name); }
            catch { return null; }
        }

        public FlightController(ApSettings cfg, Action<string, bool> report)
        {
            this.cfg = cfg;
            this.report = report;
        }

        /// <summary>Drop everything (new aircraft, death, leaving the mission).</summary>
        public void Reset()
        {
            ApEngaged = false;
            AtEngaged = false;
            Lateral = LateralMode.Hdg;
            Vertical = VerticalMode.Balt;
            State = default;
            prevSpeed = float.NaN;
            playerThrottle = float.NaN;
            playerThrottleAtEngage = float.NaN;
            if (flightAssistParked && rotorAc != null) rotorAc.SetFlightAssist(storedFlightAssist);
            flightAssistParked = false;
            rotorAc = null;
            prevFwd = float.NaN;
            PitchAxis.Reset();
            RollAxis.Reset();
            RouteVertical = false;
            TerrainAhead = SpeedProtectActive = false;
            gammaMaxSpd = float.NaN;
        }

        // =====================================================================
        // State sampling
        // =====================================================================
        public static FlightState Sample(Aircraft ac)
        {
            var s = new FlightState();
            Rigidbody rb = ac.CockpitRB();
            if (rb == null) return s;
            Transform t = rb.transform;

            Vector3 v = rb.velocity;
            Vector3 fwd = t.forward;
            Vector3 w = t.InverseTransformDirection(rb.angularVelocity);

            s.Speed = ac.speed;
            s.Altitude = t.position.GlobalY();
            SampleSurface(ac, ref s);
            s.VerticalSpeed = v.y;
            s.Heading = Mathf.Repeat(Mathf.Atan2(fwd.x, fwd.z) * Mathf.Rad2Deg, 360f);
            s.Pitch = Mathf.Asin(Mathf.Clamp(fwd.y, -1f, 1f)) * Mathf.Rad2Deg;
            s.Bank = Mathf.Atan2(-t.right.y, t.up.y) * Mathf.Rad2Deg;
            float spd = v.magnitude;
            s.Gamma = spd > 1f ? Mathf.Asin(Mathf.Clamp(v.y / spd, -1f, 1f)) * Mathf.Rad2Deg : 0f;
            // Unity: +x local rotation is nose-DOWN, +z local rotation is roll-LEFT.
            s.PitchRate = -w.x;
            s.RollRate = -w.z;
            s.Throttle = ac.GetInputs().throttle;
            s.AirDensity = Mathf.Max(ac.airDensity, 0.05f);
            Vector3 fwdH = new Vector3(fwd.x, 0f, fwd.z).normalized;
            s.FwdSpeed = v.x * fwdH.x + v.z * fwdH.z;
            s.SideSpeed = v.x * fwdH.z - v.z * fwdH.x;
            GlobalPosition gp = ac.GlobalPosition();
            s.PosX = gp.x;
            s.PosZ = gp.z;
            s.VelX = v.x;
            s.VelZ = v.z;
            s.Valid = true;
            return s;
        }

        /// <summary>
        /// Our own radar altimeter, every physics tick. The game's Aircraft.radarAlt only refreshes
        /// every 0.1 s and raycasts terrain only, so over shallow water it measures to the seabed.
        /// Here the surface is the higher of terrain/ship deck and sea level, like the game's AI uses.
        /// </summary>
        private static void SampleSurface(Aircraft ac, ref FlightState s)
        {
            Transform t = ac.transform;
            Vector3 pos = t.position;
            float surfaceY = Datum.LocalSeaY;
            if (Physics.Linecast(pos, pos - Vector3.up * 10000f, out RaycastHit hit,
                    (int)PhysicsLayers.StaticsMask | (int)PhysicsLayers.ShipsMask))
                surfaceY = Mathf.Max(hit.point.y, surfaceY);

            AircraftDefinition def = ac.definition;
            s.GearDrop = Mathf.Max(def.spawnOffset.y, 0f);
            s.HalfSpan = Mathf.Max(def.width * 0.5f, 0f);
            float halfLength = Mathf.Max(def.length * 0.5f, 0f);

            float gearY = pos.y - s.GearDrop;
            s.RadarAlt = gearY - surfaceY;

            // Lowest of: gear reference, both wingtips, nose, tail (wing/fuselage line ~ origin height).
            float lowest = gearY;
            Vector3 span = t.right * s.HalfSpan;
            Vector3 len = t.forward * halfLength;
            lowest = Mathf.Min(lowest, pos.y - Mathf.Abs(span.y));
            lowest = Mathf.Min(lowest, pos.y - Mathf.Abs(len.y));
            s.Clearance = lowest - surfaceY;
            s.LowestDrop = pos.y - lowest;
        }

        // =====================================================================
        // Engage / disengage / modes
        // =====================================================================
        public void SetAutopilot(Aircraft ac, bool on, string reason = null)
        {
            if (!on)
            {
                if (!ApEngaged) return;
                ApEngaged = false;
                if (Rotorcraft) RotorDisengage(reason);
                report(reason ?? "AUTOPILOT OFF", reason != null);
                ApDisconnected?.Invoke();
                return;
            }
            if (ApEngaged) return;

            string fail = CheckEngageable(ac);
            if (fail != null)
            {
                report("A/P FAIL - " + fail, true);
                return;
            }

            // Like an F-16/F-18 autopilot, engaging holds what you have: targets sync to the
            // current state, then you dial them with the rockers.
            SyncLateral();
            SyncVertical();
            // Bumpless transfer: start the rate integrators at the pilot's last raw stick. (Not the
            // live ControlInputs - by now FilterInputs() has overwritten those with FBW output.)
            qI = Mathf.Clamp(-pilotPitch, -0.6f, 0.6f);
            pI = Mathf.Clamp(pilotRoll, -0.4f, 0.4f);
            gammaCmdSlewed = State.Gamma * Mathf.Deg2Rad;
            gammaPrev = float.NaN;
            gammaRate = 0f;
            bankCmdSlewed = State.Bank;
            raltFiltered = State.RadarAlt;
            gammaMaxSpd = float.NaN;
            TfReset();
            if (Rotorcraft) RotorEngage();
            ApEngaged = true;
            report("AUTOPILOT ENGAGED", false);
        }

        public void SetAutothrottle(Aircraft ac, PilotPlayerState pps, bool on, string reason = null)
        {
            if (Rotorcraft)
            {
                SetRotorSpeedHold(on, ac);
                return;
            }
            if (!on)
            {
                if (!AtEngaged) return;
                AtEngaged = false;
                // Leave the throttle lever where the A/T had it (keyboard/relative throttles) unless the
                // pilot disconnected it by moving the throttle themselves.
                if (pps != null && reason == null) PlayerThrottleBridge.SetLever(pps, OutThrottle);
                report(reason ?? "AUTOTHROTTLE OFF", reason != null);
                return;
            }
            if (AtEngaged) return;
            if (!State.Valid || ac == null) return;
            TgtSpeed = State.Speed;
            thrI = State.Throttle;
            OutThrottle = State.Throttle;
            accelFiltered = 0f;
            energyCoupledLast = false;
            energyFf = 0f;
            playerThrottleAtEngage = playerThrottle;
            AtEngaged = true;
            report("AUTOTHROTTLE ENGAGED", false);
        }

        private string CheckEngageable(Aircraft ac)
        {
            if (ac == null || !State.Valid) return "NO DATA";
            if (ac.IsAutoHoverEnabled()) return "AUTO HOVER ON";
            if (ac.cockpit.IsDetached()) return "NO COCKPIT";
            if (State.RadarAlt < cfg.MinEngageRadarAlt.Value) return "ON GROUND";
            if (Rotorcraft) return Mathf.Abs(State.Bank) > 45f || Mathf.Abs(State.Pitch) > 30f ? "ATTITUDE" : null;
            if (State.Speed < cfg.MinEngageSpeed.Value) return "SPEED LOW";
            if (Mathf.Abs(State.Bank) > 60f || Mathf.Abs(State.Pitch) > 45f) return "ATTITUDE";
            return null;
        }

        private static bool IsCollective(Aircraft ac)
        {
            // Same test the game uses to decide the throttle is a collective (helicopters/VTOL).
            return ac.GetAircraftParameters().takeoffDistance == 0f;
        }

        public void SelectLateral(LateralMode m)
        {
            if (m == LateralMode.Nav)
            {
                StartNav();
                return;
            }
            Lateral = m;
            RouteVertical = false;
            SyncLateral();
        }

        public void SelectVertical(VerticalMode m)
        {
            if (m == VerticalMode.Tf && Rotorcraft)
            {
                report("TF N/A - ROTORCRAFT", true);
                return;
            }
            // RALT and TF share the radar-altitude target: switching between them keeps it.
            bool keepTarget = (m == VerticalMode.Tf && Vertical == VerticalMode.Ralt)
                              || (m == VerticalMode.Ralt && Vertical == VerticalMode.Tf);
            Vertical = m;
            RouteVertical = false;
            if (!keepTarget) SyncVertical();
            if (m == VerticalMode.Tf)
            {
                TgtRadarAlt = Mathf.Max(TgtRadarAlt, TfMinClearance);
                TfReset();
            }
        }

        /// <summary>The TF button: terrain following on jets; rotorcraft (no TF) get radar-altitude hold.</summary>
        public void SelectTf()
        {
            SelectVertical(Rotorcraft ? VerticalMode.Ralt : VerticalMode.Tf);
        }

        /// <summary>NAV on/off (radar page). On: fly the route from here; legs with a height also take
        /// over the TF target, leg by leg.</summary>
        public void ToggleNav()
        {
            if (Lateral == LateralMode.Nav) SelectLateral(LateralMode.Hdg);
            else StartNav();
        }

        private void StartNav()
        {
            if (Rotorcraft)
            {
                report("NAV - FIXED WING ONLY", true);
                return;
            }
            if (!Nav.Has)
            {
                report("NAV - NO ROUTE", true);
                return;
            }
            if (State.Valid && Nav.Active == 0)
            {
                Nav.StartX = State.PosX;
                Nav.StartZ = State.PosZ;
            }
            Lateral = LateralMode.Nav;
            navSpeedLeg = -1;
            if (!float.IsNaN(Nav.Points[Nav.Active].Agl))
            {
                // jets fly leg heights in TF (its button replaced RALT; NAV is fixed-wing only)
                if (Vertical != VerticalMode.Tf)
                {
                    Vertical = VerticalMode.Tf;
                    TfReset();
                }
                RouteVertical = true;
            }
            report("NAV ENGAGED", false);
        }

        /// <summary>The route was replaced (planner): keep flying it from where the jet is.</summary>
        public void OnRouteReplaced()
        {
            if (!State.Valid) return;
            Nav.StartX = State.PosX;
            Nav.StartZ = State.PosZ;
            navSpeedLeg = -1;
            if (Lateral == LateralMode.Nav && !Nav.Has) SelectLateral(LateralMode.Hdg);
        }

        public void SyncLateral()
        {
            if (!State.Valid || Lateral == LateralMode.Nav) return;
            TgtHeading = Mathf.Round(State.Heading) % 360f;
            TgtBank = Mathf.Abs(State.Bank) < AttHoldSnapDeg ? 0f
                : Mathf.Round(Mathf.Clamp(State.Bank, -cfg.MaxBank.Value, cfg.MaxBank.Value));
        }

        public void SyncVertical()
        {
            if (!State.Valid) return;
            TgtAltitude = Mathf.Max(State.Altitude, ApMath.MinAltitudeTarget);
            TgtRadarAlt = Mathf.Max(State.RadarAlt, ApMath.MinAltitudeTarget);
            if (Vertical == VerticalMode.Tf) TgtRadarAlt = Mathf.Max(TgtRadarAlt, TfMinClearance);
            raltFiltered = State.RadarAlt;
            // V/S captures the current rate rounded to the display step
            float step = Units.VsStep(false);
            TgtVs = Mathf.Round(State.VerticalSpeed / step) * step;
        }

        public void SyncSpeed()
        {
            if (State.Valid) TgtSpeed = Rotorcraft ? Mathf.Max(0f, Mathf.Round(State.FwdSpeed)) : State.Speed;
        }

        public void SyncAll()
        {
            SyncLateral();
            SyncVertical();
            SyncSpeed();
        }

        // ---- target rockers ----
        public void NudgeLateral(int dir, bool coarse)
        {
            coarse |= CoarseStep;
            if (Lateral == LateralMode.Nav)
            {
                // a heading rocker leaves NAV for HDG on the current heading, then steps it
                Lateral = LateralMode.Hdg;
                RouteVertical = false;
                TgtHeading = Mathf.Round(State.Heading) % 360f;
            }
            if (Lateral == LateralMode.Hdg)
            {
                float step = coarse ? 10f : 1f;
                TgtHeading = Mathf.Repeat(ApMath.Snap(TgtHeading, dir, step), 360f);
            }
            else
            {
                float step = coarse ? 5f : 1f;
                TgtBank = Mathf.Clamp(ApMath.Snap(TgtBank, dir, step), -cfg.MaxBank.Value, cfg.MaxBank.Value);
            }
        }

        public void NudgeVertical(int dir, bool coarse)
        {
            coarse |= CoarseStep;
            RouteVertical = false;
            switch (Vertical)
            {
                case VerticalMode.Balt:
                    // below one ALT step, step like TF so the target reaches sea-skimming heights
                    float fine = Units.AltStep(false);
                    bool low = dir > 0 ? TgtAltitude < fine - 0.01f : TgtAltitude <= fine + 0.01f;
                    TgtAltitude = Mathf.Max(ApMath.MinAltitudeTarget, ApMath.Snap(TgtAltitude, dir, low ? Units.RaltStep(coarse) : Units.AltStep(coarse)));
                    break;
                case VerticalMode.Ralt:
                    TgtRadarAlt = Mathf.Max(cfg.GroundFloor.Value, ApMath.MinAltitudeTarget, ApMath.Snap(TgtRadarAlt, dir, Units.RaltStep(coarse)));
                    break;
                case VerticalMode.Tf:
                    TgtRadarAlt = Mathf.Max(TfMinClearance, ApMath.Snap(TgtRadarAlt, dir, Units.RaltStep(coarse)));
                    break;
                default:
                    float lim = cfg.MaxClimbRate.Value;
                    TgtVs = Mathf.Clamp(ApMath.Snap(TgtVs, dir, Units.VsStep(coarse)), -lim, lim);
                    break;
            }
        }

        public void NudgeSpeed(int dir, bool coarse)
        {
            coarse |= CoarseStep;
            TgtSpeed = Mathf.Max(Rotorcraft ? 0f : cfg.MinEngageSpeed.Value, ApMath.Snap(TgtSpeed, dir, Units.SpeedStep(coarse)));
        }

        // =====================================================================
        // Per-physics-tick entry points (called from the Harmony patches)
        // =====================================================================

        /// <summary>
        /// Called after the game wrote the pilot's stick into <paramref name="inputs"/> and before
        /// FilterInputs(). Detects stick override, then replaces pitch/roll/throttle as needed.
        /// </summary>
        public void FixedTick(Aircraft ac, PilotPlayerState pps, ControlInputs inputs, bool cursorMode)
        {
            float dt = Time.fixedDeltaTime;
            State = Sample(ac);
            if (!State.Valid) return;

            suppressStickOverride = cursorMode;
            lastPps = pps;
            Rotorcraft = IsCollective(ac);
            if (Rotorcraft) BindRotorcraft(ac);
            if (Rotorcraft && Vertical == VerticalMode.Tf) Vertical = VerticalMode.Ralt;   // jets only
            pilotPitch = inputs.pitch;
            pilotRoll = inputs.roll;

            // Speed derivative for the A/T damping term
            if (!float.IsNaN(prevSpeed))
            {
                float a = (State.Speed - prevSpeed) / dt;
                accelFiltered += (a - accelFiltered) * (dt / (0.4f + dt));
            }
            prevSpeed = State.Speed;
            UpdateNav();

            if (ApEngaged)
            {
                string why = AutoDisconnectReason(ac, inputs);
                if (why != null)
                    SetAutopilot(ac, false, why);
            }

            ControlsFilter.FlyByWire fbw = null;
            if (!Rotorcraft)
            {
                // Rate loops learn from every tick, flown by the pilot or by us: the stick applied last
                // tick against the body-rate response it produced.
                fbw = GetFbw(ac);
                float hz = cfg.IndiFilterHz.Value;
                PitchAxis.Observe(State.PitchRate, lastPitchStick, PitchRatePerStick(fbw, State), dt, hz);
                RollAxis.Observe(State.RollRate, lastRollStick, RollRatePerStick(fbw, State), dt, hz);
                PitchAxis.IncrementGain = RollAxis.IncrementGain = cfg.IndiIncrementGain.Value;
            }

            if (ApEngaged && Rotorcraft)
                FlyRotor(ac, inputs, dt);
            else if (ApEngaged)
            {
                HasFbw = fbw != null;
                pitchAgility = fbw != null ? ApMath.Agility(FbwMaxPitchRate(fbw), 0.7f, 0.4f) : 1f;
                rollAgility = fbw != null ? ApMath.Agility(FbwMaxRollRate(fbw), 5f, 0.5f) : 1f;
                FlyVertical(ac, fbw, dt);
                FlyLateral(fbw, dt);
                inputs.pitch = OutPitch;
                inputs.roll = OutRoll;
            }
            else
            {
                TerrainAhead = SpeedProtectActive = false;
            }
            lastPitchStick = -inputs.pitch;   // ControlInputs.pitch is + nose-down
            lastRollStick = inputs.roll;

            if (AtEngaged && !Rotorcraft)
            {
                if (ac.cockpit.IsDetached())
                    SetAutothrottle(ac, pps, false, "A/T DISC");
                else
                {
                    FlySpeed(dt);
                    inputs.throttle = OutThrottle;
                }
            }
        }

        /// <summary>
        /// Called after the game computed the throttle from the pilot's lever. Records the lever
        /// (to detect throttle override) and re-applies the A/T output.
        /// </summary>
        public void OnPlayerThrottle(Aircraft ac, PilotPlayerState pps, ControlInputs inputs)
        {
            playerThrottle = inputs.throttle;
            // On a rotorcraft the A/P owns the collective; otherwise only the A/T owns the throttle.
            if (Rotorcraft ? !ApEngaged : !AtEngaged) return;
            if (float.IsNaN(playerThrottleAtEngage))
                playerThrottleAtEngage = playerThrottle;
            else if (Mathf.Abs(playerThrottle - playerThrottleAtEngage) > 0.08f)
            {
                if (Rotorcraft) SetAutopilot(ac, false, "A/P DISC - COLLECTIVE");
                else SetAutothrottle(ac, pps, false, "A/T DISC - THROTTLE");
                return;
            }
            inputs.throttle = OutThrottle;
        }

        private string AutoDisconnectReason(Aircraft ac, ControlInputs pilotInputs)
        {
            if (ac.cockpit.IsDetached()) return "A/P DISC";
            if (ac.IsAutoHoverEnabled()) return "A/P DISC - AUTO HOVER";
            if (State.RadarAlt < 0.5f && State.Speed < cfg.MinEngageSpeed.Value) return "A/P DISC - ON GROUND";

            // Paddle-switch behaviour: a deliberate stick input takes the jet back.
            float thr = cfg.StickOverride.Value;
            if (!suppressStickOverride && (Mathf.Abs(pilotInputs.pitch) > thr || Mathf.Abs(pilotInputs.roll) > thr))
                return "A/P DISC - STICK";
            if (Rotorcraft && !suppressStickOverride && Mathf.Abs(pilotInputs.yaw) > thr) return "A/P DISC - PEDALS";
            return null;
        }

        /// <summary>The fly-by-wire numbers the flight planner predicts turns with; false without one (rotorcraft, old jets).</summary>
        public static bool FbwNumbers(Aircraft ac, out float gLimit, out float corner, out float maxRollRate)
        {
            ControlsFilter.FlyByWire fbw = GetFbw(ac);
            gLimit = fbw != null ? FbwGLimit(fbw) : 0f;
            corner = fbw != null ? FbwCorner(fbw) : 0f;
            maxRollRate = fbw != null ? FbwMaxRollRate(fbw) : 0f;
            return fbw != null && !IsCollective(ac);
        }

        private static ControlsFilter.FlyByWire GetFbw(Aircraft ac)
        {
            if (!FbwReadable) return null;
            ControlsFilter cf = ac.GetControlsFilter();
            if (cf == null) return null;
            ControlsFilter.FlyByWire fbw = cf.GetFlyByWire();
            return fbw != null && fbw.Enabled ? fbw : null;
        }

        // =====================================================================
        // Vertical: altitude / radar altitude / vertical speed
        // =====================================================================
        private void FlyVertical(Aircraft ac, ControlsFilter.FlyByWire fbw, float dt)
        {
            FlightState s = State;
            float v = Mathf.Max(s.Speed, 40f);

            // Gain schedule: flight path lags pitch more at low speed, so soften the loops there.
            float sched = Mathf.Clamp(v / 150f, 0.6f, 1.6f);

            // Complementary filter for radar altitude: vertical speed carries the fast motion (no lag),
            // the radar pulls the estimate in over ~0.5 s (smooths terrain spikes).
            raltFiltered += s.VerticalSpeed * dt + (s.RadarAlt - raltFiltered) * Mathf.Clamp01(2f * dt);

            if (RouteVertical && Vertical == VerticalMode.Ralt) TgtRadarAlt = RouteHeight(v);
            else if (RouteVertical && Vertical == VerticalMode.Tf) TgtRadarAlt = Mathf.Max(RouteHeight(v), TfMinClearance);

            float maxVs = cfg.MaxClimbRate.Value;
            float k = cfg.AltitudeGain.Value * Mathf.Min(sched, 1f);
            float a = cfg.AltitudeCaptureAccel.Value;
            // Flight-path command slew (rad/s). Terrain following replaces the fixed rate with its ride's g.
            bool tf = Vertical == VerticalMode.Tf;
            float slewUp = 4f * Mathf.Deg2Rad, slewDown = slewUp;
            float qPull = cfg.MaxPullG.Value * G / v, qPush = cfg.MaxPushG.Value * G / v;   // pitch-rate limits, rad/s
            // The A/P's g budget: AirframeShare of the normal acceleration full stick gives this airframe at
            // this speed (the fly-by-wire's g limit, less below corner speed). Climb gets it first, the turn the rest.
            float avail = AirframeShare * PitchRatePerStick(fbw, s) * v;
            float vsCmd;
            switch (Vertical)
            {
                case VerticalMode.Balt: vsCmd = ApMath.CaptureProfile(TgtAltitude - s.Altitude, k, a, maxVs); break;
                case VerticalMode.Ralt: vsCmd = ApMath.CaptureProfile(TgtRadarAlt - raltFiltered, k, a, maxVs); break;
                case VerticalMode.Tf:
                    // TF's own g limits, not MaxPullG/MaxPushG.
                    float aPull = Mathf.Min(Mathf.Max(cfg.TfPullG.Value, 0.3f) * G, avail);
                    float aPush = Mathf.Min(Mathf.Max(cfg.TfPushG.Value, 0.2f) * G, avail);
                    // Steeply banked, the planned pull only comes once the jet has rolled back to the bank where it
                    // fits vertically: plan the ride with that roll-out time on top of the loop's lag.
                    float pullBank = ApMath.BankLimit(avail, ApMath.TfPlanG * aPull, G) * Mathf.Rad2Deg;
                    float rollOut = Mathf.Max(0f, Mathf.Abs(s.Bank) - pullBank) / (0.6f * rollLimit);
                    vsCmd = TfClimbRate(ac, v, k, ApMath.TfPlanG * aPull, ApMath.TfPlanG * aPush, TfLoopLag + rollOut, dt);
                    slewUp = qPull = aPull / v;
                    slewDown = qPush = aPush / v;
                    break;
                default: vsCmd = Mathf.Clamp(TgtVs, -maxVs, maxVs); break;
            }

            // Surface floor, on the airframe's LOWEST point (wingtips/nose/tail/gear): the closer the
            // floor, the slower we may sink toward it; further out, no faster than a FloorPullUpG
            // pull-up can stop; below it, climb back out. Then terrain ahead, likewise.
            float maxGammaSin = Mathf.Sin(cfg.MaxFlightPathAngle.Value * Mathf.Deg2Rad);
            float pullAccel = Mathf.Max(cfg.FloorPullG.Value, 0.1f) * G * Mathf.Max(Mathf.Cos(s.Bank * Mathf.Deg2Rad), 0.3f);
            float floor = cfg.GroundFloor.Value;
            float floorVs = ApMath.FloorVs(s.Clearance - floor, cfg.FloorGain.Value, pullAccel);
            float aheadVs = cfg.TerrainLookahead.Value ? LookaheadVs(ac, s, floor, pullAccel, v * maxGammaSin) : float.NegativeInfinity;
            TerrainAhead = aheadVs > floorVs && aheadVs > vsCmd;
            floorVs = Mathf.Max(floorVs, aheadVs);

            // Speed priority: A/T maxed and still slow -> climb only what the engines can sustain.
            float gammaCmd0 = Mathf.Asin(Mathf.Clamp(vsCmd / v, -maxGammaSin, maxGammaSin));
            UpdateSpeedProtection(s, dt);
            if (SpeedProtectActive && gammaMaxSpd < gammaCmd0)
                vsCmd = Mathf.Sin(gammaMaxSpd) * v;

            vsCmd = Mathf.Max(vsCmd, floorVs);
            CmdVs = vsCmd;

            float gammaCmd = Mathf.Asin(Mathf.Clamp(vsCmd / v, -maxGammaSin, maxGammaSin));
            float gammaFloor = Mathf.Asin(Mathf.Clamp(floorVs / v, -maxGammaSin, maxGammaSin));
            float slew = gammaCmd > gammaCmdSlewed ? slewUp : slewDown;
            // TF smoothing: a small change of command is flown gently, only a big one at the full g limit.
            if (tf && cfg.TfSmoothing.Value > dt)
                slew = Mathf.Min(slew, Mathf.Abs(gammaCmd - gammaCmdSlewed) / cfg.TfSmoothing.Value);
            gammaCmdSlewed = Mathf.MoveTowards(gammaCmdSlewed, gammaCmd, slew * dt);
            // The floor is never delayed by the slew limiter.
            gammaCmdSlewed = Mathf.Max(gammaCmdSlewed, gammaFloor);
            CmdGammaDeg = gammaCmdSlewed * Mathf.Rad2Deg;

            float gamma = s.Gamma * Mathf.Deg2Rad;

            // Flight-path-rate damping. Gamma lags pitch by the airframe's lift lag (seconds at low
            // speed), which is what makes a plain gamma loop porpoise; feeding back measured
            // d(gamma)/dt damps it without needing to know that lag.
            if (!float.IsNaN(gammaPrev))
                gammaRate += ((gamma - gammaPrev) / dt - gammaRate) * (dt / (0.2f + dt));
            gammaPrev = gamma;

            // Terrain following flies a moving flight-path command: feed its rate forward and damp only
            // the difference from it, or the damping term would hold the jet seconds behind the terrain.
            if (tf) tfCmdRate += ((gammaCmdSlewed - tfCmdPrev) / dt - tfCmdRate) * (dt / (0.2f + dt));
            else tfCmdRate = 0f;
            tfCmdPrev = gammaCmdSlewed;

            // Flight-path rate wanted (rad/s, vertical).
            float gammaDot = cfg.FlightPathGain.Value * sched * pitchAgility * (gammaCmdSlewed - gamma)
                             - cfg.FlightPathDamping.Value * (gammaRate - tfCmdRate) + tfCmdRate;

            // Lift vector (m/s^2): vertically gravity plus that climb, sideways the turn the lateral mode wants
            // (HDG/NAV: its turn rate; ATT: a level turn at the bank held). The lateral law banks to point the lift
            // that way (LiftBank); the pull is the lift needed along the bank the jet has now (LiftPull), never
            // lifting it higher than the climb wanted. So a push-over in a turn keeps pulling round, and a bank held
            // shallow by the wingtip limit over the water turns slower instead of climbing.
            float bankNow = Mathf.Clamp(s.Bank, -85f, 85f) * Mathf.Deg2Rad;
            float lv = G * Mathf.Cos(gamma) + Mathf.Clamp(gammaDot, -qPush, qPull) * v;
            float ah = Lateral != LateralMode.Att ? v * HdgTurnRate(s, float.PositiveInfinity) : lv * Mathf.Tan(bankNow);
            // budget: the stick's g on top of the share of gravity the bank leaves on the lift axis (pitch rate x V = n g - g cos(bank))
            liftBank = ApMath.LiftBank(ref lv, ref ah, avail + G * Mathf.Cos(bankNow), G, out float room) * Mathf.Rad2Deg;
            turnRateMax = room / v;
            turnBank = Mathf.Min(cfg.MaxBank.Value, ApMath.BankLimit(avail, 0.2f * G, G) * Mathf.Rad2Deg); // 0.2 g kept for the climb loop
            float qCmd = (ApMath.LiftPull(lv, ah, bankNow, G) - G * Mathf.Cos(gamma) * Mathf.Cos(bankNow)) / v;

            qCmd = Mathf.Clamp(qCmd, -qPush, avail / v); // never past the g budget
            CmdQ = qCmd;

            float model = PitchRatePerStick(fbw, s);
            bool adaptive = cfg.AdaptiveEffectiveness.Value;
            float stickUp;
            if (cfg.RateLoopLaw.Value == RateLaw.Indi)
            {
                stickUp = PitchAxis.Command(qCmd, s.PitchRate, model, cfg.IndiPitchBandwidth.Value * pitchAgility, adaptive, lastPitchStick);
                qI = Mathf.Clamp(stickUp - qCmd / (model * (adaptive ? PitchAxis.Rls.Ratio : 1f)), -0.6f, 0.6f); // bumpless switch to Classic
            }
            else
            {
                float qPerStick = model * (adaptive ? PitchAxis.Rls.Ratio : 1f);
                float qErr = qCmd - s.PitchRate;
                float ki = fbw != null ? cfg.PitchRateIntegralFbw.Value : cfg.PitchRateIntegralNoFbw.Value;
                float qINext = Mathf.Clamp(qI + ki * qErr / qPerStick * dt, -0.6f, 0.6f);
                stickUp = (qCmd + cfg.PitchRateGain.Value * qErr) / qPerStick + qINext;
                // Conditional anti-windup: only keep integrating if it doesn't push further into saturation.
                if (!(stickUp > 1f && qErr > 0f) && !(stickUp < -1f && qErr < 0f))
                    qI = qINext;
                stickUp = Mathf.Clamp((qCmd + cfg.PitchRateGain.Value * qErr) / qPerStick + qI, -1f, 1f);
            }

            OutPitch = -stickUp;   // ControlInputs.pitch is + nose-down
        }

        // ---- terrain look-ahead ----
        private const int AheadRays = 12;

        /// <summary>
        /// Lowest climb rate that still clears what lies ahead: terrain, ship decks and the sea are
        /// sampled along the predicted ground track (the current turn continued, up to 90 deg), with the
        /// samples bunched near the jet. Each sample's surface is charged to the PREVIOUS sample's time,
        /// since the rise may start anywhere in between. Pull-up after a 1 s reaction, at pullAccel,
        /// up to vsMax (the steepest flight path the A/P will fly).
        /// </summary>
        private float LookaheadVs(Aircraft ac, FlightState s, float floor, float pullAccel, float vsMax)
        {
            float v = new Vector2(s.VelX, s.VelZ).magnitude;
            if (v < 20f) return float.NegativeInfinity;
            Vector3 pos = ac.transform.position;
            float sea = Datum.LocalSeaY;
            float lowestY = pos.y - s.LowestDrop;
            float horizon = Mathf.Max(cfg.LookaheadSeconds.Value, 3f);
            float psi0 = Mathf.Atan2(s.VelX, s.VelZ);
            float omega = G * Mathf.Tan(Mathf.Clamp(s.Bank, -70f, 70f) * Mathf.Deg2Rad) / v;
            int mask = (int)PhysicsLayers.StaticsMask | (int)PhysicsLayers.ShipsMask;

            float best = float.NegativeInfinity, tPrev = 0.25f;
            for (int k = 1; k <= AheadRays; k++)
            {
                float f = k / (float)AheadRays, t = horizon * f * f;
                float turn = Mathf.Clamp(omega * t, -Mathf.PI * 0.5f, Mathf.PI * 0.5f);
                float dx, dz;
                if (Mathf.Abs(omega) < 1e-3f)
                {
                    dx = v * t * Mathf.Sin(psi0);
                    dz = v * t * Mathf.Cos(psi0);
                }
                else
                {
                    float r = v / omega, psi = psi0 + turn;
                    dx = r * (Mathf.Cos(psi0) - Mathf.Cos(psi));
                    dz = r * (Mathf.Sin(psi) - Mathf.Sin(psi0));
                }
                Vector3 top = new Vector3(pos.x + dx, sea + 10000f, pos.z + dz);
                float surface = sea;
                if (Physics.Raycast(top, Vector3.down, out RaycastHit hit, 10500f, mask))
                    surface = Mathf.Max(surface, hit.point.y);
                // margin: the floor, plus 5 m and 0.5% of the distance for terrain the rays miss
                float room = lowestY - surface - floor - 5f - 0.005f * v * t;
                best = Mathf.Max(best, ApMath.LookaheadMinVs(room, tPrev, pullAccel, vsMax, 1f));
                tPrev = t;
            }
            return best;
        }

        // ---- speed priority (TECSMOD-style) ----
        private void UpdateSpeedProtection(FlightState s, float dt)
        {
            if (!AtEngaged || !cfg.SpeedProtection.Value)
            {
                SpeedProtectActive = false;
                gammaMaxSpd = float.NaN;
                return;
            }
            float hi = Mathf.Clamp(cfg.AtMaxThrottle.Value, 0f, 1f);
            float deficit = TgtSpeed - s.Speed, margin = Mathf.Max(5f, 0.05f * TgtSpeed);
            bool maxed = OutThrottle >= hi - 0.02f;
            if (!SpeedProtectActive && maxed && deficit > margin) SpeedProtectActive = true;
            else if (SpeedProtectActive && (deficit < 0.5f * margin || OutThrottle < hi - 0.05f)) SpeedProtectActive = false;

            // energy the engines are sustaining now: gamma + Vdot/g (dimensionless specific excess power)
            float energyRate = s.Gamma * Mathf.Deg2Rad + accelFiltered / G;
            float target = ApMath.SpeedProtectGamma(energyRate, deficit, margin);
            gammaMaxSpd = float.IsNaN(gammaMaxSpd) ? target : gammaMaxSpd + (target - gammaMaxSpd) * Mathf.Clamp01(dt / 1f);
        }

        /// <summary>RALT target for the leg being flown; a lower next leg is descended to early enough
        /// to arrive low before its waypoint (descending at ~8 m/s, plus the turn lead).</summary>
        private float RouteHeight(float v)
        {
            if (!Nav.Has) return TgtRadarAlt;
            float h = Nav.Points[Nav.Active].Agl;
            if (float.IsNaN(h)) return TgtRadarAlt;
            if (Nav.Active + 1 < Nav.Points.Count)
            {
                float next = Nav.Points[Nav.Active + 1].Agl;
                if (!float.IsNaN(next) && next < h && NavToGo < (h - next) / 8f * v + 2000f) h = next;
            }
            return Mathf.Max(h, cfg.GroundFloor.Value + 2f, ApMath.MinAltitudeTarget);
        }

        /// <summary>Body pitch rate (rad/s) the airframe produces per unit of stick.</summary>
        private static float PitchRatePerStick(ControlsFilter.FlyByWire fbw, FlightState s)
        {
            if (fbw != null)
            {
                // Mirrors ControlsFilter.FlyByWire.Filter(): the FBW turns stick into a pitch-rate demand.
                float corner = Mathf.Max(FbwCorner(fbw), 1f);
                float qbarRatio = s.Speed * s.Speed * s.AirDensity / (corner * corner * 1.225f);
                float q = FbwGLimit(fbw) * G / Mathf.Max(s.Speed, corner * 0.75f);
                if (qbarRatio < 1f) q *= Mathf.Clamp(qbarRatio, 0.3f, 1f);
                q = Mathf.Lerp(FbwMaxPitchRate(fbw), q, Mathf.Clamp01(FbwLimitFactor(fbw)));
                return Mathf.Max(q, 0.05f);
            }
            // Conventional controls: elevator authority grows with dynamic pressure.
            return Mathf.Clamp(0.35f * QbarRatio(s, 150f), 0.08f, 1f);
        }

        // =====================================================================
        // Lateral: heading select / attitude (bank) hold
        // =====================================================================
        private void FlyLateral(ControlsFilter.FlyByWire fbw, float dt)
        {
            FlightState s = State;
            float v = Mathf.Max(s.Speed, 40f);
            // The climb's share of the lift is already taken out of the bank (FlyVertical's lift vector).
            float maxBank = cfg.MaxBank.Value;

            float bankCmd;
            if (Lateral == LateralMode.Att)
                bankCmd = TgtBank;
            else
                bankCmd = liftBank; // HDG / NAV: points the lift at the turn and the climb together (FlyVertical)
            // Near the surface, limit bank so the low wingtip stays above the floor
            // (a 15 m span at 35 deg drops the tip ~4 m below the radar-altitude reference).
            float originHeight = s.RadarAlt + s.GearDrop;
            if (s.HalfSpan > 0.5f)
            {
                float room = (originHeight - cfg.GroundFloor.Value - 0.5f) / s.HalfSpan;
                float surfaceBank = Mathf.Asin(Mathf.Clamp01(room)) * Mathf.Rad2Deg;
                BankLimitedBySurface = surfaceBank < maxBank;
                maxBank = Mathf.Min(maxBank, surfaceBank);
            }
            else BankLimitedBySurface = false;

            bankLimit = maxBank;
            bankCmd = Mathf.Clamp(bankCmd, -maxBank, maxBank);
            // the configured rate, but never more than AirframeShare of the airframe's own full-stick roll rate
            float maxRoll = rollLimit = Mathf.Min(cfg.MaxRollRate.Value * rollAgility, AirframeShare * RollRatePerStick(fbw, s) * Mathf.Rad2Deg);
            bankCmdSlewed = Mathf.MoveTowards(bankCmdSlewed, bankCmd, 0.6f * maxRoll * dt);
            CmdBankDeg = bankCmdSlewed;

            float maxP = maxRoll * Mathf.Deg2Rad;
            float pCmd = Mathf.Clamp(cfg.BankGain.Value * rollAgility * Mathf.DeltaAngle(s.Bank, bankCmdSlewed) * Mathf.Deg2Rad, -maxP, maxP);
            CmdP = pCmd;

            float model = RollRatePerStick(fbw, s);
            bool adaptive = cfg.AdaptiveEffectiveness.Value;
            if (cfg.RateLoopLaw.Value == RateLaw.Indi)
            {
                OutRoll = RollAxis.Command(pCmd, s.RollRate, model, cfg.IndiRollBandwidth.Value * rollAgility, adaptive, lastRollStick);
                pI = Mathf.Clamp(OutRoll - pCmd / (model * (adaptive ? RollAxis.Rls.Ratio : 1f)), -0.4f, 0.4f); // bumpless switch to Classic
                return;
            }
            float pPerStick = model * (adaptive ? RollAxis.Rls.Ratio : 1f);
            float pErr = pCmd - s.RollRate;
            float pINext = Mathf.Clamp(pI + cfg.RollRateIntegral.Value * pErr / pPerStick * dt, -0.4f, 0.4f);
            float stick = (pCmd + cfg.RollRateGain.Value * pErr) / pPerStick + pINext;
            if (!(stick > 1f && pErr > 0f) && !(stick < -1f && pErr < 0f))
                pI = pINext;
            OutRoll = Mathf.Clamp((pCmd + cfg.RollRateGain.Value * pErr) / pPerStick + pI, -1f, 1f);
        }

        /// <summary>
        /// NAV, once per physics tick: L1 guidance along the active leg, sequencing to the next one a turn-radius
        /// lead before the waypoint so the turn rolls out on the new leg. Leaves the turn rate (rad/s, + right) in
        /// navTurnRate for the HDG law (HdgTurnRate), which flies it through the lift vector like any other turn.
        /// </summary>
        private void UpdateNav()
        {
            if (!ApEngaged || Lateral != LateralMode.Nav) return;
            FlightState s = State;
            float maxBank = cfg.MaxBank.Value;
            float v = Mathf.Max(new Vector2(s.VelX, s.VelZ).magnitude, 40f);
            float maxAcc = G * Mathf.Tan(Mathf.Clamp(maxBank, 5f, 70f) * Mathf.Deg2Rad);
            float acc = 0f;
            for (int guard = 0; guard < 4 && Nav.Has; guard++)
            {
                Nav.LegStart(out float ax, out float az);
                Waypoint w = Nav.Points[Nav.Active];
                acc = L1Guidance.LateralAccel(s.PosX, s.PosZ, s.VelX, s.VelZ, ax, az, w.X, w.Z,
                    cfg.L1Period.Value, cfg.L1Damping.Value, maxAcc, out NavCrossTrack, out NavToGo);
                TgtHeading = L1Guidance.Course(ax, az, w.X, w.Z);
                float lead = 0f;
                if (Nav.Active + 1 < Nav.Points.Count)
                {
                    Waypoint n = Nav.Points[Nav.Active + 1];
                    float change = Mathf.DeltaAngle(TgtHeading, L1Guidance.Course(w.X, w.Z, n.X, n.Z));
                    lead = L1Guidance.TurnLead(v, maxBank, change, new Vector2(n.X - w.X, n.Z - w.Z).magnitude);
                }
                if (NavToGo > Mathf.Max(lead, 0f)) break;
                Nav.Active++;   // passed (or turning onto the next leg)
                Nav.Version++;
            }
            if (Nav.Has)
            {
                navTurnRate = acc / v;
                // a leg's speed, if set, becomes the A/T target as the leg starts (the rockers override it until the next)
                if (navSpeedLeg != Nav.Active)
                {
                    navSpeedLeg = Nav.Active;
                    float spd = Nav.Points[Nav.Active].Speed;
                    if (spd > 0f) TgtSpeed = spd;
                }
                return;
            }
            // route finished or cleared
            Lateral = LateralMode.Hdg;
            RouteVertical = false;
            navTurnRate = 0f;
            TgtHeading = Mathf.Round(s.Heading) % 360f;
            report("NAV - ROUTE END", false);
        }

        private float[] etaPath = new float[16];

        /// <summary>
        /// Seconds to the first selected target (along the route while in NAV, else straight), or with no
        /// target to the route's last point (NAV only), at the current ground speed. False = nothing to time.
        /// </summary>
        public bool Eta(Aircraft ac, out float seconds, out bool toTarget)
        {
            seconds = 0f;
            List<Unit> targets = TargetPanel.Targets();
            GlobalPosition t = default;
            bool nav = Lateral == LateralMode.Nav && Nav.Has;
            toTarget = targets != null && targets.Count > 0 && targets[0] != null && TargetPanel.KnownPosition(targets[0], out t);
            if (ac == null || !toTarget && !nav) return false;
            Vector3 v = ac.CockpitRB().velocity;
            float gs = new Vector2(v.x, v.z).magnitude;
            if (gs < 5f) return false; // parked or hovering: no meaningful ETA

            GlobalPosition g = ac.GlobalPosition();
            int n = nav ? Nav.Points.Count - Nav.Active : 0;
            if (etaPath.Length < 2 * n) etaPath = new float[4 * n];
            for (int i = 0; i < n; i++)
            {
                etaPath[2 * i] = Nav.Points[Nav.Active + i].X - g.x;
                etaPath[2 * i + 1] = Nav.Points[Nav.Active + i].Z - g.z;
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

        /// <summary>
        /// Turn rate the HDG law commands (rad/s, + right; NAV: its L1 guidance's), no faster than
        /// <paramref name="most"/> (rad/s).
        /// </summary>
        private float HdgTurnRate(FlightState s, float most)
        {
            float max = Mathf.Min(cfg.MaxTurnRate.Value * Mathf.Deg2Rad, most);
            return Mathf.Clamp(Lateral == LateralMode.Nav ? navTurnRate : cfg.HeadingGain.Value * Mathf.DeltaAngle(s.Heading, TgtHeading) * Mathf.Deg2Rad, -max, max);
        }

        /// <summary>Body roll rate (rad/s) per unit of stick.</summary>
        private static float RollRatePerStick(ControlsFilter.FlyByWire fbw, FlightState s)
        {
            if (fbw != null)
            {
                float corner = Mathf.Max(FbwCorner(fbw), 1f);
                float qbarRatio = s.Speed * s.Speed * s.AirDensity / (corner * corner * 1.225f);
                float p = FbwMaxRollRate(fbw) * Mathf.Clamp(qbarRatio / Mathf.Max(FbwMaxRollSpeed(fbw), 0.01f), 0.5f, 1f);
                return Mathf.Clamp(p, 0.5f, 8f);
            }
            return Mathf.Clamp(2f * QbarRatio(s, 150f), 0.3f, 4f);
        }

        private static float QbarRatio(FlightState s, float refSpeed)
        {
            return s.Speed * s.Speed * s.AirDensity / (refSpeed * refSpeed * 1.225f);
        }

        // =====================================================================
        // Autothrottle
        // =====================================================================
        private float energyFf;   // throttle feedforward from energy coupling, as last applied

        private void FlySpeed(float dt)
        {
            float lo = Mathf.Clamp01(cfg.AtMinThrottle.Value);
            float hi = Mathf.Clamp(cfg.AtMaxThrottle.Value, lo, 1f);
            float e = TgtSpeed - State.Speed;
            float pd = cfg.SpeedGain.Value * e - cfg.SpeedDamping.Value * accelFiltered;

            // Energy coupling (TECS-style): the A/P's flight-path command and bank feed the throttle
            // directly, and a flight-path error counts like an acceleration error (same weight).
            bool coupled = ApEngaged && cfg.EnergyCoupling.Value;
            float ff = 0f;
            if (coupled)
            {
                ff = ApMath.EnergyFeedforward(gammaCmdSlewed, State.Bank * Mathf.Deg2Rad, cfg.ClimbFeedforward.Value, cfg.TurnFeedforward.Value);
                pd += cfg.SpeedDamping.Value * G * (gammaCmdSlewed - State.Gamma * Mathf.Deg2Rad);
            }
            // bumpless when coupling comes or goes (A/P engaged/disengaged): the integrator absorbs the step
            if (coupled != energyCoupledLast) thrI += energyFf - ff;
            energyCoupledLast = coupled;
            energyFf = ff;
            pd += ff;

            thrI += cfg.SpeedIntegral.Value * e * dt;
            thrI = Mathf.Clamp(thrI, lo - pd, hi - pd);   // back-calculation anti-windup
            OutThrottle = Mathf.Clamp(thrI + pd, lo, hi);
        }

        private bool energyCoupledLast;
    }

    /// <summary>Access to PilotPlayerState's private throttle lever, for bumpless A/T disconnect.</summary>
    internal static class PlayerThrottleBridge
    {
        private static readonly AccessTools.FieldRef<PilotPlayerState, float> SimulatedThrottle = TryRef();

        private static AccessTools.FieldRef<PilotPlayerState, float> TryRef()
        {
            try { return AccessTools.FieldRefAccess<PilotPlayerState, float>("simulatedThrottle"); }
            catch { return null; }
        }

        public static void SetLever(PilotPlayerState pps, float throttle, bool collective = false)
        {
            if (SimulatedThrottle == null || pps == null) return;
            // Inverse of PilotPlayerState.PlayerThrottleAxis1Controls' mapping.
            if (collective && PlayerSettings.invertCollective) throttle = 1f - throttle;
            SimulatedThrottle(pps) = PlayerSettings.throttleUseNegative ? throttle * 2f - 1f : throttle;
        }
    }
}
