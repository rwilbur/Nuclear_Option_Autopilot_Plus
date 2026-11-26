using System;
using System.Linq;
using BepInEx.Configuration;
using BepInEx.Logging;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>What the limit override does to the AoA limit.</summary>
    internal enum OverrideAoa
    {
        /// <summary>None: flies as with Stability Assist off (full pitch rate, past the lift peak if held).</summary>
        NoLimit,
        /// <summary>Raised to the lift peak: the most turn there is, and no further.</summary>
        LiftPeak,
    }

    internal enum FbwMode
    {
        /// <summary>Stock fly-by-wire, untouched. Telemetry can still log.</summary>
        Off,
        /// <summary>Test of the takeover: every surface and engine gets its own inputs, copied from the shared ones exactly when stock would read them. Flies exactly like stock.</summary>
        Passthrough,
        /// <summary>
        /// The FBW flies: rate laws, the envelope governor (AoA and g), INDI, every surface and nozzle allocated on its own. In flight above
        /// <see cref="FbwController.EngageSpeed"/> only; stock below it and on the ground.
        /// </summary>
        Full,
        /// <summary>
        /// The same fly-by-wire as Full, writing only the shared pitch, roll and yaw (and the throttle's engine-out cut): every
        /// surface and nozzle moves by the game's stock mixing, so other players see exactly what flies (docs/fbw/PLAN.md, phase 6).
        /// </summary>
        Multiplayer,
    }

    /// <summary>
    /// The fly-by-wire's lifecycle on the local player's aircraft: which aircraft it holds, when Full mode flies,
    /// the per-tick entry points the Harmony hooks call, telemetry, and the hand-back to stock (including the
    /// stock FBW's own switch) on leaving Full mode, on a mode change and on any error.
    ///
    /// Tick order (docs/fbw/PHASE0_FACTS.md section 1): <see cref="PreJobs"/> runs before the game's surface
    /// job takes its copy of the inputs; the pilot's stick and the stock filter run after that; then
    /// <see cref="AfterFilter"/>; then the engines' FixedUpdate reads its inputs.
    /// </summary>
    internal sealed class FbwController
    {
        /// <summary>
        /// Full mode flies above this airspeed (m/s) with no wheel carrying weight, and lets go below the lower one or on touchdown. No
        /// height limit: sea-skimming is flown (it was 5 m to engage, 2 m to let go, which also kept it off the runway; the wheels do that now).
        /// </summary>
        public const float EngageSpeed = 60f, ReleaseSpeed = 50f;

        /// <summary>Set by the plugin; the Harmony hooks call through it.</summary>
        public static FbwController Current;

        private readonly ConfigEntry<FbwMode> mode;
        private readonly ConfigEntry<bool> telemetry;
        private readonly ApSettings cfg;
        private readonly ManualLogSource log;

        private AircraftLink link;
        private FbwFlight flight;        // Full mode's computer for the held aircraft
        private bool flying;             // Full mode is flying the jet now
        private bool cutting;            // the engine-out throttle cut is on (for the log)
        private ControlsFilter.FlyByWire stockFbw;
        private bool stockWasEnabled;
        private FbwTelemetry tlm;
        private AeroCheck check;         // phase-2 diagnostics, with telemetry
        private Aircraft failed;         // the aircraft the FBW faulted on: stays stock until it is replaced

        public FbwController(ApSettings cfg, ManualLogSource log)
        {
            this.cfg = cfg;
            mode = cfg.FbwMode;
            telemetry = cfg.FbwTelemetry;
            this.log = log;
        }

        /// <summary>The aircraft held now, or null.</summary>
        public Aircraft Holding => link?.Aircraft;

        /// <summary>Full mode is flying the held jet.</summary>
        public bool Flying => flying;

        /// <summary>Full mode's computer while it flies the held jet, else null (the envelope display reads it).</summary>
        public FbwFlight Flight => flying || observing ? flight : null;

        private bool observing; // Full or Multiplayer mode, not flying: the envelope is still worked out, for the display
        private int offWheels;  // ticks with no wheel carrying weight
        private const int LiftoffTicks = 30;

        /// <summary>Start of the physics tick, before JobManager.ScheduleJobs.</summary>
        public void PreJobs()
        {
            try
            {
                Aircraft target = Target();
                if (link != null && link.Aircraft != target) Unbind();
                if (target == null) return;
                if (link == null) Bind(target);

                float dt = Time.fixedDeltaTime;
                AirframeState state = link.Sample(dt);
                Aircraft ac = link.Aircraft;
                bool shared = mode.Value == FbwMode.Multiplayer;
                bool full = mode.Value == FbwMode.Full || shared;
                if (flying && flight.Shared != shared) StopFlying("mode changed"); // Full <-> Multiplayer: a new computer next tick
                // engaging waits for half a second off the wheels (a bounce on touchdown doesn't flick it on); touchdown lets go at once
                offWheels = link.OnWheels ? 0 : offWheels + 1;
                bool airborne = flying ? offWheels > 0 && ac.speed > ReleaseSpeed : offWheels > LiftoffTicks && ac.speed > EngageSpeed;
                if (full && airborne && !flying) StartFlying(state);
                else if (flying && !(full && airborne)) StopFlying(full ? "slow or on the ground" : "mode changed");
                observing = false;

                if (flying)
                {
                    if (shared) link.Release(); // every surface and engine on the shared inputs: stock mixing moves them
                    else
                    {
                        link.HoldSurfaces();
                        link.HoldEngines();
                    }
                    KeyboardShortcut k = cfg.KeyLimitOverride.Value;
                    flight.LimitOverride = k.MainKey != KeyCode.None && Input.GetKey(k.MainKey) && k.Modifiers.All(Input.GetKey);
                    flight.OverridePeak = cfg.OverrideAoa.Value == OverrideAoa.LiftPeak;
                    flight.OverrideG = cfg.OverrideG.Value;
                    flight.SlipAllowance = cfg.RollSideslip.Value / AirData.Rad2Deg;
                    flight.Tick(state, dt);
                    float cut = flight.Computer.Cut.Cut;
                    if (cut > 0.05f != cutting)
                    {
                        cutting = !cutting;
                        log.LogInfo(cutting ? $"FBW: engine-out throttle cut ({cut:P0}): the yaw can't hold the thrust asymmetry" : "FBW: engine-out throttle cut given back");
                    }
                }
                else if (mode.Value != FbwMode.Off)
                {
                    link.HoldSurfaces();
                    // passthrough (and Full mode below its speed): the surfaces get exactly what their job would read now
                    ControlInputs from = link.Shared;
                    foreach (SurfaceLink s in link.Surfaces) Copy(from, s.Inputs);
                    if (full)
                    {
                        // standby: the stock fly-by-wire flies, the envelope display still shows the jet's envelope
                        EnsureFlight(shared);
                        flight.Observe(state, dt);
                        observing = true;
                    }
                }
                else link.Release();

                if (telemetry.Value)
                {
                    tlm ??= new FbwTelemetry(link);
                    check ??= new AeroCheck(link, log);
                    tlm.Row(link, state, mode.Value != FbwMode.Off, check, flying ? flight : null);
                }
                else CloseTelemetry();
            }
            catch (Exception e)
            {
                Fail(e);
            }
        }

        /// <summary>After Aircraft.FilterInputs, before the engines' FixedUpdate reads the inputs.</summary>
        public void AfterFilter(Aircraft aircraft)
        {
            if (link == null || aircraft != link.Aircraft || mode.Value == FbwMode.Off) return;
            try
            {
                if (flying)
                {
                    if (flight.Shared) flight.AfterFilter(); // multiplayer mode: keep the pilot's inputs, put the command back
                }
                else
                {
                    link.HoldEngines();
                    // the engines read the inputs in the same tick they are filtered, one tick sooner than the surfaces
                    ControlInputs from = link.Shared;
                    foreach (EngineLink e in link.Engines) Copy(from, e.Inputs);
                }
                EngineOutTest();
            }
            catch (Exception e)
            {
                Fail(e);
            }
        }

        /// <summary>
        /// Development test, not saved (off at every launch): -1 / +1 holds the outermost left / right jet engine at idle through its own
        /// inputs, the nearest the controls come to an engine-out (idle still gives about 7.5% thrust; the game has no per-engine
        /// switch, and a real failure comes only from hits). Any mode but Off; Passthrough flies it on the stock fly-by-wire, to compare.
        /// </summary>
        public int TestEngineOut;

        /// <summary>The engine the test holds at idle on this aircraft, or null (off, or no engine out on that side).</summary>
        public EngineLink TestEngine()
        {
            if (TestEngineOut == 0 || link == null) return null;
            EngineLink pick = null;
            float best = 0.3f; // m off the centreline: a centreline engine is no engine-out test
            Transform root = link.Aircraft.transform;
            foreach (EngineLink e in link.Engines)
            {
                Transform t = e.Transform;
                if (t == null) continue;
                float x = root.InverseTransformPoint(t.position).x * TestEngineOut;
                if (x > best) { best = x; pick = e; }
            }
            return pick;
        }

        // the test engine to idle, after everything else has written the engines' inputs and before they read them
        private void EngineOutTest()
        {
            EngineLink e = TestEngine();
            if (e == null) return;
            if (!e.Held)
            {
                e.Hold(); // multiplayer mode leaves the engines on the shared inputs: this one gets their copy
                Copy(link.Shared, e.Inputs);
            }
            e.Inputs.throttle = 0f;
        }

        /// <summary>JobManager.FixedUpdateLate: the aero job's forces are in, the physics step is next.</summary>
        public void AfterAero()
        {
            if (check == null) return;
            try { check.AfterAero(); }
            catch (Exception e) { Fail(e); }
        }

        /// <summary>After a Turbojet's or Turbofan's FixedUpdate: its thrust for this tick is applied.</summary>
        public void AfterEngine(MonoBehaviour engine)
        {
            if (check == null || !link.Owns(engine)) return;
            try { check.AfterEngine(engine); }
            catch (Exception e) { Fail(e); }
        }

        /// <summary>Plugin unload: hand everything back.</summary>
        public void Shutdown() => Unbind();

        private Aircraft Target()
        {
            if (mode.Value == FbwMode.Off && !telemetry.Value) return null;
            if (!GameManager.GetLocalAircraft(out Aircraft ac) || ac == null) return null;
            if (ac == failed || !ac.LocalSim || ac.disabled) return null;
            return ac;
        }

        private void Bind(Aircraft ac)
        {
            link = new AircraftLink(ac);
            log.LogInfo($"FBW: {ac.definition.unitName}, {link.Surfaces.Count} surfaces, {link.Engines.Count} jet engines ({mode.Value})");
        }

        private void EnsureFlight(bool shared)
        {
            if (flight != null && flight.Shared == shared) return;
            ControlsFilter cf = link.Aircraft.GetControlsFilter();
            flight = new FbwFlight(link, cf != null ? cf.GetFlyByWire() : null, shared);
        }

        private void StartFlying(in AirframeState state)
        {
            ControlsFilter cf = link.Aircraft.GetControlsFilter();
            stockFbw = cf != null ? cf.GetFlyByWire() : null;
            EnsureFlight(mode.Value == FbwMode.Multiplayer);
            // the stock FBW stops reshaping the shared inputs: they stay the pilot's stick (plus hover and aim assist)
            if (stockFbw != null)
            {
                stockWasEnabled = stockFbw.Enabled;
                stockFbw.Enabled = false;
            }
            flight.Engage(state);
            flying = true;
            RateLawLog();
        }

        private void StopFlying(string why)
        {
            flying = false;
            cutting = false;
            if (stockFbw != null) stockFbw.Enabled = stockWasEnabled;
            stockFbw = null;
            log.LogInfo("FBW: stock fly-by-wire back (" + why + ")");
        }

        private void RateLawLog()
        {
            var law = flight.Computer.Law;
            log.LogInfo($"FBW flying {link.Aircraft.definition.unitName}: {flight.Surfaces} surfaces, {flight.Splits} splits, {flight.Nozzles} vectoring engines; " +
                        $"roll {law.PMax * AirData.Rad2Deg:F0} deg/s, pitch {law.QMax * AirData.Rad2Deg:F0} deg/s, {law.NzMax:F1} g");
        }

        private void Unbind()
        {
            if (flying) StopFlying("released");
            link?.Release();
            link = null;
            flight = null;
            CloseTelemetry();
        }

        private void CloseTelemetry()
        {
            check?.Dispose();
            check = null;
            if (tlm == null) return;
            log.LogInfo("FBW telemetry: " + tlm.Path);
            tlm.Dispose();
            tlm = null;
        }

        private void Fail(Exception e)
        {
            failed = link?.Aircraft;
            log.LogError("FBW fault, back to the stock fly-by-wire for this aircraft: " + e);
            // the stock FBW's switch first: whatever else fails, the jet must not be left with no FBW at all
            try { if (stockFbw != null) stockFbw.Enabled = stockWasEnabled; }
            catch (Exception e3) { log.LogError("FBW: could not switch the stock fly-by-wire back on: " + e3); }
            try { Unbind(); }
            catch (Exception e2) { log.LogError("FBW release failed: " + e2); }
            link = null;
            flight = null;
            flying = false;
        }

        private static void Copy(ControlInputs from, ControlInputs to)
        {
            to.pitch = from.pitch;
            to.roll = from.roll;
            to.yaw = from.yaw;
            to.throttle = from.throttle;
            to.brake = from.brake;
            to.customAxis1 = from.customAxis1;
        }
    }
}
