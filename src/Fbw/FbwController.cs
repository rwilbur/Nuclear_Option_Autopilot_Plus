using System;
using System.Linq;
using BepInEx.Configuration;
using BepInEx.Logging;
using UnityEngine;

namespace AutopilotMod.Fbw
{
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
        /// <summary>Full mode flies above this airspeed (m/s) and radar altitude, and lets go below the lower pair.</summary>
        public const float EngageSpeed = 60f, ReleaseSpeed = 50f, EngageHeight = 5f, ReleaseHeight = 2f;

        /// <summary>Set by the plugin; the Harmony hooks call through it.</summary>
        public static FbwController Current;

        private readonly ConfigEntry<FbwMode> mode;
        private readonly ConfigEntry<bool> telemetry;
        private readonly ConfigEntry<KeyboardShortcut> overrideKey;
        private readonly ManualLogSource log;

        private AircraftLink link;
        private FbwFlight flight;        // Full mode's computer for the held aircraft
        private bool flying;             // Full mode is flying the jet now
        private ControlsFilter.FlyByWire stockFbw;
        private bool stockWasEnabled;
        private FbwTelemetry tlm;
        private AeroCheck check;         // phase-2 diagnostics, with telemetry
        private Aircraft failed;         // the aircraft the FBW faulted on: stays stock until it is replaced

        public FbwController(ConfigEntry<FbwMode> mode, ConfigEntry<bool> telemetry, ConfigEntry<KeyboardShortcut> overrideKey, ManualLogSource log)
        {
            this.mode = mode;
            this.telemetry = telemetry;
            this.overrideKey = overrideKey;
            this.log = log;
        }

        /// <summary>The aircraft held now, or null.</summary>
        public Aircraft Holding => link?.Aircraft;

        /// <summary>Full mode is flying the held jet.</summary>
        public bool Flying => flying;

        /// <summary>Full mode's computer while it flies the held jet, else null (the envelope display reads it).</summary>
        public FbwFlight Flight => flying ? flight : null;

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
                bool full = mode.Value == FbwMode.Full;
                bool airborne = flying
                    ? ac.speed > ReleaseSpeed && ac.radarAlt > ReleaseHeight
                    : ac.speed > EngageSpeed && ac.radarAlt > EngageHeight;
                if (full && airborne && !flying) StartFlying(state);
                else if (flying && !(full && airborne)) StopFlying(full ? "slow or low" : "mode changed");

                if (flying)
                {
                    link.HoldSurfaces();
                    link.HoldEngines();
                    KeyboardShortcut k = overrideKey.Value;
                    flight.LimitOverride = k.MainKey != KeyCode.None && Input.GetKey(k.MainKey) && k.Modifiers.All(Input.GetKey);
                    flight.Tick(state, dt);
                }
                else if (mode.Value != FbwMode.Off)
                {
                    link.HoldSurfaces();
                    // passthrough (and Full mode below its speed): the surfaces get exactly what their job would read now
                    ControlInputs from = link.Shared;
                    foreach (SurfaceLink s in link.Surfaces) Copy(from, s.Inputs);
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
            if (link == null || aircraft != link.Aircraft || flying || mode.Value == FbwMode.Off) return;
            try
            {
                link.HoldEngines();
                // the engines read the inputs in the same tick they are filtered, one tick sooner than the surfaces
                ControlInputs from = link.Shared;
                foreach (EngineLink e in link.Engines) Copy(from, e.Inputs);
            }
            catch (Exception e)
            {
                Fail(e);
            }
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

        private void StartFlying(in AirframeState state)
        {
            ControlsFilter cf = link.Aircraft.GetControlsFilter();
            stockFbw = cf != null ? cf.GetFlyByWire() : null;
            flight ??= new FbwFlight(link, stockFbw);
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
