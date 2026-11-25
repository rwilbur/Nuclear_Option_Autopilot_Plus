using System;
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
    }

    /// <summary>
    /// The fly-by-wire's lifecycle on the local player's aircraft: which aircraft it holds, the per-tick
    /// entry points the Harmony hooks call, telemetry, and the hand-back to stock on any error.
    ///
    /// Tick order (docs/fbw/PHASE0_FACTS.md section 1): <see cref="PreJobs"/> runs before the game's surface
    /// job takes its copy of the inputs; the pilot's stick and the stock filter run after that; then
    /// <see cref="AfterFilter"/>; then the engines' FixedUpdate reads its inputs.
    /// </summary>
    internal sealed class FbwController
    {
        /// <summary>Set by the plugin; the Harmony hooks call through it.</summary>
        public static FbwController Current;

        private readonly ConfigEntry<FbwMode> mode;
        private readonly ConfigEntry<bool> telemetry;
        private readonly ManualLogSource log;

        private AircraftLink link;
        private FbwTelemetry tlm;
        private AeroCheck check; // phase-2 diagnostics, with telemetry
        private Aircraft failed; // the aircraft the FBW faulted on: stays stock until it is replaced

        public FbwController(ConfigEntry<FbwMode> mode, ConfigEntry<bool> telemetry, ManualLogSource log)
        {
            this.mode = mode;
            this.telemetry = telemetry;
            this.log = log;
        }

        /// <summary>The aircraft held now, or null.</summary>
        public Aircraft Holding => link?.Aircraft;

        /// <summary>Start of the physics tick, before JobManager.ScheduleJobs.</summary>
        public void PreJobs()
        {
            try
            {
                Aircraft target = Target();
                if (link != null && link.Aircraft != target) Unbind();
                if (target == null) return;
                if (link == null) Bind(target);

                bool hold = mode.Value == FbwMode.Passthrough;
                if (hold)
                {
                    link.HoldSurfaces();
                    // passthrough: the surfaces get exactly what their job would read from the shared inputs now
                    ControlInputs from = link.Shared;
                    foreach (SurfaceLink s in link.Surfaces) Copy(from, s.Inputs);
                }
                else link.Release();

                AirframeState state = link.Sample(Time.fixedDeltaTime);
                if (telemetry.Value)
                {
                    tlm ??= new FbwTelemetry(link);
                    check ??= new AeroCheck(link);
                    tlm.Row(link, state, hold, check);
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
            if (link == null || aircraft != link.Aircraft) return;
            try
            {
                if (mode.Value != FbwMode.Passthrough) return;
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

        private void Unbind()
        {
            link?.Release();
            link = null;
            CloseTelemetry();
        }

        private void CloseTelemetry()
        {
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
            try { Unbind(); }
            catch (Exception e2) { log.LogError("FBW release failed: " + e2); }
            link = null;
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
