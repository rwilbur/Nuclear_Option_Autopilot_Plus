using System;
using System.Globalization;
using System.IO;
using System.Text;
using BepInEx;
using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// Flight recorder for tuning: while DATA is on and the A/P flies a jet, one CSV row per physics tick
    /// (about 7 KB/s) to BepInEx\AutopilotMod-flightlog.csv, started fresh each game session. Heights,
    /// clearances, the vertical mode's and the floor's climb commands, and each axis' command beside
    /// its response and the stick; the lateral mode, ground track, target heading, the turn rate HDG/NAV asks
    /// for (deg/s), NAV's cross-track (m), active waypoint and arc turn.
    /// </summary>
    internal sealed partial class FlightController
    {
        private float logModeVs, logFloorVs; // FlyVertical: the mode's climb command (after speed protection) and the floor's
        private float logModelP, logModelR;  // the FBW model's rate per stick, rad/s (the learned ratio is relative to it)
        private static StreamWriter log;
        private static bool logStarted, logFailed;
        private float logT;
        private int logRows;
        private readonly StringBuilder logRow = new StringBuilder(256);

        private void LogTick(float dt)
        {
            if (!cfg.ShowDebug.Value || logFailed)
            {
                if (log != null) log.Flush();
                return;
            }
            try
            {
                if (log == null)
                {
                    string path = Path.Combine(Paths.BepInExRootPath, "AutopilotMod-flightlog.csv");
                    log = new StreamWriter(path, logStarted, Encoding.ASCII, 1 << 16);
                    if (!logStarted)
                        log.WriteLine("t,mode,law,spd,ralt,raltF,tgt,clear,lowDrop,vsMode,vsFloor,vsCmd,gammaCmd,gamma,qCmd,q,stickP,bankCmd,bankLim,bank,pCmd,p,stickR,flags,rlsP,rlsR,modelP,modelR,detuneP,detuneR,lat,track,tgtHdg,turnCmd,xtk,wpt,arc");
                    logStarted = true;
                }
                logT += dt;
                FlightState s = State;
                CultureInfo c = CultureInfo.InvariantCulture;
                string flags = (TerrainAhead ? "A" : "") + (SpeedProtectActive ? "S" : "") + (BankLimitedBySurface ? "B" : "") + (TfClimbLimited ? "L" : "") + (RouteVertical ? "R" : "");
                logRow.Clear();
                logRow.Append(logT.ToString("F2", c)).Append(',').Append(Vertical).Append(',').Append(cfg.RateLoopLaw.Value)
                    .Append(',').Append(s.Speed.ToString("F1", c)).Append(',').Append(s.RadarAlt.ToString("F2", c)).Append(',').Append(raltFiltered.ToString("F2", c))
                    .Append(',').Append(TgtRadarAlt.ToString("F1", c)).Append(',').Append(s.Clearance.ToString("F2", c)).Append(',').Append(s.LowestDrop.ToString("F2", c))
                    .Append(',').Append(logModeVs.ToString("F2", c)).Append(',').Append(logFloorVs.ToString("F2", c)).Append(',').Append(CmdVs.ToString("F2", c))
                    .Append(',').Append(CmdGammaDeg.ToString("F2", c)).Append(',').Append(s.Gamma.ToString("F2", c))
                    .Append(',').Append((CmdQ * Mathf.Rad2Deg).ToString("F2", c)).Append(',').Append((s.PitchRate * Mathf.Rad2Deg).ToString("F2", c)).Append(',').Append(OutPitch.ToString("F3", c))
                    .Append(',').Append(CmdBankDeg.ToString("F1", c)).Append(',').Append(bankLimit.ToString("F1", c)).Append(',').Append(s.Bank.ToString("F1", c))
                    .Append(',').Append((CmdP * Mathf.Rad2Deg).ToString("F1", c)).Append(',').Append((s.RollRate * Mathf.Rad2Deg).ToString("F1", c)).Append(',').Append(OutRoll.ToString("F3", c))
                    .Append(',').Append(flags).Append(',').Append(PitchAxis.Rls.Ratio.ToString("F2", c)).Append(',').Append(RollAxis.Rls.Ratio.ToString("F2", c))
                    .Append(',').Append((logModelP * Mathf.Rad2Deg).ToString("F1", c)).Append(',').Append((logModelR * Mathf.Rad2Deg).ToString("F1", c))
                    .Append(',').Append(PitchAxis.Rls.Detune.ToString("F2", c)).Append(',').Append(RollAxis.Rls.Detune.ToString("F2", c))
                    .Append(',').Append(Lateral).Append(',').Append((Mathf.Atan2(s.VelX, s.VelZ) * Mathf.Rad2Deg).ToString("F1", c)).Append(',').Append(TgtHeading.ToString("F1", c))
                    .Append(',').Append((HdgTurnRate(s, float.PositiveInfinity) * Mathf.Rad2Deg).ToString("F2", c)).Append(',').Append(NavCrossTrack.ToString("F0", c))
                    .Append(',').Append(Nav.Active).Append(',').Append(navArc.ToString("F0", c));
                log.WriteLine(logRow.ToString());
                if (++logRows % 250 == 0) log.Flush(); // every ~5 s, so a crash to desktop keeps nearly all of it
            }
            catch (Exception)
            {
                log = null; // disk full or file locked: stop recording for this session, keep flying
                logFailed = true;
            }
        }
    }
}
