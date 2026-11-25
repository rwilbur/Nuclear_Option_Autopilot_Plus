using System;
using System.Globalization;
using System.IO;
using System.Text;
using BepInEx;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// One CSV row per physics tick for the held aircraft: flight state, the shared inputs, and per surface
    /// its target and actual deflection, per engine its nozzle angles and thrust. Written to
    /// BepInEx/FbwLogs/&lt;aircraft&gt;_&lt;time&gt;.csv. A development tool: off by default.
    /// </summary>
    internal sealed class FbwTelemetry : IDisposable
    {
        private const int FlushEvery = 60; // rows: about once a second

        private readonly StreamWriter w;
        private readonly StringBuilder sb = new StringBuilder(2048);
        private int rows;

        public string Path { get; }

        public FbwTelemetry(AircraftLink link)
        {
            string dir = System.IO.Path.Combine(Paths.BepInExRootPath, "FbwLogs");
            Directory.CreateDirectory(dir);
            string name = link.Aircraft.definition.unitName;
            foreach (char c in System.IO.Path.GetInvalidFileNameChars()) name = name.Replace(c, '_');
            Path = System.IO.Path.Combine(dir, name + "_" + DateTime.Now.ToString("yyyyMMdd_HHmmss") + ".csv");
            w = new StreamWriter(Path, false, new UTF8Encoding(false), 1 << 16);

            sb.Append("t,dt,held,speed,rho,mass,alpha,beta,p,q,r,pdot,qdot,rdot,nz,pitch,roll,yaw,throttle");
            sb.Append(",aeroErrF,aeroErrM,aeroWorst,aeroSkip,pdotPred,qdotPred,rdotPred");
            foreach (SurfaceLink s in link.Surfaces) sb.Append(',').Append(s.Name).Append("_tgt,").Append(s.Name).Append("_ang,").Append(s.Name).Append("_split");
            for (int i = 0; i < link.Engines.Count; i++) sb.Append(",e").Append(i).Append("_nzx,e").Append(i).Append("_nzy,e").Append(i).Append("_thrust,e").Append(i).Append("_thr");
            w.WriteLine(sb.ToString());
        }

        /// <summary>Angles in degrees, rates in deg/s and deg/s^2.</summary>
        public void Row(AircraftLink link, in AirframeState s, bool held, AeroCheck check)
        {
            const float D = AirData.Rad2Deg;
            sb.Clear();
            N(s.Time); N(s.Dt); sb.Append(held ? ",1" : ",0");
            N(s.Speed); N(s.Rho); N(s.Mass);
            N(s.Alpha * D); N(s.Beta * D);
            N(s.P * D); N(s.Q * D); N(s.R * D);
            N(s.PDot * D); N(s.QDot * D); N(s.RDot * D);
            N(s.Nz);
            ControlInputs i = link.Shared;
            N(i.pitch); N(i.roll); N(i.yaw); N(i.throttle);
            // the aero check of the previous tick, and its predicted acceleration over the step this row measured
            N(check.ForceError); N(check.MomentError); N(check.WorstPart); N(check.Skipped);
            if (check.Predict(out float pp, out float qp, out float rp)) { N(pp * D); N(qp * D); N(rp * D); }
            else sb.Append(",,,");
            foreach (SurfaceLink sf in link.Surfaces)
            {
                if (sf.JobReady) { N(sf.Target); N(sf.Angle); N(sf.Split); }
                else sb.Append(",,,");
            }
            foreach (EngineLink e in link.Engines)
            {
                if (e.Alive) { UnityEngine.Vector3 nz = e.Nozzle; N(nz.x); N(nz.y); N(e.Thrust); N(e.Active.throttle); }
                else sb.Append(",,,,");
            }
            w.WriteLine(sb.ToString(1, sb.Length - 1)); // every value is written with a leading comma: drop the first
            if (++rows % FlushEvery == 0) w.Flush();
        }

        public void Dispose() => w.Dispose();

        // shortcut: one string per value (~3k small allocations/s at 60 Hz); fine for an off-by-default log, write digits by hand if it shows in Perf
        private void N(float v) => sb.Append(',').Append(v.ToString("G6", CultureInfo.InvariantCulture));
    }
}
