using System;
using AutopilotMod.Fbw;
using AutopilotMod.Fbw.Control;
using BepInEx.Configuration;
using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// Slim, buttonless display of the fly-by-wire's live flight envelope (Mode = Full), drawn from the same numbers the
    /// envelope governor flies by: the airframe's lift curve at the current flow, its AoA limits and the g limit.
    /// Two views, picked with the tabs at the top (tap with the MFD cursor, or in the web mirror):
    /// - V-N: load factor against airspeed. The boundary is the lift at the AoA limit (scaled with airspeed squared) up to the
    ///   g limit; faint, the peak-lift boundary the limit override reaches. The dot is the jet now, with a 3 s trail.
    /// - LIFT: load factor against AoA at the current airspeed, the AoA limits and the governor's 0.5 s AoA forecast.
    /// Below: AoA and g against their limits, sideslip against its allowance, corner speed, the turn rate the envelope gives now and the
    /// steady roll rate the yaw can coordinate.
    /// - SET: every [FlyByWire] setting (mode, the limit override's key, AoA and g, the roll's sideslip allowance, telemetry, airframe
    ///   export). Tap a row to step it; amber = changed from its default. Saved to the .cfg at once.
    /// Shown whenever you fly an aircraft, so the mode can be switched on from here. The plots show in Full and Multiplayer mode, on the
    /// ground and at any speed too (STANDBY: the stock fly-by-wire flies below 60 m/s and with weight on the wheels).
    /// </summary>
    internal sealed class EnvelopePage : MfdFrame
    {
        private const float G = 9.81f, Deg = 57.29578f;
        private const int Trail = 30;          // points, every TrailStep s
        private const float TrailStep = 0.1f;

        private int view;           // 0 V-N, 1 LIFT, 2 SET
        private int touched = -1;   // the settings row tapped last: its hint shows
        private readonly Vector2[] trail = new Vector2[Trail]; // (airspeed m/s, nz)
        private readonly Vector2[] trailAoa = new Vector2[Trail]; // (AoA deg, nz)
        private int trailCount, trailAt;
        private float nextTrail;

        private readonly ApSettings cfg;
        private static readonly float[] GSteps = { 1f, 1.25f, 1.5f, 2f }, SlipSteps = { 0f, 3f, 6f };

        public EnvelopePage(ApSettings cfg) : base(cfg.EnvelopeSize, cfg.EnvelopeX, cfg.EnvelopeY)
        {
            this.cfg = cfg;
        }

        protected override float Bezel => 0.025f;

        protected override Osb[] Buttons => Array.Empty<Osb>();

        private static FbwFlight Flight => FbwController.Current?.Flight;

        /// <summary>In an aircraft: the settings must be reachable to switch the fly-by-wire on.</summary>
        public bool Visible => GameManager.GetLocalAircraft(out Aircraft ac) && ac != null;

        // header tabs: V-N, LIFT, SET
        private static readonly string[] TabNames = { "V-N", "LIFT", "SET" };
        private Rect TabRect(int i) => Sc(0.02f + 0.14f * i, 0.015f, 0.13f, 0.075f);

        // ---- the settings ----
        private const int Rows = 8;
        private Rect RowRect(int i) => Sc(0.03f, 0.11f + 0.08f * i, 0.94f, 0.07f);

        // the limit override's key: these, in turn (a key set in the .cfg shows by name until stepped past)
        private static readonly KeyCode[] Keys = { KeyCode.Mouse1, KeyCode.Mouse2, KeyCode.Mouse3, KeyCode.Mouse4, KeyCode.None };

        private void Step(int row)
        {
            switch (row)
            {
                case 0: cfg.FbwMode.Value = (FbwMode)(((int)cfg.FbwMode.Value + 1) % 4); break; // Off, Passthrough, Full, Multiplayer
                case 1:
                    int k = Array.IndexOf(Keys, cfg.KeyLimitOverride.Value.MainKey);
                    KeyCode next = Keys[(k + 1) % Keys.Length]; // not in the list (-1): the first
                    cfg.KeyLimitOverride.Value = next == KeyCode.None ? KeyboardShortcut.Empty : new KeyboardShortcut(next);
                    break;
                case 2: cfg.OverrideAoa.Value = cfg.OverrideAoa.Value == OverrideAoa.NoLimit ? OverrideAoa.LiftPeak : OverrideAoa.NoLimit; break;
                case 3: cfg.OverrideG.Value = Next(GSteps, cfg.OverrideG.Value); break;
                case 4: cfg.RollSideslip.Value = Next(SlipSteps, cfg.RollSideslip.Value); break;
                case 5: cfg.FbwTelemetry.Value = !cfg.FbwTelemetry.Value; break;
                case 6: cfg.ExportAirframes.Value = !cfg.ExportAirframes.Value; break;
                default:
                    FbwController fbw = FbwController.Current;
                    if (fbw != null) fbw.TestEngineOut = fbw.TestEngineOut == 0 ? -1 : fbw.TestEngineOut < 0 ? 1 : 0; // off, left, right
                    break;
            }
        }

        private static string KeyName(KeyboardShortcut k) => k.MainKey switch
        {
            KeyCode.None => "OFF",
            KeyCode.Mouse1 => "RIGHT MOUSE",
            KeyCode.Mouse2 => "MIDDLE MOUSE",
            KeyCode.Mouse3 => "MOUSE 4",
            KeyCode.Mouse4 => "MOUSE 5",
            _ => k.ToString().ToUpperInvariant(),
        };

        // label, value, changed from its default, and the hint shown once the row is tapped
        private void Row(int row, out string label, out string value, out bool changed, out string hint)
        {
            switch (row)
            {
                case 0:
                    label = "MODE";
                    value = cfg.FbwMode.Value switch { FbwMode.Off => "OFF", FbwMode.Passthrough => "PASSTHROUGH", FbwMode.Full => "FULL", _ => "MULTIPLAYER" };
                    changed = cfg.FbwMode.Value != (FbwMode)cfg.FbwMode.DefaultValue;
                    hint = cfg.FbwMode.Value switch
                    {
                        FbwMode.Off => "THE GAME'S OWN FLY-BY-WIRE",
                        FbwMode.Passthrough => "TEST OF THE TAKEOVER\nFLIES EXACTLY AS STOCK",
                        FbwMode.Full => "EVERY SURFACE MOVED ON ITS OWN\nOTHER PLAYERS SEE AN APPROXIMATION",
                        _ => "STOCK MIXING OF THE SHARED INPUTS\nOTHER PLAYERS SEE EXACTLY WHAT YOU FLY",
                    };
                    break;
                case 1:
                    label = "OVERRIDE KEY";
                    value = KeyName(cfg.KeyLimitOverride.Value);
                    changed = !cfg.KeyLimitOverride.Value.Equals((KeyboardShortcut)cfg.KeyLimitOverride.DefaultValue);
                    hint = "HOLD TO LIFT THE AOA LIMIT\nG IS STILL HELD";
                    break;
                case 2:
                    label = "OVERRIDE AOA";
                    value = cfg.OverrideAoa.Value == OverrideAoa.LiftPeak ? "LIFT PEAK" : "FREE";
                    changed = cfg.OverrideAoa.Value != (OverrideAoa)cfg.OverrideAoa.DefaultValue;
                    hint = cfg.OverrideAoa.Value == OverrideAoa.LiftPeak ? "UP TO THE MOST LIFT, NO FURTHER" : "NO AOA LIMIT, AS ASSIST OFF\nA HELD PULL CAN FLIP THE JET";
                    break;
                case 3:
                    label = "OVERRIDE G";
                    value = $"x{cfg.OverrideG.Value:0.##}";
                    changed = Math.Abs(cfg.OverrideG.Value - (float)cfg.OverrideG.DefaultValue) > 0.001f;
                    hint = "THE G LIMIT WHILE THE KEY IS HELD\nAT MOST 19 G; BLACKOUT ABOVE 9";
                    break;
                case 4:
                    label = "ROLL SLIP";
                    value = $"{cfg.RollSideslip.Value:0} DEG";
                    changed = Math.Abs(cfg.RollSideslip.Value - (float)cfg.RollSideslip.DefaultValue) > 0.001f;
                    hint = "SIDESLIP A ROLL MAY BUILD BEFORE\nIT IS HELD TO WHAT THE YAW COORDINATES";
                    break;
                case 5:
                    label = "TELEMETRY";
                    value = cfg.FbwTelemetry.Value ? "ON" : "OFF";
                    changed = cfg.FbwTelemetry.Value != (bool)cfg.FbwTelemetry.DefaultValue;
                    hint = "LOGS EVERY TICK TO\nBEPINEX\\FBWLOGS";
                    break;
                case 7:
                    FbwController fbw = FbwController.Current;
                    int test = fbw?.TestEngineOut ?? 0;
                    bool found = fbw?.TestEngine() != null;
                    label = "TEST ENGINE OUT";
                    value = test == 0 ? "OFF" : !found ? "NO ENGINE" : test < 0 ? "LEFT IDLE" : "RIGHT IDLE";
                    changed = test != 0;
                    hint = test != 0 && !found ? "NO ENGINE OFF THE CENTRELINE\nON THAT SIDE: NOTHING TO TEST"
                        : "HOLDS ONE ENGINE AT IDLE (TEST)\nNOT SAVED: OFF AT EVERY LAUNCH";
                    break;
                default:
                    label = "AIRFRAME EXPORT";
                    value = cfg.ExportAirframes.Value ? "ON" : "OFF";
                    changed = cfg.ExportAirframes.Value != (bool)cfg.ExportAirframes.DefaultValue;
                    hint = "WRITES THIS JET ONCE TO\nBEPINEX\\FBWAIRFRAMES";
                    break;
            }
        }

        protected override bool ScreenMouse(Event e)
        {
            if (e.type != EventType.MouseDown || e.button != 0) return false;
            for (int i = 0; i < TabNames.Length; i++)
            {
                if (!TabRect(i).Contains(e.mousePosition)) continue;
                view = i;
                e.Use();
                return true;
            }
            if (view != 2) return false;
            for (int i = 0; i < Rows; i++)
            {
                if (!RowRect(i).Contains(e.mousePosition)) continue;
                Step(i);
                touched = i;
                e.Use();
                return true;
            }
            return false;
        }

        private void DrawSettings()
        {
            for (int i = 0; i < Rows; i++)
            {
                Row(i, out string label, out string value, out bool changed, out _);
                Rect r = RowRect(i);
                Border(r, i == touched ? Green : GreenFaint, Mathf.Max(1f, 0.004f * S), 0.01f * S);
                Text(new Rect(r.x + 0.02f * S, r.y, r.width * 0.5f, r.height), label, 0.032f, GreenDim, TextAnchor.MiddleLeft, false);
                Text(new Rect(r.x + r.width * 0.45f, r.y, r.width * 0.53f, r.height), value, 0.036f, changed ? Amber : Green, TextAnchor.MiddleRight);
            }
            string hint = "TAP A ROW TO CHANGE IT";
            if (touched >= 0) Row(touched, out _, out _, out _, out hint);
            Text(Sc(0.03f, 0.735f + 0.02f, 0.94f, 0.2f), hint, 0.03f, GreenDim, TextAnchor.UpperCenter, false);
            Text(Sc(0.03f, 0.93f, 0.94f, 0.05f), "AMBER = CHANGED FROM DEFAULT", 0.026f, GreenFaint, TextAnchor.MiddleCenter, false);
        }

        // the step after the one at or above v, round to the first
        private static float Next(float[] steps, float v)
        {
            foreach (float s in steps)
                if (s > v + 0.01f) return s;
            return steps[0];
        }

        protected override void DrawScreen()
        {
            for (int i = 0; i < TabNames.Length; i++)
            {
                Rect r = TabRect(i);
                bool on = view == i;
                if (on) Fill(r, Green, 0.01f * S);
                else Border(r, GreenDim, Mathf.Max(1f, 0.004f * S), 0.01f * S);
                Text(r, TabNames[i], 0.03f, on ? Black : GreenDim, TextAnchor.MiddleCenter, false);
            }
            if (view == 2)
            {
                DrawSettings();
                return;
            }
            FbwFlight f = Flight;
            if (f == null || !f.Curve.Known) // the web mirror renders it out of the jet too
            {
                string why = f != null ? "ENVELOPE..." : cfg.FbwMode.Value == FbwMode.Off || cfg.FbwMode.Value == FbwMode.Passthrough
                    ? "FBW OFF\nTURN IT ON IN SET" : "FBW STANDBY";
                Text(Sc(0.05f, 0.35f, 0.9f, 0.3f), why, 0.05f, GreenDim, TextAnchor.MiddleCenter, false);
                return;
            }
            bool liftView = view == 1;
            FlightComputer fc = f.Computer;
            LiftCurve c = f.Curve;
            Fbw.Control.FlightState s = fc.Last;
            CommandGovernor gov = fc.Governor;
            float nzMax = fc.Law.NzLimit, nzMin = -FlightComputer.NegativeG * nzMax;

            // lift per airspeed squared (g per (m/s)^2) at the AoA limits and at the lift peaks
            float k = c.SweepWeight > 0f && c.SweepSpeed > 1f ? 1f / (c.SweepWeight * c.SweepSpeed * c.SweepSpeed) : 0f;
            float kLimPos = c.LiftAt(fc.AlphaMax) * k, kLimNeg = c.LiftAt(fc.AlphaMin) * k;
            float kPeakPos = c.LiftAt(c.PeakPos) * k, kPeakNeg = c.LiftAt(c.PeakNeg) * k;
            bool limited = fc.Assist;
            float kPos = limited ? kLimPos : kPeakPos, kNeg = limited ? kLimNeg : kPeakNeg;
            float v = s.Speed, aoa = s.Alpha * Deg;

            if (Time.unscaledTime >= nextTrail)
            {
                nextTrail = Time.unscaledTime + TrailStep;
                trail[trailAt] = new Vector2(v, s.Nz);
                trailAoa[trailAt] = new Vector2(aoa, s.Nz);
                trailAt = (trailAt + 1) % Trail;
                trailCount = Math.Min(Trail, trailCount + 1);
            }

            // ---- header status (right of the tabs) ----
            bool standby = !(FbwController.Current?.Flying ?? false); // the stock fly-by-wire flies: slow or on the ground
            bool governing = !standby && (Math.Abs(gov.Governed - gov.Wanted) > 0.01f || Math.Abs(gov.GovernedRoll - gov.WantedRoll) > 0.01f);
            string status = standby ? "STANDBY" : f.LimitOverride ? "AOA OVRD" : !limited ? "ASSIST OFF" : governing ? "LIMITING" : "";
            Color statusCol = f.LimitOverride || !limited ? Amber : Green;
            bool blink = f.LimitOverride && Mathf.Repeat(Time.unscaledTime, 0.6f) > 0.4f;
            if (status.Length > 0 && !blink) Text(Sc(0.4f, 0.125f, 0.56f, 0.06f), status, 0.036f, statusCol, TextAnchor.MiddleRight);

            // ---- plot ----
            Rect plot = Sc(0.1f, 0.12f, 0.87f, 0.58f);
            float nTop = nzMax + 1f, nBot = nzMin - 1f;
            if (!liftView)
            {
                float corner = kLimPos > 0f ? Mathf.Sqrt(nzMax / kLimPos) : 0f;
                float vTop = Mathf.Clamp(Mathf.Max(v * 1.4f, corner * 1.3f), 100f, 700f);
                Grid(plot, 0f, vTop, nBot, nTop, StepFor(Units.Speed(vTop)) / Units.Speed(1f), x => Units.Speed(x).ToString("0"));
                if (limited) Boundary(plot, vTop, nBot, nTop, kPeakPos, kPeakNeg, nzMax, nzMin, GreenFaint);
                Boundary(plot, vTop, nBot, nTop, kPos, kNeg, nzMax, nzMin, Green);
                if (corner > 0f && corner < vTop)
                    Line(P(plot, corner, nzMax, 0f, vTop, nBot, nTop), P(plot, corner, nBot, 0f, vTop, nBot, nTop), 1f, GreenFaint);
                Dot(plot, trail, v, s.Nz, 0f, vTop, nBot, nTop, governing || !limited);
                Text(Sc(0.1f, 0.7f, 0.87f, 0.05f), Units.SpeedUnit, 0.03f, GreenDim, TextAnchor.UpperRight, false);
            }
            else
            {
                const float aLo = -20f, aHi = 45f;
                float scale = k * v * v; // g per N of sweep lift at this airspeed
                Grid(plot, aLo, aHi, nBot, nTop, 10f, x => x.ToString("0"));
                Vector2 prev = default;
                for (int i = 0; i <= 65; i++)
                {
                    float a = aLo + i;
                    Vector2 p = P(plot, a, Mathf.Clamp(c.LiftAt(a / Deg) * scale, nBot, nTop), aLo, aHi, nBot, nTop);
                    if (i > 0) Line(prev, p, Mathf.Max(1.5f, 0.005f * S), Green);
                    prev = p;
                }
                Color lim = limited ? Amber : GreenFaint;
                foreach (float a in new[] { fc.AlphaMax * Deg, fc.AlphaMin * Deg })
                    Line(P(plot, a, nTop, aLo, aHi, nBot, nTop), P(plot, a, nBot, aLo, aHi, nBot, nTop), Mathf.Max(1f, 0.004f * S), lim);
                foreach (float n in new[] { nzMax, nzMin })
                    Line(P(plot, aLo, n, aLo, aHi, nBot, nTop), P(plot, aHi, n, aLo, aHi, nBot, nTop), Mathf.Max(1f, 0.004f * S), Amber);
                // the governor's 0.5 s AoA forecast, on the bottom axis
                Vector2 fp = P(plot, Mathf.Clamp(gov.PredAlpha * Deg, aLo, aHi), nBot, aLo, aHi, nBot, nTop);
                Triangle(new Rect(fp.x - 0.015f * S, fp.y - 0.03f * S, 0.03f * S, 0.03f * S), true, Green);
                Dot(plot, trailAoa, aoa, s.Nz, aLo, aHi, nBot, nTop, governing || !limited);
                Text(Sc(0.1f, 0.7f, 0.87f, 0.05f), "AOA DEG", 0.03f, GreenDim, TextAnchor.UpperRight, false);
            }

            // ---- readouts ----
            float nAvail = Mathf.Min(nzMax, kPos * v * v);
            float lateral = nAvail > 1f ? G * Mathf.Sqrt(nAvail * nAvail - 1f) : 0f;
            float cornerV = kLimPos > 0f ? Mathf.Sqrt(nzMax / kLimPos) : 0f;
            Readout(0, 0, "AOA", $"{aoa:0.0}", limited ? $"/{fc.AlphaMax * Deg:0}" : "/--", aoa > fc.AlphaMax * Deg - 1f || !limited);
            Readout(0, 1, "G", $"{s.Nz:0.0}", $"/{nzMax:0.0}", s.Nz > nzMax - 0.5f || s.Nz < nzMin + 0.5f);
            Readout(0, 2, "BETA", $"{s.Beta * Deg:0.0}", $"/{cfg.RollSideslip.Value:0}", Math.Abs(s.Beta * Deg) > cfg.RollSideslip.Value + 1f);
            Readout(1, 0, "CRNR", cornerV > 0f ? $"{Units.Speed(cornerV):0}" : "--", "", false);
            Readout(1, 1, "TURN", lateral > 0f ? $"{lateral / Mathf.Max(v, 1f) * Deg:0.0}" : "--", "°/S", false);
            // the steady roll rate the yaw can coordinate now (the slower side)
            float roll = Mathf.Min(fc.RollHi, -fc.RollLo) * Deg;
            Readout(1, 2, "ROLL", roll < 900f ? $"{Mathf.Max(0f, roll):0}" : "--", "°/S", roll < 30f);
        }

        private void Readout(int col, int row, string label, string value, string unit, bool warn)
        {
            float x = 0.03f + 0.49f * col, y = 0.76f + 0.075f * row;
            Text(Sc(x, y, 0.13f, 0.07f), label, 0.033f, GreenDim, TextAnchor.MiddleLeft, false);
            Text(Sc(x + 0.12f, y, 0.2f, 0.07f), value, 0.042f, warn ? Amber : Green, TextAnchor.MiddleRight);
            Text(Sc(x + 0.33f, y, 0.14f, 0.07f), unit, 0.03f, GreenDim, TextAnchor.MiddleLeft, false);
        }

        // a round step giving 3-6 grid lines over a span
        private static float StepFor(float span)
        {
            float step = Mathf.Pow(10f, Mathf.Floor(Mathf.Log10(Mathf.Max(span, 1f) / 3f)));
            if (span / step > 12f) step *= 5f;
            else if (span / step > 6f) step *= 2f;
            return step;
        }

        private static Vector2 P(Rect r, float x, float y, float x0, float x1, float y0, float y1) =>
            new Vector2(r.x + (x - x0) / (x1 - x0) * r.width, r.yMax - (y - y0) / (y1 - y0) * r.height);

        private void Grid(Rect r, float x0, float x1, float y0, float y1, float xStep, Func<float, string> xLabel)
        {
            Border(r, GreenDim, 1f, 0f);
            for (float x = Mathf.Ceil(x0 / xStep) * xStep; x <= x1; x += xStep)
            {
                Vector2 a = P(r, x, y0, x0, x1, y0, y1);
                Line(a, P(r, x, y1, x0, x1, y0, y1), 1f, GreenFaint);
                Text(new Rect(a.x - 0.06f * S, a.y, 0.12f * S, 0.04f * S), xLabel(x), 0.028f, GreenDim, TextAnchor.UpperCenter, false);
            }
            for (float y = Mathf.Ceil(y0 / 2f) * 2f; y <= y1; y += 2f)
            {
                Vector2 a = P(r, x0, y, x0, x1, y0, y1);
                Line(a, P(r, x1, y, x0, x1, y0, y1), y == 0f ? Mathf.Max(1f, 0.004f * S) : 1f, y == 0f ? GreenDim : GreenFaint);
                Text(new Rect(a.x - 0.075f * S, a.y - 0.02f * S, 0.065f * S, 0.04f * S), y.ToString("0"), 0.028f, GreenDim, TextAnchor.MiddleRight, false);
            }
        }

        // the V-n boundary: lift-limited (k v^2) up to the g limits, then the limits out to the plot's edge
        private void Boundary(Rect r, float vTop, float y0, float y1, float kPos, float kNeg, float nzMax, float nzMin, Color col)
        {
            float w = Mathf.Max(1.5f, 0.005f * S);
            const int N = 40;
            Vector2 prevP = P(r, 0f, 0f, 0f, vTop, y0, y1), prevN = prevP;
            for (int i = 1; i <= N; i++)
            {
                float vv = vTop * i / N;
                Vector2 p = P(r, vv, Mathf.Min(nzMax, kPos * vv * vv), 0f, vTop, y0, y1);
                Vector2 n = P(r, vv, Mathf.Max(nzMin, kNeg * vv * vv), 0f, vTop, y0, y1);
                Line(prevP, p, w, col);
                Line(prevN, n, w, col);
                prevP = p;
                prevN = n;
            }
        }

        private void Dot(Rect r, Vector2[] pts, float x, float y, float x0, float x1, float y0, float y1, bool amber)
        {
            for (int i = 0; i < trailCount; i++)
            {
                Vector2 t = pts[(trailAt + Trail - 1 - i) % Trail];
                Vector2 p = Clamp(r, P(r, t.x, t.y, x0, x1, y0, y1));
                float d = 0.008f * S;
                Fill(new Rect(p.x - d * 0.5f, p.y - d * 0.5f, d, d), new Color(Green.r, Green.g, Green.b, 0.5f * (1f - i / (float)Trail)), d * 0.5f);
            }
            Vector2 now = Clamp(r, P(r, x, y, x0, x1, y0, y1));
            float sz = 0.025f * S;
            Fill(new Rect(now.x - sz * 0.5f, now.y - sz * 0.5f, sz, sz), amber ? Amber : Color.white, sz * 0.5f);
        }

        private static Vector2 Clamp(Rect r, Vector2 p) => new Vector2(Mathf.Clamp(p.x, r.x, r.xMax), Mathf.Clamp(p.y, r.y, r.yMax));
    }
}
