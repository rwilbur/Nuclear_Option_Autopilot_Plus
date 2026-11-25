using System;
using AutopilotMod.Fbw;
using AutopilotMod.Fbw.Control;
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
    /// steady roll rate the yaw can coordinate. Header buttons set the limit override (AoA free or to the lift peak, its g) and the roll's sideslip allowance.
    /// </summary>
    internal sealed class EnvelopePage : MfdFrame
    {
        private const float G = 9.81f, Deg = 57.29578f;
        private const int Trail = 30;          // points, every TrailStep s
        private const float TrailStep = 0.1f;

        private bool liftView;
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

        public bool Visible => Flight != null;

        // header touch buttons: 0-1 the views, 2-4 the override's AoA mode and g, and the roll's sideslip allowance
        private Rect TabRect(int i) => i < 2 ? Sc(0.02f + 0.14f * i, 0.015f, 0.13f, 0.075f) : Sc(0.31f + 0.23f * (i - 2), 0.015f, 0.22f, 0.075f);

        protected override bool ScreenMouse(Event e)
        {
            if (e.type != EventType.MouseDown || e.button != 0) return false;
            for (int i = 0; i < 5; i++)
            {
                if (!TabRect(i).Contains(e.mousePosition)) continue;
                if (i < 2) liftView = i == 1;
                else if (i == 2) cfg.OverrideAoa.Value = cfg.OverrideAoa.Value == OverrideAoa.NoLimit ? OverrideAoa.LiftPeak : OverrideAoa.NoLimit;
                else if (i == 3) cfg.OverrideG.Value = Next(GSteps, cfg.OverrideG.Value);
                else cfg.RollSideslip.Value = Next(SlipSteps, cfg.RollSideslip.Value);
                e.Use();
                return true;
            }
            return false;
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
            FbwFlight f = Flight;
            if (f == null || !f.Curve.Known) // the web mirror renders it out of the jet too
            {
                Text(Sc(0.05f, 0.4f, 0.9f, 0.2f), f == null ? "FBW NOT FLYING" : "ENVELOPE...", 0.06f, GreenDim, TextAnchor.MiddleCenter, false);
                return;
            }
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

            // ---- header: view tabs, then the settings (their buttons cycle them) ----
            string[] tabs =
            {
                "V-N", "LIFT", cfg.OverrideAoa.Value == OverrideAoa.LiftPeak ? "OVR PEAK" : "OVR FREE",
                $"OVR G{cfg.OverrideG.Value:0.##}", $"SLIP {cfg.RollSideslip.Value:0}",
            };
            for (int i = 0; i < 5; i++)
            {
                Rect r = TabRect(i);
                bool on = i < 2 && liftView == (i == 1);
                if (on) Fill(r, Green, 0.01f * S);
                else Border(r, i < 2 ? GreenDim : Green, Mathf.Max(1f, 0.004f * S), 0.01f * S);
                Text(r, tabs[i], 0.03f, on ? Black : i < 2 ? GreenDim : Green, TextAnchor.MiddleCenter, false);
            }
            bool governing = Math.Abs(gov.Governed - gov.Wanted) > 0.01f || Math.Abs(gov.GovernedRoll - gov.WantedRoll) > 0.01f;
            string status = f.LimitOverride ? "AOA OVRD" : !limited ? "ASSIST OFF" : governing ? "LIMITING" : "";
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
