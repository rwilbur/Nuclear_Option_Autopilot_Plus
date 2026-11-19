using System;
using AutopilotMod;

/// <summary>
/// Assert-based checks of src/ApMath.cs, plus small offline models of the helicopter height loop
/// and of terrain following.
/// Exit code 0 = all passed. Run: dotnet run --project tests
/// </summary>
internal static class SelfCheck
{
    private static int failures;

    private static void Check(bool ok, string what)
    {
        if (ok) return;
        failures++;
        Console.WriteLine("FAIL: " + what);
    }

    private static bool Near(float a, float b, float tol = 1e-3f) => Math.Abs(a - b) <= tol;

    private static int Main()
    {
        // ---- rocker steps ----
        Check(Near(ApMath.Snap(1234f, +1, 50f), 1250f), "Snap up goes to the next multiple");
        Check(Near(ApMath.Snap(1250f, +1, 50f), 1300f), "Snap up from a multiple moves a full step");
        Check(Near(ApMath.Snap(1250f, -1, 50f), 1200f), "Snap down from a multiple moves a full step");
        Check(Near(ApMath.Snap(3f, -1, 50f), 0f), "Snap down reaches 0 (the caller clamps to the 1 m minimum)");

        // ---- altitude capture profile ----
        Check(ApMath.CaptureProfile(0f, 0.3f, 2.5f, 40f) == 0f, "no error, no climb");
        Check(Near(ApMath.CaptureProfile(10f, 0.3f, 2.5f, 40f), 3f), "linear near the target");
        Check(Near(ApMath.CaptureProfile(-10f, 0.3f, 2.5f, 40f), -3f), "symmetric below the target");
        Check(ApMath.CaptureProfile(5000f, 0.3f, 2.5f, 40f) == 40f, "capped at the max climb rate");
        float e0 = 2f * 2.5f / (0.3f * 0.3f);
        Check(Near(ApMath.CaptureProfile(e0 - 0.01f, 0.3f, 2.5f, 40f), ApMath.CaptureProfile(e0 + 0.01f, 0.3f, 2.5f, 40f), 0.02f),
            "linear and square-root segments meet");

        // ---- rotorcraft floor / height priority / heavy-airframe gains ----
        Check(Near(ApMath.RotorFloorVs(11.5f, 1.5f, 1f, 0f), -3f), "helicopter may sink 0.3 m/s per metre above the floor");
        Check(Near(ApMath.RotorFloorVs(11.5f, 1.5f, 1f, 1f), -10f), "airplane mode keeps the jets' floor gain");
        Check(Near(ApMath.RotorFloorVs(0.5f, 1.5f, 1f, 0f), 1f), "below the floor it climbs at the full gain");
        Check(ApMath.HeightPriority(3f, 2f) == 0f, "sinking near the ground freezes the speed loop's pitch");
        Check(ApMath.HeightPriority(3f, 30f) == 1f, "no effect well clear of the ground");
        Check(ApMath.HeightPriority(-1f, 2f) == 1f, "no effect when not sinking");
        Check(ApMath.Agility(1f, 0.7f, 0.4f) == 1f && Near(ApMath.Agility(0.3f, 0.7f, 0.4f), 0.4286f) && ApMath.Agility(0.1f, 0.7f, 0.4f) == 0.4f,
            "agility is 1 for fighters, proportional below, floored");

        // ---- route hand edits (web map / planner): never into flown legs, neighbour's height and speed ----
        {
            var r = new NavRoute();
            r.Replace(new System.Collections.Generic.List<Waypoint> { new Waypoint(0f, 1000f, 100f, 200f), new Waypoint(0f, 2000f, 300f), new Waypoint(0f, 3000f) }, 0f, 0f, true);
            r.Active = 1;
            r.Insert(0, 5f, 5f);
            Check(r.Points.Count == 4 && r.Points[1].X == 5f && r.Points[1].Agl == 300f && !r.Planned, "insert before the active leg lands on it, takes the next point's height, unplans");
            r.Insert(4, 9f, 9f);
            Check(r.Points[4].X == 9f && r.DestX == 9f && float.IsNaN(r.Points[4].Agl), "append takes the last point's height and becomes the destination");
            r.RemoveAt(0);
            Check(r.Points.Count == 5 && r.Points[0].X == 0f, "a flown point can't be removed");
            r.SetAgl(2, 1f);
            r.SetSpeed(2, 0f);
            Check(r.Points[2].Agl == 2f && float.IsNaN(r.Points[2].Speed), "height floored at 2 m, speed 0 = none");
        }

        // ---- ETA path distance ----
        float[] path = { 0f, 4000f, 3000f, 4000f }; // north 4 km, then east 3 km
        Check(Near(ApMath.PathDistance(path, 0, 300f, 400f, true), 500f), "no route: straight to the target");
        Check(Near(ApMath.PathDistance(path, 2, 0f, 0f, false), 7000f), "no target: to the route's end");
        Check(Near(ApMath.PathDistance(path, 2, 1500f, 5000f, true), 6500f), "along the route to abeam the target, then 1 km off");
        Check(Near(ApMath.PathDistance(path, 2, 3000f, 4000f, true), 7000f), "target on the last waypoint");

        // ---- lift vector: bank and pull from the climb and the turn wanted together ----
        float lv = 9.81f, ah = 9.81f * (float)Math.Tan(Math.PI / 3);
        float bk = ApMath.LiftBank(ref lv, ref ah, 80f, 9.81f, out float room);
        Check(Near(bk * 180f / (float)Math.PI, 60f, 0.1f) && Near(ApMath.LiftAlong(lv, ah, bk), 19.62f, 0.01f), "level 60 deg turn: 2 g at 60 deg bank");
        lv = -9.81f; ah = 3f * 9.81f; // push-over wanting -1 g while turning at 3 g
        bk = ApMath.LiftBank(ref lv, ref ah, 80f, 9.81f, out room);
        float turnAccel = ApMath.LiftAlong(lv, ah, bk) * (float)Math.Sin(bk);
        Check(Near(bk * 180f / (float)Math.PI, 71.6f, 0.1f) && turnAccel > 2.3f * 9.81f, $"push-over in a turn keeps the level-turn bank and keeps turning: {bk * 180f / Math.PI:F0} deg, {turnAccel / 9.81f:F1} g sideways");
        lv = 0f; ah = 0.2f * 9.81f; // a small steering correction while unloaded over a crest
        bk = ApMath.LiftBank(ref lv, ref ah, 80f, 9.81f, out room);
        Check(bk * 180f / Math.PI < 12f, $"a small correction while pushing stays a small bank: {bk * 180f / Math.PI:F1} deg");
        lv = -9.81f; ah = 0f;
        bk = ApMath.LiftBank(ref lv, ref ah, 80f, 9.81f, out room);
        Check(bk == 0f && Near(ApMath.LiftAlong(lv, ah, bk), -9.81f), "push-over without a turn: wings level, negative g");
        lv = 78f; ah = 50f;
        ApMath.LiftBank(ref lv, ref ah, 80f, 9.81f, out room);
        Check(Near(lv, 78f) && Near(ah, (float)Math.Sqrt(80f * 80f - 78f * 78f), 0.01f) && Near(room, ah, 0.01f), "over budget: the climb keeps its lift, the turn gets the rest");

        // ---- lift hold: TF's pull/push swings (+-2 g at 2 Hz) with a 1 g turn wanted must not swing the bank ----
        {
            const float g = 9.81f, dt = 0.02f;
            float hold0 = float.NaN, rawLo = 99f, rawHi = -99f, heldLo = 99f, heldHi = -99f;
            for (int k = 0; k < 250; k++)
            {
                float lvNow = g + 2f * g * (float)Math.Sin(2.0 * Math.PI * 2.0 * k * dt), a1 = g, a2 = g, l1 = lvNow;
                hold0 = ApMath.LiftHold(hold0, lvNow, dt, 2f);
                float l2 = hold0;
                float raw = ApMath.LiftBank(ref l1, ref a1, 80f, g, out _) * 57.3f, hold = ApMath.LiftBank(ref l2, ref a2, 80f, g, out _) * 57.3f;
                rawLo = Math.Min(rawLo, raw); rawHi = Math.Max(rawHi, raw);
                if (k >= 50) { heldLo = Math.Min(heldLo, hold); heldHi = Math.Max(heldHi, hold); }
            }
            Check(rawHi - rawLo > 20f, $"(model) pointing the bank at each TF swing moves it {rawHi - rawLo:F0} deg");
            Check(heldHi - heldLo < 4f, $"lift hold: bank steady through TF's swings ({heldHi - heldLo:F1} deg)");
            Check(ApMath.LiftHold(g, 4f * g, dt, 2f) == 4f * g, "lift hold: a new pull-up is followed at once (rolls toward level)");
            float rel = 4f * g;
            for (int k = 0; k < 200; k++) rel = ApMath.LiftHold(rel, g, dt, 2f);
            Check(rel < 2f * g, $"lift hold: lets go of a past pull within 4 s ({rel / g:F2} g)");
        }

        // ---- the pull at the bank the jet actually has ----
        float rad = (float)Math.PI / 180f, cos35 = (float)Math.Cos(35f * rad);
        float held = ApMath.LiftPull(9.81f, 6f * 9.81f, 35f * rad, 9.81f); // hard turn wanted, bank held at 35 deg by the wingtip limit
        Check(Near(held * cos35, 9.81f, 0.01f), $"bank held shallow: pulls for level flight ({held / 9.81f:F2} g), not for the turn (it would climb)");
        float rollOut = ApMath.LiftPull(9.81f, 0f, 70f * rad, 9.81f); // rolling out from 70 deg, no turn wanted
        Check(Near(rollOut * (float)Math.Cos(70f * rad), 9.81f, 0.01f), "rolling out: keeps its height");
        float pushTurn = ApMath.LiftPull(-9.81f, 3f * 9.81f, 71.57f * rad, 9.81f);
        Check(Near(pushTurn, ApMath.LiftAlong(-9.81f, 3f * 9.81f, 71.57f * rad), 0.01f) && pushTurn > 2.4f * 9.81f, "push-over in a turn at its bank: still pulls round");
        Check(Near(ApMath.LiftPull(-9.81f, 0f, 0f, 9.81f), -9.81f), "push-over wings level: negative g");
        Check(Near(ApMath.LiftPull(9.81f, 9.81f * (float)Math.Tan(60f * rad), 60f * rad, 9.81f), 19.62f, 0.01f), "at the bank wanted: the full turn, 2 g at 60 deg");

        // ---- lift allocation: vertical first, the turn gets the rest (g = 9.81; 7.2 g of stick) ----
        float deg = (float)(180.0 / Math.PI);
        float lvl = ApMath.BankLimit(70.6f, 0f, 9.81f) * deg;
        Check(lvl > 81.5f && lvl < 83f, $"level turn on the whole budget: {lvl:F1} deg (8.2 g)");
        Check(ApMath.BankLimit(70.6f, 70.6f, 9.81f) * deg < 1f, "a full pull-up leaves no lift for turning: wings level");
        Check(Near(ApMath.BankLimit(70.6f, -9.81f, 9.81f) * deg, 90f, 0.1f), "a 0 g push-over may roll to 90 deg");
        float slow = ApMath.BankLimit(5f, 0f, 9.81f) * deg;
        Check(slow > 38f && slow < 40f, $"slow jet with little pitch authority banks less: {slow:F1} deg");
        Check(Near(ApMath.SegmentDistance(0f, 5f, -10f, 0f, 10f, 0f), 5f) && Near(ApMath.SegmentDistance(13f, 4f, -10f, 0f, 10f, 0f), 5f),
            "segment distance: to the middle, and to an end past it");

        // ---- helicopter height loop, light to heavy rotor (m/s^2 of lift per unit of collective) ----
        foreach (float lift in new[] { 8f, 16f, 25f })
        {
            float low = HoverModel(lift, 20f, 3f, 0f);
            Check(low > 2.7f, $"lift {lift}: descent 20 m -> 3 m undershoots to {low:F2} m");
            low = HoverModel(lift, 40f, 2f, 0f);
            Check(low > 1.8f, $"lift {lift}: descent 40 m -> 2 m undershoots to {low:F2} m");
            low = HoverModel(lift, 3f, 3f, 1f);
            Check(low > 1.5f, $"lift {lift}: 1 m/s^2 push-down at 3 m sags to {low:F2} m (floor 1.5 m)");
        }

        // ---- terrain following ----
        Check(float.IsNegativeInfinity(ApMath.TfComfortSlope(-5f, 500f, 0.26f, 8e-5f)), "terrain below needs no climb");
        Check(float.IsNegativeInfinity(ApMath.TfComfortSlope(50f, 5000f, 0.26f, 8e-5f)), "a far-off hill needs no climb yet");
        Check(Near(ApMath.TfComfortSlope(40f, 1000f, 0.26f, 8e-5f), 0.08f), "on the approach parabola the slope is kPush * d");
        Check(ApMath.TfComfortSlope(600f, 1500f, 0.26f, 8e-5f) == 0.26f, "capped at the planned climb angle");
        bool lim = false;
        Check(ApMath.TfHardSlope(-100f, 1000f, 0.35f, 2e-4f, ref lim) < -0.15f && !lim, "terrain well below allows a descent");
        Check(ApMath.TfHardSlope(500f, 1000f, 0.35f, 2e-4f, ref lim) == 0.35f && lim, "an unclimbable wall is flagged");

        // Fighter at 250 m/s on the default g limits and a slow, sluggish heavy at 120 m/s whose airframe caps
        // them lower, over ridges, a cliff, rolling hills and isolated hills.
        foreach (bool heavy in new[] { false, true })
        {
            float v = heavy ? 120f : 250f;
            foreach (int terrain in new[] { 0, 1, 2, 3 })
            {
                TfModel(v, terrain, 60f, heavy ? 1.5f : 0.8f, heavy ? 0.4f : 1f, heavy ? 2f : 3f, heavy ? 1.5f : 3f, out float low, out float mean, out float crest);
                string tag = $"TF v={v} terrain {terrain}";
                Check(low > 54f, $"{tag}: clearance dipped to {low:F1} m (set 60)");
                Check(mean < (heavy ? 100f : 140f), $"{tag}: mean clearance {mean:F1} m - not following the ground");
                Check(crest > 50f && crest < 80f, $"{tag}: crossed the highest crest at {crest:F1} m");
            }
        }

        // Low and fast: a 30 m clearance must not be eaten by dips at crests.
        foreach (float v in new[] { 150f, 250f, 400f })
            foreach (int terrain in new[] { 0, 1, 2, 3 })
            {
                TfModel(v, terrain, 30f, 0.8f, 1f, 3f, 3f, out float low, out _, out _);
                Check(low > 24f, $"TF v={v} terrain {terrain}: clearance dipped to {low:F1} m (set 30)");
            }


        // ---- surface floor: linear near it, pull-up-limited (sqrt) further out ----
        Check(Near(ApMath.FloorVs(1f, 1f, 7f), -1f), "floor: linear sink limit close to the floor");
        Check(Near(ApMath.FloorVs(100f, 1f, 7f), -(float)Math.Sqrt(1400f), 0.01f), "floor: sink limited to what a pull-up can stop");
        Check(Near(ApMath.FloorVs(-2f, 1f, 7f), 2f), "floor: below it, climb");
        Check(Near(ApMath.LookaheadMinVs(50f, 5f, 5f, 80f), -22.5f), "look-ahead: sink allowed toward distant terrain");
        Check(Near(ApMath.LookaheadMinVs(-100f, 10f, 5f, 80f), -15f), "look-ahead: a ridge 100 m up in 10 s still allows -15 m/s with a 5 m/s^2 pull");
        Check(float.IsNegativeInfinity(ApMath.LookaheadMinVs(10f, 0f, 5f, 80f)), "look-ahead: no constraint at t=0");
        Check(Near(ApMath.LookaheadMinVs(-600f, 20f, 5f, 40f), 40f - (float)Math.Sqrt(2f * 5f * 200f)), "look-ahead: climb-rate cap brings the pull-up forward");
        Check(ApMath.LookaheadMinVs(-2000f, 20f, 5f, 40f) == 40f, "look-ahead: unclearable terrain = climb at the cap");
        Check(Near(ApMath.LookaheadMinVs(-100f, 10f, 5f, 80f, 1f), (100f - 0.5f * 5f * 81f) / 10f), "look-ahead: reaction time delays the pull-up");
        Check(LookaheadClearsRidge(), "look-ahead floor clears a ridge the radar altimeter can't see yet");

        // ---- energy coupling ----
        Check(Near(ApMath.EnergyFeedforward(0f, 0f, 1f, 0.3f), 0f), "no climb, wings level: no feedforward");
        Check(ApMath.EnergyFeedforward(0.1f, 0f, 1f, 0.3f) > 0.09f, "climb adds throttle");
        Check(Near(ApMath.EnergyFeedforward(0f, 60f * (float)Math.PI / 180f, 1f, 0.3f), 0.9f, 0.01f), "60 deg bank (2 g): 0.3 x (4-1)");
        Check(ApMath.SpeedProtectGamma(0.05f, 0f, 10f) == 0.05f, "speed protection: no deficit, climb what the engines sustain");
        Check(ApMath.SpeedProtectGamma(0.05f, 30f, 10f) < 0f, "speed protection: 30 m/s slow at max thrust -> descend to recover");

        // ---- L1 path following ----
        Check(L1Converges(500f, 0f) && L1Converges(-3000f, 90f) && L1Converges(200f, 180f), "L1: converges onto the track from offsets/intercepts");
        float lead = L1Guidance.TurnLead(200f, 35f, 90f, 50000f);
        Check(lead > 5000f && lead < 6000f, $"turn lead for a 90 deg turn at 200 m/s, 35 deg bank = R ({lead:F0} m)");
        foreach (float cv in new[] { 150f, 250f })
        foreach (float turn in new[] { 45f, 90f, 135f })
        {
            float over = NavCornerOvershoot(cv, turn, true), old = NavCornerOvershoot(cv, turn, false);
            Check(over < 6f, $"NAV corner {turn} deg at {cv} m/s rolls out on the new leg ({over:F0} deg past its course; L1 alone: {old:F0})");
        }
        // a slow jet at a steep bank and roll-limited, and the same held to 20 deg by the wingtip over the water
        foreach (float turn in new[] { 45f, 90f, 135f })
        {
            float steep = NavCornerOvershoot(108f, turn, true, 60f, 60f, 30f), low = NavCornerOvershoot(108f, turn, true, 20f, 20f, 30f);
            float wrong = NavCornerOvershoot(108f, turn, true, 60f, 20f, 30f);
            Check(steep < 6f && low < 6f, $"NAV corner {turn} deg at 108 m/s, 30 deg/s roll: 60 deg bank {steep:F0}, wingtip-limited 20 deg {low:F0} deg past (planned for 60, flown at 20: {wrong:F0})");
        }

        // ---- radar masking heights ----
        CheckMaskHeights();
        Check(Near(CoverageMath.DetectRange(10000f, 1f, 1f), 10000f) && Near(CoverageMath.DetectRange(10000f, 1f, 16f), 20000f)
              && Near(CoverageMath.DetectRange(10000f, 1f, 1000f), 20000f), "detect range: rcs^0.25, capped at 2x maxRange");

        // ---- route planner ----
        CheckPlanner();

        // ---- rate loops: INDI + online effectiveness, on a model rate-command airframe ----
        foreach (float tau in new[] { 0.1f, 0.3f, 0.6f })
        foreach (int delay in new[] { 0, 1, 3 })
        foreach (float ratio in new[] { 0.5f, 1f, 2f })
        {
            float rms = RateLoopModel(tau, delay, ratio, 0f, out float overshoot, out float estRatio, out bool sat);
            Check(rms < 0.035f && overshoot < 0.35f && !sat,
                $"INDI tau {tau} delay {delay} ratio {ratio}: rms {rms:F3} rad/s, overshoot {overshoot:P0}, saturated {sat}");
            Check(Math.Abs(estRatio / ratio - 1f) < 0.3f, $"learned ratio tau {tau} delay {delay}: est {estRatio:F2} vs true {ratio}");
        }
        // INDI as the game flies it: the measured airframe (about 0.04 s, 1-2 ticks late, 60 Hz) and around it, the mod's
        // prior (0.04 s) and bandwidth (10): no chatter, rate tracked within 0.4 deg/s. The old prior (0.15 s, bandwidth 3)
        // reversed the stick 13 times a second on the same airframe (the recorded 7.5 Hz limit cycle).
        foreach (float tauTrue in new[] { 0.02f, 0.04f, 0.08f })
        foreach (int delay in new[] { 1, 2, 3 })
        foreach (float ratio in new[] { 0.5f, 1f, 2f })
        {
            IndiGameModel(tauTrue, delay, ratio, 0.04f, 10f, out float rev, out float rms);
            Check(rev < 3f && rms < 0.4f, $"INDI on the game's airframe (tau {tauTrue}, {delay} ticks, ratio {ratio}): {rev:F1} reversals/s, rms {rms:F2} deg/s");
        }
        {
            IndiGameModel(0.04f, 2, 0.7f, 0.15f, 3f, out float rev, out _);
            Check(rev > 6f, $"(model) the old INDI prior chatters on the game's airframe: {rev:F1} reversals/s");
        }

        // the game: 60 Hz, a 4-5 tick answer (what pinned the old one-tick estimate at its floor)
        foreach (int delay in new[] { 4, 5 })
        foreach (float ratio in new[] { 0.5f, 1f, 2f })
        {
            float rms = RateLoopModel(0.2f, delay, ratio, 0f, out _, out float est, out bool sat, 1f / 60f);
            Check(Math.Abs(est / ratio - 1f) < 0.3f && rms < 0.05f && !sat, $"60 Hz, delay {delay} ticks, ratio {ratio}: learned {est:F2}, rms {rms:F3}");
        }
        {
            // damage at 20 s: the stick suddenly gives half the rate; 20 s later the estimate has followed
            RateLoopModel(0.2f, 4, 1f, 0f, out _, out float est, out _, 1f / 60f, 0.5f);
            Check(Math.Abs(est / 0.5f - 1f) < 0.3f, $"learns damage: stick halved at 20 s, learned {est:F2} by 40 s");
        }
        {
            // chatter guard: a 7.5 Hz +-0.1 stick cycle (the recorded INDI limit cycle) softens the loop within 2 s;
            // 0.4 manoeuvre steps every 2 s don't
            var g = new EffectivenessRls(0.15f);
            for (int i = 0; i < 120; i++) g.Observe(0f, 0f, 0.1f * (float)Math.Sin(2.0 * Math.PI * 7.5 * i / 60.0), 1f / 60f);
            float cycling = g.Detune;
            var h = new EffectivenessRls(0.15f);
            float most = 1f;
            for (int i = 0; i < 600; i++) { h.Observe(0f, 0f, (i / 120) % 2 == 0 ? 0.4f : -0.4f, 1f / 60f); most = Math.Max(most, h.Detune); }
            Check(cycling > 2f, $"chatter guard: 7.5 Hz stick cycle softens the loop ({cycling:F2}x)");
            Check(most < 1.05f, $"chatter guard: manoeuvre steps leave it alone ({most:F2}x)");
        }
        {
            float rms = RateLoopModel(0.3f, 1, 1f, 0.4f, out float os, out _, out bool sat);
            Check(rms < 0.035f && !sat, $"INDI trims out a steady pitching moment (rms {rms:F3})");
        }

        Console.WriteLine(failures == 0 ? "Self-check passed." : $"{failures} check(s) failed.");
        return failures == 0 ? 0 : 1;
    }

    private static float TfTerrain(int kind, float x)
    {
        switch (kind)
        {
            case 0:   // two sharp 400 m ridges with a valley between
                return 400f * Math.Max(0f, 1f - Math.Abs(x - 12000f) / 1500f) + 400f * Math.Max(0f, 1f - Math.Abs(x - 17000f) / 1500f);
            case 1:   // 300 m cliff up to a plateau, then a cliff back down
                return x > 10000f && x < 18000f ? 300f : 0f;
            case 3:   // isolated rounded 120 m hills, 800 m across, every 3 km
                if (x < 4500f) return 0f;
                float dx = (x - 4500f) % 3000f - 1500f;
                return Math.Abs(dx) < 400f ? 120f * (float)Math.Pow(Math.Cos(Math.PI * dx / 800f), 2) : 0f;
            default:  // rolling hills, growing out of a plain so the run starts settled
                return Math.Min(1f, x / 6000f) * (150f + 120f * (float)Math.Sin(x / 900f) + 60f * (float)Math.Sin(x / 310f + 1f));
        }
    }

    /// <summary>
    /// Point-mass jet flown by the same terrain-following law as FlightController.FlyVertical: pitch
    /// rate command -> pitch -> flight path through a lift lag; agility scales the flight-path gain
    /// as it does on the heavy jets; pullG/pushG are the g limits (extra g above / below 1g). Returns the lowest and mean
    /// clearance and the clearance over the highest point of the terrain.
    /// </summary>
    private static void TfModel(float v, int terrain, float clearance, float liftLag, float agility, float pullG, float pushG, out float low, out float mean, out float crest)
    {
        const float dt = 0.02f, g = 9.81f, smoothing = 0.4f;
        const int n = 40;
        const float maxGamma = 20f * (float)Math.PI / 180f, planGamma = 0.75f * maxGamma;
        float aPullMax = pullG * g, aPushMax = pushG * g, aPull = ApMath.TfPlanG * aPullMax, aPush = ApMath.TfPlanG * aPushMax;
        var dist = new float[n];
        var rel = new float[n];
        float x = 0f, alt = TfTerrain(terrain, 0f) + clearance, gamma = 0f, theta = 0f;
        float cmd = 0f, cmdPrev = 0f, cmdRate = 0f, gammaPrev = 0f, gammaRate = 0f;
        float travelled = 0f, altAtSweep = alt, nearSlope = 0f, sinceSweep = 1f;
        low = float.MaxValue;
        crest = 0f;
        float sum = 0f, top = float.MinValue;
        int ticks = 0;
        float raltF = clearance;   // FlyVertical's complementary-filtered radar altitude
        while (x < 40000f)
        {
            float ralt = alt - TfTerrain(terrain, x);
            raltF += v * (float)Math.Sin(gamma) * dt + (ralt - raltF) * Math.Min(1f, 2f * dt);
            if (ralt < raltF) raltF = ralt; // surface closer: at once
            sinceSweep += dt;
            if (sinceSweep >= 0.1f)
            {
                sinceSweep = 0f;
                travelled = 0f;
                altAtSweep = alt;
                float look = Math.Max(1000f, Math.Min(8000f, v * 15f));
                nearSlope = float.NegativeInfinity;
                for (int i = 0; i < n; i++)
                {
                    dist[i] = ApMath.TfSampleDistance(i, n, look);
                    rel[i] = TfTerrain(terrain, x + dist[i]) - alt;
                    if (dist[i] >= 0.5f * v && dist[i] <= 2f * v)
                        nearSlope = Math.Max(nearSlope, (rel[i] + ralt) / dist[i]);
                }
            }
            float maxVs = v * (float)Math.Sin(planGamma);
            float vsCmd = ApMath.CaptureProfile(clearance - raltF, 2f * 0.3f, 0.5f * aPull, maxVs) + ApMath.TfDescentFeedforward(nearSlope, v);
            vsCmd = Math.Max(-maxVs, Math.Min(maxVs, vsCmd));
            float slope = ApMath.TfSlope(dist, rel, n, travelled, alt - altAtSweep, clearance, v, gamma,
                planGamma, maxGamma, aPull, aPush, 1f, out _);
            if (!float.IsNegativeInfinity(slope)) vsCmd = Math.Max(vsCmd, v * (float)Math.Sin(slope));
            float target = (float)Math.Asin(Math.Max(-Math.Sin(maxGamma), Math.Min(Math.Sin(maxGamma), vsCmd / v)));
            float slew = Math.Min((target > cmd ? aPullMax : aPushMax) / v, Math.Abs(target - cmd) / smoothing);
            cmd += Math.Sign(target - cmd) * Math.Min(Math.Abs(target - cmd), slew * dt);
            cmdRate += ((cmd - cmdPrev) / dt - cmdRate) * (dt / (0.2f + dt));
            cmdPrev = cmd;
            gammaRate += ((gamma - gammaPrev) / dt - gammaRate) * (dt / (0.2f + dt));
            gammaPrev = gamma;
            float q = 2f * agility * (cmd - gamma) - 2.5f * (gammaRate - cmdRate) + cmdRate;
            q = Math.Max(-aPushMax / v, Math.Min(aPullMax / v, q));
            theta += q * dt;
            gamma += (theta - gamma) * dt / liftLag;
            x += v * (float)Math.Cos(gamma) * dt;
            alt += v * (float)Math.Sin(gamma) * dt;
            travelled += v * (float)Math.Cos(gamma) * dt;

            float h = TfTerrain(terrain, x);
            float c = alt - h;
            if (x > 2000f) low = Math.Min(low, c);   // skip the start-up transient
            sum += c;
            ticks++;
            if (h > top + 0.01f)
            {
                top = h;
                crest = c;
            }
        }
        mean = sum / ticks;
    }

    /// <summary>
    /// Point-mass helicopter: collective -> lift through a 0.3 s lag, flown by the same laws as
    /// RotorcraftLaws.FlyRotor's height loop. <paramref name="push"/> is a downward acceleration applied
    /// from 5 s to 9 s (lift lost while the rotor tilts to change speed). Returns the lowest height.
    /// </summary>
    private static float HoverModel(float lift, float start, float target, float push)
    {
        const float dt = 0.02f, floor = 1.5f, trim = 0.6f;
        float alt = start, vs = 0f, coll = trim, thrust = trim, vsAccel = 0f, prevVs = 0f, low = start;
        for (int i = 0; i < 3000; i++)
        {
            float t = i * dt;
            vsAccel += ((vs - prevVs) / dt - vsAccel) * (dt / (0.2f + dt));
            prevVs = vs;
            float vsCmd = ApMath.CaptureProfile(target - alt, 0.3f, 1f, 8f);
            vsCmd = Math.Max(vsCmd, -3f);   // vortex-ring guard at low speed
            vsCmd = Math.Max(vsCmd, ApMath.RotorFloorVs(alt, floor, 1f, 0f));
            float rate = Math.Max(-0.6f, Math.Min(0.6f, ApMath.CollectiveRate(vsCmd, vs, vsAccel)));
            coll = Math.Max(0f, Math.Min(1f, coll + rate * dt));
            thrust += (coll - thrust) * dt / 0.3f;
            vs += (lift * (thrust - trim) - (t >= 5f && t < 9f ? push : 0f)) * dt;
            alt += vs * dt;
            if (t > 1f) low = Math.Min(low, alt);
        }
        return low;
    }

    /// <summary>
    /// Jet at 250 m/s, 60 m above flat ground, flying at a 600 m ridge 6 km ahead. The plain floor
    /// only knows the ground under it; the look-ahead must start the climb in time. Vertical
    /// acceleration toward the commanded climb is limited to 1 g extra (pull-up). Returns true if the
    /// lowest point stays above the ridge.
    /// </summary>
    private static bool LookaheadClearsRidge()
    {
        const float dt = 0.02f, v = 250f, a = 9.81f * 0.8f, floor = 1.5f;
        float vsMax = v * (float)Math.Sin(20f * Math.PI / 180f), aPlan = 0.5f * 9.81f; // plan with less pull than it has
        float x = 0f, h = 60f, vs = 0f;
        float Terrain(float px) => px > 6000f && px < 6800f ? 600f : 0f;
        for (int i = 0; i < 2000; i++)
        {
            float room = h - Terrain(x) - floor;
            float vsCmd = ApMath.CaptureProfile(60f - (h - Terrain(x)), 0.3f, 2.5f, 40f);
            vsCmd = Math.Max(vsCmd, ApMath.FloorVs(room, 1f, aPlan));
            // as FlightController does it: samples bunched near, each one's terrain charged to the
            // previous sample's time (it may start anywhere in between)
            for (int k = 1; k <= 12; k++)
            {
                float t = 15f * (k / 12f) * (k / 12f), tPrev = 15f * ((k - 1) / 12f) * ((k - 1) / 12f);
                float r = h - Terrain(x + v * t) - floor - 10f;
                vsCmd = Math.Max(vsCmd, ApMath.LookaheadMinVs(r, Math.Max(tPrev, 0.25f), aPlan, vsMax, 1f));
            }
            vs += Math.Max(-a, Math.Min(a, (vsCmd - vs) * 2f)) * dt;
            vs = Math.Max(-vsMax, Math.Min(vsMax, vs));
            h += vs * dt;
            x += v * dt;
            if (h - Terrain(x) < 0f) return false;
        }
        return true;
    }

    /// <summary>Point-mass aircraft at 200 m/s (35 deg bank limit, 25 deg/s roll) flown by L1 onto a
    /// north-going track through the origin, starting crossTrack m east of it on a heading. True if it
    /// settles within 15 m and, once within 50 m, never strays past 100 m again.</summary>
    private static bool L1Converges(float crossTrack, float headingDeg)
    {
        const float dt = 0.02f, v = 200f, g = 9.81f, maxBank = 35f;
        float x = crossTrack, z = 0f, hdg = headingDeg * (float)Math.PI / 180f, bank = 0f;
        bool captured = false, strayed = false;
        for (int i = 0; i < 12000; i++)
        {
            float vx = v * (float)Math.Sin(hdg), vz = v * (float)Math.Cos(hdg);
            float acc = L1Guidance.LateralAccel(x, z, vx, vz, 0f, -100000f, 0f, 100000f, 20f, 0.75f, g * (float)Math.Tan(maxBank * Math.PI / 180), out float xt, out _);
            float bankCmd = Math.Max(-maxBank, Math.Min(maxBank, (float)(Math.Atan(acc / g) * 180 / Math.PI)));
            bank += Math.Max(-25f * dt, Math.Min(25f * dt, bankCmd - bank));
            hdg += g * (float)Math.Tan(bank * Math.PI / 180) / v * dt;
            x += vx * dt;
            z += vz * dt;
            if (Math.Abs(xt) < 50f && Math.Abs(Math.Sin(hdg)) < 0.1) captured = true;
            else if (captured && Math.Abs(xt) > 100f) strayed = true;
        }
        return Math.Abs(x) < 15f && !strayed;
    }

    /// <summary>NAV (FlightController.UpdateNav) through one corner of turnDeg between 30 km legs (long enough for each turn's lead): point mass at
    /// speed v, planning its turns for planBank, flying no steeper than maxBank, rolling at roll deg/s. Most heading (deg) past the
    /// new leg's course. arc = false: L1 alone.</summary>
    private static float NavCornerOvershoot(float v, float turnDeg, bool arc, float planBank = 35f, float maxBank = 35f, float roll = 25f)
    {
        const float dt = 0.02f, g = 9.81f;
        float maxAcc = g * (float)Math.Tan(planBank * Math.PI / 180), a = turnDeg * (float)Math.PI / 180f;
        float[] wx = { 0f, 0f, 30000f * (float)Math.Sin(a) }, wz = { 0f, 30000f, 30000f + 30000f * (float)Math.Cos(a) };
        float x = 0f, z = 0f, hdg = 0f, bank = 0f, sign = 0f, over = 0f;
        int leg = 1;
        for (int i = 0; i < 20000 && leg < 3; i++)
        {
            float vx = v * (float)Math.Sin(hdg), vz = v * (float)Math.Cos(hdg);
            float acc = L1Guidance.LateralAccel(x, z, vx, vz, wx[leg - 1], wz[leg - 1], wx[leg], wz[leg], 20f, 0.75f, maxAcc, out _, out float toGo);
            if (leg == 1 && toGo <= L1Guidance.TurnLead(v, planBank, turnDeg, 30000f, arc ? planBank / roll : 0f))
            {
                leg = 2;
                sign = arc ? 1f : 0f;
                continue;
            }
            if (sign != 0f && !L1Guidance.ArcTurn(ref acc, sign, v, maxAcc, hdg * 180f / (float)Math.PI, turnDeg)) sign = 0f;
            float bankCmd = Math.Max(-maxBank, Math.Min(maxBank, (float)(Math.Atan(acc / g) * 180 / Math.PI)));
            bank += Math.Max(-roll * dt, Math.Min(roll * dt, bankCmd - bank));
            hdg += g * (float)Math.Tan(bank * Math.PI / 180) / v * dt;
            x += vx * dt;
            z += vz * dt;
            if (leg == 2) over = Math.Max(over, (hdg - a) * 180f / (float)Math.PI);
        }
        return over;
    }

    private static void CheckPlanner()
    {
        // 100 x 100 cells of 500 m; a radar in the middle sees everything within 12 km (even 5 m up),
        // and a valley across its coverage where it can't see below 200 m.
        var p = new RoutePlanner { N = 100, Cell = 500f, OriginX = -25000f, OriginZ = -25000f, Speed = 200f };
        p.Hide = new float[100 * 100];
        for (int z = 0; z < 100; z++)
        for (int x = 0; x < 100; x++)
        {
            float cx = p.OriginX + (x + 0.5f) * p.Cell, cz = p.OriginZ + (z + 0.5f) * p.Cell;
            p.Hide[z * 100 + x] = cx * cx + cz * cz < 12000f * 12000f ? 2f : float.PositiveInfinity;
        }
        var route = p.Plan(-20000f, 0f, 20000f, 0f);
        Check(route != null && route.Count >= 2, "planner: finds a route around the radar");
        if (route != null)
        {
            Check(p.ExposedSeconds < 0.5f, $"planner: route around the radar is unexposed ({p.ExposedSeconds:F1} s)");
            float len = 0f, px = -20000f, pz = 0f, closest = float.MaxValue;
            foreach (Waypoint w in route)
            {
                len += (float)Math.Sqrt((w.X - px) * (w.X - px) + (w.Z - pz) * (w.Z - pz));
                for (int k = 0; k <= 20; k++)
                {
                    float t = k / 20f, qx = px + (w.X - px) * t, qz = pz + (w.Z - pz) * t;
                    closest = Math.Min(closest, (float)Math.Sqrt(qx * qx + qz * qz));
                }
                px = w.X;
                pz = w.Z;
            }
            Check(closest > 11500f, $"planner: stays out of the radar's reach (closest {closest:F0} m)");
            Check(len < 40000f * 1.6f, $"planner: detour is reasonable ({len:F0} m vs 40000 straight)");
            Check(route.Count <= 8, $"planner: any-angle route has few corners ({route.Count})");
            Check(route[route.Count - 1].X == 20000f && route[route.Count - 1].Z == 0f, "planner: ends on the goal");
        }

        // valley through the coverage: hidden below 200 m. Going low through it beats the detour.
        for (int z = 0; z < 100; z++)
        for (int x = 0; x < 100; x++)
        {
            float cz = p.OriginZ + (z + 0.5f) * p.Cell;
            if (Math.Abs(cz) < 1500f && p.Hide[z * 100 + x] < 100f) p.Hide[z * 100 + x] = 200f;
        }
        route = p.Plan(-20000f, 0f, 20000f, 0f);
        Check(route != null && p.ExposedSeconds < 0.5f, "planner: valley route is unexposed");
        if (route != null)
        {
            bool low = false, straight = true;
            foreach (Waypoint w in route)
            {
                if (Math.Abs(w.X) < 12000f && w.Agl <= 150f) low = true;
                if (Math.Abs(w.Z) > 1500f) straight = false;
            }
            bool legLow = false;
            foreach (Waypoint w in route) if (w.Agl < 200f) legLow = true;
            Check(straight && legLow, "planner: takes the valley, low, instead of going around");
            Check(route[0].Agl < 200f || route.Count > 1, "planner: drops to a hidden band before the valley");
        }
    }

    /// <summary>
    /// One axis of a rate-command fly-by-wire jet: the FBW demands ratio x (model rate per stick) x
    /// stick, the airframe follows that demand with time constant tau after a delay, plus an optional
    /// steady pitching acceleration the stick must hold off. Flown by IndiAxis (with its RLS) on a
    /// square-wave rate command. Returns the RMS tracking error over the last 10 s.
    /// </summary>
    /// <summary>
    /// INDI with learning on a first-order rate response (tauTrue, delay ticks, effectiveness ratio) at the game's
    /// 60 Hz, following +-0.05 rad/s rate steps every 2 s: stick reversals per second and the rate error (deg/s,
    /// from 0.8 s after each step) over the last 10 s.
    /// </summary>
    private static void IndiGameModel(float tauTrue, int delay, float ratio, float tauPrior, float bandwidth, out float reversals, out float rms)
    {
        const float dt = 1f / 60f, model = 0.25f;
        var axis = new IndiAxis(tauPrior) { IncrementGain = 0.5f };
        var hist = new float[delay + 1];
        float q = 0f, u = 0f, du0 = 0f, err2 = 0f;
        int flips = 0, n = 0;
        for (int i = 0; i < 60 * 30; i++)
        {
            float t = i * dt;
            axis.Observe(q, u, model, dt, 5f);
            float qCmd = (int)(t / 2f) % 2 == 0 ? 0.05f : -0.05f;
            float uNew = axis.Command(qCmd, q, model, bandwidth, true, u);
            if (t > 20f)
            {
                float d = uNew - u;
                if (Math.Abs(d) > 0.002f && d * du0 < 0f) flips++;
                if (Math.Abs(d) > 0.002f) du0 = d;
                if (t % 2f > 0.8f) { err2 += (qCmd - q) * (qCmd - q); n++; }
            }
            u = uNew;
            for (int k = delay; k > 0; k--) hist[k] = hist[k - 1];
            hist[0] = u;
            q += (ratio * model * hist[delay] - q) / tauTrue * dt;
        }
        reversals = flips / 10f;
        rms = (float)Math.Sqrt(err2 / Math.Max(1, n)) * 57.3f;
    }

    private static float RateLoopModel(float tau, int delay, float ratio, float bias, out float overshoot, out float estRatio, out bool saturated,
        float dt = 0.02f, float ratioAfter = float.NaN)
    {
        const float model = 0.6f, amp = 0.15f;
        var axis = new IndiAxis(0.15f) { IncrementGain = 0.5f };
        float q = 0f, u = 0f, err2 = 0f, peak = 0f;
        int n = 0, satTicks = 0;
        var hist = new float[delay + 1];
        for (int i = 0; i < (int)(40f / dt); i++)
        {
            float t = i * dt;
            axis.Observe(q, u, model, dt, 5f);
            float qCmd = (int)(t / 3f) % 2 == 0 ? amp : -amp;
            u = axis.Command(qCmd, q, model, 3f, true, 0f);
            if (Math.Abs(u) > 0.99f) satTicks++;
            for (int k = delay; k > 0; k--) hist[k] = hist[k - 1];
            hist[0] = u;
            float demand = (t >= 20f && !float.IsNaN(ratioAfter) ? ratioAfter : ratio) * model * hist[delay];
            q += ((demand - q) / tau + bias * (1f - (float)Math.Exp(-t))) * dt;
            if (t > 20f)
            {
                err2 += (qCmd - q) * (qCmd - q) * (((t % 3f) > 1.2f) ? 1f : 0f);
                n += (t % 3f) > 1.2f ? 1 : 0;
                peak = Math.Max(peak, Math.Abs(q));
            }
        }
        overshoot = peak / amp - 1f;
        estRatio = axis.Rls.Ratio;
        saturated = satTicks > 50;
        return (float)Math.Sqrt(err2 / Math.Max(1, n));
    }

    /// <summary>
    /// The masking height must give exactly the old line-of-sight answer (steepest-slope sweep plus
    /// the curved-earth horizon) for any target height, on random terrain.
    /// </summary>
    private static void CheckMaskHeights()
    {
        const int az = 4, steps = 300;
        const float step = 200f, radarAlt = 120f;
        var rnd = new Random(7);
        var height = new float[az * steps];
        var before = new float[az * steps];
        for (int a = 0; a < az; a++)
        {
            float h = 50f;
            for (int i = 0; i < steps; i++)
            {
                h = Math.Max(0f, h + (float)(rnd.NextDouble() - 0.5) * 60f);
                height[a * steps + i] = h;
                // a ridge inside some bins
                before[a * steps + i] = rnd.NextDouble() < 0.1 ? (h + 40f - radarAlt) / (i * step + step * 0.5f) : float.NegativeInfinity;
            }
        }
        var mask = (float[])before.Clone();
        CoverageMath.MaskHeights(height, mask, radarAlt, step, az, steps);
        float radarHorizon = (float)Math.Sqrt(CoverageMath.TwoR * radarAlt);
        int mismatches = 0;
        foreach (float agl in new[] { 5f, 30f, 150f, 500f, 2000f })
        for (int a = 0; a < az; a++)
        {
            float maxSlope = float.NegativeInfinity;
            for (int i = 0; i < steps; i++)
            {
                int k = a * steps + i;
                float d = (i + 1) * step, tgt = height[k] + agl;
                maxSlope = Math.Max(maxSlope, before[k]);
                bool hiddenOld = (tgt - radarAlt) / d < maxSlope || radarHorizon + Math.Sqrt(CoverageMath.TwoR * Math.Max(0f, tgt)) < d;
                bool hiddenNew = tgt < mask[k];
                // exact boundary hits can round either way; anything else is a real mismatch
                if (hiddenOld != hiddenNew && Math.Abs(tgt - mask[k]) > 0.01f) mismatches++;
                maxSlope = Math.Max(maxSlope, (height[k] - radarAlt) / d);
            }
        }
        Check(mismatches == 0, $"masking height reproduces the line-of-sight sweep ({mismatches} mismatches)");
    }
}
