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

        // ---- NAV flying a route: 250 m/s, 74 deg sustained bank (R = 1.8 km), 1 s turn-rate lag ----
        // route: 20 km north, 90 deg right, 15 km, 135 deg left, 15 km
        float[] wx = { 0f, 15000f, 4400f }, wz = { 20000f, 20000f, 30600f };
        float[] cornerDeg = { 90f, 135f, 0f };
        NavModel(250f, 74f, wx, wz, out int reached, out float[] miss, out float worstOff, out float radius, out float settledBank);
        Check(settledBank < 10f, $"NAV flies settled legs nearly wings level (bank up to {settledBank:F1} deg in the last 40% of each leg)");
        Check(reached == 3, $"NAV reaches every waypoint ({reached}/3)");
        for (int k = 0; k < miss.Length; k++)
        {
            // a fly-by of a turn of radius R passes R(1/cos(turn/2) - 1) inside the corner: the geometric best
            float best = radius * (1f / (float)Math.Cos(cornerDeg[k] * Math.PI / 360.0) - 1f);
            Check(miss[k] < best + 300f, $"NAV corner {k + 1} ({cornerDeg[k]:F0} deg): passes {miss[k]:F0} m away, geometric best {best:F0} m");
        }
        Check(worstOff < 300f, $"NAV holds the drawn line within {worstOff:F0} m once on a leg");

        // ---- NAV fly-over on the same route: 15 deg/s bank slew (0.6 x 25 deg/s) ----
        FlyOverModel(250f, 74f, 15f, wx, wz, out int foReached, out float[] foMiss, out float foBank);
        Check(foReached == 3, $"fly-over reaches every waypoint ({foReached}/3)");
        for (int k = 0; k < 2; k++)
            Check(foMiss[k] < ApMath.FlyOverMiss + 1f, $"fly-over corner {k + 1} ({cornerDeg[k]:F0} deg): passes {foMiss[k]:F1} m from the waypoint");
        Check(foBank < 2f, $"fly-over flies the direct legs wings level (bank up to {foBank:F1} deg on their second half)");

        // ---- NAV L1 guidance (flying north at 250 m/s) ----
        Check(Near(ApMath.L1TurnRate(0f, 250f, 0f, 2000f), 0f), "steering point dead ahead: no turn");
        Check(Near(ApMath.L1TurnRate(0f, 250f, 1000f, 1000f), 2f * 250f * 0.7071f / 1414.2f, 1e-3f), "point 45 deg right: 2 v sin(eta) / L, turning right");
        Check(ApMath.L1TurnRate(0f, 250f, -1000f, -1000f) < 0f && Near(Math.Abs(ApMath.L1TurnRate(0f, 250f, -1000f, -1000f)), 2f * 250f / 1414.2f, 1e-3f),
            "point behind on the left: the 90 deg rate, turning left");
        Check(Near(ApMath.AlongTrack(0f, -3000f, 0f, 7000f), 3000f), "3 km along a leg that started 3 km behind");

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

        // ---- NAV leg steering point (jet at origin, leg runs north 1 km to its right) ----
        ApMath.LegCarrot(1000f, -5000f, 1000f, 5000f, 2000f, out float cx, out float cz);
        Check(Near(cx, 1000f) && Near(cz, 2000f), "steering point sits on the leg, look-ahead past the jet's abeam point");
        ApMath.LegCarrot(1000f, -5000f, 1000f, 1500f, 2000f, out cx, out cz);
        Check(Near(cx, 1000f) && Near(cz, 1500f), "the point stops at the waypoint");

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
    /// <summary>
    /// Point-mass jet flying RouteNav's logic: L1 at a point one turn radius on (at least 3 s), next leg
    /// once abeam the fly-by turn point. Turn rate limited by the bank, lagging the command by 1 s.
    /// miss = closest approach to each waypoint, worstOff = cross-track from the leg after the turn has
    /// settled (from 3 turn radii past the corner to 3 before the next).
    /// </summary>
    private static void NavModel(float v, float bankDeg, float[] wx, float[] wz, out int reached, out float[] miss, out float worstOff, out float radius, out float settledBank)
    {
        const float g = 9.81f, dt = 0.02f;
        float wMax = g * (float)Math.Tan(bankDeg * Math.PI / 180.0) / v;
        radius = v / wMax;
        float x = 0f, z = 0f, hdg = 0f, w = 0f, lx = 0f, lz = 0f;
        int i = 0;
        reached = 0;
        worstOff = 0f;
        settledBank = 0f;
        miss = new float[wx.Length];
        for (int k = 0; k < miss.Length; k++) miss[k] = float.MaxValue;
        for (float t = 0f; t < 600f && i < wx.Length; t += dt)
        {
            for (int k = 0; k < miss.Length; k++) // closest approach, also after NAV moved on to the next leg
                miss[k] = Math.Min(miss[k], (float)Math.Sqrt((wx[k] - x) * (wx[k] - x) + (wz[k] - z) * (wz[k] - z)));
            float ax = lx - x, az = lz - z, bx = wx[i] - x, bz = wz[i] - z;
            bool next = i + 1 < wx.Length;
            float nx = next ? wx[i + 1] - x : 0f, nz = next ? wz[i + 1] - z : 0f;
            float turn = next ? Angle(bx - ax, bz - az, nx - bx, nz - bz) : 0f;
            float len = (float)Math.Sqrt((bx - ax) * (bx - ax) + (bz - az) * (bz - az));
            float along = ApMath.AlongTrack(ax, az, bx, bz);
            float lead = radius * (float)Math.Tan(Math.Min(Math.Abs(turn), 2.6f) * 0.5f);
            if (along >= len - lead || bx * bx + bz * bz < ApMath.MinWaypointLead * ApMath.MinWaypointLead)
            {
                lx = wx[i];
                lz = wz[i];
                i++;
                reached++;
                continue;
            }
            // settled on the leg: measure cross-track
            if (along > (i > 0 ? 3f * radius : 0f) && along < len - 3f * radius)
            {
                float off = Math.Abs(((bx - ax) * az - (bz - az) * ax) / Math.Max(len, 1f));
                worstOff = Math.Max(worstOff, off);
            }
            if (along > 0.6f * len && along < len - 3f * radius)
                settledBank = Math.Max(settledBank, (float)(Math.Atan(v * Math.Abs(w) / g) * 180.0 / Math.PI));
            float vx = v * (float)Math.Sin(hdg), vz = v * (float)Math.Cos(hdg);
            ApMath.LegCarrot(ax, az, bx, bz, Math.Max(radius, ApMath.NavLookSeconds * v), out float cx, out float cz);
            float cmd = Math.Max(-wMax, Math.Min(wMax, ApMath.L1TurnRate(vx, vz, cx, cz)));
            w += (cmd - w) * dt / 1f;
            hdg += w * dt;
            x += vx * dt;
            z += vz * dt;
        }
    }

    /// <summary>
    /// Point-mass jet flying RouteNav's fly-over logic: bank slews at <paramref name="rollDeg"/> deg/s up to
    /// <paramref name="bankDeg"/>; the hard turn starts FlyOverLeadTime before each waypoint, aims straight at the
    /// next and rolls out on HardTurnDone, then L1 flies a leg from there. miss = closest approach to the
    /// waypoints with a turn after them; settledBank = the most bank on the second half of each direct leg.
    /// </summary>
    private static void FlyOverModel(float v, float bankDeg, float rollDeg, float[] wx, float[] wz, out int reached, out float[] miss, out float settledBank)
    {
        const float g = 9.81f, dt = 0.01f, d2r = (float)(Math.PI / 180.0);
        float roll = rollDeg * d2r, maxBank = bankDeg * d2r;
        float x = 0f, z = 0f, hdg = 0f, bank = 0f, lx = 0f, lz = 0f;
        bool hard = false;
        int i = 0;
        reached = 0;
        settledBank = 0f;
        miss = new float[wx.Length];
        for (int k = 0; k < miss.Length; k++) miss[k] = float.MaxValue;
        for (float t = 0f; t < 600f && i < wx.Length; t += dt)
        {
            for (int k = 0; k < miss.Length; k++)
                miss[k] = Math.Min(miss[k], (float)Math.Sqrt((wx[k] - x) * (wx[k] - x) + (wz[k] - z) * (wz[k] - z)));
            float ax = lx - x, az = lz - z, bx = wx[i] - x, bz = wz[i] - z;
            float len = (float)Math.Sqrt((bx - ax) * (bx - ax) + (bz - az) * (bz - az));
            float along = ApMath.AlongTrack(ax, az, bx, bz);
            if (along >= len - v * ApMath.FlyOverLeadTime(roll, g))
            {
                lx = wx[i];
                lz = wz[i];
                i++;
                reached++;
                hard = i < wx.Length;
                continue;
            }
            float vx = v * (float)Math.Sin(hdg), vz = v * (float)Math.Cos(hdg);
            float eta = Angle(vx, vz, bx, bz), bankCmd;
            if (hard && ApMath.HardTurnDone(eta, bank, roll, v, g))
            {
                hard = false;
                lx = x;
                lz = z;
                ax = az = 0f;
            }
            if (hard) bankCmd = (eta < 0f ? -1f : 1f) * maxBank;
            else
            {
                if (along > 0.5f * len) settledBank = Math.Max(settledBank, Math.Abs(bank) / d2r);
                ApMath.LegCarrot(ax, az, bx, bz, ApMath.NavLookSeconds * v, out float cx, out float cz);
                float cmd = ApMath.L1TurnRate(vx, vz, cx, cz);
                bankCmd = Math.Max(-maxBank, Math.Min(maxBank, (float)Math.Atan(cmd * v / g)));
            }
            bank += Math.Max(-roll * dt, Math.Min(roll * dt, bankCmd - bank));
            hdg += g * (float)Math.Tan(bank) / v * dt;
            x += vx * dt;
            z += vz * dt;
        }
    }

    /// <summary>Signed angle (rad, + right) from direction a to direction b.</summary>
    private static float Angle(float ax, float az, float bx, float bz) =>
        (float)Math.Atan2(az * bx - ax * bz, ax * bx + az * bz);

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
}
