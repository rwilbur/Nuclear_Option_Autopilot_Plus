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

        // Fighter at 250 m/s and a slow, sluggish heavy at 120 m/s, over ridges, a cliff and rolling hills.
        foreach (bool heavy in new[] { false, true })
        {
            float v = heavy ? 120f : 250f;
            foreach (int terrain in new[] { 0, 1, 2 })
            {
                TfModel(v, terrain, 60f, heavy ? 1.5f : 0.8f, heavy ? 0.4f : 1f, out float low, out float mean, out float crest);
                string tag = $"TF v={v} terrain {terrain}";
                Check(low > 45f, $"{tag}: clearance dipped to {low:F1} m (set 60)");
                Check(mean < (heavy ? 100f : 160f), $"{tag}: mean clearance {mean:F1} m - not following the ground");
                Check(crest > 50f && crest < 80f, $"{tag}: crossed the highest crest at {crest:F1} m");
            }
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
            default:  // rolling hills, growing out of a plain so the run starts settled
                return Math.Min(1f, x / 6000f) * (150f + 120f * (float)Math.Sin(x / 900f) + 60f * (float)Math.Sin(x / 310f + 1f));
        }
    }

    /// <summary>
    /// Point-mass jet flown by the same terrain-following law as FlightController.FlyVertical: pitch
    /// rate command -> pitch -> flight path through a lift lag; agility scales the flight-path gain
    /// as it does on the heavy jets. Returns the lowest and mean
    /// clearance and the clearance over the highest point of the terrain.
    /// </summary>
    private static void TfModel(float v, int terrain, float clearance, float liftLag, float agility, out float low, out float mean, out float crest)
    {
        const float dt = 0.02f, g = 9.81f;
        const int n = 40;
        const float maxGamma = 20f * (float)Math.PI / 180f, planGamma = 0.75f * maxGamma;
        const float aPull = 1f * g, aPush = 0.5f * g, aPullMax = 1.5f * g, aPushMax = 1f * g;
        var dist = new float[n];
        var rel = new float[n];
        float x = 0f, alt = TfTerrain(terrain, 0f) + clearance, gamma = 0f, theta = 0f;
        float cmd = 0f, cmdPrev = 0f, cmdRate = 0f, gammaPrev = 0f, gammaRate = 0f;
        float travelled = 0f, altAtSweep = alt, nearSlope = 0f, sinceSweep = 1f;
        low = float.MaxValue;
        crest = 0f;
        float sum = 0f, top = float.MinValue;
        int ticks = 0;
        while (x < 40000f)
        {
            float ralt = alt - TfTerrain(terrain, x);
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
            float vsCmd = ApMath.CaptureProfile(clearance - ralt, 0.3f, 0.5f * aPull, maxVs) + ApMath.TfDescentFeedforward(nearSlope, v);
            vsCmd = Math.Max(-maxVs, Math.Min(maxVs, vsCmd));
            float slope = ApMath.TfSlope(dist, rel, n, travelled, alt - altAtSweep, clearance, v, gamma,
                planGamma, maxGamma, aPull, aPush, 1f, out _);
            if (!float.IsNegativeInfinity(slope)) vsCmd = Math.Max(vsCmd, v * (float)Math.Sin(slope));
            float target = (float)Math.Asin(Math.Max(-Math.Sin(maxGamma), Math.Min(Math.Sin(maxGamma), vsCmd / v)));
            cmd = target > cmd ? Math.Min(target, cmd + aPullMax / v * dt) : Math.Max(target, cmd - Math.Min(1.5f * aPush, aPushMax) / v * dt);
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
            low = Math.Min(low, c);
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
}
