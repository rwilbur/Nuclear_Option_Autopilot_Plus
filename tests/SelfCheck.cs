using System;
using AutopilotMod;

/// <summary>
/// Assert-based checks of src/ApMath.cs, plus a small offline model of the helicopter height loop.
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
            if (delay <= 1)
                Check(Math.Abs(estRatio / ratio - 1f) < 0.3f, $"RLS ratio tau {tau} delay {delay}: est {estRatio:F2} vs true {ratio}");
        }
        {
            float rms = RateLoopModel(0.3f, 1, 1f, 0.4f, out float os, out _, out bool sat);
            Check(rms < 0.035f && !sat, $"INDI trims out a steady pitching moment (rms {rms:F3})");
        }

        Console.WriteLine(failures == 0 ? "Self-check passed." : $"{failures} check(s) failed.");
        return failures == 0 ? 0 : 1;
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
    private static float RateLoopModel(float tau, int delay, float ratio, float bias, out float overshoot, out float estRatio, out bool saturated)
    {
        const float dt = 0.02f, model = 0.6f, amp = 0.15f;
        var axis = new IndiAxis(0.15f) { IncrementGain = 0.5f };
        float q = 0f, u = 0f, err2 = 0f, peak = 0f;
        int n = 0, satTicks = 0;
        var hist = new float[delay + 1];
        for (int i = 0; i < 2000; i++)
        {
            float t = i * dt;
            axis.Observe(q, u, model, dt, 5f);
            float qCmd = (int)(t / 3f) % 2 == 0 ? amp : -amp;
            u = axis.Command(qCmd, q, model, 3f, true, 0f);
            if (Math.Abs(u) > 0.99f) satTicks++;
            for (int k = delay; k > 0; k--) hist[k] = hist[k - 1];
            hist[0] = u;
            float demand = ratio * model * hist[delay];
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
}
