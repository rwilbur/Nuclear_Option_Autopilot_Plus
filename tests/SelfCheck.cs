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
}
