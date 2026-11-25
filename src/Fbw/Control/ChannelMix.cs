using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// From effector targets to the game's inputs.
    ///
    /// A control surface's deflection is the sum of three channels (pitch, roll, yaw), each with its own
    /// range and each slewed at the servo speed by the game's job (docs/fbw/PHASE0_FACTS.md section 3).
    /// To reach a target fastest, every channel with travel left moves the same way by the same amount
    /// (water-filling), so a surface with travel on all three channels slews at up to three servo speeds,
    /// as stock does when the pilot's inputs line up.
    ///
    /// A jet nozzle's pitch and yaw angles come from the engine's pitch and yaw inputs times fixed gains.
    /// </summary>
    internal static class ChannelMix
    {
        public const int Channels = 3;

        /// <summary>
        /// Channel inputs (as ControlInputs pitch/roll/yaw, −1..1) that move the surface toward
        /// <paramref name="target"/> degrees. <paramref name="range"/> and <paramref name="current"/> are per channel (deg).
        /// </summary>
        public static void Surface(float target, float[] range, float[] current, float[] inputs)
        {
            float total = 0f;
            for (int c = 0; c < Channels; c++) total += Math.Abs(range[c]);
            if (total <= 0f)
            {
                for (int c = 0; c < Channels; c++) inputs[c] = 0f;
                return;
            }
            target = Math.Max(-total, Math.Min(total, target));
            // the shared shift d with Σ clamp(current_c + d, ±|range_c|) = target: monotone in d, so bisect
            float lo = -2f * total, hi = 2f * total;
            for (int i = 0; i < 40; i++)
            {
                float mid = 0.5f * (lo + hi);
                if (Sum(range, current, mid) < target) lo = mid;
                else hi = mid;
            }
            float d = 0.5f * (lo + hi);
            for (int c = 0; c < Channels; c++)
            {
                float r = range[c];
                inputs[c] = r == 0f ? 0f : Clamp(current[c] + d, -Math.Abs(r), Math.Abs(r)) / r;
            }
        }

        /// <summary>The deflection (deg) the surface can reach this tick, down and up, from its channels' positions.</summary>
        public static void Reach(float[] range, float[] current, float servo, float dt, out float lo, out float hi)
        {
            float step = servo * dt, up = 0f, down = 0f, now = 0f;
            for (int c = 0; c < Channels; c++)
            {
                float r = Math.Abs(range[c]), x = current[c];
                now += x;
                if (r == 0f) continue;
                up += Math.Max(0f, Math.Min(step, r - x));
                down += Math.Max(0f, Math.Min(step, x + r));
            }
            lo = now - down;
            hi = now + up;
        }

        /// <summary>
        /// The engine pitch input that sets the nozzle's pitch angle to <paramref name="angle"/> (deg), with the game's
        /// mapping angle = −clamp(input × gain, ±1) × vectoring (Turbojet/Turbofan.FixedUpdate).
        /// </summary>
        public static float NozzlePitchInput(float angle, float vectoring, float gain)
        {
            if (vectoring == 0f || gain == 0f) return 0f;
            float s = Clamp(-angle / vectoring, -1f, 1f);
            return s / gain;
        }

        /// <summary>The engine yaw input for a nozzle yaw angle (deg): angle = clamp(−input, ±1) × vectoring.</summary>
        public static float NozzleYawInput(float angle, float vectoring) =>
            vectoring == 0f ? 0f : -Clamp(angle / vectoring, -1f, 1f);

        private static float Sum(float[] range, float[] current, float d)
        {
            float s = 0f;
            for (int c = 0; c < Channels; c++)
            {
                float r = Math.Abs(range[c]);
                if (r != 0f) s += Clamp(current[c] + d, -r, r);
            }
            return s;
        }

        private static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;
    }
}
