using System;

namespace AutopilotMod
{
    /// <summary>Radar coverage arithmetic free of Unity, so tests/SelfCheck can check it.</summary>
    internal static class CoverageMath
    {
        /// <summary>Twice the earth's radius (m): the game's radar horizon is sqrt(2R*h).</summary>
        public const float TwoR = 12742000f;

        /// <summary>
        /// Masking height per bin: the lowest altitude above sea level at which a target at the far edge
        /// of each range bin is in the radar's line of sight, from one sweep. The usual radial sweep
        /// keeps the steepest terrain slope seen from the radar; a target is visible when its own
        /// elevation clears it AND it is above the curved-earth radar horizon:
        ///     mask = max(radarAlt + maxSlope * d,  (d - sqrt(2R*radarAlt))^2 / 2R)
        /// Computed once per sweep, any target height is then one comparison (tgt >= mask), and the
        /// masking height itself is a map: how low you must fly here to stay unseen.
        /// </summary>
        /// <param name="height">terrain height (m ASL) at each bin's far edge, [az * steps]</param>
        /// <param name="before">in: steepest slope of samples inside each bin short of its far edge;
        /// out: the masking height (m ASL). Overwritten in place.</param>
        public static void MaskHeights(float[] height, float[] before, float radarAlt, float step, int az, int steps)
        {
            float radarHorizon = (float)Math.Sqrt(TwoR * Math.Max(0f, radarAlt));
            for (int a = 0; a < az; a++)
            {
                int b = a * steps;
                float maxSlope = float.NegativeInfinity;
                for (int i = 0; i < steps; i++)
                {
                    float d = (i + 1) * step;
                    maxSlope = Math.Max(maxSlope, before[b + i]);
                    float los = float.IsNegativeInfinity(maxSlope) ? float.NegativeInfinity : radarAlt + maxSlope * d;
                    float over = d - radarHorizon;
                    float horizon = over > 0f ? over * over / TwoR : float.NegativeInfinity;
                    before[b + i] = Math.Max(los, horizon);
                    maxSlope = Math.Max(maxSlope, (height[b + i] - radarAlt) / d);
                }
            }
        }

        /// <summary>
        /// Horizontal range (m) out to which a target of radar cross section <paramref name="rcs"/> can
        /// still beat the radar's minimum signal with no clutter (the game's signal = maxRange / dist x
        /// rcs^0.25, capped), and never past the 2 x maxRange the game checks.
        /// </summary>
        public static float DetectRange(float maxRange, float minSignal, float rcs)
        {
            float r = maxRange * (float)Math.Pow(Math.Max(rcs, 1e-6f), 0.25) / Math.Max(minSignal, 1e-6f);
            return Math.Min(r, 2f * maxRange);
        }
    }
}
