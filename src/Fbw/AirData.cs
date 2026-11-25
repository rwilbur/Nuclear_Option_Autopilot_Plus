using System;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// Flow angles and body rates in aerospace sign conventions, from Unity body-frame vectors
    /// (x right, y up, z forward; left-handed, so +x rotation is nose-down and +z is roll-left).
    /// Pure maths: no Unity types, so the offline checks run it.
    /// </summary>
    internal static class AirData
    {
        public const float Rad2Deg = 57.29578f;

        /// <summary>Angle of attack (rad), positive nose-up: the air arrives from below, so body velocity has -y.</summary>
        public static float Alpha(float vx, float vy, float vz) => (float)Math.Atan2(-vy, vz);

        /// <summary>Sideslip (rad), positive with the air from the right: body velocity has +x.</summary>
        public static float Beta(float vx, float vy, float vz)
        {
            double v = Math.Sqrt(vx * vx + vy * vy + vz * vz);
            return v > 1e-3 ? (float)Math.Asin(Math.Max(-1.0, Math.Min(1.0, vx / v))) : 0f;
        }

        /// <summary>Roll rate p (rad/s), positive right wing down, from Unity's body angular velocity.</summary>
        public static float P(float wz) => -wz;

        /// <summary>Pitch rate q (rad/s), positive nose-up.</summary>
        public static float Q(float wx) => -wx;

        /// <summary>Yaw rate r (rad/s), positive nose-right.</summary>
        public static float R(float wy) => wy;
    }
}
