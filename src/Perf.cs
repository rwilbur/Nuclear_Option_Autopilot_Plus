using System;
using System.Diagnostics;
using System.Threading;
using BepInEx.Logging;
using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// Where the mod spends CPU time: its entry points timed with a Stopwatch, averaged per frame over
    /// the last second (DATA page), and a breakdown logged every 10 s while the DATA page is on.
    /// Radar is part of Update and Mirror part of Gui. Encode runs on the mirror's own threads: not
    /// frame time, but a core the game can't use. GPU and render-thread work isn't in any of these:
    /// if the frame time grows by much more than the mod's total, that's where it went.
    /// </summary>
    internal static class Perf
    {
        public const int Update = 0, Radar = 1, Gui = 2, Mirror = 3, Autopilot = 4, Encode = 5, Sweep = 6, Raster = 7, Terrain = 8;
        private static readonly string[] Names = { "update", "radar tick", "OnGUI", "mirror render", "autopilot", "jpeg encode (worker)",
            "radar: sweeps", "radar: overlay", "radar: terrain rays" };

        /// <summary>Radar profile sweeps and coverage overlays finished since the last log line.</summary>
        public static int Sweeps, Builds, Viewsheds;
        private static readonly System.Collections.Generic.Dictionary<string, int> triggers = new System.Collections.Generic.Dictionary<string, int>();

        /// <summary>Count what started a coverage overlay build. Main thread.</summary>
        public static void Trigger(string why) => triggers[why] = triggers.TryGetValue(why, out int n) ? n + 1 : 1;
        private static readonly long[] ticks = new long[Names.Length];
        private static readonly double[] logMs = new double[Names.Length];

        /// <summary>ms per frame over the last second, per slot.</summary>
        public static readonly float[] Ms = new float[Names.Length];
        public static float ModMs, FrameMs;

        private static int frames, seconds, gcAtLog = GC.CollectionCount(0);
        private static float frameTime, next, logFrameMs;

        public static long Now => Stopwatch.GetTimestamp();

        /// <summary>Add the time since <paramref name="since"/> (from Now) to a slot. Any thread.</summary>
        public static void Add(int slot, long since) => Interlocked.Add(ref ticks[slot], Stopwatch.GetTimestamp() - since);

        /// <summary>Call once per frame, from Update.</summary>
        public static void Frame(ManualLogSource log, bool logging)
        {
            frames++;
            frameTime += Time.unscaledDeltaTime;
            if (Time.unscaledTime < next) return;
            next = Time.unscaledTime + 1f;

            double k = 1000.0 / Stopwatch.Frequency / frames;
            for (int i = 0; i < Ms.Length; i++)
            {
                Ms[i] = (float)(Interlocked.Exchange(ref ticks[i], 0) * k);
                logMs[i] += Ms[i];
            }
            ModMs = Ms[Update] + Ms[Gui] + Ms[Autopilot];
            FrameMs = frameTime * 1000f / frames;
            logFrameMs += FrameMs;
            frames = 0;
            frameTime = 0f;

            if (++seconds < 10) return;
            if (logging)
            {
                var sb = new System.Text.StringBuilder($"Perf, ms per frame over {seconds} s: frame {logFrameMs / seconds:0.00}");
                for (int i = 0; i < Names.Length; i++) sb.Append($", {Names[i]} {logMs[i] / seconds:0.00}");
                int gc = GC.CollectionCount(0);
                sb.Append($"; {Sweeps} radar sweeps, {Builds} overlays, {Viewsheds} viewsheds; overlay triggers:");
                foreach (var kv in triggers) sb.Append($" {kv.Key} x{kv.Value}");
                sb.Append($"; {gc - gcAtLog} GCs, heap {GC.GetTotalMemory(false) / 1048576} MB");
                log.LogInfo(sb.ToString());
            }
            seconds = Sweeps = Builds = Viewsheds = 0;
            triggers.Clear();
            logFrameMs = 0f;
            Array.Clear(logMs, 0, logMs.Length);
            gcAtLog = GC.CollectionCount(0);
        }
    }
}
