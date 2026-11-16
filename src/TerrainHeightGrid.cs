using System.Threading;
using Unity.Collections;
using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// Terrain height (m above sea level) on a square grid in global x/z, filled with batched
    /// multi-threaded raycasts against the same Statics layer the game's radar line-of-sight uses.
    /// One covers the whole map, once (terrain never changes, so every radar profile then reads it
    /// for free); smaller, finer ones are made around the radar map's view when zoomed in.
    /// </summary>
    internal sealed class TerrainHeightGrid
    {
        private const int BatchPerFrame = 8192;
        private const float TargetCell = 60f, QuarterMetre = 4f;

        public readonly int N;
        public readonly float Cell, CX, CZ, Size; // cell size, centre (global x/z) and edge length, m
        private readonly float half, contour;      // contour interval, m
        private readonly ushort[] height; // quarter-metres above sea level, so 2048^2 cells fit in 8 MB
        private int filled;

        public bool Ready => filled >= height.Length;
        public float Progress => filled / (float)height.Length;

        /// <summary>The whole map (centred on the global origin), 100 m contours.</summary>
        public TerrainHeightGrid(float mapSize) : this(0f, 0f, mapSize, Mathf.Clamp(Mathf.CeilToInt(mapSize / TargetCell), 64, 2048), 100f) { }

        public TerrainHeightGrid(float cx, float cz, float size, int n, float contourM)
        {
            N = n;
            CX = cx;
            CZ = cz;
            Size = size;
            Cell = size / N;
            half = size * 0.5f;
            contour = contourM;
            height = new ushort[N * N];
        }

        /// <summary>Raycast the next batch of cells. Completed in the same frame, so a
        /// floating-origin shift can't land between scheduling and reading the hits.</summary>
        public void Fill()
        {
            if (Ready) return;
            int count = Mathf.Min(BatchPerFrame, height.Length - filled);
            float sea = Datum.LocalSeaY;
            var query = new QueryParameters(PhysicsLayers.StaticsMask, false, QueryTriggerInteraction.Ignore, false);
            var cmds = new NativeArray<RaycastCommand>(count, Allocator.TempJob);
            var hits = new NativeArray<RaycastHit>(count, Allocator.TempJob);
            try
            {
                for (int k = 0; k < count; k++)
                {
                    int idx = filled + k;
                    Vector3 p = new GlobalPosition(CX + (idx % N + 0.5f) * Cell - half, 0f, CZ + (idx / N + 0.5f) * Cell - half).ToLocalPosition();
                    p.y = sea + 10000f;
                    cmds[k] = new RaycastCommand(p, Vector3.down, query, 10500f);
                }
                RaycastCommand.ScheduleBatch(cmds, hits, 64, 1).Complete();
                for (int k = 0; k < count; k++)
                {
                    float h = hits[k].collider != null ? hits[k].point.y - sea : 0f;
                    height[filled + k] = (ushort)Mathf.Clamp(Mathf.RoundToInt(h * QuarterMetre), 0, ushort.MaxValue);
                }
                filled += count;
            }
            finally
            {
                cmds.Dispose();
                hits.Dispose();
            }
        }

        private volatile Color32[] relief;
        private int reliefStarted;

        /// <summary>
        /// Relief shading of the grid for drawing over the map (row 0 = south): hillshade lit from the
        /// north-west plus contour lines (every 500 m bolder). Built once on a worker thread after
        /// Ready; null until then.
        /// </summary>
        public Color32[] Relief()
        {
            if (Ready && Interlocked.Exchange(ref reliefStarted, 1) == 0)
                ThreadPool.QueueUserWorkItem(_ =>
                {
                    try { relief = BuildRelief(); }
                    catch (System.Exception) { /* no shading; the map still draws */ }
                });
            return relief;
        }

        private Color32[] BuildRelief()
        {
            const float Exaggerate = 3f;
            float Contour = contour * QuarterMetre;
            int bold = Mathf.Max(1, Mathf.RoundToInt(500f / contour));
            var px = new Color32[N * N];
            float k = Exaggerate / (2f * Cell * QuarterMetre);
            for (int z = 0; z < N; z++)
            for (int x = 0; x < N; x++)
            {
                int i = z * N + x, h = height[i];
                if (h == 0) continue; // sea / off the map: leave the map image alone
                int e = x < N - 1 ? height[i + 1] : h, w = x > 0 ? height[i - 1] : h;
                int n = z < N - 1 ? height[i + N] : h, s = z > 0 ? height[i - N] : h;
                float gx = (e - w) * k, gz = (n - s) * k;
                // light from the north-west, 45 deg up: (-0.5, 0.707, 0.5); a flat cell scores 0
                float shade = (0.5f * gx - 0.5f * gz + 0.7071f) / Mathf.Sqrt(gx * gx + gz * gz + 1f) - 0.7071f;
                int band = (int)(h / Contour), be = (int)(e / Contour), bn = (int)(n / Contour);
                int level = band != be ? Mathf.Max(band, be) : bn != band ? Mathf.Max(band, bn) : 0; // contour crossed, in intervals
                if (level > 0) px[i] = new Color32(20, 12, 0, (byte)(level % bold == 0 ? 190 : 110));
                else if (shade < 0f) px[i] = new Color32(0, 0, 0, (byte)(Mathf.Clamp01(-shade * 1.3f) * 200f));
                else px[i] = new Color32(255, 250, 235, (byte)(Mathf.Clamp01(shade * 3f) * 90f));
            }
            return px;
        }

        /// <summary>Height above sea level at a global x/z, bilinear between cell centres; 0 off the map.</summary>
        public float Height(float gx, float gz)
        {
            float fx = (gx - CX + half) / Cell - 0.5f, fz = (gz - CZ + half) / Cell - 0.5f;
            if (fx < -0.5f || fz < -0.5f || fx > N - 0.5f || fz > N - 0.5f) return 0f;
            int ix = Mathf.Clamp((int)fx, 0, N - 2), iz = Mathf.Clamp((int)fz, 0, N - 2);
            float tx = Mathf.Clamp01(fx - ix), tz = Mathf.Clamp01(fz - iz);
            int i = iz * N + ix;
            float a = height[i] + (height[i + 1] - height[i]) * tx;
            float b = height[i + N] + (height[i + N + 1] - height[i + N]) * tx;
            return (a + (b - a) * tz) / QuarterMetre;
        }
    }
}
