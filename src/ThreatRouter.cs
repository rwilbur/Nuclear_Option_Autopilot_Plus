using System;
using System.Collections.Generic;

namespace AutopilotMod
{
    /// <summary>
    /// Least-exposure path over a square cost grid: A* on the 8-connected cells (a step costs its length times
    /// the mean cost per metre of the two cells, every cell >= 1), then pulled tight into a few turn points.
    /// Pure arithmetic (no Unity), so tests/SelfCheck runs it without the game.
    /// </summary>
    internal static class ThreatRouter
    {
        /// <summary>
        /// Turn points (global x, z) to fly from a to b, ending exactly at b. Cell (i, j) spans
        /// x0 + i*cell and z0 + j*cell, cost[j*n + i]; points off the grid are clamped onto it.
        /// </summary>
        public static List<float[]> Path(float[] cost, int n, float x0, float z0, float cell, float ax, float az, float bx, float bz)
        {
            int Cell(float x, float z) =>
                Clamp((int)Math.Floor((z - z0) / cell), 0, n - 1) * n + Clamp((int)Math.Floor((x - x0) / cell), 0, n - 1);
            int start = Cell(ax, az), goal = Cell(bx, bz);
            var g = new float[n * n];
            var from = new int[n * n];
            for (int k = 0; k < g.Length; k++) g[k] = float.PositiveInfinity;
            g[start] = 0f;
            from[start] = -1;
            var heap = new MinHeap();
            heap.Push(start, 0f);
            int gx = goal % n, gz = goal / n;
            while (heap.Count > 0)
            {
                int u = heap.Pop(out float fu);
                if (u == goal) break;
                int ux = u % n, uz = u / n;
                // stale heap entry: u was reached more cheaply after it was pushed
                if (fu > g[u] + Octile(ux, uz, gx, gz) * cell + 1e-3f) continue;
                for (int dz = -1; dz <= 1; dz++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int vx = ux + dx, vz = uz + dz;
                        if ((dx == 0 && dz == 0) || vx < 0 || vz < 0 || vx >= n || vz >= n) continue;
                        int v = vz * n + vx;
                        float step = (dx != 0 && dz != 0 ? 1.41421356f : 1f) * cell * 0.5f * (cost[u] + cost[v]);
                        if (g[u] + step >= g[v]) continue;
                        g[v] = g[u] + step;
                        from[v] = u;
                        heap.Push(v, g[v] + Octile(vx, vz, gx, gz) * cell); // octile distance x min cost 1: never overestimates
                    }
            }

            // cell chain start -> goal, with the real end points in place of their cells' centres
            var chain = new List<int>();
            for (int k = goal; k >= 0; k = from[k]) chain.Add(k);
            chain.Reverse();
            var px = new float[chain.Count];
            var pz = new float[chain.Count];
            for (int k = 0; k < chain.Count; k++)
            {
                px[k] = x0 + (chain[k] % n + 0.5f) * cell;
                pz[k] = z0 + (chain[k] / n + 0.5f) * cell;
            }
            px[0] = ax; pz[0] = az;
            px[chain.Count - 1] = bx; pz[chain.Count - 1] = bz;
            // the chain's cost to each point, measured on these points (not A*'s cell centres) so it compares with SegCost
            var run = new float[chain.Count];
            for (int k = 1; k < chain.Count; k++)
                run[k] = run[k - 1] + (float)Math.Sqrt((px[k] - px[k - 1]) * (px[k] - px[k - 1]) + (pz[k] - pz[k - 1]) * (pz[k] - pz[k - 1]))
                    * 0.5f * (cost[chain[k - 1]] + cost[chain[k]]);

            // Pull tight: from each turn point, go straight to the farthest chain point whose straight line
            // costs no more than the chain does to there (plus half a cell of slack for the grid's zig-zag).
            float SegCost(int i, int j)
            {
                float lx = px[j] - px[i], lz = pz[j] - pz[i], len = (float)Math.Sqrt(lx * lx + lz * lz);
                int steps = Math.Max(1, (int)Math.Ceiling(len / (cell * 0.5f)));
                float sum = 0f;
                for (int s = 0; s < steps; s++)
                {
                    float t = (s + 0.5f) / steps;
                    sum += cost[Cell(px[i] + lx * t, pz[i] + lz * t)];
                }
                return sum * len / steps;
            }
            var turns = new List<float[]>();
            int at = 0, last = chain.Count - 1;
            while (at < last)
            {
                int next = at + 1;
                while (next < last && SegCost(at, next + 1) <= run[next + 1] - run[at] + cell * 0.5f) next++;
                turns.Add(new[] { px[next], pz[next] });
                at = next;
            }
            if (turns.Count == 0) turns.Add(new[] { bx, bz });
            return turns;
        }

        private static float Octile(int ax, int az, int bx, int bz)
        {
            int dx = Math.Abs(ax - bx), dz = Math.Abs(az - bz);
            return Math.Max(dx, dz) + 0.41421356f * Math.Min(dx, dz);
        }

        private static int Clamp(int v, int lo, int hi) => v < lo ? lo : v > hi ? hi : v;

        /// <summary>Binary heap of (cell, priority), smallest priority first; duplicates allowed (stale ones are skipped).</summary>
        private sealed class MinHeap
        {
            private int[] id = new int[1024];
            private float[] key = new float[1024];
            public int Count;

            public void Push(int v, float k)
            {
                if (Count == id.Length)
                {
                    Array.Resize(ref id, Count * 2);
                    Array.Resize(ref key, Count * 2);
                }
                int i = Count++;
                while (i > 0)
                {
                    int p = (i - 1) / 2;
                    if (key[p] <= k) break;
                    id[i] = id[p];
                    key[i] = key[p];
                    i = p;
                }
                id[i] = v;
                key[i] = k;
            }

            public int Pop(out float k)
            {
                int top = id[0];
                k = key[0];
                int lastId = id[--Count];
                float lastKey = key[Count];
                int i = 0;
                while (true)
                {
                    int c = 2 * i + 1;
                    if (c >= Count) break;
                    if (c + 1 < Count && key[c + 1] < key[c]) c++;
                    if (key[c] >= lastKey) break;
                    id[i] = id[c];
                    key[i] = key[c];
                    i = c;
                }
                id[i] = lastId;
                key[i] = lastKey;
                return top;
            }
        }
    }
}
