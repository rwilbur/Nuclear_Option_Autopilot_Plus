using System;
using System.Collections.Generic;

namespace AutopilotMod
{
    /// <summary>
    /// Radar-avoiding route planner: Lazy Theta* (Nash, Daniel and Koenig 2010) over a grid of map
    /// cells x height bands. Pure arithmetic on arrays, so it runs on a worker thread and in tests.
    ///
    /// Inputs per cell: Hide = the highest height above ground (m) at which no known enemy radar can see
    /// you there (+inf = none can at any height). A cell is EXPOSED in a band when band height >= Hide.
    /// Cost of flying 1 m = (1 + LowPenalty * bands below the top + ExposureWeight if exposed) / Speed,
    /// i.e. seconds, where a second of exposure counts as ExposureWeight seconds of extra flying, and
    /// low flight costs a little (fuel, risk) so the route only goes low where it buys concealment.
    /// Changing band costs BandChangeSecondsPerM per metre of height.
    ///
    /// Any-angle: like Theta*, a node may take its parent's parent as its own parent when the straight
    /// line between them is clear, so routes are straight lines between few corners instead of
    /// 8-direction zigzags. "Lazy": that line is only checked when the node is expanded. Here "clear"
    /// means unexposed along its whole length in that band (the cost is then exactly length x base
    /// cost); through exposed cells the search falls back to exact grid edges.
    /// </summary>
    internal sealed class RoutePlanner
    {
        public int N;                         // cells per side
        public float OriginX, OriginZ, Cell;  // cell (0,0)'s south-west corner (global m), cell size (m)
        public float[] Hide;                  // N*N, m above ground
        public float[] Bands = { 30f, 150f, 500f };
        public float Speed = 200f;
        public float ExposureWeight = 30f;
        public float LowPenalty = 0.15f;
        public float BandChangeSecondsPerM = 0.05f;
        public float MinLeg = 1500f;          // shorter legs are merged away when the line allows it

        /// <summary>Planner output: seconds of exposure along the route (at its band heights).</summary>
        public float ExposedSeconds;
        public int Expanded;

        private float[] g;
        private int[] parent;
        private bool[] closed;
        private int nb, nc;

        public bool InGrid(float x, float z, out int cx, out int cz)
        {
            cx = (int)Math.Floor((x - OriginX) / Cell);
            cz = (int)Math.Floor((z - OriginZ) / Cell);
            return cx >= 0 && cz >= 0 && cx < N && cz < N;
        }

        private float CenterX(int c) => OriginX + (c % N + 0.5f) * Cell;
        private float CenterZ(int c) => OriginZ + (c / N + 0.5f) * Cell;

        private bool Exposed(int cell, int band) => Bands[band] >= Hide[cell];

        private float BaseRate(int band) => (1f + LowPenalty * (nb - 1 - band)) / Speed;

        private float Rate(int cell, int band) => BaseRate(band) + (Exposed(cell, band) ? ExposureWeight / Speed : 0f);

        /// <summary>
        /// Plan from (sx,sz) to (gx,gz). Returns the waypoints after the start (the last one is the
        /// goal), each with the band height of the leg ending there; null if either end is off the grid.
        /// </summary>
        public List<Waypoint> Plan(float sx, float sz, float gx, float gz)
        {
            nb = Bands.Length;
            nc = N * N;
            if (Hide == null || Hide.Length != nc || nb == 0) return null;
            if (!InGrid(sx, sz, out int scx, out int scz) || !InGrid(gx, gz, out int gcx, out int gcz)) return null;
            int startCell = scz * N + scx, goalCell = gcz * N + gcx;

            int ns = nc * nb;
            g = new float[ns];
            parent = new int[ns];
            closed = new bool[ns];
            for (int i = 0; i < ns; i++)
            {
                g[i] = float.PositiveInfinity;
                parent[i] = -1;
            }
            var open = new MinHeap(4096);
            float hScale = BaseRate(nb - 1); // cheapest possible cost per metre: admissible heuristic
            float H(int s) => Dist(s % nc, goalCell) * hScale;

            for (int b = 0; b < nb; b++)
            {
                int s = b * nc + startCell;
                g[s] = 0f;
                parent[s] = s;
                open.Push(s, H(s));
            }

            int goal = -1;
            Expanded = 0;
            while (open.Count > 0)
            {
                open.Pop(out int s, out float f);
                if (closed[s]) continue;
                if (f > g[s] + H(s) + 1e-3f) continue; // stale heap entry
                SetVertex(s);
                closed[s] = true;
                Expanded++;
                int cell = s % nc, band = s / nc;
                if (cell == goalCell)
                {
                    goal = s;
                    break;
                }
                int cx = cell % N, cz = cell / N;
                int ps = parent[s];
                bool shortcut = ps != s && ps / nc == band;

                for (int dz = -1; dz <= 1; dz++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dz == 0) continue;
                    int x = cx + dx, z = cz + dz;
                    if (x < 0 || z < 0 || x >= N || z >= N) continue;
                    int nCell = z * N + x, n = band * nc + nCell;
                    if (closed[n]) continue;
                    float cand;
                    int par;
                    if (shortcut && !Exposed(nCell, band))
                    {
                        // lazy: assume the line from our parent is clear; SetVertex checks on expansion
                        par = ps;
                        cand = g[ps] + Dist(ps % nc, nCell) * BaseRate(band);
                    }
                    else
                    {
                        par = s;
                        cand = g[s] + EdgeCost(cell, nCell, band);
                    }
                    if (cand < g[n])
                    {
                        g[n] = cand;
                        parent[n] = par;
                        open.Push(n, cand + H(n));
                    }
                }
                for (int db = -1; db <= 1; db += 2)
                {
                    int b2 = band + db;
                    if (b2 < 0 || b2 >= nb) continue;
                    int n = b2 * nc + cell;
                    if (closed[n]) continue;
                    float cand = g[s] + Math.Abs(Bands[b2] - Bands[band]) * BandChangeSecondsPerM;
                    if (cand < g[n])
                    {
                        g[n] = cand;
                        parent[n] = s;
                        open.Push(n, cand + H(n));
                    }
                }
            }
            if (goal < 0) return null;

            // walk back to the start, then turn states into legs
            var states = new List<int>();
            for (int s = goal; ; s = parent[s])
            {
                states.Add(s);
                if (parent[s] == s) break;
            }
            states.Reverse();

            var pts = new List<Waypoint>();
            float px = sx, pz = sz;
            ExposedSeconds = 0f;
            for (int i = 1; i < states.Count; i++)
            {
                int s = states[i], cell = s % nc, band = s / nc;
                if (cell == states[i - 1] % nc) continue; // band change in place: the next leg flies the new band
                bool last = i == states.Count - 1;
                float x = last ? gx : CenterX(cell), z = last ? gz : CenterZ(cell);
                ExposedSeconds += ExposedTime(px, pz, x, z, band);
                pts.Add(new Waypoint(x, z, Bands[band]));
                px = x;
                pz = z;
            }
            if (pts.Count == 0) pts.Add(new Waypoint(gx, gz, Bands[states[states.Count - 1] / nc]));
            else
            {
                Waypoint w = pts[pts.Count - 1];
                pts[pts.Count - 1] = new Waypoint(gx, gz, w.Agl);
            }
            return Simplify(pts, sx, sz);
        }

        /// <summary>Lazy Theta* check on expansion: if the assumed line to the parent is blocked, take
        /// the best already-expanded neighbour instead.</summary>
        private void SetVertex(int s)
        {
            int ps = parent[s];
            if (ps == s) return;
            int cell = s % nc, band = s / nc, pcell = ps % nc;
            if (ps / nc != band || Adjacent(pcell, cell) || LineClear(pcell, cell, band)) return;

            float best = float.PositiveInfinity;
            int bestP = -1;
            int cx = cell % N, cz = cell / N;
            for (int dz = -1; dz <= 1; dz++)
            for (int dx = -1; dx <= 1; dx++)
            {
                if (dx == 0 && dz == 0) continue;
                int x = cx + dx, z = cz + dz;
                if (x < 0 || z < 0 || x >= N || z >= N) continue;
                int n = band * nc + z * N + x;
                if (!closed[n]) continue;
                float c = g[n] + EdgeCost(n % nc, cell, band);
                if (c < best)
                {
                    best = c;
                    bestP = n;
                }
            }
            for (int db = -1; db <= 1; db += 2)
            {
                int b2 = band + db;
                if (b2 < 0 || b2 >= nb) continue;
                int n = b2 * nc + cell;
                if (!closed[n]) continue;
                float c = g[n] + Math.Abs(Bands[b2] - Bands[band]) * BandChangeSecondsPerM;
                if (c < best)
                {
                    best = c;
                    bestP = n;
                }
            }
            if (bestP >= 0)
            {
                parent[s] = bestP;
                g[s] = best;
            }
        }

        private bool Adjacent(int a, int b) => Math.Abs(a % N - b % N) <= 1 && Math.Abs(a / N - b / N) <= 1;

        private float Dist(int a, int b)
        {
            float dx = (a % N - b % N) * Cell, dz = (a / N - b / N) * Cell;
            return (float)Math.Sqrt(dx * dx + dz * dz);
        }

        private float EdgeCost(int a, int b, int band) => Dist(a, b) * 0.5f * (Rate(a, band) + Rate(b, band));

        /// <summary>True when every cell the segment between two cell centres crosses is unexposed.</summary>
        private bool LineClear(int a, int b, int band)
        {
            float ax = a % N + 0.5f, az = a / N + 0.5f, bx = b % N + 0.5f, bz = b / N + 0.5f;
            float len = (float)Math.Sqrt((bx - ax) * (bx - ax) + (bz - az) * (bz - az));
            int steps = Math.Max(1, (int)Math.Ceiling(len * 3f));
            for (int i = 0; i <= steps; i++)
            {
                float t = i / (float)steps;
                int x = (int)(ax + (bx - ax) * t), z = (int)(az + (bz - az) * t);
                if (x < 0 || z < 0 || x >= N || z >= N) return false;
                if (Exposed(z * N + x, band)) return false;
            }
            return true;
        }

        private bool LineClearWorld(float x0, float z0, float x1, float z1, int band)
        {
            if (!InGrid(x0, z0, out int ax, out int az) || !InGrid(x1, z1, out int bx, out int bz)) return false;
            return LineClear(az * N + ax, bz * N + bx, band);
        }

        /// <summary>Seconds spent exposed flying a straight segment in a band.</summary>
        private float ExposedTime(float x0, float z0, float x1, float z1, int band)
        {
            float len = (float)Math.Sqrt((x1 - x0) * (x1 - x0) + (z1 - z0) * (z1 - z0));
            int steps = Math.Max(1, (int)Math.Ceiling(len / (Cell * 0.5f)));
            int exposed = 0;
            for (int i = 0; i < steps; i++)
            {
                float t = (i + 0.5f) / steps;
                if (InGrid(x0 + (x1 - x0) * t, z0 + (z1 - z0) * t, out int cx, out int cz) && Exposed(cz * N + cx, band)) exposed++;
            }
            return len * exposed / steps / Speed;
        }

        /// <summary>
        /// Smoothing: drop corners that make a leg shorter than MinLeg, or that are nearly straight,
        /// when the leg that replaces them is unexposed in that band (or the corner sat in the same
        /// exposure anyway). Fewer, longer legs are what an aircraft can actually fly.
        /// </summary>
        private List<Waypoint> Simplify(List<Waypoint> pts, float sx, float sz)
        {
            bool changed = true;
            while (changed && pts.Count > 1)
            {
                changed = false;
                for (int i = 0; i < pts.Count - 1; i++)
                {
                    float x0 = i == 0 ? sx : pts[i - 1].X, z0 = i == 0 ? sz : pts[i - 1].Z;
                    Waypoint w = pts[i], nx = pts[i + 1];
                    if (w.Agl != nx.Agl) continue; // band changes stay where they are
                    float l1 = Len(x0, z0, w.X, w.Z), l2 = Len(w.X, w.Z, nx.X, nx.Z);
                    float turn = Math.Abs(DeltaDeg(L1Guidance.Course(x0, z0, w.X, w.Z), L1Guidance.Course(w.X, w.Z, nx.X, nx.Z)));
                    if ((l1 < MinLeg || l2 < MinLeg || turn < 4f) && LineClearWorld(x0, z0, nx.X, nx.Z, BandIndex(nx.Agl)))
                    {
                        pts.RemoveAt(i);
                        changed = true;
                        break;
                    }
                }
            }
            return pts;
        }

        private int BandIndex(float agl)
        {
            for (int b = 0; b < Bands.Length; b++) if (Bands[b] == agl) return b;
            return Bands.Length - 1;
        }

        private static float Len(float x0, float z0, float x1, float z1) => (float)Math.Sqrt((x1 - x0) * (x1 - x0) + (z1 - z0) * (z1 - z0));

        private static float DeltaDeg(float a, float b)
        {
            float d = (b - a) % 360f;
            if (d > 180f) d -= 360f;
            if (d < -180f) d += 360f;
            return d;
        }

        /// <summary>Binary min-heap of (state, key) with lazy deletion (stale entries are skipped).</summary>
        private sealed class MinHeap
        {
            private int[] ids;
            private float[] keys;
            public int Count;

            public MinHeap(int cap)
            {
                ids = new int[cap];
                keys = new float[cap];
            }

            public void Push(int id, float key)
            {
                if (Count == ids.Length)
                {
                    Array.Resize(ref ids, Count * 2);
                    Array.Resize(ref keys, Count * 2);
                }
                int i = Count++;
                while (i > 0)
                {
                    int p = (i - 1) / 2;
                    if (keys[p] <= key) break;
                    ids[i] = ids[p];
                    keys[i] = keys[p];
                    i = p;
                }
                ids[i] = id;
                keys[i] = key;
            }

            public void Pop(out int id, out float key)
            {
                id = ids[0];
                key = keys[0];
                int lastId = ids[--Count];
                float lastKey = keys[Count];
                int i = 0;
                while (true)
                {
                    int c = 2 * i + 1;
                    if (c >= Count) break;
                    if (c + 1 < Count && keys[c + 1] < keys[c]) c++;
                    if (keys[c] >= lastKey) break;
                    ids[i] = ids[c];
                    keys[i] = keys[c];
                    i = c;
                }
                ids[i] = lastId;
                keys[i] = lastKey;
            }
        }
    }
}
