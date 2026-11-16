using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using BepInEx.Configuration;
using HarmonyLib;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.UI;

namespace AutopilotMod
{
    /// <summary>
    /// Radar-coverage MFD page over the game's own map image, north-up. ALL shows every friendly
    /// radar; OWN shows only your jet's radar inside its scan cone. Green = the game would detect a
    /// target at the TGT height above the ground there: inside the radar's cone, in line of sight,
    /// above the radar horizon and with enough signal after ground clutter. Red = strong enough
    /// signal but hidden by terrain, the horizon or clutter. No colour = signal too weak.
    /// ALL/OWN default to a reference target of RCS 1. Red triangles/squares = enemy contacts the game map
    /// shows (same map filters), pointing along their map heading.
    /// ENMY shows enemy radars at the positions your side knows (its tracking data), for a target
    /// of the RCS set (default: your own jet's), colours swapped: red = the enemy sees you there,
    /// green = hidden. A jammed radar (Radar.IsJammed: it detects nothing) adds no coverage; its
    /// site turns amber. Doppler and ECM are left out: without ECM they can't change the result.
    ///
    ///            T1 ALL   T2 OWN   T3 ENMY  T4 TGT   T5 HIDE
    ///   L1 FOLW  +--------------------------------------+  R1 NAV
    ///   L2 MAP   |                                      |  R2 ^ (zoom in)
    ///   L3 ^RCS  |  map + coverage (scroll = zoom,      |  R3 RNG
    ///   L4 RCS   |  drag = pan (on a waypoint: move it), right-click = waypoint) |  R4 v (zoom out)
    ///   L5 vRCS  +--------------------------------------+  R5 CLR WPT
    ///   (L4 = back to own RCS)
    ///
    /// Under the coverage, the terrain is shaded (hillshade + 100 m contours) from the height grid.
    ///
    /// The map's terrain is raycast once into a TerrainHeightGrid. Each radar's polar profile
    /// (Az bearings x Steps range bins) is read from that grid over a few frames, and redone when
    /// the radar moves 2% of its range. The overlay is redrawn only when something changed, a few
    /// radars per frame.
    /// </summary>
    internal sealed class RadarCoveragePage : MfdFrame
    {
        public enum Mode { All, Own, Enemy }

        private const int Az = 720, Steps = 800, Tex = 1024;
        private const float MinRange = 2000f;
        private static readonly float[] AglsM = { 5f, 30f, 150f, 500f, 2000f, -1f }; // -1 = ALT: own altitude
        private const float MinAgl = 5f;

        private static readonly AccessTools.FieldRef<Radar, float> RadarConeRef =
            AccessTools.FieldRefAccess<Radar, float>("radarCone");

        private static readonly Color32 Covered = new Color32(60, 230, 90, 95);
        private static readonly Color32 Masked = new Color32(240, 50, 40, 130);
        private static readonly Color32 None = new Color32(0, 0, 0, 0);
        private static readonly Color SiteCol = new Color(0.4f, 0.75f, 1f, 1f);
        private static readonly Color EnemySiteCol = new Color(1f, 0.3f, 0.25f, 1f);
        private static readonly Color LegendBack = new Color(0f, 0f, 0f, 0.6f);

        private static readonly Osb[] RdrButtons =
        {
            new Osb { Side = Side.Top, Index = 0, Command = ApCommand.RdrAll, Glyph = Glyph.Text },
            new Osb { Side = Side.Top, Index = 1, Command = ApCommand.RdrOwn, Glyph = Glyph.Text },
            new Osb { Side = Side.Top, Index = 2, Command = ApCommand.RdrEnemy, Glyph = Glyph.Text },
            new Osb { Side = Side.Top, Index = 3, Command = ApCommand.RdrTgt, Glyph = Glyph.Text },
            new Osb { Side = Side.Top, Index = 4, Command = ApCommand.RdrHide, Glyph = Glyph.Text },
            new Osb { Side = Side.Left, Index = 0, Command = ApCommand.RdrFollow, Glyph = Glyph.Text },
            new Osb { Side = Side.Left, Index = 1, Command = ApCommand.RdrMap, Glyph = Glyph.Text },
            new Osb { Side = Side.Left, Index = 2, Command = ApCommand.RdrRcsUp, Glyph = Glyph.Up },
            new Osb { Side = Side.Left, Index = 3, Command = ApCommand.RdrRcsOwn, Glyph = Glyph.Text },
            new Osb { Side = Side.Left, Index = 4, Command = ApCommand.RdrRcsDown, Glyph = Glyph.Down },
            new Osb { Side = Side.Right, Index = 1, Command = ApCommand.RdrZoomIn, Glyph = Glyph.Up },
            new Osb { Side = Side.Right, Index = 3, Command = ApCommand.RdrZoomOut, Glyph = Glyph.Down },
            new Osb { Side = Side.Right, Index = 0, Command = ApCommand.ModeNav, Glyph = Glyph.Text },
            new Osb { Side = Side.Right, Index = 4, Command = ApCommand.WptClear, Glyph = Glyph.Text },
        };

        protected override Osb[] Buttons => RdrButtons;

        private sealed class Profile
        {
            public TargetDetector Radar;
            public GlobalPosition Origin;     // scan point the profile was taken from
            public float Range, Alt;          // Alt = scan point height above sea level
            public bool Valid, Jammed, Enemy;
            public float ConeHeading = float.NaN; // enemy: heading off its map icon (deg); NaN = unknown
            public float ConePitch = float.NaN;   // enemy: antenna pitch while that heading is live, then the last one (deg); NaN = never known
            public float[] Height; // terrain height above sea level at each bin's far edge (m); null until the first sweep is done
            public float[] Before; // steepest terrain slope seen from the radar inside the bin, short of its far edge
            public readonly byte[] Vis = new byte[Az * Steps];
            public float VisAgl = float.NaN, VisRcs, VisRadius; // target Vis was computed for (VisAgl: height key); NaN = stale

            // Sweep in progress: written into back buffers over several frames, then swapped in.
            public GlobalPosition NextOrigin;
            public float NextRange, NextAlt;
            public int RaysDone = Az;
            public float[] NextHeight, NextBefore; // rented from `spare` only while a sweep runs
        }

        // Back buffers are shared: a radar holds a pair only during its sweep, which halves the
        // memory the profiles keep (about 5 MB per radar instead of 10 MB).
        private static readonly Stack<float[]> spare = new Stack<float[]>();

        private static float[] Rent() => spare.Count > 0 ? spare.Pop() : new float[Az * Steps];

        private const int SampleBudget = 50000; // terrain lookups per frame for profile sweeps

        private readonly ApSettings cfg;
        private readonly FlightController ap;
        private Texture2D relief;
        // Zoomed in past the map grid's detail: a finer grid around the view (DetailSpan view half-widths
        // across), raycast over a second or so; redone once the view moves or zooms away from it.
        private const int DetailN = 768;
        private const float DetailSpan = 3f;
        private TerrainHeightGrid detail, detailNext;
        private Texture2D detailTex;
        private static readonly Color NavCol = new Color(1f, 0.45f, 1f, 1f), NavDim = new Color(1f, 0.45f, 1f, 0.55f);
        private readonly Dictionary<TargetDetector, Profile> profiles = new Dictionary<TargetDetector, Profile>();
        private TerrainHeightGrid grid;
        private float nextScan, nextBuild;
        private bool dirty = true;
        private string dirtyWhy = "start"; // what set `dirty` since the last build, for the Perf log

        private void Dirty(string why)
        {
            if (!dirty) dirtyWhy = why;
            else if (!dirtyWhy.Contains(why)) dirtyWhy += "+" + why;
            dirty = true;
        }

        // The overlay is rasterised a few radars per frame into `state`, then uploaded in one go.
        private const int PixelBudget = 160000;
        private const int ViewshedCost = 80000;     // one line-of-sight pass, in pixels of that budget
        private readonly List<Profile> building = new List<Profile>();
        private int buildIdx;
        private bool buildActive;
        private float bX, bZ, bR;                    // view being built
        private bool bEnemy;

        private readonly byte[] state = new byte[Tex * Tex]; // 0 none, 1 masked, 2 covered
        private readonly Color32[] pixels = new Color32[Tex * Tex];
        private Texture2D overlay;

        // Enemy contacts as the game map shows them, re-filtered a few times a second.
        private struct Contact
        {
            public Unit Unit;
            public TrackingInfo Track;
            public Image Icon;       // the map icon's image: sprite, colour and rotation (= map heading)
            public bool Orient;
        }
        private readonly List<Contact> contacts = new List<Contact>();
        private float nextContacts;
        private static readonly Dictionary<Texture2D, Texture2D> iconAlpha = new Dictionary<Texture2D, Texture2D>();

        /// <summary>
        /// Most map icons are DXT1 (white shape on opaque black; the map's shader hides the black).
        /// Bake a copy once per texture: white, with the shape's brightness as alpha.
        /// </summary>
        private static void BakeIcon(Texture2D src)
        {
            if (src == null || iconAlpha.ContainsKey(src)) return;
            int w = src.width, h = src.height;
            RenderTexture rt = RenderTexture.GetTemporary(w, h, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            Graphics.Blit(src, rt); // GPU copy: the source isn't CPU-readable
            RenderTexture prev = RenderTexture.active;
            RenderTexture.active = rt;
            var t = new Texture2D(w, h, TextureFormat.RGBA32, true) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            t.ReadPixels(new Rect(0, 0, w, h), 0, 0);
            RenderTexture.active = prev;
            RenderTexture.ReleaseTemporary(rt);
            Color32[] px = t.GetPixels32();
            for (int i = 0; i < px.Length; i++)
            {
                Color32 c = px[i];
                px[i] = new Color32(255, 255, 255, (byte)(Mathf.Max(c.r, Mathf.Max(c.g, c.b)) * c.a / 255));
            }
            int id = src.GetInstanceID();
            ThreadPool.QueueUserWorkItem(_ => // for the browser-drawn map (/icon)
            {
                try { iconPng[id] = ImageConversion.EncodeArrayToPNG(px, GraphicsFormat.R8G8B8A8_UNorm, (uint)w, (uint)h); }
                catch (Exception) { /* that icon is left out of the web map */ }
            });
            t.SetPixels32(px);
            t.Apply(true, true); // mipmaps keep small icons smooth
            iconAlpha[src] = t;
        }

        private static readonly ConcurrentDictionary<int, byte[]> iconPng = new ConcurrentDictionary<int, byte[]>();

        /// <summary>A baked map icon (white, shape as alpha) as PNG, by its texture's instance id; null if unknown. Any thread.</summary>
        public static byte[] IconPng(int id) => iconPng.TryGetValue(id, out byte[] b) ? b : null;

        private MapSettings mapSettings;
        private Aircraft me;

        public Mode Current = Mode.All;
        public bool Visible;                         // drawn on the game screen (the web mirror shows it regardless)
        private bool follow = true, panning;
        private float viewX, viewZ, viewR = 40000f;  // view centre (global x/z) and half-width (m)
        private float texX, texZ, texR;              // view the overlay texture was built for
        private Vector2 lastMouse;
        private int draggedWp = -1;                  // waypoint being dragged with the left button, or -1
        private bool ownRadarMissing;
        private int siteCount, jammedCount;

        public RadarCoveragePage(ApSettings cfg, FlightController ap) : base(cfg.RadarSize, cfg.RadarX, cfg.RadarY)
        {
            this.cfg = cfg;
            this.ap = ap;
            if (System.Array.IndexOf(AglsM, cfg.RadarTargetAgl.Value) < 0) cfg.RadarTargetAgl.Value = 150f;
        }

        /// <summary>F9: hidden -> ALL -> OWN -> ENMY -> hidden.</summary>
        public void CycleMode()
        {
            if (!Visible)
            {
                Visible = true;
                Current = Mode.All;
            }
            else if (Current == Mode.All) Current = Mode.Own;
            else if (Current == Mode.Own) Current = Mode.Enemy;
            else Visible = false;
            Dirty("mode");
            nextScan = 0f; // friendly <-> enemy swaps the radar set
        }

        public void Reset()
        {
            profiles.Clear();
            building.Clear();
            contacts.Clear();
            buildActive = false;
            me = null;
            nextScan = 0f;
            Dirty("reset");
        }

        public void Command(ApCommand cmd)
        {
            switch (cmd)
            {
                case ApCommand.RdrAll: Current = Mode.All; nextScan = 0f; break;
                case ApCommand.RdrOwn: Current = Mode.Own; nextScan = 0f; break;
                case ApCommand.RdrEnemy: Current = Mode.Enemy; nextScan = 0f; break;
                case ApCommand.RdrHide: Visible = false; break;
                case ApCommand.RdrTgt:
                    int i = System.Array.IndexOf(AglsM, cfg.RadarTargetAgl.Value);
                    cfg.RadarTargetAgl.Value = AglsM[(i + 1) % AglsM.Length];
                    break;
                case ApCommand.RdrRcsUp: RcsSetting.Value = Mathf.Min(TargetRcs() * 2f, 1000f); break;
                case ApCommand.RdrRcsDown: RcsSetting.Value = Mathf.Max(TargetRcs() * 0.5f, 0.001f); break;
                case ApCommand.RdrRcsOwn: RcsSetting.Value = 0f; break;
                case ApCommand.RdrZoomIn: Zoom(0.5f); break;
                case ApCommand.RdrZoomOut: Zoom(2f); break;
                case ApCommand.RdrFollow: follow = true; break;
                case ApCommand.RdrMap:
                    follow = false;
                    viewX = viewZ = 0f; // global origin is the map centre
                    viewR = MapHalfSize() * 1.02f;
                    break;
            }
            Dirty("command");
        }

        /// <summary>RCS setting of the current view: ENMY, or ALL/OWN (friendly radars).</summary>
        private ConfigEntry<float> RcsSetting => Current == Mode.Enemy ? cfg.RadarEnemyRcs : cfg.RadarFriendlyRcs;

        /// <summary>RCS the current view is drawn for: the set value, or 0 = own jet's live RCS (grows with stores).</summary>
        private float TargetRcs() => RcsSetting.Value > 0f ? RcsSetting.Value : me != null ? me.RCS : 1f;
        private float lastRcs, lastOpacity;
        private float altSnap = -1000f; // own altitude (m above sea level) TGT ALT is drawn for

        private float MapHalfSize() => mapSettings != null ? Mathf.Max(mapSettings.MapSize.x, mapSettings.MapSize.y) * 0.5f : 100000f;

        private void Zoom(float k) => viewR = Mathf.Clamp(viewR * k, MinRange, MapHalfSize() * 1.5f);

        // =====================================================================
        // Update: map the terrain once, keep radar profiles current, rasterise the overlay
        // =====================================================================
        /// <param name="active">false = nobody is looking (hidden in-game and not mirrored): do nothing.</param>
        public void Tick(Aircraft aircraft, bool active)
        {
            me = aircraft;
            float now = Time.unscaledTime;
            if (!active)
            {
                // nobody looking: still map the terrain, but only if the flight planner doesn't have this map yet
                if (now >= nextScan)
                {
                    nextScan = now + 1f;
                    FindMap();
                }
                if (plannerDone) return;
                if (grid != null && !grid.Ready) grid.Fill();
                else if (grid != null) ExportForPlanner();
                return;
            }
            if (now >= nextScan)
            {
                nextScan = now + 1f;
                FindMap();
                CollectRadars();
            }
            if (now >= nextContacts)
            {
                nextContacts = now + 0.25f;
                CollectContacts();
                float rcs = TargetRcs();
                if (rcs != lastRcs) Dirty(float.IsNaN(rcs) ? "rcs NaN" : "rcs"); // stores released, or set from the web mirror
                lastRcs = rcs;
                if (cfg.RadarOpacity.Value != lastOpacity) Dirty("opacity");
                lastOpacity = cfg.RadarOpacity.Value;
                jammedCount = 0;
                // enemy jam state only when the game map shows it too (its jammed markers)
                MapOptions mo = SceneSingleton<MapOptions>.i;
                bool showJam = mo != null && mo.showJamming, enemyView = Current == Mode.Enemy;
                foreach (Profile p in profiles.Values)
                {
                    bool shown = p.Enemy == enemyView;
                    bool j = (!p.Enemy || showJam) && p.Radar is Radar rr && rr.IsJammed();
                    if (j != p.Jammed && shown) Dirty("jam");
                    p.Jammed = j;
                    if (j && shown) jammedCount++;

                    // kept up in every mode so the last known pitch survives leaving ENMY
                    if (p.Enemy) UpdateEnemyCone(p);
                }
            }

            long t = Perf.Now;
            if (grid != null && !grid.Ready)
            {
                grid.Fill();
                Perf.Add(Perf.Terrain, t);
            }
            else if (grid != null)
            {
                if (relief == null) relief = ReliefTexture(grid);
                ExportForPlanner();
                UpdateDetail();
                Perf.Add(Perf.Terrain, t);
                t = Perf.Now;
                // Own radar first so the OWN page fills quickly.
                int budget = SampleBudget;
                if (me.radar != null && profiles.TryGetValue(me.radar, out Profile own))
                    budget -= Advance(own, budget);
                foreach (Profile p in profiles.Values)
                {
                    if (budget <= 0) break;
                    budget -= Advance(p, budget);
                }
                Perf.Add(Perf.Sweep, t);
            }

            if (follow)
            {
                GlobalPosition g = me.GlobalPosition();
                viewX = g.x;
                viewZ = g.z;
            }

            // TGT ALT: redraw once own altitude moves 3% (at least 25 m) from what was drawn
            float alt = me.GlobalPosition().y;
            if (cfg.RadarTargetAgl.Value < 0f && Mathf.Abs(alt - altSnap) > Mathf.Max(25f, altSnap * 0.03f))
            {
                altSnap = alt;
                Dirty("tgt alt");
            }

            t = Perf.Now;
            if (buildActive) ContinueBuild();
            else if (now >= nextBuild && (dirty || ViewMoved()))
            {
                nextBuild = now + 0.25f;
                Perf.Trigger(dirty ? dirtyWhy : Zoomed() ? "zoom" : "view moved");
                StartBuild();
            }
            Perf.Add(Perf.Raster, t);
        }

        private bool plannerDone; // this map handed to the flight planner

        private void FindMap()
        {
            if (mapSettings == null) // Unity null also after a map unload
            {
                mapSettings = UnityEngine.Object.FindObjectOfType<MapSettings>();
                grid = null;
                plannerDone = mapSettings != null && FlightPlanner.HasMap(mapSettings.name);
                if (relief != null) UnityEngine.Object.Destroy(relief);
                relief = null;
                DropDetail();
                profiles.Clear();
            }
            if (mapSettings != null && grid == null && MapHalfSize() > 0f)
                grid = new TerrainHeightGrid(MapHalfSize() * 2f);
        }

        /// <summary>Hand the map to the flight planner once its shading is ready (it keeps one export per map per session).</summary>
        private void ExportForPlanner()
        {
            if (plannerDone || mapSettings == null || grid.Relief() == null) return;
            plannerDone = true;
            FlightPlanner.ExportMap(mapSettings.name, mapSettings.MapImage, mapSettings.MapSize, grid);
        }

        private static Texture2D ReliefTexture(TerrainHeightGrid g)
        {
            Color32[] shade = g.Relief();
            if (shade == null) return null;
            var t = new Texture2D(g.N, g.N, TextureFormat.RGBA32, true) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            t.SetPixels32(shade);
            t.Apply(true, true);
            return t;
        }

        private void DropDetail()
        {
            if (detailTex != null) UnityEngine.Object.Destroy(detailTex);
            detailTex = null;
            detail = detailNext = null;
        }

        /// <summary>Keep a fine relief tile around the view while zoomed in; the old one shows until the next is shaded.</summary>
        private void UpdateDetail()
        {
            float size = DetailSpan * viewR;
            if (size / DetailN > grid.Cell * 0.7f)
            {
                DropDetail(); // the map grid is fine enough here
                return;
            }
            TerrainHeightGrid want = detailNext ?? detail;
            float slack = (detailNext != null ? 0.5f : 0.25f) * viewR; // a tile being filled still covers the view at 0.5: let it finish
            if (want == null || Mathf.Abs(viewX - want.CX) > slack || Mathf.Abs(viewZ - want.CZ) > slack
                || want.Size > size * 1.4f || want.Size < size / 1.4f)
                detailNext = new TerrainHeightGrid(viewX, viewZ, size, DetailN, 50f);
            if (detailNext == null) return;
            if (!detailNext.Ready)
            {
                detailNext.Fill();
                return;
            }
            Texture2D t = ReliefTexture(detailNext);
            if (t == null) return; // still shading on the worker thread
            if (detailTex != null) UnityEngine.Object.Destroy(detailTex);
            detailTex = t;
            detail = detailNext;
            detailNext = null;
        }

        // The overlay is built Margin x wider than the view and drawn where it was built, so panning
        // (and follow mode) only needs a rebuild once the view nears the overlay's edge.
        // shortcut: a web window taller than wide shows coverage only in a band 1.4x its width; scale by the shape if phones matter
        private const float Margin = 1.4f;

        private bool ViewMoved()
        {
            float slack = viewR * 0.15f;
            return Zoomed() || Mathf.Abs(viewX - texX) > slack || Mathf.Abs(viewZ - texZ) > slack;
        }

        // Not `viewR * Margin != texR`: Mono computes the product at higher precision than the stored float,
        // so that was always true and rebuilt the overlay every 0.25 s.
        private bool Zoomed() => !Mathf.Approximately(viewR * Margin, texR);

        private void CollectRadars()
        {
            // radar -> scan point to sweep from; enemy radars sit where our side last saw them
            var live = new Dictionary<TargetDetector, GlobalPosition>();
            bool enemy = Current == Mode.Enemy;
            siteCount = 0;
            foreach (Unit u in UnitRegistry.allUnits)
            {
                TargetDetector r = u != null ? u.radar : null;
                // fire-control radars are also set on their launchers (FireControl.RegisterRadar): take
                // each radar once, from the unit it sits on, so ENMY places it at that unit's known position
                if (r == null || u.disabled || r.GetAttachedUnit() != u || r.GetRadarRange() <= 0f || r.GetScanPoint() == null) continue;
                GlobalPosition scan = r.GetScanPoint().GlobalPosition();
                bool isOwn = u == me;
                if (enemy)
                {
                    // activated is ignored (worst case: on). Damage is all-or-nothing in the game: a radar
                    // knocked out destroys its component, so u.radar goes null above and the site drops off.
                    // Only the antenna height (fixed per unit type) is added to the known position.
                    if (u.NetworkHQ == null || me.NetworkHQ == null || u.NetworkHQ == me.NetworkHQ || !r.IsOperational()
                        || !me.NetworkHQ.TryGetKnownPosition(u, out GlobalPosition known)) continue;
                    scan = known + new Vector3(0f, (scan - u.GlobalPosition()).y, 0f);
                }
                else if (u.NetworkHQ != me.NetworkHQ || (!isOwn && (!r.activated || !r.IsOperational()))) continue;
                live[r] = scan;
                if (!isOwn) siteCount++;
            }
            ownRadarMissing = me.radar == null || !live.ContainsKey(me.radar);

            foreach (KeyValuePair<TargetDetector, GlobalPosition> kv in live)
            {
                TargetDetector r = kv.Key;
                if (!profiles.TryGetValue(r, out Profile p) || p.Enemy != enemy) // new, or changed side
                {
                    p = new Profile { Radar = r, Enemy = enemy };
                    profiles[r] = p;
                    TakeProfile(p, kv.Value);
                }
                else if (p.RaysDone >= Az)
                {
                    // Redo the profile once the radar has moved 2% of its range (others' 5%: many moving
                    // friendly radars would otherwise keep sweeping every frame), or its range changed.
                    float moved = (kv.Value - p.Origin).magnitude;
                    if (moved > p.Range * (r == me.radar ? 0.02f : 0.05f) || Mathf.Abs(SweepRange(r) - p.Range) > 1f) TakeProfile(p, kv.Value);
                }
            }
            // The other side's profiles are kept (switching modes would otherwise reallocate ~2.5 MB per
            // radar in one frame); they go once their radar is destroyed.
            var dead = new List<TargetDetector>();
            foreach (KeyValuePair<TargetDetector, Profile> kv in profiles)
                if (kv.Key == null || (kv.Value.Enemy == enemy && !live.ContainsKey(kv.Key))) dead.Add(kv.Key);
            foreach (TargetDetector r in dead) profiles.Remove(r);
            if (dead.Count > 0) Dirty("radar lost");
        }

        /// <summary>
        /// Enemy units the game map has an icon for, minus what the map hides (pilot toggle) or its
        /// target filters exclude.
        /// </summary>
        private void CollectContacts()
        {
            contacts.Clear();
            DynamicMap map = SceneSingleton<DynamicMap>.i;
            if (map == null) return;
            TargetListSelector filter = SceneSingleton<TargetListSelector>.i;
            foreach (MapIcon icon in map.mapIcons)
            {
                if (!(icon is UnitMapIcon ui) || !TargetPanel.ShownEnemy(ui, filter)) continue;
                Unit u = ui.unit;
                // never tracked = position unknown: no true-position fallback
                TrackingInfo track = map.HQ != null ? map.HQ.GetTrackingData(u.persistentID) : null;
                if (track == null) continue;
                if (ui.iconImage.sprite != null) BakeIcon(ui.iconImage.sprite.texture);
                contacts.Add(new Contact
                {
                    Unit = u,
                    Track = track, // GetPosition: live while spotted, else the last known position
                    Icon = ui.iconImage,
                    Orient = u.definition.mapOrient,
                });
            }
        }

        /// <summary>
        /// Enemy cone aim from what the map gives: heading off the icon (which the map freezes once the
        /// track is stale), and antenna pitch only while that heading is live, else the last pitch seen.
        /// </summary>
        private void UpdateEnemyCone(Profile p)
        {
            Unit u = p.Radar.GetAttachedUnit();
            float h = MapHeading(u), pitch = p.ConePitch;
            TrackingInfo t = u != null && me.NetworkHQ != null ? me.NetworkHQ.GetTrackingData(u.persistentID) : null;
            Transform scan = p.Radar.GetScanPoint();
            if (!float.IsNaN(h) && t != null && t.Observed() && scan != null)
                pitch = Mathf.Asin(Mathf.Clamp(scan.forward.y, -1f, 1f)) * Mathf.Rad2Deg;
            if (Differs(h, p.ConeHeading) || Differs(pitch, p.ConePitch))
            {
                p.ConeHeading = h;
                p.ConePitch = pitch;
                if (Current == Mode.Enemy) Dirty("enemy cone");
            }
        }

        private static bool Differs(float a, float b) =>
            float.IsNaN(a) != float.IsNaN(b) || Mathf.Abs(Mathf.DeltaAngle(a, b)) > 3f;

        /// <summary>Heading the game map's icon shows for a unit (deg from north), NaN if the map shows none.</summary>
        private static float MapHeading(Unit u)
        {
            DynamicMap map = SceneSingleton<DynamicMap>.i;
            if (u == null || map == null || map.mapImage == null || !u.definition.mapOrient
                || !DynamicMap.TryGetMapIcon(u, out UnitMapIcon icon) || icon.iconImage == null) return float.NaN;
            return map.mapImage.transform.eulerAngles.z - icon.iconImage.transform.eulerAngles.z;
        }

        /// <summary>
        /// The game checks targets out to 2x maxRange (Radar.RadarCheck); signal strength decides the
        /// rest, and a large RCS reaches past maxRange.
        /// </summary>
        private static float SweepRange(TargetDetector r) => r.GetRadarRange() * 2f;

        /// <summary>Start a new sweep from the radar's scan point (true or known position).</summary>
        private static void TakeProfile(Profile p, GlobalPosition origin)
        {
            p.NextOrigin = origin;
            p.NextRange = SweepRange(p.Radar);
            p.NextAlt = p.NextOrigin.ToLocalPosition().y - Datum.LocalSeaY;
            p.RaysDone = 0;
        }

        /// <summary>
        /// Continue a sweep from the height grid (no raycasts). Each range bin stores the terrain at
        /// its far edge plus the steepest slope of grid-spaced samples inside it, so ridges narrower
        /// than a bin still mask what lies behind them. Returns the lookups used.
        /// </summary>
        private int Advance(Profile p, int budget)
        {
            if (p.RaysDone >= Az) return 0;
            if (p.NextHeight == null)
            {
                p.NextHeight = Rent();
                p.NextBefore = Rent();
            }
            float step = p.NextRange / Steps;
            int fine = Mathf.Clamp(Mathf.CeilToInt(step / grid.Cell), 1, 32);
            float sub = step / fine, ox = p.NextOrigin.x, oz = p.NextOrigin.z, alt = p.NextAlt;
            int used = 0;
            while (used < budget && p.RaysDone < Az)
            {
                int a = p.RaysDone++, b = a * Steps;
                float brg = (a + 0.5f) * (2f * Mathf.PI / Az), sx = Mathf.Sin(brg), sz = Mathf.Cos(brg);
                for (int i = 0; i < Steps; i++)
                {
                    float before = float.NegativeInfinity, d0 = i * step;
                    for (int j = 1; j < fine; j++)
                    {
                        float d = d0 + j * sub;
                        before = Mathf.Max(before, (grid.Height(ox + sx * d, oz + sz * d) - alt) / d);
                    }
                    float de = d0 + step;
                    p.NextHeight[b + i] = grid.Height(ox + sx * de, oz + sz * de);
                    p.NextBefore[b + i] = before;
                }
                used += Steps * fine;
            }
            if (p.RaysDone >= Az)
            {
                if (p.Height != null)
                {
                    spare.Push(p.Height);
                    spare.Push(p.Before);
                }
                p.Height = p.NextHeight;
                p.Before = p.NextBefore;
                p.NextHeight = p.NextBefore = null;
                p.Origin = p.NextOrigin;
                p.Range = p.NextRange;
                p.Alt = p.NextAlt;
                p.Valid = true;
                p.VisAgl = float.NaN;
                Dirty("sweep");
                Perf.Sweeps++;
            }
            return used;
        }

        /// <summary>
        /// Target height above sea level over terrain t: terrain + agl, or (tgtAsl not NaN) a fixed
        /// altitude such as the own jet's, kept MinAgl above the ground.
        /// </summary>
        private static float TargetAsl(float t, float agl, float tgtAsl) =>
            float.IsNaN(tgtAsl) ? t + agl : Mathf.Max(tgtAsl, t + MinAgl);

        /// <summary>
        /// Line-of-sight along each bearing: a target at TargetAsl at the far edge of bin i is
        /// visible when its elevation angle from the radar clears the steepest terrain before it
        /// (earlier bins, plus samples inside bin i), it is above the radar horizon, and its signal
        /// beats the radar's minimum after clutter. vis: 0 = signal too weak, 1 = hidden, 2 = detected.
        /// Mirrors DetectorManager.RequestRadarCheck + RadarParams.GetSignalStrength with no ECM, where
        /// the doppler boost only applies once the signal already passes, so it can't change the result.
        /// </summary>
        internal static void Viewshed(float[] height, float[] before, float radarAlt, float agl, float tgtAsl, float step,
            RadarParams rp, float rcs, float tgtRadius, byte[] vis)
        {
            // The map is flat, but the game drops targets past the curved-earth horizon:
            // sqrt(2R*radarASL) + sqrt(2R*targetASL) < horizontal distance.
            const float TwoR = 12742000f;
            float radarHorizon = Mathf.Sqrt(TwoR * Mathf.Max(0f, radarAlt));
            float rcsGain = Mathf.Pow(rcs, 0.25f), maxDist = rp.maxRange * 2f;
            float size2 = tgtRadius * tgtRadius * 2f;
            for (int a = 0; a < Az; a++)
            {
                int b = a * Steps;
                float maxSlope = float.NegativeInfinity;
                for (int i = 0; i < Steps; i++)
                {
                    float d = (i + 1) * step, t = height[b + i], tgt = TargetAsl(t, agl, tgtAsl), dy = tgt - radarAlt;
                    float h = tgt - t; // target's radar altitude
                    maxSlope = Mathf.Max(maxSlope, before[b + i]);
                    float dist = Mathf.Sqrt(d * d + dy * dy);
                    float signal = Mathf.Min(rp.maxRange / dist * rcsGain, rp.maxSignal);
                    byte v;
                    if (signal < rp.minSignal || dist > maxDist) v = 0;
                    else if (dy / d < maxSlope || radarHorizon + Mathf.Sqrt(TwoR * Mathf.Max(0f, tgt)) < d) v = 1;
                    else
                    {
                        // below the radar's line to its horizon, the ground behind the target adds clutter
                        float clutter = size2 / (h * h); // the ground right under the target
                        if (d < radarHorizon && tgt < radarAlt * (1f - d / radarHorizon))
                            clutter += Mathf.Min(dist, 1000f) * (radarAlt - tgt) / (dist * h);
                        v = signal - clutter * rp.clutterFactor >= rp.minSignal ? (byte)2 : (byte)1;
                    }
                    vis[b + i] = v;
                    maxSlope = Mathf.Max(maxSlope, (t - radarAlt) / d);
                }
            }
        }

        private void StartBuild()
        {
            dirty = false;
            bX = viewX;
            bZ = viewZ;
            bR = viewR * Margin;
            bEnemy = Current == Mode.Enemy;
            System.Array.Clear(state, 0, state.Length);
            building.Clear();
            foreach (Profile p in profiles.Values)
                if (p.Valid && !p.Jammed && p.Enemy == bEnemy && (Current != Mode.Own || p.Radar == me.radar)) building.Add(p);
            buildIdx = 0;
            buildRow = -1;
            buildActive = true;
            ContinueBuild();
        }

        private void ContinueBuild()
        {
            int budget = PixelBudget;
            while (budget > 0 && buildIdx < building.Count)
            {
                budget -= Rasterize(building[buildIdx], budget);
                if (buildRow < 0) buildIdx++; // that radar is done
            }
            if (buildIdx < building.Count) return;

            // enemy view: seen by them is the danger (red), terrain-masked is safe (green)
            // opacity setting = red's alpha; green stays a little fainter, as drawn before (95 vs 130)
            float op = Mathf.Clamp01(cfg.RadarOpacity.Value);
            builtOpacity = op;
            Color32 cov = Covered, mask = Masked;
            cov.a = (byte)(op * 0.73f * 255f);
            mask.a = (byte)(op * 255f);
            Color32 seen = bEnemy ? mask : cov, hidden = bEnemy ? cov : mask;
            for (int k = 0; k < pixels.Length; k++)
                pixels[k] = state[k] == 2 ? seen : state[k] == 1 ? hidden : None;
            if (overlay == null)
                overlay = new Texture2D(Tex, Tex, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            overlay.SetPixels32(pixels); // row 0 = south, so the texture draws north-up
            overlay.Apply();
            texX = bX;
            texZ = bZ;
            texR = bR;
            buildActive = false;
            builtSeq++;
            Perf.Builds++;
        }

        private int buildRow = -1; // row to resume the current radar at; -1 = start the next radar

        /// <summary>
        /// Merge one radar's coverage into `state`, a budget of pixels at a time (one long-range
        /// radar can cover the whole texture). Returns the pixels it looked at; buildRow is -1
        /// once the radar is finished.
        /// </summary>
        private int Rasterize(Profile p, int budget)
        {
            Transform scan = p.Radar != null ? p.Radar.GetScanPoint() : null;
            int resume = buildRow;
            buildRow = -1;
            if (scan == null || !(p.Radar is Radar radar)) return 0;
            float half = Tex * 0.5f, mPerPx = bR / half, agl = cfg.RadarTargetAgl.Value;
            float radarAlt = p.Alt;
            // target: own jet's size (ground clutter) and the RCS set for this view; TGT ALT (agl < 0)
            // = own altitude above sea level, as snapped in Tick
            float rcs = TargetRcs(), tgtRadius = me.maxRadius;
            float tgtAsl = agl < 0f ? altSnap : float.NaN, hKey = agl < 0f ? -1f - altSnap : agl;

            // Scan cone (half-angle, 0 = all round) is checked in 3D, so it also covers the
            // radar's up/down limits for a target at that spot's height.
            float cone = RadarConeRef(radar);
            Vector3 fwd = scan.forward;
            bool flat = false, elevOnly = false; // flat = azimuth-only check; elevOnly = pitch band, any azimuth
            if (bEnemy)
            {
                // Enemy: heading + pitch -> full cone; heading only -> azimuth check; pitch only (heading
                // lost) -> the band of elevations the cone can reach turned any way; neither -> all-round.
                bool hKnown = !float.IsNaN(p.ConeHeading), pKnown = !float.IsNaN(p.ConePitch);
                float hr = p.ConeHeading * Mathf.Deg2Rad, pr = pKnown ? p.ConePitch * Mathf.Deg2Rad : 0f;
                if (hKnown) fwd = new Vector3(Mathf.Sin(hr) * Mathf.Cos(pr), Mathf.Sin(pr), Mathf.Cos(hr) * Mathf.Cos(pr));
                flat = hKnown && !pKnown;
                elevOnly = !hKnown && pKnown;
                if (!hKnown && !pKnown) cone = 0f;
            }

            // Only touch pixels inside this radar's range box.
            float ox = (p.Origin.x - bX) / mPerPx + half, oy = (p.Origin.z - bZ) / mPerPx + half, rp = p.Range / mPerPx;
            int x0 = Mathf.Max(0, (int)(ox - rp)), x1 = Mathf.Min(Tex - 1, (int)(ox + rp));
            int y0 = Mathf.Max(0, (int)(oy - rp)), y1 = Mathf.Min(Tex - 1, (int)(oy + rp));
            if (x1 < x0 || y1 < y0) return 0;
            int used = 0;
            // Approximately: hKey is computed, and Mono's extra precision would never match the stored float
            if (!Mathf.Approximately(p.VisAgl, hKey) || p.VisRcs != rcs || p.VisRadius != tgtRadius) // only for radars in view
            {
                used = ViewshedCost;
                Perf.Viewsheds++;
                Viewshed(p.Height, p.Before, radarAlt, agl, tgtAsl, p.Range / Steps, radar.RadarParameters, rcs, tgtRadius, p.Vis);
                p.VisAgl = hKey;
                p.VisRcs = rcs;
                p.VisRadius = tgtRadius;
            }
            int y = Mathf.Max(y0, resume);
            // Vector3.Angle(v, fwd) > cone  <=>  dot(v, fwd) < cos(cone)|v|  (fwd is unit): no acos per pixel
            bool coneCull = cone > 0f && cone < 180f;
            float cosCone = Mathf.Cos(cone * Mathf.Deg2Rad);
            float binsPerM = Steps / p.Range, azPerRad = Az / (2f * Mathf.PI), r2 = p.Range * p.Range;
            for (; y <= y1 && used < budget; y++, used += x1 - x0 + 1)
            {
                float dz = (y + 0.5f - oy) * mPerPx;
                for (int x = x0; x <= x1; x++)
                {
                    float dx = (x + 0.5f - ox) * mPerPx, d2 = dx * dx + dz * dz;
                    if (d2 >= r2) continue;
                    int k = y * Tex + x;
                    if (state[k] == 2) continue; // already covered by another radar
                    float brg = Mathf.Atan2(dx, dz);
                    if (brg < 0f) brg += 2f * Mathf.PI;
                    int bin = Mathf.Min(Az - 1, (int)(brg * azPerRad)) * Steps + Mathf.Min(Steps - 1, (int)(Mathf.Sqrt(d2) * binsPerM));
                    if (coneCull)
                    {
                        float dy = TargetAsl(p.Height[bin], agl, tgtAsl) - radarAlt;
                        if (elevOnly)
                        {
                            if (Mathf.Abs(Mathf.Atan2(dy, Mathf.Sqrt(d2)) * Mathf.Rad2Deg - p.ConePitch) > cone) continue;
                        }
                        else
                        {
                            float vy = flat ? 0f : dy;
                            if (dx * fwd.x + vy * fwd.y + dz * fwd.z < cosCone * Mathf.Sqrt(d2 + vy * vy)) continue;
                        }
                    }
                    byte v = p.Vis[bin];
                    if (v > state[k]) state[k] = v;
                }
            }
            if (y <= y1) buildRow = y;
            return used;
        }

        // =====================================================================
        // Mouse: scroll = zoom about the cursor, drag = pan
        // =====================================================================
        protected override bool ScreenMouse(Event e)
        {
            Vector2 mp = e.mousePosition;
            float mPerScreenPx = viewR / (screen.width * 0.5f);
            switch (e.type)
            {
                case EventType.ScrollWheel when screen.Contains(mp) && WaypointAt(mp - screen.position) >= 0:
                    NudgeWaypoint(WaypointAt(mp - screen.position), e.delta.y > 0f ? -1f : 1f, e.shift);
                    e.Use();
                    return true;
                case EventType.ScrollWheel when screen.Contains(mp):
                    float before = viewR;
                    Zoom(e.delta.y > 0f ? 1.25f : 0.8f);
                    float k = viewR / before;
                    if (!follow)
                    {
                        // keep the map point under the cursor fixed
                        float gx = viewX + (mp.x - screen.center.x) * mPerScreenPx;
                        float gz = viewZ - (mp.y - screen.center.y) * mPerScreenPx;
                        viewX = gx + (viewX - gx) * k;
                        viewZ = gz + (viewZ - gz) * k;
                    }
                    e.Use();
                    return true;
                case EventType.MouseDown when e.button == 1 && screen.Contains(mp):
                    EditWaypoint(mp, mPerScreenPx);
                    e.Use();
                    return true;
                case EventType.MouseDown when e.button == 0:
                    draggedWp = screen.Contains(mp) ? WaypointAt(mp - screen.position) : -1;
                    if (draggedWp < 0 && ClickContact(mp))
                    {
                        e.Use();
                        return true;
                    }
                    panning = draggedWp < 0;
                    lastMouse = mp;
                    e.Use();
                    return true;
                case EventType.MouseDrag when draggedWp >= 0:
                    ap.MoveWaypoint(draggedWp, viewX + (mp.x - screen.center.x) * mPerScreenPx, viewZ - (mp.y - screen.center.y) * mPerScreenPx);
                    e.Use();
                    return true;
                case EventType.MouseUp when draggedWp >= 0:
                    draggedWp = -1;
                    e.Use();
                    return true;
                case EventType.MouseDrag when panning:
                    Vector2 d = mp - lastMouse;
                    lastMouse = mp;
                    follow = false;
                    viewX -= d.x * mPerScreenPx;
                    viewZ += d.y * mPerScreenPx;
                    e.Use();
                    return true;
                case EventType.MouseUp when panning:
                    panning = false;
                    e.Use();
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Right-click: on a waypoint, delete it; on a leg of the route line, insert one there; else
        /// add one at the end of the route.
        /// </summary>
        private void EditWaypoint(Vector2 mp, float mPerScreenPx)
        {
            float gx = viewX + (mp.x - screen.center.x) * mPerScreenPx, gz = viewZ - (mp.y - screen.center.y) * mPerScreenPx;
            EditWaypointAt(gx, gz, Mathf.Max(10f, 0.025f * S) * mPerScreenPx);
        }

        /// <summary>Right-click at global x/z, with the hit radius in metres (also from the browser-drawn map).</summary>
        public void EditWaypointAt(float gx, float gz, float hitM)
        {
            if (me == null) return;
            for (int i = 0; i < ap.Route.Count; i++)
                if (new Vector2(ap.Route[i].X - gx, ap.Route[i].Z - gz).magnitude < hitM)
                {
                    ap.RemoveWaypoint(i);
                    return;
                }
            GlobalPosition jet = me.GlobalPosition();
            float ax = jet.x, az = jet.z; // the first leg starts at the jet, as drawn
            for (int i = 0; i < ap.Route.Count; i++)
            {
                float bx = ap.Route[i].X, bz = ap.Route[i].Z;
                if (ApMath.SegmentDistance(gx, gz, ax, az, bx, bz) < hitM * 0.6f)
                {
                    ap.AddWaypoint(i, gx, gz);
                    return;
                }
                ax = bx;
                az = bz;
            }
            ap.AddWaypoint(ap.Route.Count, gx, gz);
        }

        /// <summary>Wheel over waypoint wi: notch +1 = higher clearance (TF/RALT height for the leg to it); with shift, its speed.</summary>
        public void NudgeWaypoint(int wi, float notch, bool shift)
        {
            if (wi < 0 || wi >= ap.Route.Count) return;
            Waypoint ww = ap.Route[wi];
            if (shift)
                ap.SetSpeed(wi, (float.IsNaN(ww.Speed) ? me != null ? me.speed : 200f : ww.Speed) + notch * Units.SpeedStep(true));
            else
                ap.SetClearance(wi, ww.Clearance + notch * Units.RaltStep(true));
        }

        /// <summary>The browser-drawn map's view (it pans and zooms on its own, then tells the game where it is).</summary>
        public void SetView(float x, float z, float r, bool followJet)
        {
            follow = followJet;
            if (!followJet)
            {
                viewX = x;
                viewZ = z;
            }
            viewR = Mathf.Clamp(r, MinRange, MapHalfSize() * 1.5f);
        }

        /// <summary>Index of the waypoint under p (inside the screen rect), or -1.</summary>
        private int WaypointAt(Vector2 p)
        {
            float hit = Mathf.Max(10f, 0.025f * S);
            for (int i = 0; i < ap.Route.Count; i++)
                if ((ToScreen(ap.Route[i].X, ap.Route[i].Z) - p).magnitude < hit) return i;
            return -1;
        }

        /// <summary>Map state for the web page's controls.</summary>
        public string Json()
        {
            GlobalPosition g = me != null ? me.GlobalPosition() : default;
            string mode = Current == Mode.All ? "ALL" : Current == Mode.Own ? "OWN" : "ENMY";
            var sb = new StringBuilder(FormattableString.Invariant(
                $"{{\"mode\":\"{mode}\",\"tgt\":\"{TgtLabel()}\",\"folw\":{(follow ? "true" : "false")},\"rng\":{viewR / 1000f:F0},\"rcs\":{TargetRcs():0.###},\"nav\":{(ap.Nav ? "true" : "false")},\"eta\":\"{(ap.ApEngaged ? ap.EtaText(me) : null)}\",\"opa\":{cfg.RadarOpacity.Value:0.##},\"unit\":\"{Units.AltUnit}\",\"sunit\":\"{Units.SpeedUnit}\",\"wps\":["));
            for (int i = 0; i < ap.Route.Count; i++)
            {
                Waypoint w = ap.Route[i];
                float d = new Vector2(w.X - g.x, w.Z - g.z).magnitude / 1000f;
                string spd = float.IsNaN(w.Speed) ? "null" : FormattableString.Invariant($"{Units.Speed(w.Speed):F0}");
                sb.Append(i > 0 ? "," : "").Append(FormattableString.Invariant($"{{\"c\":{Units.Alt(w.Clearance):F0},\"s\":{spd},\"d\":{d:F1}}}"));
            }
            return sb.Append("]}").ToString();
        }

        private string TgtLabel() => cfg.RadarTargetAgl.Value < 0f ? "TGT ALT" : $"TGT {cfg.RadarTargetAgl.Value:F0}";

        // =====================================================================
        // Display
        // =====================================================================
        protected override void DrawScreen()
        {
            if (me == null) return;
            GUI.BeginGroup(screen);

            // the game's own map image: global origin is the map centre
            Sprite img = mapSettings != null ? mapSettings.MapImage : null;
            if (img != null)
            {
                Vector2 ms = mapSettings.MapSize;
                Vector2 tl = ToScreen(-ms.x * 0.5f, ms.y * 0.5f), br = ToScreen(ms.x * 0.5f, -ms.y * 0.5f);
                Texture2D t = img.texture;
                Rect tr = img.textureRect;
                Color saved = GUI.color;
                GUI.color = new Color(0.8f, 0.8f, 0.8f, 1f);
                GUI.DrawTextureWithTexCoords(Rect.MinMaxRect(tl.x, tl.y, br.x, br.y), t,
                    new Rect(tr.x / t.width, tr.y / t.height, tr.width / t.width, tr.height / t.height));
                GUI.color = saved;
            }

            // terrain relief from the height grid (it covers the map's larger side, centred), and the
            // finer tile when zoomed in; the map grid's shading is left out where the tile covers the view
            Rect dr = default;
            if (detailTex != null)
            {
                float h = detail.Size * 0.5f;
                Vector2 tl = ToScreen(detail.CX - h, detail.CZ + h), br = ToScreen(detail.CX + h, detail.CZ - h);
                dr = Rect.MinMaxRect(tl.x, tl.y, br.x, br.y);
            }
            bool tileCovers = detailTex != null && dr.xMin <= 0f && dr.yMin <= 0f && dr.xMax >= screen.width && dr.yMax >= screen.height;
            if (relief != null && !tileCovers)
            {
                float h = MapHalfSize();
                Vector2 tl = ToScreen(-h, h), br = ToScreen(h, -h);
                GUI.DrawTexture(Rect.MinMaxRect(tl.x, tl.y, br.x, br.y), relief);
            }
            if (tileCovers) GUI.DrawTexture(dr, detailTex);

            // coverage overlay, placed where it was built (it lags pans by up to 0.1 s)
            if (overlay != null)
            {
                Vector2 tl = ToScreen(texX - texR, texZ + texR), br = ToScreen(texX + texR, texZ - texR);
                GUI.DrawTexture(Rect.MinMaxRect(tl.x, tl.y, br.x, br.y), overlay);
            }

            // radar sites (friendly blue / enemy red, amber = jammed)
            if (Current != Mode.Own)
            {
                foreach (Profile p in profiles.Values)
                {
                    if (p.Radar == me.radar || !p.Valid || p.Enemy != (Current == Mode.Enemy)) continue;
                    Vector2 s = ToScreen(p.Origin.x, p.Origin.z);
                    Fill(new Rect(s.x - 3f, s.y - 3f, 6f, 6f), p.Jammed ? Amber : Current == Mode.Enemy ? EnemySiteCol : SiteCol, 0f);
                }
            }

            // enemy contacts drawn with the map icon's own sprite and colour; position from the same
            // track the icon uses, heading read off the icon (the map freezes it while a track is stale)
            DynamicMap map = SceneSingleton<DynamicMap>.i;
            float mapRot = map != null && map.mapImage != null ? map.mapImage.transform.eulerAngles.z : 0f;
            float es = ContactSize;
            List<Unit> targets = TargetPanel.Targets();
            Color savedCol = GUI.color;
            foreach (Contact k in contacts)
            {
                if (k.Unit == null || k.Icon == null || k.Icon.sprite == null) continue;
                Vector2 s = ContactPos(k);
                if (s.x < -es || s.y < -es || s.x > screen.width + es || s.y > screen.height + es) continue;
                Sprite sp = k.Icon.sprite;
                if (!iconAlpha.TryGetValue(sp.texture, out Texture2D t)) continue; // baked in CollectContacts
                Rect tr = sp.textureRect;
                float w = es * Mathf.Clamp(k.Unit.definition.mapIconSize, 0.6f, 1.2f), h = w * tr.height / Mathf.Max(1f, tr.width);
                if (targets != null && targets.Contains(k.Unit))
                    Border(new Rect(s.x - es * 0.8f, s.y - es * 0.8f, es * 1.6f, es * 1.6f), Color.white, Mathf.Max(1.5f, 0.004f * S), 0f);
                Matrix4x4 saved = GUI.matrix;
                if (k.Orient) GUIUtility.RotateAroundPivot(mapRot - k.Icon.transform.eulerAngles.z, s);
                GUI.color = k.Icon.color;
                GUI.DrawTextureWithTexCoords(new Rect(s.x - w * 0.5f, s.y - h * 0.5f, w, h), t,
                    new Rect(tr.x / t.width, tr.y / t.height, tr.width / t.width, tr.height / t.height));
                GUI.matrix = saved;
            }
            GUI.color = savedCol;

            // NAV route: jet -> next waypoint -> ..., each marked with its number and clearance
            GlobalPosition g = me.GlobalPosition();
            Vector2 c = ToScreen(g.x, g.z), prev = c;
            float lw = Mathf.Max(1.5f, 0.003f * S), wr = Mathf.Max(5f, 0.009f * S);
            for (int i = 0; i < ap.Route.Count; i++)
            {
                Vector2 s = ToScreen(ap.Route[i].X, ap.Route[i].Z);
                Line(prev, s, lw, i == 0 && ap.Nav ? NavCol : NavDim);
                prev = s;
            }
            for (int i = 0; i < ap.Route.Count; i++)
            {
                Vector2 s = ToScreen(ap.Route[i].X, ap.Route[i].Z);
                Border(new Rect(s.x - wr, s.y - wr, 2f * wr, 2f * wr), NavCol, lw, wr);
                Text(new Rect(s.x + wr * 1.4f, s.y - wr * 2f, 0.3f * S, wr * 4f), WaypointLabel(i), 0.024f, NavCol, TextAnchor.MiddleLeft);
            }

            // own jet: dot + heading stick
            Vector3 f = me.transform.forward;
            Matrix4x4 m = GUI.matrix;
            GUIUtility.RotateAroundPivot(Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg, c);
            Fill(new Rect(c.x - 1f, c.y - 0.05f * S, 2f, 0.05f * S), Color.white, 0f);
            GUI.matrix = m;
            Fill(new Rect(c.x - 3.5f, c.y - 3.5f, 7f, 7f), Color.white, 3.5f);
            GUI.EndGroup();

            if (!bare) DrawLegends(); // the web page has its own controls

            // bottom status line: what is shown + colour key
            Fill(Sc(0f, 0.92f, 1f, 0.08f), LegendBack, 0f);
            string status = Status(out bool amber);
            Text(Sc(0.02f, 0.92f, 0.5f, 0.08f), status, 0.03f, amber ? Amber : Green, TextAnchor.MiddleLeft);
            Color green = new Color(0.35f, 0.95f, 0.45f), red = new Color(1f, 0.3f, 0.25f);
            bool en = Current == Mode.Enemy;
            Text(Sc(0.5f, 0.92f, 0.48f, 0.08f), "SEEN", 0.026f, en ? red : green, TextAnchor.MiddleLeft);
            Text(Sc(0.66f, 0.92f, 0.32f, 0.08f), "HIDDEN", 0.026f, en ? green : red, TextAnchor.MiddleRight);
        }

        /// <summary>Bottom status line: what is shown; amber = a problem (no radar, jammed).</summary>
        private string Status(out bool amber)
        {
            bool noRadar = Current == Mode.Own && ownRadarMissing;
            bool ownJammed = !noRadar && me.radar != null && profiles.TryGetValue(me.radar, out Profile op) && op.Jammed;
            amber = noRadar || (Current == Mode.Own ? ownJammed : jammedCount > 0);
            return grid != null && !grid.Ready ? $"MAPPING TERRAIN {grid.Progress * 100f:F0}%"
                : mapSettings == null ? "NO MAP"
                : Current == Mode.Own
                ? (noRadar ? "NO RADAR" : ownJammed ? "OWN RDR JAMMED" : me.radar.activated ? "OWN RDR" : "OWN RDR OFF")
                : (Current == Mode.Enemy ? $"{siteCount} ENMY RDR" : $"{siteCount} RDR SITES") + (jammedCount > 0 ? $" {jammedCount} JAM" : "");
        }

        private string WaypointLabel(int i) =>
            $"{i + 1}  {Units.Alt(ap.Route[i].Clearance):F0} {Units.AltUnit}" + (float.IsNaN(ap.Route[i].Speed) ? "" : $"  {Units.Speed(ap.Route[i].Speed):F0} {Units.SpeedUnit}");

        // =====================================================================
        // Browser-drawn map (WebMirror): the page draws the map itself from this data, so the game
        // renders and encodes nothing for it. The map image and terrain shading come from the flight
        // planner's map file; the coverage overlay and the fine terrain tile are sent when they change.
        // =====================================================================
        private int builtSeq, sentSeq = -1, detailSeq = -1;
        private float builtOpacity;
        private TerrainHeightGrid detailSent;

        /// <summary>
        /// The coverage overlay for /overlay: x, z, half-width (m), opacity (float32 each), enemy view (int32), then
        /// the Tex x Tex states (0 none, 1 masked, 2 covered; row 0 = south) run-length coded as (state byte, count uint16) pairs.
        /// </summary>
        public volatile byte[] OverlayData;

        /// <summary>The fine terrain tile for /detail: centre x, z and size (m, float32 each), then its shading as PNG.</summary>
        public volatile byte[] DetailData;

        private byte[] EncodeOverlay()
        {
            var ms = new MemoryStream(64 * 1024);
            var w = new BinaryWriter(ms);
            w.Write(texX);
            w.Write(texZ);
            w.Write(texR);
            w.Write(builtOpacity);
            w.Write(bEnemy ? 1 : 0);
            for (int k = 0; k < state.Length;)
            {
                byte v = state[k];
                int n = 1;
                while (k + n < state.Length && state[k + n] == v && n < ushort.MaxValue) n++;
                w.Write(v);
                w.Write((ushort)n);
                k += n;
            }
            return ms.ToArray();
        }

        /// <summary>Encode the fine tile's shading as PNG on a worker thread, once per tile.</summary>
        private void EncodeDetail()
        {
            if (detailTex == null || detail == detailSent) return;
            TerrainHeightGrid d = detailSent = detail;
            Color32[] px = d.Relief();
            ThreadPool.QueueUserWorkItem(_ =>
            {
                try
                {
                    byte[] png = ImageConversion.EncodeArrayToPNG(px, GraphicsFormat.R8G8B8A8_UNorm, (uint)d.N, (uint)d.N);
                    var ms = new MemoryStream(png.Length + 12);
                    var w = new BinaryWriter(ms);
                    w.Write(d.CX);
                    w.Write(d.CZ);
                    w.Write(d.Size);
                    w.Write(png);
                    DetailData = ms.ToArray();
                    Interlocked.Increment(ref detailSeq);
                }
                catch (Exception) { /* the web map shows the coarse shading there */ }
            });
        }

        /// <summary>
        /// The map's live state as JSON for the browser: view, jet, overlay / tile versions, radar sites, contacts,
        /// route and status line. Positions are global metres. Main thread, a few times a second.
        /// </summary>
        public void WebState(StringBuilder sb)
        {
            if (!buildActive && sentSeq != builtSeq) // state[] holds the finished overlay until the next build starts
            {
                sentSeq = builtSeq;
                OverlayData = EncodeOverlay();
            }
            EncodeDetail();
            var inv = CultureInfo.InvariantCulture;
            sb.Append("{\"map\":\"").Append(mapSettings != null ? FlightPlanner.Safe(mapSettings.name) : "").Append('"');
            sb.Append(",\"rmax\":").Append((int)(MapHalfSize() * 1.5f));
            sb.Append(",\"view\":[").Append((int)viewX).Append(',').Append((int)viewZ).Append(',').Append((int)viewR).Append(']');
            sb.Append(",\"folw\":").Append(follow ? "true" : "false");
            sb.Append(",\"ov\":").Append(OverlayData != null ? sentSeq : -1);
            sb.Append(",\"det\":").Append(detailTex != null && DetailData != null ? Volatile.Read(ref detailSeq) : -1);
            sb.Append(",\"en\":").Append(Current == Mode.Enemy ? "true" : "false");
            sb.Append(",\"nav\":").Append(ap.Nav ? "true" : "false");
            if (me == null)
            {
                sb.Append('}');
                return;
            }
            string status = Status(out bool amber);
            sb.Append(",\"status\":\"").Append(status).Append("\",\"amber\":").Append(amber ? "true" : "false");

            // jet: x, z, heading (deg), ground velocity x, z (m/s) so the page can move it between updates
            GlobalPosition g = me.GlobalPosition();
            Vector3 f = me.transform.forward;
            Rigidbody rb = me.CockpitRB();
            Vector3 v = rb != null ? rb.velocity : Vector3.zero;
            sb.Append(",\"jet\":[").Append((int)g.x).Append(',').Append((int)g.z).Append(',').Append((int)(Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg))
              .Append(',').Append((int)v.x).Append(',').Append((int)v.z).Append(']');

            // radar sites: x, z, colour (0 friendly, 1 enemy, 2 jammed)
            sb.Append(",\"sites\":[");
            bool first = true;
            if (Current != Mode.Own)
                foreach (Profile p in profiles.Values)
                {
                    if (p.Radar == me.radar || !p.Valid || p.Enemy != (Current == Mode.Enemy)) continue;
                    sb.Append(first ? "[" : ",[").Append((int)p.Origin.x).Append(',').Append((int)p.Origin.z).Append(',').Append(p.Jammed ? 2 : p.Enemy ? 1 : 0).Append(']');
                    first = false;
                }

            // contacts: unit id, x, z, rotation (deg clockwise, null = upright), colour, icon texture id, sprite rect in
            // the PNG (x, y from the top, w, h), size factor, selected
            DynamicMap map = SceneSingleton<DynamicMap>.i;
            float mapRot = map != null && map.mapImage != null ? map.mapImage.transform.eulerAngles.z : 0f;
            List<Unit> targets = TargetPanel.Targets();
            sb.Append("],\"con\":[");
            first = true;
            foreach (Contact k in contacts)
            {
                if (k.Unit == null || k.Icon == null || k.Icon.sprite == null) continue;
                Texture2D tex = k.Icon.sprite.texture;
                if (tex == null || !iconAlpha.ContainsKey(tex)) continue;
                GlobalPosition cp = k.Track.GetPosition();
                Rect tr = k.Icon.sprite.textureRect;
                sb.Append(first ? "{" : ",{").Append("\"u\":").Append(k.Unit.GetInstanceID()).Append(",\"x\":").Append((int)cp.x).Append(",\"z\":").Append((int)cp.z);
                sb.Append(",\"r\":");
                if (k.Orient) sb.Append((int)(mapRot - k.Icon.transform.eulerAngles.z));
                else sb.Append("null");
                sb.Append(",\"c\":\"#").Append(ColorUtility.ToHtmlStringRGBA(k.Icon.color)).Append("\",\"i\":").Append(tex.GetInstanceID());
                sb.Append(",\"uv\":[").Append((int)tr.x).Append(',').Append((int)(tex.height - tr.y - tr.height)).Append(',').Append((int)tr.width).Append(',').Append((int)tr.height).Append(']');
                sb.Append(",\"k\":").Append(Mathf.Clamp(k.Unit.definition.mapIconSize, 0.6f, 1.2f).ToString("0.###", inv));
                sb.Append(",\"s\":").Append(targets != null && targets.Contains(k.Unit) ? "true" : "false").Append('}');
                first = false;
            }

            // route: x, z, label
            sb.Append("],\"wps\":[");
            for (int i = 0; i < ap.Route.Count; i++)
                sb.Append(i > 0 ? ",[" : "[").Append((int)ap.Route[i].X).Append(',').Append((int)ap.Route[i].Z).Append(",\"").Append(WaypointLabel(i)).Append("\"]");
            sb.Append("]}");
        }

        /// <summary>Global x/z -> position inside the screen rect (0,0 = its top-left).</summary>
        private Vector2 ToScreen(float gx, float gz)
        {
            float pxPerM = screen.width * 0.5f / viewR;
            return new Vector2(screen.width * 0.5f + (gx - viewX) * pxPerM, screen.height * 0.5f - (gz - viewZ) * pxPerM);
        }

        private float ContactSize => Mathf.Max(8f, 0.016f * S);

        private Vector2 ContactPos(Contact k)
        {
            GlobalPosition p = k.Track.GetPosition();
            return ToScreen(p.x, p.z);
        }

        /// <summary>Click on a contact: select it as a target, or deselect it if already selected.</summary>
        private bool ClickContact(Vector2 mp)
        {
            List<Unit> targets = TargetPanel.Targets();
            if (targets == null) return false;
            float best = Mathf.Max(10f, 0.025f * S); // hit radius: larger than the icon so it is easy to tap
            Unit hit = null;
            foreach (Contact k in contacts)
            {
                if (k.Unit == null) continue;
                float d = (ContactPos(k) + screen.position - mp).magnitude;
                if (d < best)
                {
                    best = d;
                    hit = k.Unit;
                }
            }
            if (hit == null) return false;
            CombatHUD hud = SceneSingleton<CombatHUD>.i;
            if (targets.Contains(hit)) hud.DeSelectUnit(hit);
            else hud.SelectUnit(hit);
            return true;
        }

        private void DrawLegends()
        {
            foreach (Osb b in Buttons)
            {
                Rect r = LabelRect(b);
                bool lit = flashCmd == b.Command && Time.unscaledTime < flashUntil;
                if (b.Glyph != Glyph.Text)
                {
                    DrawArrow(r, b, b.Glyph == Glyph.Up);
                    continue;
                }
                TextAnchor anchor = b.Side == Side.Left ? TextAnchor.MiddleLeft : b.Side == Side.Right ? TextAnchor.MiddleRight : TextAnchor.UpperCenter;
                string label;
                bool boxed = false;
                switch (b.Command)
                {
                    case ApCommand.RdrAll: label = "ALL"; boxed = Current == Mode.All; break;
                    case ApCommand.RdrOwn: label = "OWN"; boxed = Current == Mode.Own; break;
                    case ApCommand.RdrEnemy: label = "ENMY"; boxed = Current == Mode.Enemy; break;
                    case ApCommand.RdrTgt: label = TgtLabel(); break;
                    case ApCommand.ModeNav: label = "NAV"; boxed = ap.Nav; break;
                    case ApCommand.WptClear: label = "CLR\nWPT"; break;
                    case ApCommand.RdrHide: label = "HIDE"; break;
                    case ApCommand.RdrFollow: label = "FOLW"; boxed = follow; break;
                    case ApCommand.RdrRcsOwn:
                        // the RCS this view is drawn for (boxed = own jet's live RCS)
                        label = $"RCS\n{TargetRcs():0.###}";
                        boxed = RcsSetting.Value <= 0f;
                        break;
                    default: label = "MAP"; break;
                }
                Legend(r, label, anchor, boxed, false, lit);
            }

            // range readout between the zoom arrows (half-width of the view)
            Rect rr = Sc(0.8f, 0.45f, 0.19f, 0.1f);
            Fill(rr, LegendBack, 0f);
            Text(new Rect(rr.x, rr.y, rr.width - 0.01f * S, rr.height * 0.45f), "RNG KM", 0.022f, GreenDim, TextAnchor.UpperRight);
            Text(new Rect(rr.x, rr.y + rr.height * 0.4f, rr.width - 0.01f * S, rr.height * 0.6f), $"{viewR / 1000f:F0}", 0.036f, Green, TextAnchor.UpperRight);
        }
    }
}
