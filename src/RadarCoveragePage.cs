using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
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
    /// ENMY also shows in yellow where an enemy radar paints you without detecting you: the game sends
    /// the RWR warning (and the map ping) from every radar check that reaches you - out to 2x its range,
    /// in its cone, above the horizon, in line of sight - before it looks at signal strength.
    ///
    /// EYES adds visual spotting (TargetDetector.VisualCheck): a detector sees a target closer than
    /// min(visualRange, target visibility x magnification) in line of sight, if the target is no faster than
    /// its maxSpeed. Cyan (friendly) / orange (enemy). The target is your own jet's visibility.
    /// SAM draws anti-air rings (ALL: ours, ENMY: theirs), one per unit / battery: WeaponInfo.targetRequirements.maxRange (slant range), which the
    /// game's turrets engage inside. AVOID re-plans the route's legs around what ENMY shows (ThreatRouter).
    ///
    ///            T1 ALL   T2 OWN   T3 ENMY  T4 TGT   T5 HIDE
    ///   L1 FOLW  +--------------------------------------+  R1 NAV
    ///   L2 MAP   |                                      |  R2 ^ (zoom in)
    ///   L3 ^RCS  |  map + coverage (scroll = zoom,      |  R3 RNG
    ///   L4 RCS   |  drag = pan (on a waypoint: move it), right-click = waypoint) |  R4 v (zoom out)
    ///   L5 vRCS  +--------------------------------------+  R5 CLR WPT
    ///            B1 EYES  B2 SAM            B4 AVOID
    ///   (L4 = back to own RCS)
    ///
    /// Under the coverage, the terrain is shaded (hillshade + 100 m contours) from the height grid.
    ///
    /// The map's terrain is raycast once into a TerrainHeightGrid. Each radar's polar profile
    /// (Az bearings x Steps range bins) is read from that grid on a worker thread, and redone when
    /// the radar moves 2% of its range. The overlay is redrawn only when something changed, also on
    /// workers (line of sight, rasterising, colours, the web copy); the main thread only uploads it.
    /// Workers run below normal priority so the game's own threads always come first.
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
        private static readonly AccessTools.FieldRef<TargetDetector, float> EyeMaxSpeedRef =
            AccessTools.FieldRefAccess<TargetDetector, float>("maxSpeed");

        // Overlay states, in merge priority (a pixel keeps the highest any source gives it).
        private const byte SNone = 0, SMasked = 1, SPainted = 2, SEye = 3, SRadar = 4;
        private static readonly Color32 None = new Color32(0, 0, 0, 0);
        private static readonly Color KeyGreen = new Color(0.35f, 0.95f, 0.45f), KeyRed = new Color(1f, 0.3f, 0.25f),
            KeyOrange = new Color(1f, 0.55f, 0.15f), KeyYellow = new Color(1f, 0.9f, 0.25f), KeyCyan = new Color(0.3f, 0.8f, 1f);
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
            new Osb { Side = Side.Bottom, Index = 0, Command = ApCommand.RdrEyes, Glyph = Glyph.Text },
            new Osb { Side = Side.Bottom, Index = 1, Command = ApCommand.RdrSam, Glyph = Glyph.Text },
            new Osb { Side = Side.Bottom, Index = 3, Command = ApCommand.RdrAvoid, Glyph = Glyph.Text },
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
            public int Sweep;      // sweeps swapped in so far

            // build worker only: Vis and the sweep / target it was computed for (VisAgl: height key; NaN = stale)
            public readonly byte[] Vis = new byte[Az * Steps];
            public float VisAgl = float.NaN, VisRcs, VisRadius;
            public int VisSweep;

            // Sweep wanted (Pending) / running (Job): written into back buffers on a worker, then swapped in.
            public GlobalPosition NextOrigin;
            public float NextRange, NextAlt;
            public bool Pending;
            public Task Job;
            public float[] NextHeight, NextBefore; // rented from `spare` only while a sweep runs
        }

        /// <summary>
        /// One detector's visual coverage: a small polar line-of-sight table taken straight from the height
        /// grid whenever it is needed (EyeAz x EyeSteps lookups), out to the range it spots the target at.
        /// </summary>
        private sealed class Eye
        {
            public GlobalPosition Origin;  // scan point (enemy: at the known position)
            public float Alt, Range;       // scan point height above sea level; spotting range for the target (m)
            public int Version;            // bumped when Origin / Alt / Range change
            // build worker only: SEye / SMasked / SNone at each bin's far edge, and the version / target height key it is for
            public readonly byte[] Vis = new byte[EyeAz * EyeSteps];
            public float VisKey = float.NaN;
            public int VisVersion;
        }

        private const int EyeAz = 360, EyeSteps = 128;
        private readonly Dictionary<TargetDetector, Eye> eyes = new Dictionary<TargetDetector, Eye>();
        private readonly Dictionary<Unit, TargetDetector[]> detectorsOf = new Dictionary<Unit, TargetDetector[]>();

        /// <summary>An anti-air weapon's engagement ring: slant range from the unit (m).</summary>
        private struct Ring
        {
            public float X, Z, Y, Max, Min;
            public bool Missile;
        }

        private readonly List<Ring> rings = new List<Ring>();      // every enemy AA weapon (AVOID)
        private readonly List<Ring> shownRings = new List<Ring>(); // the ones drawn (ShowRing)
        private readonly Dictionary<Unit, Ring[]> ringsOf = new Dictionary<Unit, Ring[]>(); // per unit: its weapons' ranges, no position

        // Back buffers are shared: a radar holds a pair only during its sweep, which halves the
        // memory the profiles keep (about 5 MB per radar instead of 10 MB).
        private static readonly Stack<float[]> spare = new Stack<float[]>();

        private static float[] Rent() => spare.Count > 0 ? spare.Pop() : new float[Az * Steps];

        // Replaced profiles' arrays: a running build or AVOID may still read them, so they go back to `spare` only after.
        private readonly List<float[]> retired = new List<float[]>();

        // Radar work runs on at most this many pool threads (a quarter of the cores), below normal priority.
        private static readonly int Workers = Mathf.Clamp(Environment.ProcessorCount / 4, 1, 4);

        private static void Background(Action work)
        {
            Thread t = Thread.CurrentThread;
            System.Threading.ThreadPriority was = t.Priority;
            t.Priority = System.Threading.ThreadPriority.BelowNormal;
            try { work(); }
            finally { t.Priority = was; }
        }

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

        // The overlay is built on workers from a snapshot taken on the main thread (StartBuild), into
        // `state` and `pixels`, then uploaded in one go (FinishBuild). Nothing here changes while it runs.
        private struct RadarSrc
        {
            public Profile P;
            public float[] Height, Before;
            public float Ox, Oz, Alt, Range;
            public int Sweep;
            public Cone Cone;
            public RadarParams Rp;
        }

        private struct EyeSrc
        {
            public Eye E;
            public float Ox, Oz, Alt, Range;
            public int Version;
        }

        private const int Band = 32; // texture rows per parallel work item
        private readonly List<RadarSrc> building = new List<RadarSrc>();
        private readonly List<EyeSrc> buildingEyes = new List<EyeSrc>(); // rasterised after the radars
        private Task buildJob;
        private bool buildActive => buildJob != null;
        private float bX, bZ, bR;                    // view being built
        private bool bEnemy;
        private float bAgl, bTgtAsl, bHKey, bRcs, bRadius, bOpacity; // target and opacity being built for
        private TerrainHeightGrid bGrid;
        private Color32[] bPal;
        private byte[] bEncoded;                     // the build's overlay for the web page

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
        /// Bake a copy once per texture: white, with the shape's brightness as alpha. False if nothing was baked.
        /// </summary>
        private static bool BakeIcon(Texture2D src)
        {
            if (src == null || iconAlpha.ContainsKey(src)) return false;
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
            return true;
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
            profiles.Clear(); // building / buildingEyes belong to a build that may still be running
            eyes.Clear();
            detectorsOf.Clear();
            ringsOf.Clear();
            rings.Clear();
            shownRings.Clear();
            avoid = null;
            contacts.Clear();
            me = null; // a build still running finishes on its own; the dirty rebuild below replaces it
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
                case ApCommand.RdrEyes: cfg.RadarEyes.Value = !cfg.RadarEyes.Value; nextScan = 0f; break;
                case ApCommand.RdrSam: cfg.RadarSams.Value = !cfg.RadarSams.Value; nextScan = 0f; break;
                case ApCommand.RdrAvoid: StartAvoid(); break;
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
                CollectEyesAndRings();
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
                UpdateSweeps();
            }
            if (avoid != null) StepAvoid();
            if (!buildActive && avoid == null && retired.Count > 0)
            {
                foreach (float[] a in retired) spare.Push(a);
                retired.Clear();
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
            if (buildActive)
            {
                if (buildJob.IsCompleted) FinishBuild();
            }
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
            {
                grid = new TerrainHeightGrid(MapHalfSize() * 2f);
                // the terrain only changes with the game: a new game version (or map size) maps it again
                grid.UseCache(Path.Combine(FlightPlanner.Dir, "terrain", FlightPlanner.Safe(mapSettings.name) + ".bin"),
                    $"{Application.version} {mapSettings.MapSize.x.ToString("R", CultureInfo.InvariantCulture)} {mapSettings.MapSize.y.ToString("R", CultureInfo.InvariantCulture)}");
            }
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
                else if (!p.Pending)
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
            enemyScanned = enemy;
        }

        private bool enemyScanned; // the last radar scan was for ENMY (AVOID waits for one)

        // reused by CollectEyesAndRings each second instead of new collections
        private readonly HashSet<TargetDetector> liveEyes = new HashSet<TargetDetector>();
        private readonly List<TargetDetector> scratchEyes = new List<TargetDetector>();
        private readonly List<Unit> scratchUnits = new List<Unit>();

        /// <summary>
        /// Visual detectors and anti-air rings of the side the view shows (OWN: own jet's eyes only). Enemy
        /// units sit at the position our side knows; never tracked = left out, as for radars.
        /// </summary>
        private void CollectEyesAndRings()
        {
            rings.Clear();
            shownRings.Clear();
            bool enemy = Current == Mode.Enemy, own = Current == Mode.Own;
            liveEyes.Clear();
            HashSet<TargetDetector> live = liveEyes;
            if (me.NetworkHQ != null)
                foreach (Unit u in UnitRegistry.allUnits)
                {
                    if (u == null || u.disabled || u is Missile || u.NetworkHQ == null || (u.NetworkHQ != me.NetworkHQ) != enemy || (own && u != me)) continue;
                    GlobalPosition truePos = u.GlobalPosition(), at = truePos;
                    if (enemy && !me.NetworkHQ.TryGetKnownPosition(u, out at)) continue;
                    if (cfg.RadarEyes.Value || avoid != null) // AVOID plans with them even when not drawn
                        foreach (TargetDetector d in Detectors(u))
                        {
                            // activated is ignored for the enemy (worst case: on), as for their radars
                            if (d == null || !d.IsOperational() || d.GetVisualRange() <= 0f || d.GetScanPoint() == null || (!enemy && !d.activated)) continue;
                            if (enemy && me.speed > EyeMaxSpeedRef(d)) continue; // too fast for these eyes to pick up
                            float range = Mathf.Min(d.GetVisualRange(), me.GetVisibility() * d.GetVisualMagnification());
                            if (range < 50f) continue;
                            live.Add(d);
                            UpdateEye(d, at + (d.GetScanPoint().GlobalPosition() - truePos), range);
                        }
                    // ALL: our air defence, to see the gaps; ENMY: the threats (AVOID plans with every enemy weapon)
                    if ((cfg.RadarSams.Value || avoid != null) && !own && !(u is Aircraft))
                    {
                        Ring widest = default;
                        foreach (Ring r in RingsOf(u))
                        {
                            Ring placed = r;
                            placed.X = at.x;
                            placed.Z = at.z;
                            placed.Y = at.y;
                            if (enemy) rings.Add(placed); // a gun can cover a missile's minimum range
                            if (r.Max > widest.Max) widest = placed;
                        }
                        if (widest.Max > 0f && cfg.RadarSams.Value) ShowRing(widest);
                    }
                }
            var gone = scratchEyes;
            gone.Clear();
            foreach (TargetDetector d in eyes.Keys)
                if (!live.Contains(d)) gone.Add(d);
            foreach (TargetDetector d in gone) eyes.Remove(d);
            if (gone.Count > 0) Dirty("eye lost");

            // forget destroyed units (Unity null)
            var deadUnits = scratchUnits;
            deadUnits.Clear();
            foreach (Unit u in detectorsOf.Keys)
                if (u == null) deadUnits.Add(u);
            foreach (Unit u in ringsOf.Keys)
                if (u == null) deadUnits.Add(u);
            foreach (Unit u in deadUnits)
            {
                detectorsOf.Remove(u);
                ringsOf.Remove(u);
            }
        }

        private TargetDetector[] Detectors(Unit u)
        {
            if (!detectorsOf.TryGetValue(u, out TargetDetector[] d)) detectorsOf[u] = d = u.GetComponentsInChildren<TargetDetector>(true);
            return d;
        }

        /// <summary>The unit's anti-air weapons, one ring per weapon type (cached: a unit's weapons don't change).</summary>
        /// <summary>
        /// Drawn rings: each unit's widest only, and one for a battery: a ring whose centre and radius are within
        /// 10% / 5% of one already drawn would just thicken that line.
        /// </summary>
        private void ShowRing(Ring r)
        {
            foreach (Ring o in shownRings)
            {
                float dx = o.X - r.X, dz = o.Z - r.Z, near = 0.1f * r.Max;
                if (Mathf.Abs(o.Max - r.Max) <= 0.05f * r.Max && dx * dx + dz * dz <= near * near) return;
            }
            shownRings.Add(r);
        }

        private Ring[] RingsOf(Unit u)
        {
            if (ringsOf.TryGetValue(u, out Ring[] r)) return r;
            var seen = new HashSet<WeaponInfo>();
            var list = new List<Ring>();
            foreach (Weapon w in u.GetComponentsInChildren<Weapon>(true))
            {
                WeaponInfo i = w != null ? w.info : null;
                if (i == null || !seen.Add(i) || i.cargo || i.troops || i.effectiveness.antiAir <= 0f || i.targetRequirements.maxRange <= 0f) continue;
                list.Add(new Ring { Max = i.targetRequirements.maxRange, Min = i.targetRequirements.minRange, Missile = i.missile });
            }
            return ringsOf[u] = list.ToArray();
        }

        private void UpdateEye(TargetDetector d, GlobalPosition pos, float range)
        {
            bool fresh = !eyes.TryGetValue(d, out Eye e);
            if (fresh) eyes[d] = e = new Eye();
            // 5% of its range, as friendly radars: moving aircraft would otherwise redraw the map every second
            else if ((pos - e.Origin).magnitude <= Mathf.Max(30f, e.Range * 0.05f) && Mathf.Abs(range - e.Range) <= e.Range * 0.05f) return;
            // redraw only if the drawn overlay shows it, before or after (a build in progress may already have used the old one)
            bool shown = cfg.RadarEyes.Value && (buildActive || OnOverlay(pos, range) || (!fresh && OnOverlay(e.Origin, e.Range)));
            e.Origin = pos;
            e.Alt = pos.ToLocalPosition().y - Datum.LocalSeaY;
            e.Range = range;
            e.Version++;
            if (shown) Dirty("eye");
        }

        private bool OnOverlay(GlobalPosition p, float r) => Mathf.Abs(p.x - texX) < texR + r && Mathf.Abs(p.z - texZ) < texR + r;

        /// <summary>
        /// Line of sight from an eye along each bearing, as the game's LoS raycast: a target at TargetAsl at
        /// the far edge of a bin is seen when it clears the steepest terrain before it and lies inside the
        /// spotting range (3D, as FastMath.InRange). No horizon or cone for eyes. Any thread.
        /// </summary>
        private static void EyeViewshed(TerrainHeightGrid grid, float ox, float oz, float alt, float range, float agl, float tgtAsl, byte[] vis)
        {
            float step = range / EyeSteps, r2 = range * range;
            int fine = Mathf.Clamp(Mathf.CeilToInt(step / grid.Cell), 1, 8);
            float sub = step / fine;
            for (int a = 0; a < EyeAz; a++)
            {
                float brg = (a + 0.5f) * (2f * Mathf.PI / EyeAz), sx = Mathf.Sin(brg), sz = Mathf.Cos(brg);
                float maxSlope = float.NegativeInfinity;
                int b = a * EyeSteps;
                for (int i = 0; i < EyeSteps; i++)
                {
                    float d0 = i * step;
                    for (int j = 1; j < fine; j++)
                    {
                        float dj = d0 + j * sub;
                        maxSlope = Mathf.Max(maxSlope, (grid.Height(ox + sx * dj, oz + sz * dj) - alt) / dj);
                    }
                    float d = d0 + step, t = grid.Height(ox + sx * d, oz + sz * d), dy = TargetAsl(t, agl, tgtAsl) - alt;
                    vis[b + i] = d * d + dy * dy > r2 ? SNone : dy / d >= maxSlope ? SEye : SMasked;
                    maxSlope = Mathf.Max(maxSlope, (t - alt) / d);
                }
            }
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
            bool baked = false; // one icon per call: each bake waits for the GPU (ReadPixels); an unbaked contact is drawn once its icon is
            foreach (MapIcon icon in map.mapIcons)
            {
                if (!(icon is UnitMapIcon ui) || !TargetPanel.ShownEnemy(ui, filter)) continue;
                Unit u = ui.unit;
                // never tracked = position unknown: no true-position fallback
                TrackingInfo track = map.HQ != null ? map.HQ.GetTrackingData(u.persistentID) : null;
                if (track == null) continue;
                if (!baked && ui.iconImage.sprite != null) baked = BakeIcon(ui.iconImage.sprite.texture);
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
            p.Pending = true;
        }

        /// <summary>Swap in finished sweeps and start waiting ones on workers, own radar first so the OWN page fills quickly.</summary>
        private void UpdateSweeps()
        {
            int running = 0;
            foreach (Profile p in profiles.Values)
                if (p.Job != null && !FinishSweep(p)) running++;
            if (me.radar != null && profiles.TryGetValue(me.radar, out Profile own)) StartSweep(own, ref running);
            foreach (Profile p in profiles.Values) StartSweep(p, ref running);
        }

        private void StartSweep(Profile p, ref int running)
        {
            if (!p.Pending || p.Job != null || running >= Workers) return;
            running++;
            if (p.NextHeight == null)
            {
                p.NextHeight = Rent();
                p.NextBefore = Rent();
            }
            TerrainHeightGrid g = grid;
            float ox = p.NextOrigin.x, oz = p.NextOrigin.z, alt = p.NextAlt, range = p.NextRange;
            float[] height = p.NextHeight, before = p.NextBefore;
            p.Job = Task.Run(() => Background(() =>
            {
                long t = Perf.Now;
                Sweep(g, ox, oz, alt, range, height, before);
                Perf.Add(Perf.Sweep, t);
            }));
        }

        /// <summary>Swap a finished sweep in; false while it still runs.</summary>
        private bool FinishSweep(Profile p)
        {
            if (!p.Job.IsCompleted) return false;
            Task job = p.Job;
            p.Job = null;
            p.Pending = false;
            if (job.IsFaulted)
            {
                Debug.LogException(job.Exception);
                p.NextHeight = p.NextBefore = null; // the profile keeps its last sweep
                return true;
            }
            if (p.Height != null)
            {
                retired.Add(p.Height);
                retired.Add(p.Before);
            }
            p.Height = p.NextHeight;
            p.Before = p.NextBefore;
            p.NextHeight = p.NextBefore = null;
            p.Origin = p.NextOrigin;
            p.Range = p.NextRange;
            p.Alt = p.NextAlt;
            p.Valid = true;
            p.Sweep++;
            Dirty("sweep");
            Perf.Sweeps++;
            return true;
        }

        /// <summary>
        /// A radar's profile from the height grid (no raycasts). Each range bin stores the terrain at
        /// its far edge plus the steepest slope of grid-spaced samples inside it, so ridges narrower
        /// than a bin still mask what lies behind them. Any thread.
        /// </summary>
        private static void Sweep(TerrainHeightGrid grid, float ox, float oz, float alt, float range, float[] height, float[] before)
        {
            float step = range / Steps;
            int fine = Mathf.Clamp(Mathf.CeilToInt(step / grid.Cell), 1, 32);
            float sub = step / fine;
            for (int a = 0; a < Az; a++)
            {
                int b = a * Steps;
                float brg = (a + 0.5f) * (2f * Mathf.PI / Az), sx = Mathf.Sin(brg), sz = Mathf.Cos(brg);
                for (int i = 0; i < Steps; i++)
                {
                    float steepest = float.NegativeInfinity, d0 = i * step;
                    for (int j = 1; j < fine; j++)
                    {
                        float d = d0 + j * sub;
                        steepest = Mathf.Max(steepest, (grid.Height(ox + sx * d, oz + sz * d) - alt) / d);
                    }
                    float de = d0 + step;
                    height[b + i] = grid.Height(ox + sx * de, oz + sz * de);
                    before[b + i] = steepest;
                }
            }
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
        /// beats the radar's minimum after clutter. vis: SNone = out of reach, SMasked = signal strong enough
        /// but hidden, SRadar = detected, SPainted (only with <paramref name="rwr"/>) = in line of sight but
        /// too weak to detect: Aircraft.GetRadarReturn still fires the target's RWR warning there.
        /// Mirrors DetectorManager.RequestRadarCheck + RadarParams.GetSignalStrength with no ECM, where
        /// the doppler boost only applies once the signal already passes, so it can't change the result.
        /// </summary>
        internal static void Viewshed(float[] height, float[] before, float radarAlt, float agl, float tgtAsl, float step,
            RadarParams rp, float rcs, float tgtRadius, bool rwr, byte[] vis)
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
                    if (dist > maxDist) v = SNone;
                    else if (dy / d < maxSlope || radarHorizon + Mathf.Sqrt(TwoR * Mathf.Max(0f, tgt)) < d)
                        v = signal >= rp.minSignal ? SMasked : SNone;
                    else
                    {
                        // below the radar's line to its horizon, the ground behind the target adds clutter
                        float clutter = size2 / (h * h); // the ground right under the target
                        if (d < radarHorizon && tgt < radarAlt * (1f - d / radarHorizon))
                            clutter += Mathf.Min(dist, 1000f) * (radarAlt - tgt) / (dist * h);
                        v = signal - clutter * rp.clutterFactor >= rp.minSignal ? SRadar
                            : rwr ? SPainted
                            : signal >= rp.minSignal ? SMasked : SNone; // friendly: hidden by clutter, as before
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
            TargetHeight(out bAgl, out bTgtAsl, out bHKey);
            bRcs = TargetRcs(); // target: the RCS set for this view and own jet's size (ground clutter)
            bRadius = me.maxRadius;
            bGrid = grid;
            building.Clear();
            foreach (Profile p in profiles.Values)
            {
                if (!p.Valid || p.Jammed || p.Enemy != bEnemy || (Current == Mode.Own && p.Radar != me.radar)) continue;
                Transform scan = p.Radar != null ? p.Radar.GetScanPoint() : null;
                if (scan == null || !(p.Radar is Radar radar)) continue;
                building.Add(new RadarSrc
                {
                    P = p, Height = p.Height, Before = p.Before, Ox = p.Origin.x, Oz = p.Origin.z, Alt = p.Alt, Range = p.Range,
                    Sweep = p.Sweep, Cone = ConeOf(p, radar, scan), Rp = radar.RadarParameters,
                });
            }
            buildingEyes.Clear();
            if (cfg.RadarEyes.Value && grid != null && grid.Ready)
                foreach (Eye e in eyes.Values)
                    buildingEyes.Add(new EyeSrc { E = e, Ox = e.Origin.x, Oz = e.Origin.z, Alt = e.Alt, Range = e.Range, Version = e.Version });

            // enemy view: seen by them is the danger (radar red, eyes orange, RWR-only yellow), masked is safe (green)
            // opacity setting = red's alpha; green stays a little fainter, as drawn before (95 vs 130)
            float op = bOpacity = Mathf.Clamp01(cfg.RadarOpacity.Value);
            Color32 C(byte r, byte g, byte b, float a) => new Color32(r, g, b, (byte)(Mathf.Clamp01(a) * 255f));
            bPal = bEnemy
                ? new[] { None, C(60, 230, 90, op * 0.73f), C(255, 225, 60, op * 0.6f), C(255, 140, 30, op), C(240, 50, 40, op) }
                : new[] { None, C(240, 50, 40, op), None, C(70, 200, 255, op * 0.85f), C(60, 230, 90, op * 0.73f) };
            buildJob = Task.Run(() => Background(Build));
        }

        /// <summary>
        /// Worker: stale line-of-sight tables first, then the texture in bands of rows, each band merging every source
        /// (no two threads write the same pixel), then the web page's copy.
        /// </summary>
        private void Build()
        {
            long t = Perf.Now;
            var opts = new ParallelOptions { MaxDegreeOfParallelism = Workers };
            int radars = building.Count;
            Parallel.For(0, radars + buildingEyes.Count, opts, i => Background(() =>
            {
                if (i < radars) RadarVis(building[i]);
                else EyeVis(buildingEyes[i - radars]);
            }));
            Parallel.For(0, Tex / Band, opts, band => Background(() =>
            {
                int y0 = band * Band, y1 = y0 + Band, k1 = y1 * Tex;
                System.Array.Clear(state, y0 * Tex, Band * Tex);
                foreach (RadarSrc s in building) Rasterize(s, y0, y1);
                foreach (EyeSrc s in buildingEyes) RasterizeEye(s, y0, y1);
                for (int k = y0 * Tex; k < k1; k++) pixels[k] = bPal[state[k]];
            }));
            bEncoded = EncodeOverlay();
            Perf.Add(Perf.Build, t);
        }

        /// <summary>Main thread: upload a finished build.</summary>
        private void FinishBuild()
        {
            Task job = buildJob;
            buildJob = null;
            if (job.IsFaulted)
            {
                Debug.LogException(job.Exception);
                return; // the last overlay stays; `dirty` brings the next try
            }
            if (overlay == null)
                overlay = new Texture2D(Tex, Tex, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            overlay.SetPixels32(pixels); // row 0 = south, so the texture draws north-up
            overlay.Apply();
            texX = bX;
            texZ = bZ;
            texR = bR;
            OverlayData = bEncoded;
            builtSeq++;
            Perf.Builds++;
        }

        /// <summary>Texture pixel box a source at global x/z reaching `range` covers in the build's view; false if none.</summary>
        private bool Box(float gx, float gz, float range, out float ox, out float oy, out int x0, out int x1, out int y0, out int y1)
        {
            float half = Tex * 0.5f, mPerPx = bR / half, rp = range / mPerPx;
            ox = (gx - bX) / mPerPx + half;
            oy = (gz - bZ) / mPerPx + half;
            x0 = Mathf.Max(0, (int)(ox - rp));
            x1 = Mathf.Min(Tex - 1, (int)(ox + rp));
            y0 = Mathf.Max(0, (int)(oy - rp));
            y1 = Mathf.Min(Tex - 1, (int)(oy + rp));
            return x1 >= x0 && y1 >= y0;
        }

        /// <summary>Worker: redo a radar's line-of-sight table if it is in view and was taken for another sweep or target.</summary>
        private void RadarVis(RadarSrc s)
        {
            Profile p = s.P;
            // Approximately: hKey is computed, and Mono's extra precision would never match the stored float
            if ((p.VisSweep == s.Sweep && Mathf.Approximately(p.VisAgl, bHKey) && p.VisRcs == bRcs && p.VisRadius == bRadius)
                || !Box(s.Ox, s.Oz, s.Range, out _, out _, out _, out _, out _, out _)) return;
            Interlocked.Increment(ref Perf.Viewsheds);
            Viewshed(s.Height, s.Before, s.Alt, bAgl, bTgtAsl, s.Range / Steps, s.Rp, bRcs, bRadius, p.Enemy, p.Vis);
            p.VisSweep = s.Sweep;
            p.VisAgl = bHKey;
            p.VisRcs = bRcs;
            p.VisRadius = bRadius;
        }

        /// <summary>Worker: as RadarVis, for an eye.</summary>
        private void EyeVis(EyeSrc s)
        {
            Eye e = s.E;
            if ((e.VisVersion == s.Version && Mathf.Approximately(e.VisKey, bHKey)) || !Box(s.Ox, s.Oz, s.Range, out _, out _, out _, out _, out _, out _)) return;
            EyeViewshed(bGrid, s.Ox, s.Oz, s.Alt, s.Range, bAgl, bTgtAsl, e.Vis);
            e.VisVersion = s.Version;
            e.VisKey = bHKey;
        }

        /// <summary>Target the view is drawn for: height above ground, or (TGT ALT, agl &lt; 0) own altitude as snapped in Tick; hKey identifies it.</summary>
        private void TargetHeight(out float agl, out float tgtAsl, out float hKey)
        {
            agl = cfg.RadarTargetAgl.Value;
            tgtAsl = agl < 0f ? altSnap : float.NaN;
            hKey = agl < 0f ? -1f - altSnap : agl;
        }

        /// <summary>
        /// A radar's scan cone (half-angle, 0 = all round), checked in 3D so it also covers the radar's up/down
        /// limits for a target at that spot's height. Vector3.Angle(v, fwd) &gt; cone  &lt;=&gt;  dot(v, fwd) &lt; cos(cone)|v|
        /// (fwd is unit): no acos per pixel.
        /// </summary>
        private struct Cone
        {
            public bool Cull, Flat, ElevOnly; // Flat = azimuth-only check; ElevOnly = pitch band, any azimuth
            public float Half, Cos, Pitch;
            public Vector3 Fwd;

            /// <summary>Target at horizontal offset dx/dz (d2 = dx^2 + dz^2) and dy above the radar inside the cone?</summary>
            public bool Holds(float dx, float dz, float dy, float d2)
            {
                if (!Cull) return true;
                if (ElevOnly) return Mathf.Abs(Mathf.Atan2(dy, Mathf.Sqrt(d2)) * Mathf.Rad2Deg - Pitch) <= Half;
                float vy = Flat ? 0f : dy;
                return dx * Fwd.x + vy * Fwd.y + dz * Fwd.z >= Cos * Mathf.Sqrt(d2 + vy * vy);
            }
        }

        private static Cone ConeOf(Profile p, Radar radar, Transform scan)
        {
            float cone = RadarConeRef(radar);
            Vector3 fwd = scan.forward;
            bool flat = false, elevOnly = false;
            if (p.Enemy)
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
            return new Cone { Cull = cone > 0f && cone < 180f, Flat = flat, ElevOnly = elevOnly, Half = cone, Cos = Mathf.Cos(cone * Mathf.Deg2Rad), Pitch = p.ConePitch, Fwd = fwd };
        }

        /// <summary>Worker: merge one radar's coverage into rows ry0..ry1-1 of `state`.</summary>
        private void Rasterize(RadarSrc s, int ry0, int ry1)
        {
            if (!Box(s.Ox, s.Oz, s.Range, out float ox, out float oy, out int x0, out int x1, out int y0, out int y1)) return;
            float mPerPx = bR / (Tex * 0.5f), binsPerM = Steps / s.Range, r2 = s.Range * s.Range, radarAlt = s.Alt, agl = bAgl, tgtAsl = bTgtAsl;
            byte[] vis = s.P.Vis;
            float[] height = s.Height;
            Cone cone = s.Cone;
            for (int y = Mathf.Max(y0, ry0), ye = Mathf.Min(y1, ry1 - 1); y <= ye; y++)
            {
                float dz = (y + 0.5f - oy) * mPerPx;
                RowSpan(r2 - dz * dz, ox, mPerPx, x0, x1, out int xa, out int xb);
                for (int x = xa; x <= xb; x++)
                {
                    float dx = (x + 0.5f - ox) * mPerPx, d2 = dx * dx + dz * dz;
                    if (d2 >= r2) continue;
                    int k = y * Tex + x;
                    if (state[k] == SRadar) continue; // already detected by another radar
                    int bin = Bin(dx, dz, d2, binsPerM);
                    if (cone.Cull && !cone.Holds(dx, dz, TargetAsl(height[bin], agl, tgtAsl) - radarAlt, d2)) continue;
                    byte v = vis[bin];
                    if (v > state[k]) state[k] = v;
                }
            }
        }

        /// <summary>
        /// Columns of a row that can lie inside the circle (w2 = r^2 - dz^2): the box's corners are skipped, the
        /// per-pixel test still decides (one column of slack each side for rounding).
        /// </summary>
        private static void RowSpan(float w2, float ox, float mPerPx, int x0, int x1, out int xa, out int xb)
        {
            if (w2 <= 0f)
            {
                xa = 0;
                xb = -1;
                return;
            }
            float w = Mathf.Sqrt(w2) / mPerPx;
            xa = Mathf.Max(x0, (int)(ox - w) - 1);
            xb = Mathf.Min(x1, (int)(ox + w) + 1);
        }

        /// <summary>Profile bin (bearing x range step) of a point dx/dz from the radar; d2 = dx^2 + dz^2.</summary>
        private static int Bin(float dx, float dz, float d2, float binsPerM)
        {
            float brg = Mathf.Atan2(dx, dz);
            if (brg < 0f) brg += 2f * Mathf.PI;
            return Mathf.Min(Az - 1, (int)(brg * (Az / (2f * Mathf.PI)))) * Steps + Mathf.Min(Steps - 1, (int)(Mathf.Sqrt(d2) * binsPerM));
        }

        private static int EyeBin(float dx, float dz, float d2, float binsPerM)
        {
            float brg = Mathf.Atan2(dx, dz);
            if (brg < 0f) brg += 2f * Mathf.PI;
            return Mathf.Min(EyeAz - 1, (int)(brg * (EyeAz / (2f * Mathf.PI)))) * EyeSteps + Mathf.Min(EyeSteps - 1, (int)(Mathf.Sqrt(d2) * binsPerM));
        }

        /// <summary>Worker: merge one eye's coverage into rows ry0..ry1-1 of `state`, as Rasterize.</summary>
        private void RasterizeEye(EyeSrc s, int ry0, int ry1)
        {
            if (!Box(s.Ox, s.Oz, s.Range, out float ox, out float oy, out int x0, out int x1, out int y0, out int y1)) return;
            float mPerPx = bR / (Tex * 0.5f), binsPerM = EyeSteps / s.Range, r2 = s.Range * s.Range;
            byte[] vis = s.E.Vis;
            for (int y = Mathf.Max(y0, ry0), ye = Mathf.Min(y1, ry1 - 1); y <= ye; y++)
            {
                float dz = (y + 0.5f - oy) * mPerPx;
                RowSpan(r2 - dz * dz, ox, mPerPx, x0, x1, out int xa, out int xb);
                for (int x = xa; x <= xb; x++)
                {
                    float dx = (x + 0.5f - ox) * mPerPx, d2 = dx * dx + dz * dz;
                    if (d2 >= r2) continue;
                    int k = y * Tex + x;
                    if (state[k] >= SEye) continue;
                    byte v = vis[EyeBin(dx, dz, d2, binsPerM)];
                    if (v > state[k]) state[k] = v;
                }
            }
        }

        // =====================================================================
        // AVOID: re-plan the route's legs around what ENMY shows, at each leg's clearance
        // =====================================================================
        // Routing cells of RouteCell metres (2.5 height-grid cells), at most RouteMaxN a side: past that a big map gets coarser cells.
        // shortcut: a valley narrower than a cell is not found; lower RouteCell if routes miss gaps (work grows with 1 / cell^2)
        private const float RouteCell = 150f;
        private const int RouteMaxN = 1200;
        private const int StampBudget = 200000; // routing cells looked at per frame
        private IEnumerator<bool> avoid;  // the planning in progress, one step per frame
        private byte[] routeVis;          // a radar's line-of-sight table for a leg's clearance
        private string note;              // shown in the status line until noteUntil
        private float noteUntil;
        private bool noteAmber;

        private void Note(string text, bool amber)
        {
            note = text;
            noteAmber = amber;
            noteUntil = Time.unscaledTime + 5f;
        }

        private void StartAvoid()
        {
            if (me == null || ap.Route.Count == 0)
            {
                Note("AVOID - NO ROUTE", true);
                return;
            }
            if (Current != Mode.Enemy)
            {
                Current = Mode.Enemy; // the plan needs the enemy picture; it stays on show afterwards
                enemyScanned = false;
            }
            nextScan = 0f;
            avoid = Avoid();
        }

        private void StepAvoid()
        {
            try
            {
                if (!avoid.MoveNext()) avoid = null;
            }
            catch (Exception ex)
            {
                avoid = null;
                Note("AVOID FAILED", true);
                Debug.LogException(ex);
            }
        }

        private IEnumerator<bool> Avoid()
        {
            // wait for the full enemy picture: the terrain, an ENMY scan, and every radar's sweep
            float give = Time.unscaledTime + 60f;
            while (grid == null || !grid.Ready || !enemyScanned || Sweeping())
            {
                if (Time.unscaledTime > give || Current != Mode.Enemy)
                {
                    Note("AVOID - NO ENEMY DATA", true);
                    yield break;
                }
                yield return true;
            }

            var plan = new List<Waypoint>();
            foreach (Waypoint w in ap.Route) plan.Add(new Waypoint { X = w.X, Z = w.Z, Clearance = w.Clearance, Speed = w.Speed });
            GlobalPosition jet = me.GlobalPosition();
            float size = MapHalfSize() * 2f, x0 = -size * 0.5f;
            int n = Mathf.Clamp(Mathf.CeilToInt(size / RouteCell), 64, RouteMaxN);
            float cell = size / n;
            var costs = new Dictionary<int, float[]>(); // by clearance (whole metres)
            var result = new List<Waypoint>();
            float ax = jet.x, az = jet.z;
            foreach (Waypoint w in plan)
            {
                int key = Mathf.RoundToInt(w.Clearance);
                if (!costs.TryGetValue(key, out float[] cost))
                {
                    var flags = new byte[n * n];
                    for (IEnumerator<bool> s = ThreatFlags(w.Clearance, flags, n, x0, cell); s.MoveNext();) yield return true;
                    costs[key] = cost = CostOf(flags);
                }
                // A* over up to RouteMaxN^2 cells takes a while: off the main thread (the cost grid is not touched again)
                float sx = ax, sz = az;
                var job = System.Threading.Tasks.Task.Run(() => ThreatRouter.Path(cost, n, x0, x0, cell, sx, sz, w.X, w.Z));
                while (!job.IsCompleted) yield return true;
                List<float[]> path = job.Result; // a failure rethrows here: StepAvoid reports it
                for (int i = 0; i < path.Count - 1; i++)
                    result.Add(new Waypoint { X = path[i][0], Z = path[i][1], Clearance = w.Clearance, Speed = w.Speed });
                result.Add(w);
                ax = w.X;
                az = w.Z;
                yield return true;
            }

            // NAV may have flown past a waypoint, or the pilot edited the route, while this ran: don't undo that
            bool same = ap.Route.Count == plan.Count;
            for (int i = 0; same && i < plan.Count; i++) same = ap.Route[i].X == plan[i].X && ap.Route[i].Z == plan[i].Z;
            if (!same)
            {
                Note("ROUTE CHANGED - AVOID AGAIN", true);
                yield break;
            }
            ap.LoadRoute(result);
            Note($"AVOID +{result.Count - plan.Count} WPT", false);
            Dirty("avoid"); // the eyes' tables were taken at the legs' clearances
        }

        private bool Sweeping()
        {
            foreach (Profile p in profiles.Values)
                if (p.Enemy && (!p.Valid || p.Pending)) return true;
            return false;
        }

        // routing grid flags per cell
        private const byte FPainted = 1, FEye = 2, FRadar = 4, FSam = 8;

        /// <summary>
        /// What sees a target <paramref name="agl"/> above the ground at each routing cell: every known enemy radar
        /// (one line-of-sight table per frame; jammed ones too, jamming comes and goes), their eyes and their AA rings.
        /// </summary>
        private IEnumerator<bool> ThreatFlags(float agl, byte[] flags, int n, float x0, float cell)
        {
            float rcs = TargetRcs(), radius = me.maxRadius;
            if (routeVis == null) routeVis = new byte[Az * Steps];
            foreach (Profile p in new List<Profile>(profiles.Values))
            {
                Transform scan = p.Radar != null ? p.Radar.GetScanPoint() : null;
                if (!p.Enemy || !p.Valid || scan == null || !(p.Radar is Radar radar)) continue;
                Cone cone = ConeOf(p, radar, scan);
                float binsPerM = Steps / p.Range;
                // the sweep routeVis was taken from, even if a new one lands while this runs over several frames
                // (its arrays are only recycled once AVOID is done)
                float[] height = p.Height, before = p.Before;
                float alt = p.Alt, step = p.Range / Steps;
                RadarParams rp = radar.RadarParameters;
                Task los = Task.Run(() => Background(() => Viewshed(height, before, alt, agl, float.NaN, step, rp, rcs, radius, true, routeVis)));
                while (!los.IsCompleted) yield return true;
                los.Wait(); // rethrows a failure: StepAvoid reports it
                for (IEnumerator<bool> s = Stamp(p.Origin.x, p.Origin.z, p.Range, n, x0, cell, (dx, dz, d2) =>
                {
                    int bin = Bin(dx, dz, d2, binsPerM);
                    if (cone.Cull && !cone.Holds(dx, dz, height[bin] + agl - alt, d2)) return 0;
                    byte v = routeVis[bin];
                    return v == SRadar ? FRadar : v == SPainted ? FPainted : (byte)0;
                }, flags); s.MoveNext();) yield return true;
                yield return true;
            }
            foreach (Eye e in new List<Eye>(eyes.Values))
            {
                // its own table: the display's build worker owns e.Vis
                var vis = new byte[EyeAz * EyeSteps];
                EyeViewshed(grid, e.Origin.x, e.Origin.z, e.Alt, e.Range, agl, float.NaN, vis);
                float binsPerM = EyeSteps / e.Range;
                for (IEnumerator<bool> s = Stamp(e.Origin.x, e.Origin.z, e.Range, n, x0, cell,
                    (dx, dz, d2) => vis[EyeBin(dx, dz, d2, binsPerM)] == SEye ? FEye : (byte)0, flags); s.MoveNext();) yield return true;
            }
            foreach (Ring r in new List<Ring>(rings))
            {
                Ring rr = r;
                for (IEnumerator<bool> s = Stamp(r.X, r.Z, r.Max, n, x0, cell, (dx, dz, d2) =>
                {
                    float dy = grid.Height(rr.X + dx, rr.Z + dz) + agl - rr.Y, s2 = d2 + dy * dy;
                    return s2 <= rr.Max * rr.Max && s2 >= rr.Min * rr.Min ? FSam : (byte)0;
                }, flags); s.MoveNext();) yield return true;
            }
        }

        /// <summary>
        /// OR <paramref name="at"/>'s flags into every routing cell within <paramref name="range"/> of x/z, pausing
        /// (yield) every StampBudget cells: one long-range radar can cover the whole map.
        /// </summary>
        private static IEnumerator<bool> Stamp(float x, float z, float range, int n, float x0, float cell, Func<float, float, float, byte> at, byte[] flags)
        {
            int i0 = Mathf.Max(0, (int)((x - range - x0) / cell)), i1 = Mathf.Min(n - 1, (int)((x + range - x0) / cell));
            int j0 = Mathf.Max(0, (int)((z - range - x0) / cell)), j1 = Mathf.Min(n - 1, (int)((z + range - x0) / cell));
            float r2 = range * range;
            int done = 0;
            for (int j = j0; j <= j1; j++)
            {
                float dz = x0 + (j + 0.5f) * cell - z;
                for (int i = i0; i <= i1; i++)
                {
                    float dx = x0 + (i + 0.5f) * cell - x, d2 = dx * dx + dz * dz;
                    if (d2 < r2 && d2 > 1f) flags[j * n + i] |= at(dx, dz, d2);
                }
                done += i1 - i0 + 1;
                if (done >= StampBudget)
                {
                    done = 0;
                    yield return true;
                }
            }
        }

        /// <summary>
        /// Cost per metre of flying through each cell: 1 in the clear, more where something sees you; an AA ring is
        /// the worst place to be seen, but it can't shoot what nobody tracks, so a masked ring costs little.
        /// </summary>
        private static float[] CostOf(byte[] flags)
        {
            var cost = new float[flags.Length];
            for (int k = 0; k < flags.Length; k++)
            {
                byte f = flags[k];
                bool seen = (f & (FRadar | FEye)) != 0;
                cost[k] = 1f + ((f & FPainted) != 0 ? 2f : 0f) + ((f & FEye) != 0 ? 15f : 0f) + ((f & FRadar) != 0 ? 30f : 0f)
                    + ((f & FSam) != 0 ? (seen ? 80f : 6f) : 0f);
            }
            return cost;
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
                $"{{\"mode\":\"{mode}\",\"tgt\":\"{TgtLabel()}\",\"folw\":{(follow ? "true" : "false")},\"rng\":{viewR / 1000f:F0},\"rcs\":{TargetRcs():0.###},\"nav\":{(ap.Nav ? "true" : "false")},\"eta\":\"{(ap.ApEngaged ? ap.EtaText(me) : null)}\",\"opa\":{cfg.RadarOpacity.Value:0.##},\"unit\":\"{Units.AltUnit}\",\"sunit\":\"{Units.SpeedUnit}\",\"eyes\":{(cfg.RadarEyes.Value ? "true" : "false")},\"sam\":{(cfg.RadarSams.Value ? "true" : "false")},\"avoid\":{(avoid != null ? "true" : "false")},\"wps\":["));
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

            // AA rings at their slant range: missiles solid, guns faint
            if (cfg.RadarSams.Value)
            {
                float pxPerM = screen.width * 0.5f / viewR, rw = Mathf.Max(1.2f, 0.0025f * S);
                foreach (Ring r in shownRings)
                {
                    Vector2 s = ToScreen(r.X, r.Z);
                    float rad = r.Max * pxPerM;
                    if (rad < 3f || s.x + rad < 0f || s.y + rad < 0f || s.x - rad > screen.width || s.y - rad > screen.height) continue;
                    Color col = Current == Mode.Enemy ? EnemySiteCol : SiteCol;
                    if (!r.Missile) col.a = 0.5f;
                    Border(new Rect(s.x - rad, s.y - rad, 2f * rad, 2f * rad), col, rw, rad); // a fully rounded border: one draw call per ring
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

            // bottom status line: what is shown + colour key (above the bottom row's legends in the game)
            float ly = bare ? 0.92f : 0.82f;
            Fill(Sc(0f, ly, 1f, 0.08f), LegendBack, 0f);
            string status = Status(out bool amber);
            Text(Sc(0.02f, ly, 0.48f, 0.08f), status, 0.03f, amber ? Amber : Green, TextAnchor.MiddleLeft);
            bool en = Current == Mode.Enemy;
            string[] keys = en ? new[] { "RDR", "EYE", "RWR", "HID" } : new[] { "RDR", "EYE", "HID" };
            Color[] cols = en ? new[] { KeyRed, KeyOrange, KeyYellow, KeyGreen } : new[] { KeyGreen, KeyCyan, KeyRed };
            for (int i = 0; i < keys.Length; i++)
                Text(Sc(0.5f + i * 0.12f, ly, 0.12f, 0.08f), keys[i], 0.026f, cols[i], TextAnchor.MiddleLeft);
        }

        /// <summary>Bottom status line: what is shown, or a passing note; amber = a problem (no radar, jammed).</summary>
        private string Status(out bool amber)
        {
            amber = false;
            if (avoid != null) return "AVOID: PLANNING";
            if (Time.unscaledTime < noteUntil)
            {
                amber = noteAmber;
                return note;
            }
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
        private int builtSeq, detailSeq = -1;
        private TerrainHeightGrid detailSent;

        /// <summary>
        /// The coverage overlay for /overlay: x, z, half-width (m), opacity (float32 each), enemy view (int32), then
        /// the Tex x Tex states (0 none, 1 masked, 2 covered; row 0 = south) run-length coded as (state byte, count uint16) pairs.
        /// </summary>
        public volatile byte[] OverlayData;

        /// <summary>The fine terrain tile for /detail: centre x, z and size (m, float32 each), then its shading as PNG.</summary>
        public volatile byte[] DetailData;

        /// <summary>Worker, at the end of a build.</summary>
        private byte[] EncodeOverlay()
        {
            var ms = new MemoryStream(64 * 1024);
            var w = new BinaryWriter(ms);
            w.Write(bX);
            w.Write(bZ);
            w.Write(bR);
            w.Write(bOpacity);
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
            EncodeDetail();
            var inv = CultureInfo.InvariantCulture;
            sb.Append("{\"map\":\"").Append(mapSettings != null ? FlightPlanner.Safe(mapSettings.name) : "").Append('"');
            sb.Append(",\"rmax\":").Append((int)(MapHalfSize() * 1.5f));
            sb.Append(",\"view\":[").Append((int)viewX).Append(',').Append((int)viewZ).Append(',').Append((int)viewR).Append(']');
            sb.Append(",\"folw\":").Append(follow ? "true" : "false");
            sb.Append(",\"ov\":").Append(OverlayData != null ? builtSeq : -1);
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

            // AA rings: x, z, slant range (m), 1 = missile
            sb.Append("],\"sams\":[");
            first = true;
            if (cfg.RadarSams.Value)
                foreach (Ring r in shownRings)
                {
                    sb.Append(first ? "[" : ",[").Append((int)r.X).Append(',').Append((int)r.Z).Append(',').Append((int)r.Max).Append(',').Append(r.Missile ? 1 : 0).Append(']');
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
                TextAnchor anchor = b.Side == Side.Left ? TextAnchor.MiddleLeft : b.Side == Side.Right ? TextAnchor.MiddleRight
                    : b.Side == Side.Bottom ? TextAnchor.LowerCenter : TextAnchor.UpperCenter;
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
                    case ApCommand.RdrEyes: label = "EYES"; boxed = cfg.RadarEyes.Value; break;
                    case ApCommand.RdrSam: label = "SAM"; boxed = cfg.RadarSams.Value; break;
                    case ApCommand.RdrAvoid: label = "AVOID"; boxed = avoid != null; break;
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
