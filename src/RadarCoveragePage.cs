using System.Collections.Generic;
using BepInEx.Configuration;
using HarmonyLib;
using UnityEngine;
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
    ///   L1 FOLW  +--------------------------------------+  R1 -
    ///   L2 MAP   |                                      |  R2 ^ (zoom in)
    ///   L3 ^RCS  |  map + coverage (scroll = zoom,      |  R3 RNG
    ///   L4 RCS   |  drag = pan)                         |  R4 v (zoom out)
    ///   L5 vRCS  +--------------------------------------+
    ///            B1 WPT   B2 UNDO  B3 CLR   B4 RTE   B5 NAV
    ///   (L4 = back to own RCS)
    ///
    /// Each sweep is turned into a MASKING HEIGHT per bin (CoverageMath.MaskHeights): the lowest
    /// altitude the radar can see there. Any target height is then one comparison, and TGT MASK draws
    /// the masking height itself: in ENMY, how low you must fly to stay unseen; in ALL/OWN, how low an
    /// intruder could slip under your radars.
    ///
    /// Route: WPT turns map clicks into waypoints (the last one is the destination), RTE plans a
    /// radar-avoiding route there (RoutePlanner, on a worker thread) from the enemy radars your side
    /// knows, and NAV flies it. A planned route is re-planned when that radar picture changes.
    ///
    /// The map's terrain is raycast once into a TerrainHeightGrid. Each radar's polar profile
    /// (Az bearings x Steps range bins) is read from that grid over a few frames, and redone when
    /// the radar moves 2% of its range. The overlay is redrawn only when something changed, a few
    /// radars per frame.
    /// </summary>
    internal sealed class RadarCoveragePage : MfdFrame
    {
        public enum Mode { All, Own, Enemy }

        private const int Az = 360, Steps = 400, Tex = 512;
        private const float MinRange = 2000f;
        private static readonly float[] AglsM = { 5f, 30f, 150f, 500f, 2000f, -1f, MaskView }; // -1 = ALT: own altitude
        private const float MaskView = -2f;   // TGT MASK: draw the masking height
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
            new Osb { Side = Side.Bottom, Index = 0, Command = ApCommand.RteWpt, Glyph = Glyph.Text },
            new Osb { Side = Side.Bottom, Index = 1, Command = ApCommand.RteUndo, Glyph = Glyph.Text },
            new Osb { Side = Side.Bottom, Index = 2, Command = ApCommand.RteClear, Glyph = Glyph.Text },
            new Osb { Side = Side.Bottom, Index = 3, Command = ApCommand.RtePlan, Glyph = Glyph.Text },
            new Osb { Side = Side.Bottom, Index = 4, Command = ApCommand.NavToggle, Glyph = Glyph.Text },
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
            public float[] Mask;   // masking height (m ASL): lowest altitude the radar sees at each bin's far edge
            public readonly byte[] Vis = new byte[Az * Steps];
            public float VisAgl = float.NaN, VisRcs, VisRadius; // target Vis was computed for (VisAgl: height key); NaN = stale

            // Sweep in progress: written into back buffers over several frames, then swapped in.
            public GlobalPosition NextOrigin;
            public float NextRange, NextAlt;
            public int RaysDone = Az;
            public float[] NextHeight, NextBefore; // rented from `spare` only while a sweep runs (NextBefore becomes Mask)
        }

        // Back buffers are shared: a radar holds a pair only during its sweep, which halves the
        // memory the profiles keep (about 1.2 MB per radar instead of 2.4 MB).
        private static readonly Stack<float[]> spare = new Stack<float[]>();

        private static float[] Rent() => spare.Count > 0 ? spare.Pop() : new float[Az * Steps];

        private const int SampleBudget = 50000; // terrain lookups per frame for profile sweeps

        private readonly ApSettings cfg;
        private readonly Dictionary<TargetDetector, Profile> profiles = new Dictionary<TargetDetector, Profile>();
        private TerrainHeightGrid grid;
        private float nextScan, nextBuild;
        private bool dirty = true;

        // The overlay is rasterised a few radars per frame into `state`, then uploaded in one go.
        private const int PixelBudget = 60000;
        private const int ViewshedCost = 20000;     // one line-of-sight pass, in pixels of that budget
        private readonly List<Profile> building = new List<Profile>();
        private int buildIdx;
        private bool buildActive;
        private float bX, bZ, bR;                    // view being built
        private bool bEnemy;

        private readonly byte[] state = new byte[Tex * Tex]; // 0 none, 1 masked, 2 covered
        private readonly float[] maskPx = new float[Tex * Tex]; // TGT MASK: lowest masking height (m above ground)
        private bool bMask;                                       // overlay being built is the MASK view

        // MASK view colour bands, by masking height above ground (m): upper bounds and colours for the
        // ENMY view (low = you are seen even low down = danger); ALL/OWN use them reversed.
        private static readonly float[] MaskBands = { 5f, 30f, 150f, 500f, 2000f };
        private static readonly Color32[] MaskCols =
        {
            new Color32(240, 40, 35, 140), new Color32(250, 120, 30, 130), new Color32(245, 200, 40, 120),
            new Color32(170, 230, 60, 105), new Color32(60, 220, 90, 90),
        };
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
            t.SetPixels32(px);
            t.Apply(true, true); // mipmaps keep small icons smooth
            iconAlpha[src] = t;
        }

        private MapSettings mapSettings;
        private Aircraft me;

        public Mode Current = Mode.All;
        public bool Visible;                         // drawn on the game screen (the web mirror shows it regardless)
        private bool follow = true, panning;
        private float viewX, viewZ, viewR = 40000f;  // view centre (global x/z) and half-width (m)
        private float texX, texZ, texR;              // view the overlay texture was built for
        private Vector2 lastMouse;
        private bool ownRadarMissing;
        private int siteCount, jammedCount;

        // ---- route ----
        private readonly NavRoute nav;
        /// <summary>Messages for the pilot (A/P MFD + HUD).</summary>
        public System.Action<string, bool> Report;
        /// <summary>The planner replaced the route (the flight controller re-anchors leg 0).</summary>
        public System.Action RouteReplaced;
        /// <summary>Whether NAV is the A/P's lateral mode (lights the NAV legend).</summary>
        public System.Func<bool> NavActive;
        private bool wptMode;
        private Vector2 downPos;
        private bool planWanted;                 // RTE pressed or a replan is due: plan once enemy sweeps are in
        private float planWantedAt, lastPlan = -100f;
        private int enemyPicture, plannedPicture = -1; // bumps when the known enemy radar picture changes
        private volatile RoutePlanner planJob;   // running on a worker thread
        private volatile List<Waypoint> planResult;
        private volatile bool planDone;
        private float planStartX, planStartZ;
        private string routeInfo;

        private bool Routing => planWanted || planJob != null || (nav.Planned && nav.Has && cfg.RouteAutoReplan.Value);

        public RadarCoveragePage(ApSettings cfg, NavRoute nav) : base(cfg.RadarSize, cfg.RadarX, cfg.RadarY)
        {
            this.cfg = cfg;
            this.nav = nav;
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
            dirty = true;
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
            dirty = true;
            planWanted = false;
            planJob = null;
            planDone = false;
            plannedPicture = -1;
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
                case ApCommand.RteWpt: wptMode = !wptMode; return;
                case ApCommand.RteUndo:
                    if (nav.Planned) nav.Clear();
                    else if (nav.Points.Count > 0)
                    {
                        nav.Points.RemoveAt(nav.Points.Count - 1);
                        nav.Active = Mathf.Min(nav.Active, nav.Points.Count);
                        nav.Version++;
                    }
                    routeInfo = null;
                    return;
                case ApCommand.RteClear:
                    nav.Clear();
                    planWanted = false;
                    routeInfo = null;
                    return;
                case ApCommand.RtePlan:
                    if (float.IsNaN(nav.DestX) && nav.Points.Count == 0)
                    {
                        Report?.Invoke("RTE - SET A WPT FIRST", true);
                        return;
                    }
                    if (float.IsNaN(nav.DestX))
                    {
                        nav.DestX = nav.Points[nav.Points.Count - 1].X;
                        nav.DestZ = nav.Points[nav.Points.Count - 1].Z;
                    }
                    planWanted = true;
                    planWantedAt = Time.unscaledTime;
                    routeInfo = "RTE PLANNING";
                    nextScan = 0f;
                    return;
            }
            dirty = true;
        }

        /// <summary>RCS setting of the current view: ENMY, or ALL/OWN (friendly radars).</summary>
        private ConfigEntry<float> RcsSetting => Current == Mode.Enemy ? cfg.RadarEnemyRcs : cfg.RadarFriendlyRcs;

        /// <summary>RCS the current view is drawn for: the set value, or 0 = own jet's live RCS (grows with stores).</summary>
        private float TargetRcs() => RcsSetting.Value > 0f ? RcsSetting.Value : me != null ? me.RCS : 1f;
        private float lastRcs;
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
            bool routing = Routing;
            if (!active && !routing) return;
            float now = Time.unscaledTime;
            if (now >= nextScan)
            {
                nextScan = now + 1f;
                if (mapSettings == null) // Unity null also after a map unload
                {
                    mapSettings = Object.FindObjectOfType<MapSettings>();
                    grid = null;
                    profiles.Clear();
                }
                if (mapSettings != null && grid == null && MapHalfSize() > 0f)
                    grid = new TerrainHeightGrid(MapHalfSize() * 2f);
                CollectRadars(Current == Mode.Enemy, true);
                if (routing && Current != Mode.Enemy) CollectRadars(true, false); // the planner needs the enemy picture
            }
            if (now >= nextContacts)
            {
                nextContacts = now + 0.25f;
                if (active) CollectContacts();
                float rcs = TargetRcs();
                if (rcs != lastRcs) dirty = true; // stores released, or set from the web mirror
                lastRcs = rcs;
                jammedCount = 0;
                // enemy jam state only when the game map shows it too (its jammed markers)
                MapOptions mo = SceneSingleton<MapOptions>.i;
                bool showJam = mo != null && mo.showJamming, enemyView = Current == Mode.Enemy;
                foreach (Profile p in profiles.Values)
                {
                    bool shown = p.Enemy == enemyView;
                    bool j = (!p.Enemy || showJam) && p.Radar is Radar rr && rr.IsJammed();
                    if (j != p.Jammed && shown) dirty = true;
                    if (j != p.Jammed && p.Enemy) enemyPicture++;
                    p.Jammed = j;
                    if (j && shown) jammedCount++;

                    // kept up in every mode so the last known pitch survives leaving ENMY
                    if (p.Enemy) UpdateEnemyCone(p);
                }
            }

            if (grid != null && !grid.Ready) grid.Fill();
            else if (grid != null)
            {
                // Own radar first so the OWN page fills quickly.
                int budget = SampleBudget;
                if (me.radar != null && profiles.TryGetValue(me.radar, out Profile own))
                    budget -= Advance(own, budget);
                foreach (Profile p in profiles.Values)
                {
                    if (budget <= 0) break;
                    budget -= Advance(p, budget);
                }
            }

            UpdateRouting(now);
            if (!active) return;

            if (follow)
            {
                GlobalPosition g = me.GlobalPosition();
                viewX = g.x;
                viewZ = g.z;
            }

            // TGT ALT: redraw once own altitude moves 3% (at least 25 m) from what was drawn
            float alt = me.GlobalPosition().y;
            if (cfg.RadarTargetAgl.Value == -1f && Mathf.Abs(alt - altSnap) > Mathf.Max(25f, altSnap * 0.03f))
            {
                altSnap = alt;
                dirty = true;
            }

            if (buildActive) ContinueBuild();
            else if (now >= nextBuild && (dirty || ViewMoved()))
            {
                nextBuild = now + 0.25f;
                StartBuild();
            }
        }

        // The overlay is built Margin x wider than the view and drawn where it was built, so panning
        // (and follow mode) only needs a rebuild once the view nears the overlay's edge.
        private const float Margin = 1.4f;

        private bool ViewMoved()
        {
            float slack = viewR * 0.15f;
            return viewR * Margin != texR || Mathf.Abs(viewX - texX) > slack || Mathf.Abs(viewZ - texZ) > slack;
        }

        /// <param name="enemy">which side's radars to collect</param>
        /// <param name="display">this is the side the page shows (site count, OWN radar status)</param>
        private void CollectRadars(bool enemy, bool display)
        {
            // radar -> scan point to sweep from; enemy radars sit where our side last saw them
            var live = new Dictionary<TargetDetector, GlobalPosition>();
            int sites = 0;
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
                if (!isOwn) sites++;
            }
            if (display)
            {
                siteCount = sites;
                ownRadarMissing = me.radar == null || !live.ContainsKey(me.radar);
            }

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
            if (dead.Count > 0)
            {
                dirty = true;
                if (enemy) enemyPicture++;
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
            foreach (MapIcon icon in map.mapIcons)
            {
                if (!(icon is UnitMapIcon ui) || !TargetPanel.ShownEnemy(ui, filter)) continue;
                Unit u = ui.unit;
                if (ui.iconImage.sprite != null) BakeIcon(ui.iconImage.sprite.texture);
                contacts.Add(new Contact
                {
                    Unit = u,
                    Track = map.HQ != null ? map.HQ.GetTrackingData(u.persistentID) : null,
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
                if (Current == Mode.Enemy) dirty = true;
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
                CoverageMath.MaskHeights(p.NextHeight, p.NextBefore, p.NextAlt, step, Az, Steps);
                if (p.Height != null)
                {
                    spare.Push(p.Height);
                    spare.Push(p.Mask);
                }
                p.Height = p.NextHeight;
                p.Mask = p.NextBefore;
                if (p.Enemy) enemyPicture++;
                p.NextHeight = p.NextBefore = null;
                p.Origin = p.NextOrigin;
                p.Range = p.NextRange;
                p.Alt = p.NextAlt;
                p.Valid = true;
                p.VisAgl = float.NaN;
                dirty = true;
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
        /// Per bin: a target at TargetAsl at the far edge of bin i is in line of sight when it is at
        /// or above the bin's masking height (terrain and radar horizon, see CoverageMath.MaskHeights),
        /// and detected when its signal beats the radar's minimum after clutter.
        /// vis: 0 = signal too weak, 1 = hidden, 2 = detected.
        /// Mirrors DetectorManager.RequestRadarCheck + RadarParams.GetSignalStrength with no ECM, where
        /// the doppler boost only applies once the signal already passes, so it can't change the result.
        /// </summary>
        internal static void Viewshed(float[] height, float[] mask, float radarAlt, float agl, float tgtAsl, float step,
            RadarParams rp, float rcs, float tgtRadius, byte[] vis)
        {
            float radarHorizon = Mathf.Sqrt(CoverageMath.TwoR * Mathf.Max(0f, radarAlt));
            float rcsGain = Mathf.Pow(rcs, 0.25f), maxDist = rp.maxRange * 2f;
            float size2 = tgtRadius * tgtRadius * 2f;
            for (int a = 0; a < Az; a++)
            {
                int b = a * Steps;
                for (int i = 0; i < Steps; i++)
                {
                    float d = (i + 1) * step, t = height[b + i], tgt = TargetAsl(t, agl, tgtAsl), dy = tgt - radarAlt;
                    float h = tgt - t; // target's radar altitude
                    float dist = Mathf.Sqrt(d * d + dy * dy);
                    float signal = Mathf.Min(rp.maxRange / dist * rcsGain, rp.maxSignal);
                    byte v;
                    if (signal < rp.minSignal || dist > maxDist) v = 0;
                    else if (tgt < mask[b + i]) v = 1;
                    else
                    {
                        // below the radar's line to its horizon, the ground behind the target adds clutter
                        float clutter = size2 / (h * h); // the ground right under the target
                        if (d < radarHorizon && tgt < radarAlt * (1f - d / radarHorizon))
                            clutter += Mathf.Min(dist, 1000f) * (radarAlt - tgt) / (dist * h);
                        v = signal - clutter * rp.clutterFactor >= rp.minSignal ? (byte)2 : (byte)1;
                    }
                    vis[b + i] = v;
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
            bMask = cfg.RadarTargetAgl.Value == MaskView;
            System.Array.Clear(state, 0, state.Length);
            if (bMask) for (int i = 0; i < maskPx.Length; i++) maskPx[i] = float.PositiveInfinity;
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

            if (bMask)
            {
                for (int k = 0; k < pixels.Length; k++)
                {
                    float m = maskPx[k];
                    int band = 0;
                    while (band < MaskBands.Length && m >= MaskBands[band]) band++;
                    // past the top band (or out of every radar's reach): no colour
                    pixels[k] = band >= MaskBands.Length ? None : MaskCols[bEnemy ? band : MaskBands.Length - 1 - band];
                }
            }
            else
            {
                // enemy view: seen by them is the danger (red), terrain-masked is safe (green)
                Color32 seen = bEnemy ? Masked : Covered, hidden = bEnemy ? Covered : Masked;
                for (int k = 0; k < pixels.Length; k++)
                    pixels[k] = state[k] == 2 ? seen : state[k] == 1 ? hidden : None;
            }
            if (overlay == null)
                overlay = new Texture2D(Tex, Tex, TextureFormat.RGBA32, false) { hideFlags = HideFlags.HideAndDontSave, wrapMode = TextureWrapMode.Clamp };
            overlay.SetPixels32(pixels); // row 0 = south, so the texture draws north-up
            overlay.Apply();
            texX = bX;
            texZ = bZ;
            texR = bR;
            buildActive = false;
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
            if (bMask) return RasterizeMask(p, cone, fwd, flat, elevOnly, ox, oy, x0, x1, y0, y1, mPerPx, rcs, resume, budget);
            if (p.VisAgl != hKey || p.VisRcs != rcs || p.VisRadius != tgtRadius) // only for radars in view
            {
                used = ViewshedCost;
                Viewshed(p.Height, p.Mask, radarAlt, agl, tgtAsl, p.Range / Steps, radar.RadarParameters, rcs, tgtRadius, p.Vis);
                p.VisAgl = hKey;
                p.VisRcs = rcs;
                p.VisRadius = tgtRadius;
            }
            int y = Mathf.Max(y0, resume);
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
                    if (cone > 0f)
                    {
                        float dy = TargetAsl(p.Height[bin], agl, tgtAsl) - radarAlt;
                        if (elevOnly ? Mathf.Abs(Mathf.Atan2(dy, Mathf.Sqrt(d2)) * Mathf.Rad2Deg - p.ConePitch) > cone
                            : Vector3.Angle(new Vector3(dx, flat ? 0f : dy, dz), fwd) > cone) continue;
                    }
                    byte v = p.Vis[bin];
                    if (v > state[k]) state[k] = v;
                }
            }
            if (y <= y1) buildRow = y;
            return used;
        }

        /// <summary>
        /// MASK view: merge one radar's masking height above ground into maskPx (lowest wins), inside
        /// its scan azimuth and out to the range the set RCS can be detected at without clutter. Clutter
        /// and the cone's elevation limits are left out, so the map errs low (the safe side for ENMY).
        /// </summary>
        private int RasterizeMask(Profile p, float cone, Vector3 fwd, bool flat, bool elevOnly, float ox, float oy,
            int x0, int x1, int y0, int y1, float mPerPx, float rcs, int resume, int budget)
        {
            RadarParams rp = ((Radar)p.Radar).RadarParameters;
            float reach = Mathf.Min(p.Range, CoverageMath.DetectRange(rp.maxRange, rp.minSignal, rcs));
            float binsPerM = Steps / p.Range, azPerRad = Az / (2f * Mathf.PI), r2 = reach * reach;
            Vector3 fwdFlat = new Vector3(fwd.x, 0f, fwd.z);
            bool azCheck = cone > 0f && !elevOnly && fwdFlat.sqrMagnitude > 1e-4f;
            int used = 0, y = Mathf.Max(y0, resume);
            for (; y <= y1 && used < budget; y++, used += x1 - x0 + 1)
            {
                float dz = (y + 0.5f - oy) * mPerPx;
                for (int x = x0; x <= x1; x++)
                {
                    float dx = (x + 0.5f - ox) * mPerPx, d2 = dx * dx + dz * dz;
                    if (d2 >= r2) continue;
                    if (azCheck && Vector3.Angle(new Vector3(dx, 0f, dz), fwdFlat) > cone) continue;
                    float brg = Mathf.Atan2(dx, dz);
                    if (brg < 0f) brg += 2f * Mathf.PI;
                    int bin = Mathf.Min(Az - 1, (int)(brg * azPerRad)) * Steps + Mathf.Min(Steps - 1, (int)(Mathf.Sqrt(d2) * binsPerM));
                    float agl = Mathf.Max(0f, p.Mask[bin] - p.Height[bin]);
                    int k = y * Tex + x;
                    if (agl < maskPx[k]) maskPx[k] = agl;
                }
            }
            if (y <= y1) buildRow = y;
            return used;
        }

        // =====================================================================
        // Route: waypoints, planning, re-planning
        // =====================================================================
        private void AddWaypoint(float gx, float gz)
        {
            if (nav.Planned) nav.Clear(); // a click after a planned route starts a new one
            nav.Points.Add(new Waypoint(gx, gz));
            nav.DestX = gx;
            nav.DestZ = gz;
            nav.Version++;
            routeInfo = null;
        }

        /// <summary>
        /// Hand a finished plan to NAV; start a plan (or a re-plan when the known enemy radar picture
        /// changed) once the terrain is mapped and the enemy radars' sweeps are in (or 8 s passed).
        /// </summary>
        private void UpdateRouting(float now)
        {
            if (planDone)
            {
                planDone = false;
                RoutePlanner job = planJob;
                List<Waypoint> r = planResult;
                planJob = null;
                planResult = null;
                if (r != null && r.Count > 0 && !float.IsNaN(nav.DestX))
                {
                    bool first = !nav.Planned;
                    float dx = nav.DestX, dz = nav.DestZ;
                    nav.Replace(r, planStartX, planStartZ, true);
                    nav.DestX = dx;
                    nav.DestZ = dz;
                    RouteReplaced?.Invoke();
                    routeInfo = $"RTE {r.Count} LEG{(r.Count == 1 ? "" : "S")} - SEEN {job.ExposedSeconds:0}S";
                    Report?.Invoke(first ? $"RTE READY {r.Count} LEGS" : "RTE UPDATED", job.ExposedSeconds > 0.5f);
                }
                else if (r == null)
                {
                    routeInfo = "RTE FAILED";
                    Report?.Invoke("RTE - NO ROUTE FOUND", true);
                }
            }
            if (planJob != null || me == null) return;

            if (!planWanted && nav.Planned && nav.Has && cfg.RouteAutoReplan.Value
                && enemyPicture != plannedPicture && now - lastPlan > 10f)
            {
                planWanted = true;
                planWantedAt = now;
            }
            if (!planWanted || float.IsNaN(nav.DestX)) return;
            if (grid == null || !grid.Ready)
            {
                routeInfo = "RTE - MAPPING TERRAIN";
                return;
            }
            bool sweeping = false;
            foreach (Profile p in profiles.Values)
                if (p.Enemy && (!p.Valid || p.RaysDone < Az)) sweeping = true;
            if (sweeping && now - planWantedAt < 8f) return;
            StartPlan(now);
        }

        private void StartPlan(float now)
        {
            planWanted = false;
            lastPlan = now;
            plannedPicture = enemyPicture;
            GlobalPosition g = me.GlobalPosition();
            float sx = g.x, sz = g.z, gx = nav.DestX, gz = nav.DestZ;

            // square planning box around both ends, clamped to the map
            float dist = new Vector2(gx - sx, gz - sz).magnitude, margin = Mathf.Max(15000f, 0.35f * dist);
            float cx = (sx + gx) * 0.5f, cz = (sz + gz) * 0.5f;
            float size = Mathf.Max(Mathf.Abs(gx - sx), Mathf.Abs(gz - sz)) + 2f * margin;
            float half = MapHalfSize();
            size = Mathf.Min(size, 2f * half);
            float minX = Mathf.Clamp(cx - size * 0.5f, -half, half - size), minZ = Mathf.Clamp(cz - size * 0.5f, -half, half - size);
            int n = Mathf.Clamp(Mathf.CeilToInt(size / 250f), 32, 160);
            float cell = size / n;

            var job = new RoutePlanner
            {
                N = n,
                OriginX = minX,
                OriginZ = minZ,
                Cell = cell,
                Hide = BuildHide(minX, minZ, cell, n, cfg.RadarEnemyRcs.Value > 0f ? cfg.RadarEnemyRcs.Value : me.RCS),
                Bands = ParseBands(cfg.RouteHeights.Value),
                Speed = Mathf.Max(me.speed, 100f),
                ExposureWeight = Mathf.Max(0f, cfg.RouteExposureWeight.Value),
                LowPenalty = Mathf.Max(0f, cfg.RouteLowPenalty.Value),
                MinLeg = Mathf.Max(1500f, 3f * cell),
            };
            planStartX = sx;
            planStartZ = sz;
            planJob = job;
            routeInfo = "RTE PLANNING";
            System.Threading.ThreadPool.QueueUserWorkItem(_ =>
            {
                List<Waypoint> r = null;
                try { r = job.Plan(sx, sz, gx, gz); }
                catch (System.Exception) { r = null; }
                planResult = r;
                planDone = true;
            });
        }

        /// <summary>
        /// Planner input: per cell, the highest height above ground hidden from every known enemy radar
        /// (the lowest masking height among them, inside each one's scan azimuth when known and out to
        /// where the RCS can be detected). Read from the radars' polar sweeps on the main thread.
        /// </summary>
        private float[] BuildHide(float minX, float minZ, float cell, int n, float rcs)
        {
            var hide = new float[n * n];
            for (int i = 0; i < hide.Length; i++) hide[i] = float.PositiveInfinity;
            foreach (Profile p in profiles.Values)
            {
                if (!p.Enemy || !p.Valid || p.Jammed || p.Mask == null || !(p.Radar is Radar radar)) continue;
                RadarParams rp = radar.RadarParameters;
                float reach = Mathf.Min(p.Range, CoverageMath.DetectRange(rp.maxRange, rp.minSignal, rcs));
                float cone = RadarConeRef(radar);
                bool azCheck = cone > 0f && !float.IsNaN(p.ConeHeading);
                float hr = p.ConeHeading * Mathf.Deg2Rad;
                Vector3 fwd = azCheck ? new Vector3(Mathf.Sin(hr), 0f, Mathf.Cos(hr)) : Vector3.forward;
                float binsPerM = Steps / p.Range, azPerRad = Az / (2f * Mathf.PI), r2 = reach * reach;
                int x0 = Mathf.Max(0, (int)((p.Origin.x - reach - minX) / cell)), x1 = Mathf.Min(n - 1, (int)((p.Origin.x + reach - minX) / cell));
                int z0 = Mathf.Max(0, (int)((p.Origin.z - reach - minZ) / cell)), z1 = Mathf.Min(n - 1, (int)((p.Origin.z + reach - minZ) / cell));
                for (int z = z0; z <= z1; z++)
                for (int x = x0; x <= x1; x++)
                {
                    float dx = minX + (x + 0.5f) * cell - p.Origin.x, dz = minZ + (z + 0.5f) * cell - p.Origin.z, d2 = dx * dx + dz * dz;
                    if (d2 >= r2) continue;
                    if (azCheck && Vector3.Angle(new Vector3(dx, 0f, dz), fwd) > cone) continue;
                    float brg = Mathf.Atan2(dx, dz);
                    if (brg < 0f) brg += 2f * Mathf.PI;
                    int bin = Mathf.Min(Az - 1, (int)(brg * azPerRad)) * Steps + Mathf.Min(Steps - 1, (int)(Mathf.Sqrt(d2) * binsPerM));
                    float agl = Mathf.Max(0f, p.Mask[bin] - p.Height[bin]);
                    int k = z * n + x;
                    if (agl < hide[k]) hide[k] = agl;
                }
            }
            return hide;
        }

        private static float[] ParseBands(string text)
        {
            var list = new List<float>();
            foreach (string part in (text ?? "").Split(','))
                if (float.TryParse(part.Trim(), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out float v) && v >= 5f)
                    list.Add(v);
            list.Sort();
            return list.Count > 0 ? list.ToArray() : new[] { 30f, 150f, 500f };
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
                case EventType.MouseDown when e.button == 0:
                    // WPT mode: a click (press and release without dragging) drops a waypoint instead
                    if (!wptMode && ClickContact(mp))
                    {
                        e.Use();
                        return true;
                    }
                    panning = true;
                    lastMouse = downPos = mp;
                    e.Use();
                    return true;
                case EventType.MouseDrag when panning:
                    if (wptMode && (mp - downPos).magnitude < 6f) return true; // not a drag yet
                    Vector2 d = mp - lastMouse;
                    lastMouse = mp;
                    follow = false;
                    viewX -= d.x * mPerScreenPx;
                    viewZ += d.y * mPerScreenPx;
                    e.Use();
                    return true;
                case EventType.MouseUp when panning:
                    panning = false;
                    if (wptMode && (mp - downPos).magnitude < 6f)
                        AddWaypoint(viewX + (mp.x - screen.center.x) * mPerScreenPx, viewZ - (mp.y - screen.center.y) * mPerScreenPx);
                    e.Use();
                    return true;
            }
            return false;
        }

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

            DrawRoute();

            // own jet: dot + heading stick
            GlobalPosition g = me.GlobalPosition();
            Vector2 c = ToScreen(g.x, g.z);
            Vector3 f = me.transform.forward;
            Matrix4x4 m = GUI.matrix;
            GUIUtility.RotateAroundPivot(Mathf.Atan2(f.x, f.z) * Mathf.Rad2Deg, c);
            Fill(new Rect(c.x - 1f, c.y - 0.05f * S, 2f, 0.05f * S), Color.white, 0f);
            GUI.matrix = m;
            Fill(new Rect(c.x - 3.5f, c.y - 3.5f, 7f, 7f), Color.white, 3.5f);
            GUI.EndGroup();

            DrawLegends();

            // status line (above the bottom OSB legends): what is shown + colour key
            Fill(Sc(0f, 0.835f, 1f, 0.065f), LegendBack, 0f);
            bool noRadar = Current == Mode.Own && ownRadarMissing;
            bool ownJammed = !noRadar && me.radar != null && profiles.TryGetValue(me.radar, out Profile op) && op.Jammed;
            string status = grid != null && !grid.Ready ? $"MAPPING TERRAIN {grid.Progress * 100f:F0}%"
                : mapSettings == null ? "NO MAP"
                : Current == Mode.Own
                ? (noRadar ? "NO RADAR" : ownJammed ? "OWN RDR JAMMED" : me.radar.activated ? "OWN RDR" : "OWN RDR OFF")
                : (Current == Mode.Enemy ? $"{siteCount} ENMY RDR" : $"{siteCount} RDR SITES") + (jammedCount > 0 ? $" {jammedCount} JAM" : "");
            Text(Sc(0.02f, 0.835f, 0.5f, 0.065f), status, 0.03f, noRadar || (Current == Mode.Own ? ownJammed : jammedCount > 0) ? Amber : Green, TextAnchor.MiddleLeft);
            if (routeInfo != null || nav.Points.Count > 0)
            {
                string info = routeInfo ?? (nav.Planned ? "RTE" : $"{nav.Points.Count} WPT{(nav.Points.Count == 1 ? "" : "S")}");
                if (wptMode) info += " - CLICK MAP";
                Fill(Sc(0f, 0.775f, 1f, 0.06f), LegendBack, 0f);
                Text(Sc(0.02f, 0.775f, 0.96f, 0.06f), info, 0.026f, RouteCol, TextAnchor.MiddleLeft);
            }
            else if (wptMode)
            {
                Fill(Sc(0f, 0.775f, 1f, 0.06f), LegendBack, 0f);
                Text(Sc(0.02f, 0.775f, 0.96f, 0.06f), "WPT - CLICK MAP TO ADD", 0.026f, RouteCol, TextAnchor.MiddleLeft);
            }
            bool en = Current == Mode.Enemy;
            if (cfg.RadarTargetAgl.Value == MaskView)
            {
                // colour key: masking height bands (m above ground, display units)
                Text(Sc(0.5f, 0.835f, 0.16f, 0.065f), en ? "HIDE <" : "SEES >", 0.022f, Green, TextAnchor.MiddleLeft);
                for (int i = 0; i < MaskBands.Length; i++)
                {
                    Color32 mc = MaskCols[en ? i : MaskBands.Length - 1 - i];
                    Rect r = Sc(0.64f + i * 0.07f, 0.845f, 0.065f, 0.045f);
                    Fill(r, new Color32(mc.r, mc.g, mc.b, 255), 0f);
                    float lo = i == 0 ? 0f : MaskBands[i - 1];
                    Text(r, lo < 1000f ? $"{Units.Alt(lo):0}" : $"{Units.Alt(lo) / 1000f:0}K", 0.018f, Black, TextAnchor.MiddleCenter, false);
                }
            }
            else
            {
                Color green = new Color(0.35f, 0.95f, 0.45f), red = new Color(1f, 0.3f, 0.25f);
                Text(Sc(0.5f, 0.835f, 0.48f, 0.065f), "SEEN", 0.026f, en ? red : green, TextAnchor.MiddleLeft);
                Text(Sc(0.66f, 0.835f, 0.32f, 0.065f), "HIDDEN", 0.026f, en ? green : red, TextAnchor.MiddleRight);
            }
        }

        private static readonly Color RouteCol = new Color(1f, 0.35f, 0.95f, 1f);
        private static readonly Color RouteDim = new Color(1f, 0.35f, 0.95f, 0.45f);

        /// <summary>Route legs (active leg bright, flown legs dim), waypoints, leg heights, destination.</summary>
        private void DrawRoute()
        {
            if (nav.Points.Count == 0) return;
            float lw = Mathf.Max(1.5f, 0.004f * S);
            nav.LegStart(out float sx0, out float sz0);
            if (nav.Active == 0)
            {
                GlobalPosition g = me.GlobalPosition();
                sx0 = nav.Planned ? nav.StartX : g.x;
                sz0 = nav.Planned ? nav.StartZ : g.z;
            }
            float px = nav.Active == 0 ? sx0 : nav.Points[0].X, pz = nav.Active == 0 ? sz0 : nav.Points[0].Z;
            for (int i = nav.Active == 0 ? 0 : 1; i < nav.Points.Count; i++)
            {
                Waypoint w = nav.Points[i];
                bool flown = i < nav.Active;
                Line(ToScreen(px, pz), ToScreen(w.X, w.Z), i == nav.Active ? lw * 1.6f : lw, flown ? RouteDim : RouteCol);
                if (!float.IsNaN(w.Agl) && !flown)
                {
                    Vector2 mid = (ToScreen(px, pz) + ToScreen(w.X, w.Z)) * 0.5f;
                    Text(new Rect(mid.x - 0.05f * S, mid.y - 0.03f * S, 0.1f * S, 0.025f * S), $"{Units.Alt(w.Agl):0}", 0.02f, RouteCol, TextAnchor.MiddleCenter, false);
                }
                px = w.X;
                pz = w.Z;
            }
            for (int i = 0; i < nav.Points.Count; i++)
            {
                Vector2 c = ToScreen(nav.Points[i].X, nav.Points[i].Z);
                float r = i == nav.Points.Count - 1 ? 5f : 3f;
                Fill(new Rect(c.x - r, c.y - r, 2f * r, 2f * r), i < nav.Active ? RouteDim : RouteCol, i == nav.Points.Count - 1 ? 0f : r);
            }
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
            GlobalPosition p = k.Track != null ? k.Track.GetPosition() : k.Unit.GlobalPosition();
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
                TextAnchor anchor = b.Side == Side.Left ? TextAnchor.MiddleLeft : b.Side == Side.Bottom ? TextAnchor.LowerCenter : TextAnchor.UpperCenter;
                string label;
                bool boxed = false;
                switch (b.Command)
                {
                    case ApCommand.RdrAll: label = "ALL"; boxed = Current == Mode.All; break;
                    case ApCommand.RdrOwn: label = "OWN"; boxed = Current == Mode.Own; break;
                    case ApCommand.RdrEnemy: label = "ENMY"; boxed = Current == Mode.Enemy; break;
                    case ApCommand.RdrTgt:
                        label = cfg.RadarTargetAgl.Value == MaskView ? "TGT MASK" : cfg.RadarTargetAgl.Value < 0f ? "TGT ALT" : $"TGT {cfg.RadarTargetAgl.Value:F0}";
                        break;
                    case ApCommand.RteWpt: label = "WPT"; boxed = wptMode; break;
                    case ApCommand.RteUndo: label = "UNDO"; break;
                    case ApCommand.RteClear: label = "CLR"; break;
                    case ApCommand.RtePlan: label = "RTE"; boxed = nav.Planned && nav.Has; break;
                    case ApCommand.NavToggle: label = "NAV"; boxed = NavActive != null && NavActive(); break;
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
