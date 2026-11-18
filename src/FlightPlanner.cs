using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using BepInEx;
using BepInEx.Logging;
using UnityEngine;
using UnityEngine.Experimental.Rendering;

namespace AutopilotMod
{
    /// <summary>
    /// The flight planner: a web app in BepInEx\FlightPlanner that works with the game closed (double-click
    /// planner.html) or from the web mirror (/planner, which can also send plans to the jet). The game feeds
    /// it data the first time it sees each map (map image + terrain shading) and each jet (fly-by-wire
    /// numbers for the predicted path). Data files are JavaScript (PLANNER.map({...})) so a page opened
    /// from disk can load them with script tags, which browsers allow where fetch is blocked.
    /// </summary>
    internal static class FlightPlanner
    {
        public static readonly string Dir = Path.Combine(Paths.BepInExRootPath, "FlightPlanner");
        private static string MapsDir => Path.Combine(Dir, "maps");
        private static string AirframesFile => Path.Combine(Dir, "airframes.js");

        private class Airframe
        {
            public string name;
            public float gLimit, corner, roll; // FBW g limit, corner speed (m/s), max roll rate (rad/s)
        }

        // Written and read by hand: JsonUtility silently dropped the list.
        private static readonly List<Airframe> airframes = new List<Airframe>();
        private static readonly Regex AirframeJson = new Regex(
            @"\{""name"":""((?:[^""\\]|\\.)*)"",""gLimit"":([-+0-9.eE]+),""corner"":([-+0-9.eE]+),""roll"":([-+0-9.eE]+)\}");

        private static ManualLogSource log;
        private static ApSettings cfg;
        private static bool loaded;
        private static readonly HashSet<string> exported = new HashSet<string>();

        public static void Init(ManualLogSource logger, ApSettings settings)
        {
            log = logger;
            cfg = settings;
            try
            {
                Directory.CreateDirectory(MapsDir);
                using (Stream s = typeof(FlightPlanner).Assembly.GetManifestResourceStream("planner.html"))
                using (var f = File.Create(Path.Combine(Dir, "planner.html")))
                    s.CopyTo(f);
                Load();
                loaded = true;
                SaveAirframes();
                WriteMapList();
            }
            catch (Exception ex)
            {
                log.LogWarning($"Flight planner: can't write {Dir}: {ex.Message}");
            }
        }

        private static float F(string s) => float.Parse(s, System.Globalization.CultureInfo.InvariantCulture);

        private static string N(float v) => v.ToString("R", System.Globalization.CultureInfo.InvariantCulture);

        private static void Load()
        {
            string js;
            try { js = File.ReadAllText(AirframesFile); }
            catch (Exception) { return; } // none yet
            foreach (Match m in AirframeJson.Matches(js))
                airframes.Add(new Airframe { name = Regex.Unescape(m.Groups[1].Value), gLimit = F(m.Groups[2].Value), corner = F(m.Groups[3].Value), roll = F(m.Groups[4].Value) });
        }

        private static void SaveAirframes()
        {
            var sb = new StringBuilder("PLANNER.airframes({");
            sb.Append(FormattableString.Invariant($"\"metric\":{(Units.Metric ? "true" : "false")},\"maxBank\":{N(cfg.MaxBank.Value)},\"maxTurnRate\":{N(cfg.MaxTurnRate.Value)},"));
            sb.Append(FormattableString.Invariant($"\"share\":{N(FlightController.AirframeShare)},\"minSpeed\":{N(cfg.MinEngageSpeed.Value)},"));
            sb.Append(FormattableString.Invariant($"\"l1Period\":{N(cfg.L1Period.Value)},\"l1Damping\":{N(cfg.L1Damping.Value)},\"list\":["));
            for (int i = 0; i < airframes.Count; i++)
            {
                Airframe a = airframes[i];
                string name = a.name.Replace("\\", "\\\\").Replace("\"", "\\\"");
                sb.Append(i > 0 ? "," : "").Append($"{{\"name\":\"{name}\",\"gLimit\":{N(a.gLimit)},\"corner\":{N(a.corner)},\"roll\":{N(a.roll)}}}");
            }
            File.WriteAllText(AirframesFile, sb.Append("]});\n").ToString());
        }

        /// <summary>Remember a jet's fly-by-wire numbers (once per jet type, again if they change).</summary>
        public static void RecordAirframe(string name, float gLimit, float corner, float roll)
        {
            if (!loaded || string.IsNullOrEmpty(name)) return;
            Airframe a = airframes.FirstOrDefault(x => x.name == name);
            if (a != null && Mathf.Approximately(a.gLimit, gLimit) && Mathf.Approximately(a.corner, corner) && Mathf.Approximately(a.roll, roll)) return;
            if (a == null) airframes.Add(a = new Airframe { name = name });
            a.gLimit = gLimit;
            a.corner = corner;
            a.roll = roll;
            try { SaveAirframes(); }
            catch (Exception ex) { log.LogWarning($"Flight planner: can't save airframes: {ex.Message}"); }
        }

        internal static string Safe(string s) => Regex.Replace(s ?? "", "[^A-Za-z0-9_-]", "");

        /// <summary>The planner already has this map, written by this build of the mod (a missing file dates from 1601).</summary>
        // shortcut: a game update that changes a map's image isn't noticed; delete FlightPlanner\maps\<map>.js to re-export it
        public static bool HasMap(string mapName)
        {
            try { return File.GetLastWriteTimeUtc(Path.Combine(MapsDir, Safe(mapName) + ".js")) > File.GetLastWriteTimeUtc(typeof(FlightPlanner).Assembly.Location); }
            catch (Exception) { return false; }
        }

        /// <summary>
        /// Write the map's data file for the planner, once per map per session: the game's map image
        /// (at most 4096 px a side) and the terrain shading, encoded on a worker thread.
        /// </summary>
        public static void ExportMap(string mapName, Sprite image, Vector2 mapSize, TerrainHeightGrid grid)
        {
            string name = Safe(mapName);
            Color32[] relief = grid.Relief();
            if (name.Length == 0 || image == null || relief == null || !exported.Add(name)) return;

            // GPU copy of the sprite's rect: the map texture isn't CPU-readable
            Texture2D src = image.texture;
            Rect r = image.textureRect;
            float k = Mathf.Min(1f, 4096f / Mathf.Max(r.width, r.height));
            int w = Mathf.Max(1, (int)(r.width * k)), h = Mathf.Max(1, (int)(r.height * k));
            RenderTexture rt = RenderTexture.GetTemporary(w, h, 0, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
            Graphics.Blit(src, rt, new Vector2(r.width / src.width, r.height / src.height), new Vector2(r.x / src.width, r.y / src.height));
            RenderTexture prev = RenderTexture.active;
            RenderTexture.active = rt;
            var tex = new Texture2D(w, h, TextureFormat.RGBA32, false);
            tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
            RenderTexture.active = prev;
            RenderTexture.ReleaseTemporary(rt);
            Color32[] img = tex.GetPixels32();
            UnityEngine.Object.Destroy(tex);

            int n = grid.N;
            float gridSize = grid.Size;
            ThreadPool.QueueUserWorkItem(_ =>
            {
                try
                {
                    byte[] jpg = ImageConversion.EncodeArrayToJPG(img, GraphicsFormat.R8G8B8A8_UNorm, (uint)w, (uint)h, 0, 90);
                    byte[] png = ImageConversion.EncodeArrayToPNG(relief, GraphicsFormat.R8G8B8A8_UNorm, (uint)n, (uint)n);
                    var sb = new StringBuilder("PLANNER.map(");
                    sb.Append(FormattableString.Invariant($"{{\"name\":\"{name}\",\"w\":{mapSize.x},\"h\":{mapSize.y},\"grid\":{gridSize},"));
                    sb.Append("\"img\":\"data:image/jpeg;base64,").Append(Convert.ToBase64String(jpg));
                    sb.Append("\",\"relief\":\"data:image/png;base64,").Append(Convert.ToBase64String(png)).Append("\"});\n");
                    File.WriteAllText(Path.Combine(MapsDir, name + ".js"), sb.ToString());
                    WriteMapList();
                    log.LogInfo($"Flight planner: map '{name}' saved to {MapsDir}");
                }
                catch (Exception ex)
                {
                    log.LogWarning($"Flight planner: can't save map '{name}': {ex.Message}");
                }
            });
        }

        private static readonly object listLock = new object();

        private static void WriteMapList()
        {
            lock (listLock)
            {
                string[] names = Directory.GetFiles(MapsDir, "*.js").Select(Path.GetFileNameWithoutExtension).Select(Safe).Where(s => s.Length > 0).ToArray();
                File.WriteAllText(Path.Combine(Dir, "maps.js"), "PLANNER.maps([" + string.Join(",", names.Select(s => "\"" + s + "\"")) + "]);\n");
            }
        }

        private static readonly Regex Servable = new Regex(@"^(maps/)?[A-Za-z0-9_-]+\.(js|html)$");

        /// <summary>A planner file for the web mirror (/planner/...), or null. The name is checked: no other files.</summary>
        public static byte[] Serve(string rel, out string type)
        {
            if (rel == "") rel = "planner.html";
            type = rel.EndsWith(".html") ? "text/html; charset=utf-8" : "application/javascript; charset=utf-8";
            if (!Servable.IsMatch(rel)) return null;
            try { return File.ReadAllBytes(Path.Combine(Dir, rel.Replace('/', Path.DirectorySeparatorChar))); }
            catch (Exception) { return null; }
        }
    }
}
