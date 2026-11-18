using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using BepInEx.Logging;
using Unity.Collections;
using UnityEngine;
using UnityEngine.Experimental.Rendering;
using UnityEngine.Rendering;

namespace AutopilotMod
{
    /// <summary>
    /// Mirrors the MFDs to a web page on the local network, e.g. a second computer at
    /// http://&lt;this PC&gt;:8080. Each MFD is drawn into a render texture, read back from the GPU
    /// without stalling, JPEG-encoded on a worker thread, and served by a tiny HTTP server. Clicks,
    /// drags and scrolls in the browser come back as mouse events on that MFD.
    /// Pages are only rendered while a browser is streaming them.
    /// </summary>
    internal sealed class WebMirror : IDisposable
    {
        private struct Remote
        {
            public int Page;
            public EventType Type;
            public Vector2 Pos01;
            public float Wheel;
            public int Button;  // Unity numbering: 0 left, 1 right
            public bool Shift;
            public int At; // Environment.TickCount when it arrived
        }

        // Browser input waits in the queues while the game is paused or nobody is flying; anything
        // older than this is dropped instead of being replayed on resume.
        private const int InputMaxAgeMs = 1000;
        private const int MaxRequestLine = 16000; // a flight plan travels in one request line (/plan)
        private const int MaxPlanWaypoints = 200;

        private static bool Stale(int at) => unchecked(Environment.TickCount - at) > InputMaxAgeMs;

        private readonly ApSettings cfg;
        private readonly ManualLogSource log;
        private readonly Action<string, bool> report;
        private readonly FlightController ap;
        private readonly RadarCoveragePage map;
        private readonly GunHeatPage gunHeat;
        private readonly MfdFrame[] pages;
        private readonly RenderTexture[] rts;
        // Encoded frames live in a few reused buffers per page (no per-frame garbage: a GC pass over
        // the game's heap is a visible stutter). A buffer is rewritten only when it isn't the current
        // frame and no stream is still sending it.
        private sealed class Frame
        {
            public byte[] Data = new byte[0];
            public int Length;
            public int Readers;
        }
        private const int FramesPerPage = 4;
        private readonly Frame[][] frames;
        private readonly Frame[] current;
        private readonly NativeArray<byte>[] raw; // per-page readback target (only one readback in flight per page)
        private readonly byte[][] rowTmp;         // per-page scratch row for the vertical flip
        private readonly int[] seq;        // bumped on each new frame; streams wait on `frameSignal` for it
        private readonly object frameSignal = new object();
        private readonly int[] busy;       // 1 while a readback/encode is in flight
        private readonly int[] lastPoll;   // Environment.TickCount of the last browser request
        private readonly ConcurrentQueue<Remote> input = new ConcurrentQueue<Remote>();
        private readonly ConcurrentQueue<(string act, string arg, int at)> targetActions = new ConcurrentQueue<(string act, string arg, int at)>();
        private volatile string targetsJson = "{\"t\":{\"filters\":[],\"selected\":[],\"contacts\":[]},\"m\":{\"wps\":[]}}";
        // shortcut: one map shape for every browser (the last one to connect), stretched on the others; fine for one viewer
        private volatile float mapAspect = 16f / 9f;
        private readonly ConcurrentQueue<List<Waypoint>> plans = new ConcurrentQueue<List<Waypoint>>();
        private volatile string routeJson = "{\"wps\":[]}";
        private float nextRoute;
        private int lastTargetsPoll = Environment.TickCount - 60000;
        private float nextTargets;
        private float nextFrame;

        private TcpListener listener;
        private volatile bool running;

        // JPEG encoding runs on our own below-normal-priority threads (one per page), not the shared
        // thread pool, so it never competes with the game's worker threads.
        private readonly BlockingCollection<Action> encodeJobs = new BlockingCollection<Action>();

        public WebMirror(ApSettings cfg, ManualLogSource log, Action<string, bool> report, FlightController ap, MfdPage apPage, RadarCoveragePage map, GunHeatPage gunHeat)
        {
            this.cfg = cfg;
            this.log = log;
            this.report = report;
            this.ap = ap;
            this.map = map;
            this.gunHeat = gunHeat;
            pages = new MfdFrame[] { apPage, map, gunHeat };
            rts = new RenderTexture[pages.Length];
            frames = new Frame[pages.Length][];
            current = new Frame[pages.Length];
            raw = new NativeArray<byte>[pages.Length];
            rowTmp = new byte[pages.Length][];
            seq = new int[pages.Length];
            busy = new int[pages.Length];
            lastPoll = new int[pages.Length];
            for (int i = 0; i < pages.Length; i++)
            {
                lastPoll[i] = Environment.TickCount - 60000;
                frames[i] = new Frame[FramesPerPage];
                for (int k = 0; k < FramesPerPage; k++) frames[i][k] = new Frame();
                new Thread(() => { foreach (Action job in encodeJobs.GetConsumingEnumerable()) job(); })
                    { IsBackground = true, Priority = System.Threading.ThreadPriority.BelowNormal, Name = "WebMfdEncode" }.Start();
            }
        }

        /// <summary>True while a browser has shown this page in the last few seconds (the map: as video or drawn from data).</summary>
        public bool Watching(int page) => running && (Recent(Volatile.Read(ref lastPoll[page])) || page == 1 && Recent(Volatile.Read(ref lastEvents)));

        private static bool Recent(int tick) => unchecked(Environment.TickCount - tick) < 3000;

        // Browser-drawn map: its live state as JSON, pushed to /mapevents streams 20 times a second.
        private string mapJson = "{}";
        private int mapSeq;
        private int lastEvents = Environment.TickCount - 60000;
        private float nextMapState;
        private bool mapFailed;
        private readonly StringBuilder mapSb = new StringBuilder(16 * 1024);

        /// <summary>
        /// Page 0, the A/P MFD: square. Page 1, the map: bezel-less at the browser window's shape,
        /// MapResolution on its long side. Page 2, gun heat: square, half the A/P's size. All fit inside the game window.
        /// </summary>
        private Vector2Int Size(int page)
        {
            if (page != 1)
            {
                int s = Mathf.Clamp(cfg.MirrorResolution.Value / (page == 2 ? 2 : 1), 256, Mathf.Min(Screen.width, Screen.height));
                return new Vector2Int(s, s);
            }
            float a = mapAspect, l = cfg.MirrorMapResolution.Value;
            float w = a >= 1f ? l : l * a, h = a >= 1f ? l / a : l;
            float k = Mathf.Min(1f, Mathf.Min(Screen.width / w, Screen.height / h));
            return new Vector2Int(Mathf.Max(64, (int)(w * k)), Mathf.Max(64, (int)(h * k)));
        }

        // =====================================================================
        // Main thread
        // =====================================================================
        /// <summary>Start / stop the server to follow the config. Call every frame.</summary>
        public void Update(Action<ApCommand> execute)
        {
            if (cfg.MirrorEnabled.Value && listener == null) Start();
            else if (!cfg.MirrorEnabled.Value && listener != null) Stop();
            if (!running) return;
            // flight plans from the planner apply in the menus too: the route waits for the next flight
            while (plans.TryDequeue(out List<Waypoint> plan)) map.LoadPlan(plan);
            Actions(execute);
            if (Recent(Volatile.Read(ref lastEvents)) && Time.unscaledTime >= nextMapState)
            {
                nextMapState = Time.unscaledTime + 0.05f;
                mapSb.Clear();
                try
                {
                    map.WebState(mapSb);
                    string json = mapSb.ToString();
                    lock (frameSignal)
                    {
                        mapJson = json;
                        mapSeq++;
                        Monitor.PulseAll(frameSignal);
                    }
                }
                catch (Exception ex)
                {
                    if (!mapFailed) log.LogWarning($"Web MFD: map data failed: {ex}");
                    mapFailed = true;
                }
            }
            if (Time.unscaledTime < nextRoute) return;
            nextRoute = Time.unscaledTime + 0.5f;
            var sb = new StringBuilder("{\"wps\":[");
            NavRoute nav = ap.Nav; // still to fly
            for (int i = nav.Active; i < nav.Points.Count; i++)
            {
                Waypoint w = nav.Points[i];
                string s = float.IsNaN(w.Speed) ? "null" : w.Speed.ToString("F1", CultureInfo.InvariantCulture);
                string h = float.IsNaN(w.Agl) ? "null" : w.Agl.ToString("F1", CultureInfo.InvariantCulture);
                sb.Append(i > nav.Active ? "," : "").Append(FormattableString.Invariant($"{{\"x\":{w.X:F0},\"z\":{w.Z:F0},\"h\":{h},\"s\":{s}}}"));
            }
            routeJson = sb.Append("]}").ToString();
        }

        /// <summary>Browser actions (target filters, map and route edits, MFD commands) and the page's data, every frame, in the menus too.</summary>
        private void Actions(Action<ApCommand> execute)
        {
            while (targetActions.TryDequeue(out (string act, string arg, int at) a))
            {
                if (Stale(a.at)) continue;
                if (a.act == "opa") // coverage opacity 0..1
                {
                    if (float.TryParse(a.arg, NumberStyles.Float, CultureInfo.InvariantCulture, out float o) && o >= 0f && o <= 1f)
                        cfg.RadarOpacity.Value = o;
                    continue;
                }
                if (a.act == "rcse" || a.act == "rcsf") // coverage-map RCS, enemy / friendly view; 0 = own jet's
                {
                    if (float.TryParse(a.arg, NumberStyles.Float, CultureInfo.InvariantCulture, out float v) && v >= 0f && v <= 1000f)
                        (a.act == "rcse" ? cfg.RadarEnemyRcs : cfg.RadarFriendlyRcs).Value = v;
                    continue;
                }
                if (a.act == "view") // browser-drawn map panned / zoomed: "x:z:halfWidth:follow"
                {
                    float[] v = Nums(a.arg, 4);
                    if (v != null) map.SetView(v[0], v[1], v[2], v[3] != 0f);
                    continue;
                }
                nextTargets = 0f; // show the result on the next poll
                if (a.act == "wnudge") // wheel over waypoint "i:notch:shift"
                {
                    float[] v = Nums(a.arg, 3);
                    if (v != null) map.NudgeWaypoint((int)v[0], Mathf.Sign(v[1]), v[2] != 0f);
                    continue;
                }
                if (a.act == "wmove") // waypoint dragged: "i:x:z" (m)
                {
                    float[] v = Nums(a.arg, 3);
                    if (v != null) map.MoveWaypoint((int)v[0], v[1], v[2]);
                    continue;
                }
                if (a.act == "wedit") // right-click at "x:z:hitRadius" (m)
                {
                    float[] v = Nums(a.arg, 3);
                    if (v != null) map.EditWaypointAt(v[0], v[1], Mathf.Clamp(v[2], 0f, 100000f));
                    continue;
                }
                if (a.act == "cmd") // any MFD button, by its ApCommand name
                {
                    if (Enum.TryParse(a.arg, out ApCommand c) && Enum.IsDefined(typeof(ApCommand), c) && c != ApCommand.None) execute(c);
                    continue;
                }
                if (a.act == "wdel" || a.act == "wclr" || a.act == "wspd") // waypoint i, or "i:height" / "i:speed" in display units (0 = none)
                {
                    string[] parts = a.arg.Split(':');
                    if (!int.TryParse(parts[0], NumberStyles.Integer, CultureInfo.InvariantCulture, out int wi)) continue;
                    if (a.act == "wdel") map.RemoveWaypoint(wi);
                    else if (parts.Length == 2 && float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float c))
                        if (a.act == "wclr") map.SetWaypointHeight(wi, c / Units.Alt(1f));
                        else map.SetWaypointSpeed(wi, c / Units.Speed(1f));
                    continue;
                }
                TargetPanel.Apply(a.act, a.arg);
            }
            if (unchecked(Environment.TickCount - Volatile.Read(ref lastTargetsPoll)) < 3000 && Time.unscaledTime >= nextTargets)
            {
                nextTargets = Time.unscaledTime + 0.25f;
                // built in two halves: a failure in one (logged once) leaves the other on the page
                targetsJson = "{\"t\":" + Safely(TargetPanel.Json, "{\"filters\":[],\"selected\":[],\"contacts\":[]}") + ",\"m\":" + Safely(map.Json, "{\"wps\":[]}") + ",\"g\":" + (gunHeat.Visible ? "true" : "false") + "}";
            }
        }

        private bool jsonFailed;

        private string Safely(Func<string> json, string fallback)
        {
            try { return json(); }
            catch (Exception ex)
            {
                if (!jsonFailed) log.LogWarning($"Web MFD: page data failed: {ex}");
                jsonFailed = true;
                return fallback;
            }
        }

        /// <summary>Call from OnGUI: applies browser input, and on Repaint renders watched pages.</summary>
        public void OnGUI(Action<ApCommand> execute)
        {
            if (!running) return;
            while (input.TryDequeue(out Remote r))
            {
                if (Stale(r.At)) continue;
                Vector2 size = Size(r.Page);
                pages[r.Page].RemoteMouse(r.Type, Vector2.Scale(r.Pos01, size), r.Wheel, r.Button, r.Shift, size, r.Page == 1, execute);
            }

            if (Event.current.type != EventType.Repaint || Time.unscaledTime < nextFrame) return;
            nextFrame = Time.unscaledTime + 1f / Mathf.Clamp(cfg.MirrorFps.Value, 1, 60);
            bool flip = cfg.MirrorFlipVertical.Value;
            int quality = Mathf.Clamp(cfg.MirrorJpegQuality.Value, 10, 100);

            for (int i = 0; i < pages.Length; i++)
            {
                if (!Recent(Volatile.Read(ref lastPoll[i])) || Volatile.Read(ref busy[i]) != 0) continue; // video streams only
                Vector2Int size = Size(i);
                if (rts[i] == null || rts[i].width != size.x || rts[i].height != size.y)
                {
                    if (rts[i] != null) rts[i].Release();
                    rts[i] = new RenderTexture(size.x, size.y, 0, RenderTextureFormat.ARGB32) { hideFlags = HideFlags.HideAndDontSave };
                    rts[i].Create();
                }
                pages[i].RenderMirror(rts[i], i == 1);
                // read straight into our buffer: no main-thread copy of the frame in the callback
                int bytes = size.x * size.y * 4;
                if (!raw[i].IsCreated || raw[i].Length != bytes)
                {
                    if (raw[i].IsCreated) raw[i].Dispose(); // not busy: no readback or encode is using it
                    raw[i] = new NativeArray<byte>(bytes, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
                }
                busy[i] = 1;
                int page = i;
                AsyncGPUReadback.RequestIntoNativeArray(ref raw[i], rts[i], 0, TextureFormat.RGBA32, req => OnReadback(req, page, size.x, size.y, flip, quality));
            }
        }

        private void OnReadback(AsyncGPUReadbackRequest req, int page, int w, int h, bool flip, int quality)
        {
            if (req.hasError || !running)
            {
                Volatile.Write(ref busy[page], 0);
                return;
            }
            NativeArray<byte> px = raw[page];
            encodeJobs.Add(() =>
            {
                long t = Perf.Now;
                try
                {
                    if (flip) FlipRows(px, w * 4, h, page);
                    NativeArray<byte> enc = ImageConversion.EncodeNativeArrayToJPG(px, GraphicsFormat.R8G8B8A8_UNorm, (uint)w, (uint)h, 0, quality);
                    try
                    {
                        Frame f = FreeFrame(page);
                        if (f == null) return; // every buffer still being sent: drop this frame
                        if (f.Data.Length < enc.Length) f.Data = new byte[enc.Length + enc.Length / 4];
                        NativeArray<byte>.Copy(enc, 0, f.Data, 0, enc.Length);
                        f.Length = enc.Length;
                        lock (frameSignal)
                        {
                            current[page] = f;
                            seq[page]++;
                            Monitor.PulseAll(frameSignal);
                        }
                    }
                    finally
                    {
                        enc.Dispose();
                    }
                }
                catch (Exception ex)
                {
                    log.LogWarning($"Web MFD: JPEG encode failed: {ex.Message}");
                }
                finally
                {
                    Perf.Add(Perf.Encode, t);
                    Volatile.Write(ref busy[page], 0);
                }
            });
        }

        /// <summary>A buffer that is neither the current frame nor being sent. Encoder thread only.</summary>
        private Frame FreeFrame(int page)
        {
            foreach (Frame f in frames[page])
                if (f != current[page] && Volatile.Read(ref f.Readers) == 0) return f;
            return null;
        }

        /// <summary>Take the current frame for sending (null if none yet); pair with Release.</summary>
        private Frame Acquire(int page)
        {
            lock (frameSignal)
            {
                Frame f = current[page];
                if (f != null) Interlocked.Increment(ref f.Readers);
                return f;
            }
        }

        private static void Release(Frame f) => Interlocked.Decrement(ref f.Readers);

        private void FlipRows(NativeArray<byte> px, int stride, int rows, int page)
        {
            if (rowTmp[page] == null || rowTmp[page].Length != stride * 2) rowTmp[page] = new byte[stride * 2];
            byte[] tmp = rowTmp[page];
            for (int top = 0, bottom = rows - 1; top < bottom; top++, bottom--)
            {
                NativeArray<byte>.Copy(px, top * stride, tmp, 0, stride);
                NativeArray<byte>.Copy(px, bottom * stride, tmp, stride, stride);
                NativeArray<byte>.Copy(tmp, stride, px, top * stride, stride);
                NativeArray<byte>.Copy(tmp, 0, px, bottom * stride, stride);
            }
        }

        private void Start()
        {
            int port = cfg.MirrorPort.Value;
            try
            {
                listener = new TcpListener(IPAddress.Any, port);
                listener.Start();
            }
            catch (Exception ex)
            {
                log.LogError($"Web MFD: can't listen on port {port}: {ex.Message}");
                report($"WEB MFD OFF - PORT {port} BUSY", true);
                cfg.MirrorEnabled.Value = false;
                listener = null;
                return;
            }
            running = true;
            new Thread(AcceptLoop) { IsBackground = true, Name = "WebMfd" }.Start();

            string ip = Dns.GetHostAddresses(Dns.GetHostName())
                .FirstOrDefault(a => a.AddressFamily == AddressFamily.InterNetwork && !IPAddress.IsLoopback(a))?.ToString() ?? "this-pc";
            string url = $"http://{ip}:{port}";
            log.LogInfo($"Web MFD mirror running at {url}");
            report($"WEB MFD {url}", false);
        }

        private void Stop()
        {
            running = false;
            try { listener?.Stop(); }
            catch (Exception) { /* already closed */ }
            listener = null;
        }

        public void Dispose()
        {
            Stop();
            foreach (RenderTexture rt in rts)
                if (rt != null) rt.Release();
            for (int i = 0; i < raw.Length; i++)
                if (raw[i].IsCreated && Volatile.Read(ref busy[i]) == 0) raw[i].Dispose(); // in use by an encode: leave it to process exit
        }

        // =====================================================================
        // Server threads
        // =====================================================================
        private void AcceptLoop()
        {
            TcpListener l = listener;
            while (running)
            {
                TcpClient client;
                try { client = l.AcceptTcpClient(); }
                catch (Exception) { break; } // listener stopped
                // a thread of its own: MJPEG streams run for as long as the page is open, and on the shared
                // pool they starved every other request (route list, buttons) once a few had piled up
                new Thread(() => Serve(client)) { IsBackground = true, Name = "WebMfdConn" }.Start();
            }
        }

        private void Serve(TcpClient client)
        {
            try
            {
                using (client)
                using (NetworkStream ns = client.GetStream())
                {
                    client.ReceiveTimeout = client.SendTimeout = 5000;
                    var reader = new StreamReader(ns, Encoding.ASCII);
                    string[] request = ReadLine(reader).Split(' ');
                    for (int h = 0; ReadLine(reader).Length > 0; h++) // skip headers
                        if (h > 100) throw new IOException("too many headers");
                    if (request.Length < 2 || request[0] != "GET")
                    {
                        Send(ns, "405 Method Not Allowed", "text/plain", Encoding.ASCII.GetBytes("GET only"));
                        return;
                    }

                    string target = request[1];
                    int q = target.IndexOf('?');
                    string path = q < 0 ? target : target.Substring(0, q);
                    string query = q < 0 ? "" : target.Substring(q + 1);

                    switch (path)
                    {
                        case "/":
                            Send(ns, "200 OK", "text/html; charset=utf-8", Encoding.UTF8.GetBytes(Html.Replace("__MAPVIDEO__", cfg.MirrorMapJpeg.Value ? "true" : "false")));
                            break;
                        case "/mapevents":
                            ServeEvents(ns, Arg(query, "c"));
                            break;
                        case "/overlay":
                            SendOr404(ns, "application/octet-stream", map.OverlayData);
                            break;
                        case "/detail":
                            SendOr404(ns, "application/octet-stream", map.DetailData);
                            break;
                        case "/icon":
                            SendOr404(ns, "image/png", int.TryParse(Arg(query, "i"), NumberStyles.Integer, CultureInfo.InvariantCulture, out int icon) ? RadarCoveragePage.IconPng(icon) : null);
                            break;
                        case "/stream":
                            int sp = PageArg(query);
                            if (sp == 1) mapAspect = Mathf.Clamp(FloatArg(query, "a", mapAspect), 0.25f, 4f);
                            if (sp < 0) Send(ns, "400 Bad Request", "text/plain", Encoding.ASCII.GetBytes("bad page"));
                            else ServeStream(ns, sp, Arg(query, "c"));
                            break;
                        case "/targets":
                            Volatile.Write(ref lastTargetsPoll, Environment.TickCount);
                            Send(ns, "200 OK", "application/json", Encoding.UTF8.GetBytes(targetsJson));
                            break;
                        case "/target":
                            string act = Arg(query, "a");
                            bool known = act == "toggle" || act == "only" || act == "reset" || act == "clear" || act == "desel" || act == "sel" || act == "rcse" || act == "rcsf" || act == "opa"
                                         || act == "cmd" || act == "wdel" || act == "wclr" || act == "wspd" || act == "view" || act == "wnudge" || act == "wedit" || act == "wmove";
                            if (known && targetActions.Count < 64) targetActions.Enqueue((act, Arg(query, "i") ?? "", Environment.TickCount));
                            Send(ns, known ? "204 No Content" : "400 Bad Request", "text/plain", new byte[0]);
                            break;
                        case "/route":
                            Send(ns, "200 OK", "application/json", Encoding.UTF8.GetBytes(routeJson));
                            break;
                        case "/plan":
                            List<Waypoint> plan = ParsePlan(Arg(query, "d"));
                            if (plan != null && plans.Count < 4) plans.Enqueue(plan);
                            Send(ns, plan != null ? "204 No Content" : "400 Bad Request", "text/plain", new byte[0]);
                            break;
                        case "/planner": // relative links in the planner need the trailing slash
                            Send(ns, "200 OK", "text/html", Encoding.ASCII.GetBytes("<meta http-equiv=\"refresh\" content=\"0;url=/planner/\">"));
                            break;
                        case "/input":
                            bool ok = EnqueueInput(query);
                            Send(ns, ok ? "204 No Content" : "400 Bad Request", "text/plain", new byte[0]);
                            break;
                        default:
                            string type = null;
                            byte[] file = path.StartsWith("/planner/") ? FlightPlanner.Serve(path.Substring(9), out type) : null;
                            if (file != null) Send(ns, "200 OK", type, file);
                            else Send(ns, "404 Not Found", "text/plain", Encoding.ASCII.GetBytes("not found"));
                            break;
                    }
                }
            }
            catch (Exception)
            {
                // client went away mid-request; nothing to clean up
            }
        }

        /// <summary>
        /// MJPEG push: one connection per page, each new frame sent once as it is encoded (no
        /// per-frame request, no duplicate frames). Runs until the browser disconnects.
        /// </summary>
        // Newest stream per (browser tab, page). Browsers often keep an MJPEG connection open after the
        // image's src changes; a tab's newer stream ends its older ones here, which frees the browser's
        // few connections per server for the page's other requests.
        private readonly ConcurrentDictionary<string, int> streamGen = new ConcurrentDictionary<string, int>();

        private void ServeStream(Stream s, int page, string tab)
        {
            string key = page + ":" + (tab != null && tab.Length <= 32 ? tab : "");
            int gen = streamGen.AddOrUpdate(key, 1, (_, g) => g + 1);
            byte[] head = Encoding.ASCII.GetBytes(
                "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=mfdframe\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
            s.Write(head, 0, head.Length);
            int sent = -1;
            while (running && streamGen.TryGetValue(key, out int newest) && newest == gen)
            {
                Volatile.Write(ref lastPoll[page], Environment.TickCount); // keeps the page rendering
                lock (frameSignal)
                {
                    if (seq[page] == sent) Monitor.Wait(frameSignal, 500);
                    if (seq[page] == sent) continue;
                    sent = seq[page];
                }
                Frame f = Acquire(page);
                if (f == null) continue;
                try
                {
                    byte[] part = Encoding.ASCII.GetBytes($"--mfdframe\r\nContent-Type: image/jpeg\r\nContent-Length: {f.Length}\r\n\r\n");
                    s.Write(part, 0, part.Length);
                    s.Write(f.Data, 0, f.Length);
                    s.Write(Crlf, 0, 2);
                    s.Flush();
                }
                finally
                {
                    Release(f);
                }
            }
        }

        private static readonly byte[] Crlf = { 13, 10 };

        /// <summary>
        /// Server-sent events: the map's state each time it is rebuilt (20 a second), until the browser
        /// disconnects or the same tab opens a newer stream. EventSource reconnects by itself.
        /// </summary>
        private void ServeEvents(Stream s, string tab)
        {
            string key = "e:" + (tab != null && tab.Length <= 32 ? tab : "");
            int gen = streamGen.AddOrUpdate(key, 1, (_, g) => g + 1);
            byte[] head = Encoding.ASCII.GetBytes(
                "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
            s.Write(head, 0, head.Length);
            int sent = -1;
            while (running && streamGen.TryGetValue(key, out int newest) && newest == gen)
            {
                Volatile.Write(ref lastEvents, Environment.TickCount); // keeps the map page live
                string json = null;
                lock (frameSignal)
                {
                    if (mapSeq == sent) Monitor.Wait(frameSignal, 500);
                    if (mapSeq != sent)
                    {
                        sent = mapSeq;
                        json = mapJson;
                    }
                }
                // nothing new: a comment line, so a closed browser shows up as a failed write
                byte[] b = Encoding.UTF8.GetBytes(json != null ? "data: " + json + "\n\n" : ":\n\n");
                s.Write(b, 0, b.Length);
            }
        }

        private static void SendOr404(Stream s, string type, byte[] body)
        {
            if (body != null) Send(s, "200 OK", type, body);
            else Send(s, "404 Not Found", "text/plain", Encoding.ASCII.GetBytes("not yet"));
        }

        /// <summary>One request line without its line ending ("" at end of stream); throws if it is too long.</summary>
        private static string ReadLine(StreamReader reader)
        {
            var sb = new StringBuilder();
            for (int c = reader.Read(); c >= 0 && c != '\n'; c = reader.Read())
            {
                if (sb.Length >= MaxRequestLine) throw new IOException("request line too long");
                if (c != '\r') sb.Append((char)c);
            }
            return sb.ToString();
        }

        private static void Send(Stream s, string status, string type, byte[] body)
        {
            int length = body.Length;
            byte[] head = Encoding.ASCII.GetBytes(
                $"HTTP/1.1 {status}\r\nContent-Type: {type}\r\nContent-Length: {length}\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
            s.Write(head, 0, head.Length);
            s.Write(body, 0, length);
        }

        private static string Arg(string query, string key)
        {
            foreach (string kv in query.Split('&'))
            {
                int eq = kv.IndexOf('=');
                if (eq > 0 && kv.Substring(0, eq) == key) return Uri.UnescapeDataString(kv.Substring(eq + 1));
            }
            return null;
        }

        private int PageArg(string query) =>
            int.TryParse(Arg(query, "p"), NumberStyles.Integer, CultureInfo.InvariantCulture, out int p) && p >= 0 && p < pages.Length ? p : -1;

        private static float FloatArg(string query, string key, float fallback) =>
            float.TryParse(Arg(query, key), NumberStyles.Float, CultureInfo.InvariantCulture, out float v) && !float.IsNaN(v) ? v : fallback;

        /// <summary>
        /// A flight plan from the planner, "x,z,h,s;..." (global m, leg height m, speed m/s; empty or 0 = none),
        /// checked (it comes from the network); null if malformed.
        /// </summary>
        internal static List<Waypoint> ParsePlan(string d)
        {
            if (string.IsNullOrEmpty(d)) return null;
            string[] wps = d.Split(';');
            if (wps.Length > MaxPlanWaypoints) return null;
            var plan = new List<Waypoint>();
            foreach (string wp in wps)
            {
                string[] f = wp.Split(',');
                if (f.Length != 4 || !Num(f[0], 1e6f, out float x) || !Num(f[1], 1e6f, out float z) || !Num(f[2], 1e4f, out float h)) return null;
                float s = 0f;
                if (f[3].Length > 0 && !Num(f[3], 1000f, out s)) return null;
                plan.Add(new Waypoint(x, z, Mathf.Max(h, 2f), s > 0f ? s : float.NaN));
            }
            return plan;
        }

        /// <summary>Exactly n numbers separated by ':', each within +-1e7 (it comes from the network); null otherwise.</summary>
        private static float[] Nums(string s, int n)
        {
            string[] parts = s.Split(':');
            if (parts.Length != n) return null;
            var v = new float[n];
            for (int i = 0; i < n; i++)
                if (!Num(parts[i], 1e7f, out v[i])) return null;
            return v;
        }

        private static bool Num(string s, float limit, out float v) =>
            float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out v) && !float.IsNaN(v) && Mathf.Abs(v) <= limit;

        /// <summary>Validate browser input (it comes from the network) and queue it for the main thread.</summary>
        private bool EnqueueInput(string query)
        {
            int page = PageArg(query);
            EventType type;
            switch (Arg(query, "t"))
            {
                case "down": type = EventType.MouseDown; break;
                case "drag": type = EventType.MouseDrag; break;
                case "up": type = EventType.MouseUp; break;
                case "wheel": type = EventType.ScrollWheel; break;
                default: return false;
            }
            if (page < 0 || input.Count > 256) return false;
            input.Enqueue(new Remote
            {
                Page = page,
                Type = type,
                Pos01 = new Vector2(Mathf.Clamp01(FloatArg(query, "x", 0.5f)), Mathf.Clamp01(FloatArg(query, "y", 0.5f))),
                Wheel = Mathf.Clamp(FloatArg(query, "w", 0f), -1f, 1f),
                Button = Arg(query, "b") == "1" ? 1 : 0,
                Shift = Arg(query, "sh") == "1",
                At = Environment.TickCount,
            });
            return true;
        }

        private const string Html = @"<!doctype html>
<html lang=""en""><head><meta charset=""utf-8""><meta name=""viewport"" content=""width=device-width,initial-scale=1"">
<title>MFD Mirror</title>
<style>
html,body{margin:0;height:100%;background:#000;overflow:hidden;font:12px Consolas,monospace;color:#5cff73}
img{display:block;touch-action:none;user-select:none;-webkit-user-drag:none;background:#111}
#m1,#mc{position:fixed;inset:0;width:100vw;height:100vh}
#mc{touch-action:none;background:#000}
[hidden]{display:none!important}
.win{position:fixed;z-index:1;display:flex;flex-direction:column;max-height:calc(100vh - 16px);background:#000d;border:1px solid #349442;border-radius:4px}
.bar{display:flex;align-items:center;gap:4px;padding:2px 2px 2px 8px;color:#349442;border-bottom:1px solid #349442;cursor:move;user-select:none;touch-action:none}
.bar span{flex:1}
.bar button{min-height:22px;padding:0 8px}
.win.min>:not(.bar){display:none}
#ap{left:8px;top:8px}
#m0{width:var(--s,360px);height:var(--s,360px)}
#gh{left:8px;top:400px}
#m2{width:var(--s,180px);height:var(--s,180px)}
#cp{right:8px;top:8px;width:min(360px,calc(100vw - 16px))}
#cp .body{overflow-y:auto;padding:4px 8px 8px}
.row{margin:0 0 4px}
.gh{display:block;margin:6px 0 2px;font-size:11px;color:#349442}
button{font:inherit;margin:1px;padding:3px 7px;min-height:28px;background:#111;color:#666;border:1px solid #333;border-radius:3px;cursor:pointer}
button.on{color:#5cff73;border-color:#5cff73}
button.act{color:#ffbd33;border-color:#ffbd33}
button.sel{color:#fff;border-color:#ff4a3d}
button.nav{color:#ff73ff;border-color:#ff73ff}
input{font:inherit;width:9ch;margin:1px;padding:3px 5px;min-height:20px;background:#111;color:#5cff73;border:1px solid #333;border-radius:3px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:3px}
.grid button{display:flex;align-items:center;gap:6px;width:100%;height:28px;margin:0;padding:0 6px;white-space:nowrap}
.grid .n{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;text-align:left}
.grid .d{flex:none;width:7ch;text-align:right;font-variant-numeric:tabular-nums}
.wpt{width:100%;border-collapse:collapse;color:#ff73ff;font-variant-numeric:tabular-nums}
.wpt th{font-weight:normal;font-size:11px;color:#349442;text-align:left;padding:2px 4px;white-space:nowrap}
.wpt td{padding:1px 4px;white-space:nowrap}
.wpt td:nth-child(2){text-align:right;min-width:8ch}
.wpt input{width:9ch;box-sizing:content-box}
.wpt button{min-height:24px;padding:0 8px}
.wp .d{width:9ch;text-align:right;font-variant-numeric:tabular-nums}
.none{color:#349442}
</style></head><body>
<img id=""m1"" alt=""Radar coverage map""><canvas id=""mc"" hidden></canvas>
<div id=""ap"" class=""win""><div class=""bar""><span>A/P</span><button class=""act"" data-z=""0.87"" title=""Smaller"">&minus;</button><button class=""act"" data-z=""1.15"" title=""Bigger"">+</button><button class=""act mn"" title=""Minimise"">_</button></div>
<img id=""m0"" alt=""A/P MFD""></div>
<div id=""gh"" class=""win"" hidden><div class=""bar""><span>GUN HEAT</span><button class=""act"" data-z=""0.87"" title=""Smaller"">&minus;</button><button class=""act"" data-z=""1.15"" title=""Bigger"">+</button><button class=""act mn"" title=""Minimise"">_</button></div>
<img id=""m2"" alt=""Gun heat""></div>
<div id=""cp"" class=""win""><div class=""bar""><span>MAP / ROUTE / TARGETS</span><button class=""act mn"" title=""Minimise"">_</button></div><div class=""body"">
<span class=""gh"">RADARS SHOWN &middot; TARGET HEIGHT</span>
<div class=""row""><button data-c=""RdrAll"" data-m=""ALL"">ALL</button><button data-c=""RdrOwn"" data-m=""OWN"">OWN</button><button data-c=""RdrEnemy"" data-m=""ENMY"">ENMY</button><button class=""act"" data-c=""RdrTgt"" id=""tgt"">TGT</button><button data-c=""RdrEyes"" id=""eyes"" title=""Visual spotting coverage"">EYES</button><button data-c=""RdrSam"" id=""sam"" title=""Anti-air engagement rings"">SAM</button></div>
<div class=""row""><button data-c=""RdrFollow"" id=""folw"">FOLLOW JET</button><button class=""act"" data-c=""RdrMap"">WHOLE MAP</button><button class=""act"" data-c=""RdrZoomOut"">&minus;</button><span id=""rng""></span><button class=""act"" data-c=""RdrZoomIn"">+</button></div>
<div class=""row"">COVERAGE OPACITY <input id=""opa"" type=""range"" min=""0"" max=""1"" step=""0.05"" style=""width:14ch;vertical-align:middle""></div>
<span class=""gh"">TARGET RCS: ALL/OWN &middot; ENMY (0 = OWN JET) &middot; NOW <span id=""rcs""></span></span>
<div class=""row""><input id=""rcsf"" type=""number"" min=""0"" max=""1000"" step=""any"" value=""1""><button class=""act"" data-r=""rcsf"">SET</button><button class=""act"" data-r=""rcsf"" data-own>OWN</button>
&nbsp;<input id=""rcse"" type=""number"" min=""0"" max=""1000"" step=""any"" value=""0""><button class=""act"" data-r=""rcse"">SET</button><button class=""act"" data-r=""rcse"" data-own>OWN</button></div>
<span class=""gh"">ROUTE &middot; RIGHT-CLICK MAP: ADD / INSERT ON LINE / DELETE &middot; HEIGHT: WHEEL OVER A WAYPOINT &middot; SPEED: SHIFT+WHEEL &middot; OR TYPE BELOW + ENTER (SPEED 0 = NONE)</span>
<div class=""row""><button data-c=""NavToggle"" id=""nav"">NAV</button><button class=""act"" data-c=""RteUndo"" title=""Remove the last waypoint (a planned route: clear it)"">UNDO</button><button class=""act"" data-c=""RteClear"">CLEAR ROUTE</button><button class=""act"" data-c=""RtePlan"" id=""rte"" title=""Plan a route to the last waypoint that stays out of the known enemy radars' sight"">PLAN RTE</button><button class=""act"" onclick=""window.open('/planner/')"">FLIGHT PLANNER</button><span id=""eta""></span><span id=""rinfo""></span></div>
<div id=""stale"" class=""row"" style=""color:#ffbd33"" hidden>NO DATA FROM GAME - IS IT RUNNING? RELOAD THIS PAGE IF THIS STAYS</div>
<table class=""wpt""><thead><tr><th>#</th><th>DIST</th><th>HEIGHT <span id=""hu""></span></th><th>SPEED <span id=""su""></span></th><th></th></tr></thead><tbody id=""wps""></tbody></table>
<div id=""flt""></div>
<span class=""gh"">SELECTED</span><div id=""sel"" class=""grid""></div>
<span class=""gh"">AVAILABLE (NEAREST FIRST)</span><div id=""con"" class=""grid""></div>
<p class=""none"">Map: drag = pan (on a waypoint: move it), wheel = zoom (over a waypoint: its height), click enemy = select, right-click = waypoint. Touch: pinch = zoom, long-press = waypoint. Filter: tap = toggle, right-click / long-press = only that one.</p></div></div>
<script>
const $ = id => document.getElementById(id);
const act = (a, i = '') => fetch(`/target?a=${a}&i=${encodeURIComponent(i)}`).then(refresh).catch(() => {});
const btn = (text, cls, onclick, only) => {
  const b = document.createElement('button');
  b.textContent = text; b.className = cls; b.onclick = onclick;
  if (only) b.oncontextmenu = e => { e.preventDefault(); only(); };
  return b;
};
const span = (text, cls = '') => Object.assign(document.createElement('span'), { className: cls, textContent: text });
// fixed-size target cell: name (ellipsised) + distance in a fixed-width column
const tgt = (name, dist, cls, onclick) => {
  const b = btn('', cls, onclick);
  b.title = name; b.append(span(name, 'n'), span(dist, 'd'));
  return b;
};
const none = t => span(t, 'none');
for (const b of document.querySelectorAll('button[data-c]')) b.onclick = () => act('cmd', b.dataset.c);
let last = '';
function refresh() {
  return fetch('/targets').then(r => r.text()).then(t => {
    lastOk = Date.now();
    if (t === last) return;
    last = t;
    const all = JSON.parse(t), s = all.t, m = all.m;
    if ($('gh').hidden === !!all.g) { $('gh').hidden = !all.g; loads[2](); } // only in jets whose gun can overheat
    for (const b of document.querySelectorAll('button[data-m]')) b.className = b.dataset.m === m.mode ? 'on' : '';
    $('tgt').textContent = m.tgt; $('folw').className = m.folw ? 'on' : 'act';
    $('rng').textContent = `RNG ${m.rng} KM`; $('rcs').textContent = m.rcs;
    if (document.activeElement !== $('opa')) $('opa').value = m.opa;
    $('nav').className = m.nav ? 'nav' : 'act'; $('eta').textContent = m.eta;
    $('eyes').className = m.eyes ? 'on' : ''; $('sam').className = m.sam ? 'on' : ''; $('rte').className = m.rte ? 'on' : 'act'; $('rinfo').textContent = m.info || '';
    routeList(m);
    const flt = $('flt');
    flt.replaceChildren();
    let row = null, g = null;
    for (const f of s.filters) {
      if (f.g !== g) {
        g = f.g; row = document.createElement('div'); row.className = 'row';
        row.append(span(g, 'gh'));
        flt.append(row);
      }
      row.append(btn(f.l, f.on ? 'on' : '', () => act('toggle', f.i), () => act('only', f.i)));
    }
    const ar = document.createElement('div'); ar.className = 'row';
    ar.append(btn('RESET FILTERS', 'act', () => act('reset')), btn('CLEAR SELECTION', 'act', () => act('clear')));
    flt.append(ar);
    $('sel').replaceChildren(...(s.selected.length
      ? s.selected.map(u => tgt(u.n, '✕', 'sel', () => act('desel', u.u))) : [none('none')]));
    $('con').replaceChildren(...(s.contacts.length
      ? s.contacts.map(c => tgt((c.s ? '✓ ' : '+ ') + c.n, c.d + ' km', c.s ? 'sel' : 'on', () => act(c.s ? 'desel' : 'sel', c.u)))
      : [none('no enemies shown on the map')]));
  }).catch(() => {});
}
// Route list: rows are rebuilt only when waypoints are added or removed, otherwise updated in place (the
// distances change every poll in flight; rebuilding then threw away the button or box being clicked).
const wpRows = [];
function routeList(m) {
  const box = $('wps');
  $('hu').textContent = m.unit || ''; $('su').textContent = m.sunit || '';
  if (!m.wps.length) {
    if (wpRows.length || !box.firstChild) {
      wpRows.length = 0;
      const tr = document.createElement('tr'), td = Object.assign(document.createElement('td'), { colSpan: 5, className: 'none', textContent: 'no waypoints' });
      tr.append(td); box.replaceChildren(tr);
    }
    return;
  }
  if (wpRows.length !== m.wps.length) {
    wpRows.length = 0;
    box.replaceChildren(...m.wps.map((w, i) => {
      const tr = document.createElement('tr'), cell = c => { const td = document.createElement('td'); td.append(c); tr.append(td); };
      const num = title => Object.assign(document.createElement('input'), { type: 'number', min: 0, step: 'any', title });
      const r = { d: span(''), c: num('Height above ground (RALT / TF target) for the leg to this waypoint (empty / 0 = leave the height alone)'), sp: num('A/T speed for the leg to this waypoint (empty / 0 = leave the speed alone)') };
      r.sp.placeholder = r.c.placeholder = '-';
      r.c.onchange = () => act('wclr', `${i}:${r.c.value || 0}`);
      r.sp.onchange = () => act('wspd', `${i}:${r.sp.value || 0}`);
      for (const e of [r.c, r.sp]) e.onkeydown = k => { if (k.key === 'Enter') e.blur(); };
      cell(String(i + 1)); cell(r.d); cell(r.c); cell(r.sp); cell(btn('\u2715', 'act', () => act('wdel', i)));
      wpRows.push(r);
      return tr;
    }));
  }
  m.wps.forEach((w, i) => {
    const r = wpRows[i];
    r.d.textContent = `${w.d} km`;
    if (document.activeElement !== r.c) r.c.value = w.c ?? '';
    if (document.activeElement !== r.sp) r.sp.value = w.s ?? '';
  });
}
// map target RCS boxes: rcsf = ALL/OWN, rcse = ENMY; Enter or SET sends, OWN = 0 (own jet's RCS)
const setRcs = (id, own) => { const box = $(id); if (own) box.value = 0; act(id, box.value || 0); };
for (const b of document.querySelectorAll('button[data-r]')) b.onclick = () => setRcs(b.dataset.r, 'own' in b.dataset);
$('opa').onchange = () => act('opa', $('opa').value);
for (const id of ['rcsf', 'rcse']) $(id).onkeydown = e => { if (e.key === 'Enter') setRcs(id); };
// one poll in flight at a time; if none has answered for 3 s, say so instead of showing stale data
let polling = false, lastOk = Date.now();
setInterval(() => {
  $('stale').hidden = Date.now() - lastOk < 3000;
  if (polling) return;
  polling = true;
  refresh().finally(() => { polling = false; });
}, 400);

const tab = Math.random().toString(36).slice(2, 12); // lets the game end this tab's older streams
// The map is drawn here from the game's data (webMap); MapAsVideo = true in the game's config streams it as JPEG instead.
const MAP_VIDEO = __MAPVIDEO__;
// streams: the map is rendered at the window's shape; a minimised or hidden window stops its stream (the game stops rendering it)
const loads = [];
for (const p of MAP_VIDEO ? [0, 1, 2] : [0, 2]) {
  const img = $('m' + p), win = img.closest('.win');
  const load = loads[p] = () => {
    if (win && (win.hidden || win.classList.contains('min'))) return img.removeAttribute('src');
    img.src = `/stream?p=${p}&c=${tab}&a=${(innerWidth / innerHeight).toFixed(3)}&t=${Date.now()}`;
  };
  img.onerror = () => setTimeout(load, 1000);
  const send = (t, e, w = 0, b = 0) => {
    const r = img.getBoundingClientRect();
    const x = ((e.clientX - r.left) / r.width).toFixed(4), y = ((e.clientY - r.top) / r.height).toFixed(4);
    fetch(`/input?p=${p}&t=${t}&x=${x}&y=${y}&w=${w}&b=${b}&sh=${e.shiftKey ? 1 : 0}`).then(() => b && refresh()).catch(() => {});
  };
  let down = false, lastDrag = 0;
  img.addEventListener('pointerdown', e => { if (e.button) return; down = true; img.setPointerCapture(e.pointerId); send('down', e); e.preventDefault(); });
  img.addEventListener('pointermove', e => {
    if (!down || e.timeStamp - lastDrag < 33) return;
    lastDrag = e.timeStamp; send('drag', e);
  });
  const up = e => { if (down) { down = false; send('up', e); } };
  img.addEventListener('pointerup', up);
  img.addEventListener('pointercancel', up);
  // Shift+wheel arrives as a sideways scroll in most browsers
  img.addEventListener('wheel', e => { send('wheel', e, Math.sign(e.deltaY || e.deltaX)); e.preventDefault(); }, { passive: false });
  img.addEventListener('contextmenu', e => { e.preventDefault(); if (p === 1) send('down', e, 0, 1); }); // right-click / long-press
}
let resized = 0;
if (MAP_VIDEO) addEventListener('resize', () => { clearTimeout(resized); resized = setTimeout(loads[1], 500); });

// floating windows: drag by the title bar, _ minimises, -/+ resizes the A/P MFD; remembered per browser
const store = (k, v) => { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) {} };
const recall = k => { try { return JSON.parse(localStorage.getItem(k)); } catch (e) { return null; } };
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
for (const w of document.querySelectorAll('.win')) {
  const bar = w.querySelector('.bar'), key = 'win-' + w.id;
  const place = (x, y) => Object.assign(w.style, { left: clamp(x, 0, innerWidth - 80) + 'px', top: clamp(y, 0, innerHeight - 30) + 'px', right: 'auto' });
  const save = () => { const r = w.getBoundingClientRect(); store(key, { x: r.left, y: r.top, min: w.classList.contains('min'), s: w.style.getPropertyValue('--s') }); };
  const st = recall(key);
  if (st) { place(st.x, st.y); w.classList.toggle('min', !!st.min); if (st.s) w.style.setProperty('--s', st.s); }
  bar.addEventListener('pointerdown', e => {
    if (e.target.closest('button')) return;
    const r = w.getBoundingClientRect(), ox = e.clientX - r.left, oy = e.clientY - r.top;
    bar.setPointerCapture(e.pointerId);
    bar.onpointermove = m => place(m.clientX - ox, m.clientY - oy);
    bar.onpointerup = () => { bar.onpointermove = bar.onpointerup = null; save(); };
  });
  const img = w.querySelector('img');
  w.querySelector('.mn').onclick = () => { w.classList.toggle('min'); save(); if (img) loads[img.id.slice(1)](); };
  for (const z of w.querySelectorAll('[data-z]')) z.onclick = () => {
    const s = clamp(img.offsetWidth * z.dataset.z, 120, innerHeight - 40);
    w.style.setProperty('--s', s + 'px'); save();
  };
}
loads.forEach(load => load());

// ---- the map drawn from data: the game pushes its state 20 times a second (/mapevents); the map image and
// terrain shading come from the flight planner's map file; the coverage overlay (/overlay) and the fine
// terrain tile (/detail) are fetched when they change. Pan and zoom happen here at once, then go to the game.
function webMap() {
  const cv = $('mc'), ctx = cv.getContext('2d');
  $('m1').hidden = true; cv.hidden = false;
  const OV = 1024, COV = [60, 230, 90], MASK = [240, 50, 40];
  const GREEN = '#5cff73', AMBER = '#ffbd33', RED = '#ff4d40', KEY_GREEN = '#59f273', NAV = '#ff73ff', NAV_DIM = 'rgba(255,115,255,0.55)';
  const SITE = ['#66bfff', RED, AMBER]; // friendly, enemy, jammed
  let st = null, stAt = 0, view = null, userAt = -1e9;
  let mapName = '', mapTry = 0, mapData = null, mapImg = null, reliefImg = null;
  let ov = null, ovHave = -1, ovBusy = false, det = null, detHave = -1, detBusy = false;
  const icons = {}, tinted = new Map();
  const image = src => Object.assign(new Image(), { src });
  const ready = im => im && im.complete && im.naturalWidth > 0;
  window.PLANNER = { map: m => { if (m.name === mapName) { mapData = m; mapImg = image(m.img); reliefImg = image(m.relief); } } };

  // the planner's map file appears once the game has mapped the terrain: retry every 5 s until then
  function loadMap() {
    if (!st.map || (st.map === mapName && (mapData || Date.now() - mapTry < 5000))) return;
    if (st.map !== mapName) { mapName = st.map; mapData = null; }
    mapTry = Date.now();
    const s = document.createElement('script');
    s.src = `/planner/maps/${mapName}.js?${mapTry}`;
    s.onload = s.onerror = () => s.remove();
    document.head.append(s);
  }

  // overlay: header (x, z, half-width, opacity, enemy view), then (state, count) runs from the south row up;
  // states 0 none, 1 masked, 2 RWR only (enemy view), 3 eyes, 4 radar - coloured as the game's page;
  // 5..9 TGT MASK masking-height bands, lowest first (RadarCoveragePage.MaskCols; reversed in ALL/OWN)
  const MASKCOLS = [[240, 40, 35, 140], [250, 120, 30, 130], [245, 200, 40, 120], [170, 230, 60, 105], [60, 220, 90, 90]];
  function decodeOverlay(buf) {
    const dv = new DataView(buf), op = dv.getFloat32(12, true), en = dv.getInt32(16, true);
    const a = f => Math.floor(Math.min(1, op * f) * 255);
    const pal = (en // enemy view: seen by them is the danger
      ? [null, [...COV, a(0.73)], [255, 225, 60, a(0.6)], [255, 140, 30, a(1)], [...MASK, a(1)]]
      : [null, [...MASK, a(1)], null, [70, 200, 255, a(0.85)], [...COV, a(0.73)]])
      .concat(en ? MASKCOLS : MASKCOLS.slice().reverse());
    const c = Object.assign(document.createElement('canvas'), { width: OV, height: OV }), g = c.getContext('2d');
    const im = g.createImageData(OV, OV), px = im.data;
    for (let o = 20, k = 0; o + 2 < buf.byteLength; o += 3) {
      const v = dv.getUint8(o), n = dv.getUint16(o + 1, true), col = pal[v];
      if (!col) { k += n; continue; }
      for (const e = k + n; k < e; k++) {
        const i = ((OV - 1 - Math.floor(k / OV)) * OV + k % OV) * 4;
        px[i] = col[0]; px[i + 1] = col[1]; px[i + 2] = col[2]; px[i + 3] = col[3];
      }
    }
    g.putImageData(im, 0, 0);
    return { c, x: dv.getFloat32(0, true), z: dv.getFloat32(4, true), r: dv.getFloat32(8, true) };
  }

  function sync() {
    if (!ovBusy && st.ov >= 0 && st.ov !== ovHave) {
      ovBusy = true; const want = st.ov;
      fetch(`/overlay?n=${want}`).then(r => r.ok ? r.arrayBuffer() : Promise.reject(r.status))
        .then(b => { ov = decodeOverlay(b); ovHave = want; }).catch(() => {}).finally(() => { ovBusy = false; });
    }
    if (!detBusy && st.det >= 0 && st.det !== detHave) {
      detBusy = true; const want = st.det;
      fetch(`/detail?n=${want}`).then(r => r.ok ? r.arrayBuffer() : Promise.reject(r.status))
        .then(b => {
          const dv = new DataView(b);
          return createImageBitmap(new Blob([b.slice(12)], { type: 'image/png' }))
            .then(img => { det = { img, x: dv.getFloat32(0, true), z: dv.getFloat32(4, true), s: dv.getFloat32(8, true) }; detHave = want; });
        }).catch(() => {}).finally(() => { detBusy = false; });
    }
    for (const c of st.con || []) if (!(c.i in icons)) {
      icons[c.i] = null;
      const im = new Image();
      im.onload = () => { icons[c.i] = im; };
      im.onerror = () => setTimeout(() => { delete icons[c.i]; }, 2000); // not encoded yet: ask again
      im.src = `/icon?i=${c.i}`;
    }
  }

  // a contact's sprite in its map colour (white shape x colour, as the game draws it)
  function tint(c) {
    const key = `${c.i}|${c.uv}|${c.c}`;
    let t = tinted.get(key);
    if (t) return t;
    const im = icons[c.i];
    if (!im) return null;
    if (tinted.size > 300) tinted.clear(); // colours fade with track age: don't keep every shade
    const [ux, uy, uw, uh] = c.uv;
    t = Object.assign(document.createElement('canvas'), { width: Math.max(1, uw), height: Math.max(1, uh) });
    const g = t.getContext('2d');
    g.drawImage(im, ux, uy, uw, uh, 0, 0, uw, uh);
    g.globalCompositeOperation = 'source-in'; g.fillStyle = c.c; g.fillRect(0, 0, uw, uh);
    tinted.set(key, t);
    return t;
  }

  const es = new EventSource(`/mapevents?c=${tab}`);
  es.onmessage = e => {
    st = JSON.parse(e.data); stAt = performance.now();
    // the game's view, unless this page panned or zoomed in the last second (the game hasn't caught up yet)
    if (!view || performance.now() - userAt > 1000) view = { x: st.view[0], z: st.view[1], r: st.view[2], folw: st.folw };
    loadMap(); sync();
  };

  const scale = () => cv.width / 2 / view.r; // px per m: the view's half-width spans half the canvas
  const sx = x => cv.width / 2 + (x - view.x) * scale(), sy = z => cv.height / 2 - (z - view.z) * scale();
  const world = ([px, py]) => [view.x + (px - cv.width / 2) / scale(), view.z - (py - cv.height / 2) / scale()];
  const hitR = () => Math.max(10, 0.025 * Math.min(cv.width, cv.height));
  // the jet moved on since the last update (at most 0.3 s: don't run away if updates stop)
  const jetNow = () => { const j = st.jet, t = Math.min((performance.now() - stAt) / 1000, 0.3); return [j[0] + j[3] * t, j[1] + j[4] * t]; };
  const font = (frac, S) => `bold ${Math.max(8, Math.round(frac * S))}px Consolas,monospace`;

  function draw() {
    requestAnimationFrame(draw);
    const dpr = devicePixelRatio || 1, W = Math.round(cv.clientWidth * dpr), H = Math.round(cv.clientHeight * dpr);
    if (cv.width !== W || cv.height !== H) { cv.width = W; cv.height = H; }
    ctx.fillStyle = '#000'; ctx.fillRect(0, 0, W, H);
    if (!st || !view) return;
    ctx.imageSmoothingQuality = 'high';
    const S = Math.min(W, H), jet = st.jet ? jetNow() : null;
    if (view.folw && jet) { view.x = jet[0]; view.z = jet[1]; }
    const k = scale(), m = mapData;

    // the game's map image (tinted to 80%, as the game draws it), then the terrain shading; the fine tile
    // replaces the map's shading where it covers the whole view, as in the game
    if (m && ready(mapImg)) {
      const x = sx(-m.w / 2), y = sy(m.h / 2);
      ctx.drawImage(mapImg, x, y, m.w * k, m.h * k);
      ctx.fillStyle = 'rgba(0,0,0,0.2)'; ctx.fillRect(x, y, m.w * k, m.h * k);
    }
    const d = st.det >= 0 ? det : null;
    if (d && sx(d.x - d.s / 2) <= 0 && sy(d.z + d.s / 2) <= 0 && sx(d.x + d.s / 2) >= W && sy(d.z - d.s / 2) >= H)
      ctx.drawImage(d.img, sx(d.x - d.s / 2), sy(d.z + d.s / 2), d.s * k, d.s * k);
    else if (m && ready(reliefImg)) ctx.drawImage(reliefImg, sx(-m.grid / 2), sy(m.grid / 2), m.grid * k, m.grid * k);
    if (ov) ctx.drawImage(ov.c, sx(ov.x - ov.r), sy(ov.z + ov.r), 2 * ov.r * k, 2 * ov.r * k);

    for (const [x, z, c] of st.sites || []) { ctx.fillStyle = SITE[c]; ctx.fillRect(sx(x) - 3 * dpr, sy(z) - 3 * dpr, 6 * dpr, 6 * dpr); }
    // AA rings at their slant range: missiles solid, guns faint
    ctx.lineWidth = Math.max(1.2, 0.0025 * S);
    for (const [x, z, r, mis] of st.sams || []) {
      ctx.globalAlpha = mis ? 1 : 0.5; ctx.strokeStyle = st.en ? RED : SITE[0];
      ctx.beginPath(); ctx.arc(sx(x), sy(z), r * k, 0, 2 * Math.PI); ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // contacts: the map's own icons, colours and headings; selected targets boxed
    const cs = Math.max(8, 0.016 * S);
    for (const c of st.con || []) {
      const x = sx(c.x), y = sy(c.z);
      if (x < -cs || y < -cs || x > W + cs || y > H + cs) continue;
      const t = tint(c);
      if (!t) continue;
      const w = cs * c.k, h = w * c.uv[3] / Math.max(1, c.uv[2]);
      if (c.s) { ctx.strokeStyle = '#fff'; ctx.lineWidth = Math.max(1.5, 0.004 * S); ctx.strokeRect(x - cs * 0.8, y - cs * 0.8, cs * 1.6, cs * 1.6); }
      ctx.save(); ctx.translate(x, y);
      if (c.r !== null) ctx.rotate(c.r * Math.PI / 180);
      ctx.drawImage(t, -w / 2, -h / 2, w, h);
      ctx.restore();
    }

    // NAV route: jet -> next waypoint -> ..., each marked with its number, clearance and speed
    const lw = Math.max(1.5, 0.003 * S), wr = Math.max(5, 0.009 * S), wps = st.wps || [];
    let px = jet ? sx(jet[0]) : 0, py = jet ? sy(jet[1]) : 0;
    ctx.lineWidth = lw;
    wps.forEach(([x, z], i) => {
      const qx = sx(x), qy = sy(z);
      if (jet || i) { ctx.strokeStyle = i === 0 && st.nav ? NAV : NAV_DIM; ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(qx, qy); ctx.stroke(); }
      px = qx; py = qy;
    });
    ctx.font = font(0.024, S); ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
    ctx.strokeStyle = NAV; ctx.fillStyle = NAV;
    for (const [x, z, label] of wps) {
      const qx = sx(x), qy = sy(z);
      ctx.beginPath(); ctx.arc(qx, qy, wr - lw / 2, 0, 2 * Math.PI); ctx.stroke();
      ctx.fillText(label, qx + wr * 1.4, qy);
    }

    // own jet: heading stick + dot
    if (jet) {
      const x = sx(jet[0]), y = sy(jet[1]);
      ctx.fillStyle = '#fff';
      ctx.save(); ctx.translate(x, y); ctx.rotate(st.jet[2] * Math.PI / 180); ctx.fillRect(-dpr, -0.05 * S, 2 * dpr, 0.05 * S); ctx.restore();
      ctx.beginPath(); ctx.arc(x, y, 3.5 * dpr, 0, 2 * Math.PI); ctx.fill();
    }

    // bottom status line: what is shown + colour key
    const bh = 0.08 * H, by = H - bh / 2;
    ctx.fillStyle = 'rgba(0,0,0,0.6)'; ctx.fillRect(0, H - bh, W, bh);
    ctx.font = font(0.03, S); ctx.fillStyle = st.amber ? AMBER : GREEN; ctx.fillText(st.status || 'NO AIRCRAFT', 0.02 * W, by);
    ctx.font = font(0.026, S);
    const key = st.en ? [['RDR', RED], ['EYE', '#ff8c26'], ['RWR', '#ffe640'], ['HID', KEY_GREEN]] : [['RDR', KEY_GREEN], ['EYE', '#4dccff'], ['HID', RED]];
    key.forEach(([t, c], i) => { ctx.fillStyle = c; ctx.fillText(t, (0.5 + i * 0.12) * W, by); });
  }

  // ---- mouse / touch: as on the game's map page ----
  const pos = e => { const r = cv.getBoundingClientRect(), d = devicePixelRatio || 1; return [(e.clientX - r.left) * d, (e.clientY - r.top) * d]; };
  let sentAt = 0, sendTimer = 0;
  function moved() { // panned / zoomed here: tell the game, at most 10 times a second, always the last view
    userAt = performance.now();
    clearTimeout(sendTimer);
    const send = () => { sentAt = performance.now(); fetch(`/target?a=view&i=${Math.round(view.x)}:${Math.round(view.z)}:${Math.round(view.r)}:${view.folw ? 1 : 0}`).catch(() => {}); };
    const wait = 100 - (performance.now() - sentAt);
    if (wait <= 0) send(); else sendTimer = setTimeout(send, wait);
  }
  const wpAt = p => (st.wps || []).findIndex(([x, z]) => Math.hypot(sx(x) - p[0], sy(z) - p[1]) < hitR());
  let drag = null, pinch = null, press = 0, touch = false, start = null, wdrag = -1; // wdrag: waypoint being dragged
  let wmAt = 0, wmTimer = 0;
  function wmove(p) { // drag a waypoint: drawn here at once, sent at most 10 times a second, always the last position
    const i = wdrag, g = world(p), w = (st.wps || [])[i];
    if (w) { w[0] = g[0]; w[1] = g[1]; }
    clearTimeout(wmTimer);
    const send = () => { wmAt = performance.now(); act('wmove', `${i}:${Math.round(g[0])}:${Math.round(g[1])}`); };
    const wait = 100 - (performance.now() - wmAt);
    if (wait <= 0) send(); else wmTimer = setTimeout(send, wait);
  }
  const pts = new Map(); // pointers down on the map, for pinch
  const pair = () => { const [a, b] = pts.values(); return { d: Math.hypot(a[0] - b[0], a[1] - b[1]) || 1, m: [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2] }; };
  const wedit = p => { const [gx, gz] = world(p); act('wedit', `${Math.round(gx)}:${Math.round(gz)}:${Math.round(hitR() / scale())}`); };
  cv.addEventListener('pointerdown', e => {
    touch = e.pointerType === 'touch';
    if (e.button || !st || !view) return;
    const p = pos(e);
    clearTimeout(press);
    if (drag || pinch) { // second finger: pinch zoom instead of the pan
      pts.set(e.pointerId, p); cv.setPointerCapture(e.pointerId); drag = null; wdrag = -1;
      if (pts.size === 2) { const { d, m } = pair(); pinch = { d, r: view.r, g: world(m) }; }
      return;
    }
    wdrag = wpAt(p); // on a waypoint: drag moves it
    let best = hitR(), hit = null; // on a contact: select / deselect it
    for (const c of st.con || []) { const dd = Math.hypot(sx(c.x) - p[0], sy(c.z) - p[1]); if (dd < best) { best = dd; hit = c; } }
    if (hit && wdrag < 0) { act(hit.s ? 'desel' : 'sel', hit.u); return; }
    pts.set(e.pointerId, p);
    drag = start = p; cv.setPointerCapture(e.pointerId); e.preventDefault();
    if (touch) press = setTimeout(() => { if (drag) { const p = drag; drag = null; wedit(p); } }, 500); // long-press = right-click (iOS sends no contextmenu)
  });
  cv.addEventListener('pointermove', e => {
    if (!pts.has(e.pointerId)) return;
    const p = pos(e);
    pts.set(e.pointerId, p);
    if (pinch) {
      if (pts.size < 2) return;
      const { d, m } = pair();
      view.r = clamp(pinch.r * pinch.d / d, 2000, st.rmax);
      if (!view.folw) { const k = scale(); view.x = pinch.g[0] - (m[0] - cv.width / 2) / k; view.z = pinch.g[1] + (m[1] - cv.height / 2) / k; } // the pinched point stays under the fingers
      return moved();
    }
    if (!drag) return;
    if (Math.hypot(p[0] - start[0], p[1] - start[1]) > 8 * (devicePixelRatio || 1)) clearTimeout(press);
    if (wdrag >= 0) { drag = p; return wmove(p); }
    const k = scale();
    view.x -= (p[0] - drag[0]) / k; view.z += (p[1] - drag[1]) / k; view.folw = false; drag = p;
    moved();
  });
  const up = e => { pts.delete(e.pointerId); clearTimeout(press); drag = null; wdrag = -1; if (!pts.size) pinch = null; };
  cv.addEventListener('pointerup', up);
  cv.addEventListener('pointercancel', up);
  cv.addEventListener('wheel', e => {
    e.preventDefault();
    if (!st || !view) return;
    const p = pos(e), dir = Math.sign(e.deltaY || e.deltaX); // Shift+wheel arrives as a sideways scroll in most browsers
    if (!dir) return;
    const wi = wpAt(p);
    if (wi >= 0) { act('wnudge', `${wi}:${-dir}:${e.shiftKey ? 1 : 0}`); return; } // over a waypoint: its height (Shift: speed)
    const [gx, gz] = world(p), f = clamp(view.r * (dir > 0 ? 1.25 : 0.8), 2000, st.rmax) / view.r;
    view.r *= f;
    if (!view.folw) { view.x = gx + (view.x - gx) * f; view.z = gz + (view.z - gz) * f; } // keep the point under the cursor
    moved();
  }, { passive: false });
  cv.addEventListener('contextmenu', e => { // right-click / long-press: delete a waypoint, insert on a leg, or add one
    e.preventDefault();
    if (!st || !view || touch) return; // touch long-press is handled on pointerdown
    wedit(pos(e));
  });
  requestAnimationFrame(draw);
}
if (!MAP_VIDEO) webMap();
</script></body></html>";
    }
}
