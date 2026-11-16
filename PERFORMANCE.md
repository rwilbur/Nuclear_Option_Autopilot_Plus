# Performance review

Where the mod spends frame time, ranked by expected cost, with fixes. Line numbers are as of this
review.

This is from reading the code, not from profiling: nothing below has been measured in the game
yet. The first item is therefore to add timing, so the rest can be checked and re-ranked against
real numbers.

## What changed recently (likely regressions)

Several of this session's changes raised per-frame cost. If the frame-rate drop started recently,
these are the first suspects:

| Change | Cost factor |
|---|---|
| Web map stream default `MapResolution` 1440 → 2560 (your config was migrated) | ~3.2× pixels rendered, read back and JPEG-encoded per frame (1440×810 → 2560×1440) |
| Radar profiles 360×400 → 720×800 bins, overlay texture 512² → 1024² | 4× sweep work, 4× viewshed work, 4× rasterisation and upload |
| Fine terrain tile when zoomed in (`DetailN` 768) | 590k extra raycasts per tile, redone as the view moves |
| Terrain mapped in the background on every mission, even with the map closed (for the planner export) | ~4M raycasts at 8192 per frame, about 500 frames per mission |
| Flight planner map export | One 4096² main-thread GPU copy plus ~64 MB managed allocation per map per session |

## Done (output unchanged)

- **Mirror readback into our own buffer** (`RequestIntoNativeArray`): the up-to-14.7 MB main-thread
  copy per streamed frame is gone (item 1.1).
- **Planner map already current:** if `FlightPlanner/maps/<map>.js` is newer than the mod DLL, the
  background terrain mapping and the 64 MB export are skipped (items 4.1 and 5). The radar page
  then maps the terrain when first opened, as it did before the planner existed.
- **`useGUILayout = false`** on the plugin: nothing uses GUILayout, so Unity no longer runs the
  Layout pass of `OnGUI` every frame. *New, not in the review below.*
- **Scan-cone test without `acos`:** `Rasterize` compared `Vector3.Angle` per pixel; it now
  compares the dot product with `cos(cone)·|v|`, which gives the same answer. *New.*

- **Overlay rebuilt every 0.25 s for nothing** (found with the timing): `ViewMoved` compared
  `viewR * Margin != texR`, always true under Mono's extra float precision. ~9 ms/frame. Fixed.
- **Web map drawn by the browser** from data (`/mapevents`, `/overlay`, `/detail`, `/icon`): no
  render, readback or JPEG encode for the map. `MapAsVideo = true` brings the old stream back.
  The A/P MFD stays a 30 fps JPEG stream.

## Corrections from re-reading

- **Profile sweeps (item 3) are not continuous.** The own radar is re-swept every 2% of its sweep
  range (2× max range): with an 80 km sweep at 250 m/s that is every ~6 s, and a sweep takes
  ~12-23 frames. That's about 5% of frames, so it ranks below the mirror and IMGUI.
- **Each finished sweep triggers a full overlay rebuild** (`Advance` sets `dirty`), including a new
  viewshed. That rebuild, not the view moving, is what usually starts rasterising in follow mode.
- **Watch for continuous rebuilds:** at 4 Hz, any change in `TargetRcs()` (with RCS = own jet's,
  `me.RCS`) or a 3% change in altitude (TGT ALT) rebuilds every viewshed. If the timing shows
  rebuilds back to back, check whether `me.RCS` jitters.

**Immediate workaround, no code:** in `[WebMirror]` set `MapResolution = 1440` and
`FramesPerSecond = 15`. Close the web page when you don't need it (pages only render while
watched).

---

## 0. Measure first — **done** ([Perf.cs](src/Perf.cs))

DATA page, amber line: `MOD <mod ms>/<frame ms>MS U<update> R<radar> G<OnGUI> W<mirror> A<autopilot>`,
ms per frame averaged over the last second. While DATA is on, `LogOutput.log` gets a breakdown every
10 s, including JPEG encode time on the worker thread, GC count and heap size. The original plan
follows.

There's no profiler in the shipped game.
- **Timing:** wrap the main entry points in a `System.Diagnostics.Stopwatch` and keep a rolling
  average:
  - `RadarCoveragePage.Tick` ([RadarCoveragePage.cs](src/RadarCoveragePage.cs))
  - `WebMirror.Update`, `WebMirror.OnGUI` and the readback callback ([WebMirror.cs](src/WebMirror.cs))
  - each page's `OnGUI`
  - `FlightController.FixedTick` ([FlightController.cs](src/FlightController.cs))
- **Where to show it:** add one line to the A/P MFD's DATA page (`MOD 3.2 MS`) and log a breakdown
  every 10 s.
- **Effort:** ~40 lines.
- **Why first:** every item below is an estimate, and this turns them into numbers.

---

## 1. Web mirror: render, readback and JPEG encode (likely the biggest cost)

[WebMirror.cs:230-271](src/WebMirror.cs#L230-L271)

Per streamed frame (30 fps by default), for the map at 2560×1440 and the A/P MFD at 1024²:

| Step | Where | Cost |
|---|---|---|
| IMGUI render of the whole page into a 3.7 MPx render texture | `pages[i].RenderMirror` (L244) | GPU fill plus dozens of draw calls; on the main thread |
| GPU → CPU readback, 14.7 MB per map frame | `AsyncGPUReadback.Request` (L247) | ~440 MB/s of bus traffic at 30 fps |
| `px.CopyFrom(data)`, 14.7 MB memcpy | L265, inside the readback callback, **on the main thread** | ~3-5 ms per map frame |
| JPEG encode of 3.7 MPx | L271, worker thread | ~40-80 ms per frame: one core flat out at 30 fps (frames are dropped when busy), competing with the game's own threads |

**Fixes, in order of payoff per effort:**
1. **Read back into our own buffer:** use `AsyncGPUReadback.RequestIntoNativeArray(ref raw[page], rt, …)`.
   Unity 2022.3 (this game) has it. It removes the 14.7 MB main-thread copy entirely: the callback
   just hands the buffer to the encoder. *Small change, removes the largest main-thread cost.*
2. **Separate frame rates:** give the map its own frame rate (`MapFramesPerSecond`, default 10–15)
   and keep the A/P MFD at 30. The map changes slowly, while the attitude indicator needs the rate.
3. **Lower default `MapResolution`:** 1440 or 1920. 2560 pays mostly for sharp text the overlay
   barely has.
4. **Skip unchanged frames:** don't re-render the map when nothing moved. Track a dirty flag:
   - view moved, or jet moved more than 1 px
   - overlay rebuilt
   - contacts, route or legend changed

   When it isn't following the jet, the map is often static.
5. **Lower JPEG quality** for the map (70), which encodes faster and sends less.
6. **Render less often for the browser:** only render when a browser has taken the last frame (the
   `Readers` count is already tracked). Today a slow client still costs full-rate renders that get
   dropped.

---

## 2. Radar coverage overlay: rasterised on the main thread

[RadarCoveragePage.cs:682-790](src/RadarCoveragePage.cs#L682)

- **What it costs:**
  - **Rasterising:** `ContinueBuild`/`Rasterize` process up to `PixelBudget` = **160,000 pixels
    per frame**, with an `Atan2`, a `Sqrt` and often a `Vector3.Angle` per pixel (~2-4 ms).
  - **Viewsheds:** `Viewshed` (L628) is 576k iterations per radar, rerun whenever the RCS, TGT
    height or own altitude changes.
  - **Upload:** each finished overlay is a 1M-pixel colour pass plus `SetPixels32` and `Apply`
    (L703-704), so 4 MB is uploaded on the main thread.
- **When it rebuilds:** in follow mode the view moves with the jet. `ViewMoved` (L425) starts a new
  build every time the view shifts 15% of the range. At 250 m/s and 10 km range that's every ~6 s,
  and with several radars a build spans many frames, so the budget is spent nearly every frame.
  TGT ALT mode adds a rebuild every 3% of altitude (L335).

**Fixes:**
1. **Move rasterising to a worker thread:**
   - **Snapshot on the main thread:** each radar's cone, heading, pitch, origin and range, and the
     view.
   - **Worker thread:** viewsheds and rasterising into a back buffer. Profiles, heights and `Vis`
     are plain arrays.
   - **Main thread again:** only the upload.

   *Biggest main-thread saving in this file.*
2. **Upload without the copy:** write colours straight into `overlay.GetPixelData<Color32>(0)` (a
   NativeArray view of the texture) and skip `SetPixels32`'s extra copy.
3. **Match the overlay to the display:** the in-game radar MFD is ~300-500 px on screen and doesn't
   need 1024². Pick `Tex` per display, or only build 1024² when the web map is watched.
4. **Pan instead of rebuild:** in follow mode, shift the existing overlay by the view offset and
   rebuild only when needed. It's already drawn where it was built, so a larger `Margin` (e.g. 2.0)
   with a 25% slack halves the rebuilds.

## 3. Radar profile sweeps: 50,000 terrain lookups per frame

[RadarCoveragePage.cs:563](src/RadarCoveragePage.cs#L563) (`Advance`), budget `SampleBudget` (L104)

- **What it costs:** each radar's 720×800 polar profile is swept from the height grid, so it costs
  no raycasts, but still 50k bilinear lookups per frame (~1-2 ms).
- **Why it's nearly continuous:** your own radar is re-swept every time the jet moves 2% of its
  radar range. With 4× the bins (this session) a sweep takes ~4× longer.

**Fixes:**
1. **Worker thread:** `TerrainHeightGrid.Height` only reads a `ushort[]`, so the whole sweep can run
   there and swap the finished arrays in on the main thread. *Removes the cost from the frame
   entirely.*
2. **Re-sweep less often:** for your own radar, use 5% of range instead of 2%. The coverage
   difference is invisible at map scale.

## 4. Raycasts: terrain grid, fine tile, terrain following

| Source | Where | Cost |
|---|---|---|
| Whole-map height grid | `TerrainHeightGrid.Fill`, [TerrainHeightGrid.cs](src/TerrainHeightGrid.cs) (`BatchPerFrame` 8192) | 8192 raycasts **waited on in the same frame** (`ScheduleBatch(...).Complete()`). Now runs on every mission even with the map closed (planner export) |
| Fine tile when zoomed in | [RadarCoveragePage.cs:409](src/RadarCoveragePage.cs#L409) | 590k raycasts per tile at 8192 per frame; while following zoomed in, a new tile every few seconds, so nearly continuous |
| TF look-ahead | [TerrainFollowing.cs:143](src/TerrainFollowing.cs#L143) | 120 single raycasts every 0.1 s, only while TF flies the jet: small |
| Pull-up warning | [TerrainWarning.cs:101](src/TerrainWarning.cs#L101) | per physics tick: small |

**Fixes:**
1. **Skip the background mapping when it's not needed:** skip it if `FlightPlanner/maps/<map>.js`
   already exists (the planner already has this map) and the radar page isn't open. *A one-line
   check that removes ~500 frames of raycasts from every later mission on that map.*
2. **Stop waiting on the batch:** let it run while the frame continues and read the hits next
   frame. The floating-origin concern in the comment can be handled by recording the origin offset
   at scheduling, and redoing the batch if it shifted.
3. **Smaller batches when it isn't urgent:** use 2048 per frame. The grid then takes 4× longer, but
   every frame costs less.
4. **Fine tile:**
   - only build it when the view stops moving (~0.5 s after the last pan or zoom), or when not
     following;
   - or build it ahead of the jet, so it isn't rebuilt every few seconds;
   - and lower `DetailN` to 512 (2.25× fewer rays).
5. **TF sweep in one batch:** use one `RaycastCommand` batch (multi-threaded) instead of 120
   `Physics.Raycast` calls. A minor win, only if the timing shows TF matters.

## 5. One-off hitches: map export and relief textures

- **Map export** ([FlightPlanner.cs:128-145](src/FlightPlanner.cs#L128-L145)):
  - **What it costs:** `ReadPixels` of up to a 4096² texture (a main-thread GPU sync), then
    `GetPixels32()`, a **64 MB managed array**. Then base64 strings of ~10 MB, and string-builder
    copies (another ~50 MB of short-lived managed memory).
  - **Fixes:**
    - do it once per map *ever*: skip if the file exists and is newer than the mod;
    - read back with `AsyncGPUReadback` into a `NativeArray` (no managed memory, no stall), encode
      from it, and write the base64 straight to a `FileStream`.
- **Relief textures** ([RadarCoveragePage.cs:381](src/RadarCoveragePage.cs#L381)):
  - **What it costs:** 2048² RGBA plus mipmaps (~21 MB of video memory), uploaded on the main thread
    once per map (a hitch).
  - **Memory held:** `TerrainHeightGrid.relief` keeps its 16 MB `Color32[]` for the whole mission,
    and every fine tile keeps 2.4 MB.
  - **Fixes:**
    - drop the array once uploaded;
    - consider an 8-bit alpha-only texture (`Alpha8`, 4 MB) tinted at draw time, since the shading
      is black/white plus alpha.

## 6. Garbage collection pressure

Unity's Mono garbage collector pauses the game on full collections, and pause time grows with
heap size. The big arrays above (64 MB export, 16 MB relief, 8 MB height grid) raise every pause.

Smaller repeated allocations:

| Allocation | Where | Rate |
|---|---|---|
| `new Dictionary` and `new List` | `CollectRadars`, [RadarCoveragePage.cs:434](src/RadarCoveragePage.cs#L434), [:479](src/RadarCoveragePage.cs#L479) | 1/s |
| Page data strings (`TargetPanel.Json` + `map.Json`) | [WebMirror.cs:201](src/WebMirror.cs#L201) | 4/s while a page is open |
| `routeJson` | [WebMirror.cs:158](src/WebMirror.cs#L158) | 2/s |
| `Encoding.GetBytes` of the page data | per `/targets` request | 2.5/s |
| MJPEG part header string and bytes | [WebMirror.cs:501](src/WebMirror.cs#L501) | per frame per stream |
| A thread per HTTP request | [WebMirror.cs:392](src/WebMirror.cs#L392) | ~3/s plus 30/s while dragging on the web map |

**Fixes:**
- reuse the dictionary and lists;
- build the JSON into one reused `StringBuilder`, and only rebuild it when its inputs changed;
- format the MJPEG header into a reused byte buffer;
- serve short requests from a small fixed set of 4 worker threads, keeping dedicated threads for
  the endless streams.

## 7. IMGUI drawing in game

[AutopilotPlugin.cs:321-335](src/AutopilotPlugin.cs#L321), [MfdFrame.cs:501](src/MfdFrame.cs#L501)

- **What it costs:**
  - **Every frame:** both in-game MFDs are redrawn with IMGUI, which has no batching.
  - **Text:** every glowing text is drawn **5 times** (4 glow copies plus the text).
  - **Radar page:** it draws every contact icon (up to 80) with a rotation each, plus each route
    leg as a rotated texture.
  - **Per input event:** `OnGUI` also runs for layout and input events, with `Layout()` and
    `HandleMouse` each time.
- **Fixes:**
  - draw each in-game MFD into its own render texture at 15–20 fps and blit it each frame (the
    mirror path already renders pages into textures);
  - or drop the glow copies (one shadowed label, or none);
  - return early from `OnGUI` for event types the pages ignore.

## 8. Smaller items

- **`WebMirror.Size()`** reads `Screen.width/height` and recomputes every frame for each page: cache
  it until the window size or config changes.
- **`TargetPanel.Json` / `CollectContacts`** walk every map icon 4×/s: fine. Share one walk between
  them.
- **The flight planner** (browser):
  - it re-runs the whole prediction (dt 0.05 s, up to 2 h of flight) on every edit, and drags it
    per pointer move;
  - fine for normal routes, but cap it or use dt 0.1 s past 200 waypoints;
  - it doesn't affect the game's frame rate.

---

## Suggested order

| # | Item | Effort | Expected effect |
|---|---|---|---|
| 0 | Frame timing on the DATA page | small | Tells us which of these really matter |
| 1 | ~~`RequestIntoNativeArray` in the mirror~~ **done** | small | Removes a 4–15 MB main-thread copy per streamed frame |
| 2 | Separate map frame rate (10–15) and default `MapResolution` 1440–1920 | small | ~2–6× less mirror work |
| 3 | ~~Skip background terrain mapping when the map is already exported~~ **done** (export skipped too) | one line | ~500 frames of raycasts saved per mission |
| 4 | Profile sweeps on a worker thread | medium | 1–2 ms/frame while moving |
| 5 | Coverage rasterising on a worker thread | medium | 2–4 ms/frame in follow mode |
| 6 | Map only re-rendered when changed | medium | Large when not following |
| 7 | Fine tile only when the view rests; `DetailN` 512 | small | Continuous raycasting when zoomed in and following |
| 8 | Export once per map, no managed 64 MB | small | Removes a hitch and heap growth |
| 9 | In-game MFDs to render textures / drop glow | medium | IMGUI cost every frame |
| 10 | Allocation clean-up (JSON, headers, request threads) | small | Fewer GC spikes |
