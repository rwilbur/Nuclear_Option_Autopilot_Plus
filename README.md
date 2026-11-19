# Nuclear Option Autopilot MFD

A BepInEx plugin that adds a fighter-style autopilot to Nuclear Option. It runs entirely inside
the game: the control laws run in the physics tick, and you fly it from MFD pages with 20 bezel
push-buttons and a green-phosphor display. No Python, no virtual gamepad, no ViGEmBus.

**What it does**

- **Autopilot:** heading, bank, NAV, barometric altitude, terrain following (down to 2 m) and
  vertical speed modes, plus an autothrottle. Covers jets, heavy jets, helicopters and the tiltwing.
- **Low-level safety:** an airframe floor, terrain look-ahead along the predicted path, and a
  "Terrain, pull up" warning.
- **Radar coverage map:** which friendly or known enemy radars would see a target at a given
  height. It can also show the masking height, i.e. how low you must fly to stay unseen.
- **Routes:** click waypoints on the map, or let the planner find a radar-avoiding route. The
  autopilot then flies it in NAV mode.
- **Web mirror:** the A/P MFD and a browser-drawn radar map on a second computer, plus a flight
  planner that works with the game closed.

```
  A/P MFD                                         Radar MFD (F9)
            [A/P]   [HDG]   [ATT]   [BALT]  [TF]            [ALL]  [OWN]  [ENMY]  [TGT]  [HIDE]
  [STEP] +--------------------------------------+ [V/S]   [FOLW] +-----------------------+ [EYES]
  [ ^ ]  |  heading tape + bug                  | [ ^ ]   [MAP]  |  game map + coverage  | [ ^ ]
  [HDG]  |  HDG SEL | ALT HLD | SPD             | [ALT]   [^RCS] |  overlay, contacts,   | [RNG]
  [ v ]  |  attitude indicator (+ FD bank cue)  | [ v ]   [RCS]  |  route                | [ v ]
  [SET]  |  ALT  SPD  V/S  THR                  | [SYNC]  [vRCS] +-----------------------+ [SAM]
         +--------------------------------------+                [WPT] [UNDO] [CLR] [RTE] [NAV]
            [A/T]   [ v ]   [SPD]   [ ^ ]   [HIDE]
```

**SET** opens the A/P settings page:
- Left: **SND** (terrain warning sound), **DATA** (debug line), **RESET ALL** (press twice within
  3 s: every setting on this page back to its default), **BACK**.
- Right: **PREV / NEXT** pick a setting from the list (TF smoothing, look-ahead, pull/push g and
  minimum clearance, ground floor, max bank, turn rate, climb rate, A/T max throttle, terrain
  warning time). **▲ / ▼** change it (hold or STEP for 5x steps). **DFLT** puts that one back.
  Values changed from the default show in amber.
- Every change is saved to the .cfg file straight away.

## Contents

- [Install and build](#install-and-build)
- [Flying it](#flying-it)
- [Low level and terrain safety](#low-level-and-terrain-safety)
- [Radar coverage map](#radar-coverage-map)
- [Routes and NAV mode](#routes-and-nav-mode)
- [Rotorcraft](#rotorcraft-chicane-ibis-tarantula)
- [Second screen (web mirror)](#second-screen-web-mirror)
- [Flight planner](#flight-planner)
- [How the control laws work](#how-the-control-laws-work)
- [Tuning](#tuning)
- [Files and tests](#files-and-tests)
- [References](#references)

## Install and build

1. Install BepInEx 5 (Mono) into the game folder. The Thunderstore `BepInExPack` for Nuclear
   Option is the easiest option.
2. Point the build at your game: edit `NuclearOptionDir` in `AutopilotMod.csproj`, or pass
   `-p:NuclearOptionDir="C:\path\to\Nuclear Option"` to `dotnet build`.
3. Run `dotnet build -c Release`. The build copies `AutopilotMod.dll` into
   `<game>\BepInEx\plugins` automatically. Set `<DeployToGame>false</DeployToGame>` to stop that.
4. Start the game. `BepInEx\LogOutput.log` should show `Autopilot MFD ready.`

All settings are in `BepInEx\config\com.yourname.autopilotmod.cfg`, which is created the first
time the game starts with the mod. Each entry has a description. Changes take effect when you
restart the game, or straight away with the BepInEx ConfigurationManager plugin (F1 in-game).

## Flying it

| Key (default)   | Config name (`[Keys]`) | What it does |
|-----------------|------------------------|--------------|
| **F5**          | `AutopilotEngage`      | A/P engage / disconnect. Engaging holds what you have: current heading and altitude become the targets. |
| **F8**          | `AutothrottleEngage`   | Autothrottle engage / disconnect (holds current speed). |
| **F6**          | `ShowHideMfd`          | Show / hide the A/P MFD. |
| **F7**          | `MfdCursor`            | MFD cursor on/off, to click the push-buttons. Drag the bezel to move an MFD; drag its bottom-right corner to resize it. |
| **F9**          | `RadarCoveragePage`    | Radar MFD: off → ALL → OWN → ENMY → off. See [Radar coverage map](#radar-coverage-map). |
| **PgUp / PgDn** | `VerticalTargetUp` / `VerticalTargetDown` | Vertical target up/down (altitude, TF clearance / radar altitude, or V/S, depending on mode). |
| **] / [**       | `HeadingRight` / `HeadingLeft` | Heading bug right/left (bank target in ATT mode). |
| **= / -**       | `SpeedUp` / `SpeedDown` | Speed target up/down. |
| **Shift** + any rocker | `CoarseStepModifier` | 10x step. The STEP button does the same for the on-screen arrows. |

Hold a rocker key, or hold the mouse on an MFD arrow, and it keeps stepping. After 1.5 s it
switches to the 10x step.

**Rebinding keys.** Edit the `[Keys]` section of the config file.
- Values are Unity `KeyCode` names, such as `F5`, `Keypad8`, `PageUp`, `LeftBracket`, `Alpha1`
  or `Equals` (see the [Unity KeyCode reference](https://docs.unity3d.com/ScriptReference/KeyCode.html)).
- Add modifiers with `+`, for example `AutopilotEngage = LeftControl + A`. Holding extra keys,
  such as Shift for the 10x step, does not block a binding.
- `None` unbinds a key. `CoarseStepModifier` is a single key; `LeftShift` or `RightShift` accepts
  either Shift key.
- Keys aren't checked for conflicts with each other or with the game's controls.

### Modes

- **Lateral:**
  - **HDG** flies to the heading bug, banking up to `MaxBank` (35°) for at most a `MaxTurnRate`
    turn (standard rate, 3°/s). It steers on the ground track, which stays steady while the nose
    yaws in a roll, so fast low-level turns don't rock the wings.
  - **ATT** holds a bank angle; below 7° of bank it snaps to wings-level.
  - **NAV** flies a route (see [Routes and NAV mode](#routes-and-nav-mode)).
- **Vertical:**
  - **BALT** holds barometric altitude, down to 2 m for sea skimming: below one ALT step the
    rocker moves in TF-sized steps (1 m / 5 ft).
  - **TF** (terrain following) holds a clearance above the ground or sea with a forward look,
    also down to 2 m. See [Low level and terrain safety](#low-level-and-terrain-safety).
    Rotorcraft have no TF: their button reads **RALT** and holds radar altitude.
  - **V/S** holds a vertical speed.
- **One lift vector.** On jets in HDG and NAV, the climb or descent the vertical mode wants and the
  turn the lateral mode wants are combined into one direction for the lift. The A/P banks to point
  the lift that way and pulls as hard as that needs.
  - **Push-over in a turn:** when terrain drops away mid-turn, the jet keeps its turn's bank and
    keeps pulling round instead of unloading.
  - **Never steeper than a level turn:** a small steering correction over a crest stays a small
    bank, and straight flight is wings level.
- **Climbing comes first, turning gets the rest.** On jets, the A/P uses up to 80% of the g that
  full stick gives the airframe at its current speed. The lift the climb or descent needs is taken
  first, and the turn gets the rest. Terrain that needs a pull-up takes lift from the turn, so the
  jet rolls back toward wings level. `MaxBank` still caps the bank.
  - **No bank swings in TF:** a new pull-up rolls toward level at once, but the bank lets go of it
    over 2 s, so TF's quick pull/push cycles don't rock the wings with them.
- **A/T** is independent of the A/P, as in real jets. It holds speed with the throttle.
- **Re-syncing.** Pressing an already-selected mode, or the target readout beside a rocker,
  re-syncs that target to the current value. **SYNC** re-syncs everything.

**Autothrottle with the A/P (energy coupling).** With both engaged, the A/T adds throttle as soon
as the A/P commands a climb or a banked turn, instead of waiting for the speed to bleed off. The
amount is set by `ClimbFeedforward` and `TurnFeedforward`. This is the total-energy idea behind
TECS [[7]](#ref-tecs).

**Speed protection.** If the A/T is already at `MaxThrottle` and the speed still falls well below
target, the A/P climbs only what the engines can sustain. If needed it descends to regain speed,
and the MFD shows `SPD PROT`. This gives speed priority, as in TECSMOD [[8]](#ref-tecsmod). The
terrain floor always wins.

**Heavy jets.** On airframes whose fly-by-wire allows little pitch or roll rate (the heavy bomber
class), the outer loops automatically use lower gains and a lower roll rate.

### Engaging and disconnects

The A/P refuses to engage when the jet is on the ground, below 50 m/s, banked more than 60°, or
pitched more than 45°.

It drops out, with a warble tone and an amber message, when:
- you move the stick past `StickOverrideThreshold` (paddle-switch behaviour);
- auto-hover turns on;
- you land;
- you leave the cockpit.

Moving the throttle disconnects the A/T. The throttle then stays where the A/T left it.

**Mouse flyers.** While the MFD cursor is on, stick-override detection is paused and the
virtual stick is held centred, so clicking around doesn't fight the A/P. Leave the game's
weapon-safety option on, or a left click on a button could also fire the gun.

## Low level and terrain safety

**Floor and bank limit.** These work in every vertical mode.
- **Height source.** The mod runs its own radar altimeter every physics tick. The game's own
  value updates only every 0.1 s, and over shallow water it measures to the seabed.
- **Smoothing that never lags low.** RALT and TF hold a smoothed radar height so brief spikes
  don't jerk the jet, but a surface coming closer (a rock, a ship, the far side of a crest) is
  taken at once: at 2 m and 350 m/s, half a second of lag was 175 m of ground.
- **What TF (and RALT) hold.** The same reference as the HUD's `R` (the bottom of the gear).
- **Floor.** The A/P keeps the lowest point of the airframe (wingtips, nose, tail, gear, and
  every hanging store and pylon) at least `GroundFloorMeters` (1.5 m) above the surface.
- **Sink limit.** Close to the floor, the allowed sink rate shrinks with clearance
  (`FloorGain`). Further out, it is limited to what a `FloorPullUpG` (1 g) pull-up can still
  stop at the floor. This is the barrier-function idea used in automatic ground-collision
  avoidance [[9]](#ref-gcas), [[10]](#ref-vista).
- **Bank limit.** Near the surface, bank is limited so the low wingtip can't dip in. The MFD
  shows `BANK LIM - LOW` while this is active. The jet then turns as hard as that bank allows,
  more slowly, and stays down, with the wingtip 0.5 m above the floor. For a wingtip about 1 m off
  the surface in a hard low turn, set **FLOOR** on the SET page to 0.5 m.

**Terrain look-ahead** (`TerrainLookahead`, on by default).
- **What it samples.** Every physics tick, 12 rays sample terrain, ship decks and the sea along
  the predicted ground track (including the current turn), out to `LookaheadSeconds` (15 s).
- **When it climbs.** The A/P starts climbing early enough to clear what it finds. It allows for
  a 1 s reaction and a `FloorPullUpG` pull-up, and never climbs steeper than
  `MaxFlightPathAngle`.
- **On the MFD.** `TERRAIN AHEAD - CLIMB` shows while the look-ahead is driving the climb.
- **All modes but TF.** It works in BALT, RALT and V/S, so BALT over mountains climbs too. TF uses
  its own, finer scan instead (below): this one's 5 m margin would hold TF, and a wingtip in a hard
  low bank, well above the clearance you set.

**Terrain following (TF).** Jets only. Press **TF** (the MFD shows `TERR FLW`). The vertical
rocker sets the clearance to hold, never less than `MinClearanceMeters` (2 m). TF can dip several
metres below the target over sharp crests, more the faster you fly, so a very low target over land
is a real risk; over flat sea it is fine.
- **What it does.** It holds the set clearance above the surface, but scans the terrain, ships and sea
  along the track it is about to fly, including the turn HDG or ATT is making, across a corridor
  wider than the wingspan. It starts each climb early enough to cross the high ground at the set
  clearance and level, so it doesn't balloon over the crest, then follows the far side back down.
- **In turns.** TF takes bank away when terrain needs a pull-up, as described above.
  - **Early climbs when banked:** rolling back toward level takes time, so in a steep bank TF
    allows for that time and starts its climbs earlier.
  - **Turn prediction:** it predicts the turn at the rate the current bank limit allows, not the
    rate HDG asks for.
- **Ride.** `PullUpG` (3.0) and `PushOverG` (3.0) in `[TerrainFollowing]` are the g limits, as
  extra g above and below 1 g: by default it pulls up to 4 g and pushes down to -2 g on the G
  meter. They are separate from `MaxPullG`/`MaxPushG`, which limit the other modes. TF never
  uses more than 80% of the pitch rate the airframe's fly-by-wire gives at the current speed,
  so slow or heavy jets fly a softer ride than the setting. It plans the ride with 60% of the
  limits and keeps the rest to catch up. Higher values hug the ground closer. The faster you
  fly and the softer the ride, the more it rides high over short, sharp hills: the jet can't
  turn its flight path that fast.
- **Limits.** Climbs and descents go up to `MaxFlightPathAngle` (20°), not `MaxClimbRate`. Use the
  A/T: nothing else holds your speed in a long climb. If terrain ahead is too steep to clear at
  that angle, the MFD shows `TF CLIMB LIM` in amber. Take the jet and turn away. The terrain
  warning below still works in TF.
- **How far it looks.** `LookAheadSeconds` (15 s of flight, 1 to 8 km).
- **What it can miss.** The scan is 40 distances ahead, three points across the corridor at each,
  refreshed ten times a second, not a solid picture.
  A thin mast between points, or anything that isn't terrain, a building or a ship, isn't seen.
  Aircraft and missiles aren't either.
- **Cost.** About 1200 downward raycasts a second, and only while TF is flying the jet.

**Terrain warning ("Terrain, pull up").** This is on whether or not the A/P is engaged.
- **How it decides.** Each physics tick it looks along your velocity vector for terrain, ships
  or water. It checks whether you could still escape after `ReactionTime` (1.5 s), a roll to wings
  level, and a `PullUpG` (4 g) pull.
- **What you get.** If you couldn't, Windows text-to-speech says "Terrain, terrain. Pull up!" and
  repeats "Pull up!", with a flashing PULL UP on screen. Without a TTS voice, a whoop siren plays.
- **Custom recordings.** Put WAV, OGG or MP3 files in `BepInEx\plugins\AutopilotMod\sounds` and
  restart the game:
  - `terrain_pull_up.*` plays on the first call;
  - `pull_up.*` plays on repeats;
  - `ap_disconnect.*` replaces the disconnect tone.

  No clips are bundled; supply your own and respect the clip's licence.
- **When it stays quiet.** Level sea skimming never triggers it. Shallow descents with the gear
  down (landing approaches) don't either, unless you set `WarnWithGearDown`.

## Radar coverage map

F9 opens a radar MFD over the game's map. The top row picks the view.
- **ALL:** every friendly radar.
- **OWN:** your own radar only, inside its scan cone.
- **ENMY:** enemy radars at the positions your side knows. The colours are swapped here, so red
  means *they* see you.

**Reading the map.** In ALL and OWN:
- **Green:** the game would detect a target at the **TGT** height above ground there.
- **Red:** the signal is strong enough, but the target is hidden by terrain, the earth's curvature
  (radar horizon) or ground clutter.
- **No colour:** the signal is too weak.

**Target height and RCS.**
- **TGT** cycles the target height: 5, 30, 150, 500 and 2000 m above ground; **ALT** (your own
  altitude); and **MASK** (below).
- Each view has its own target RCS. ALL and OWN default to 1; ENMY defaults to your jet's current
  RCS, which grows with stores.
- Set the RCS with the **RCS** arrows (the middle button means your own RCS) or in the web mirror.

**Map controls** (F7 cursor on):
- Scroll to zoom and drag to pan.
- **MAP** shows the whole map; **FOLW** re-centres on your jet.
- Enemy units the game map shows (with its target filters) are drawn with their map icons and
  heading. Click one to select it as a target, and again to deselect it.

**Jamming.** A jammed radar adds no coverage, and its site turns amber.

**EYES** (R1) adds visual spotting: cyan (friendly) / orange (enemy) where a unit would see your
jet by eye (its visual range, your jet's visibility × its magnification, line of sight; enemy
eyes are skipped while you fly faster than they can track).

**SAM** (R5) rings anti-air units at their longest engagement range (ALL: yours, to see the gaps
in your air defence; ENMY: theirs). Solid = missiles, faint = guns; a battery's overlapping rings
are drawn as one.

**RWR** (ENMY, yellow) marks where an enemy radar paints you without detecting you: the game
fires your radar warning and map ping on every radar check that reaches you (out to 2× its range,
in line of sight), even when the signal is too weak to detect you.

**Terrain shading.** Under the coverage the terrain is shaded (lit from the north-west) with
contour lines every 100 m (bolder every 500 m). Zoomed in, a finer height map of the view is
raycast in about a second, with 50 m contours.

**TGT MASK** draws the **masking height**: the lowest height above ground the radars can see at
each spot.
- **In ENMY:** fly below it to stay hidden. Red means seen even at 5 m; green means hidden up to
  2 km.
- **In ALL/OWN:** an intruder below it would slip under your radars.
- **It errs low.** It leaves out ground clutter and the cone's elevation limits, which makes it
  conservative for hiding.

**How it's computed.**
- The map's terrain is raycast once into a height grid and saved to
  `BepInEx\FlightPlanner\terrain\<map>.bin`, so later flights on that map load it instead
  (until a game update; delete the file to force a new mapping).
- Each radar is swept radially (bearings × range bins), keeping the steepest terrain slope seen
  from the radar. This is the standard radial-sweep viewshed [[14]](#ref-viewshed).
- Each sweep is then reduced to a masking height per bin, combining the terrain and the radar
  horizon. This minimum-visible-altitude product is also what professional terrain-analysis tools
  provide [[15]](#ref-luciad).
- With masking heights, any target height is a single comparison per bin. The self-check confirms
  that this gives exactly the same result as the earlier per-height sweep.
- **Off the game's thread.** Sweeps, line-of-sight tables, the overlay's pixels and the web map's
  copy are all worked out on background threads at below-normal priority (at most a quarter of
  your cores), so filling the map doesn't cost frame rate; the game only uploads the finished
  picture. See `PERFORMANCE.md`.

## Routes and NAV mode

The radar MFD's bottom row plans and flies routes.

| Button | What it does |
|--------|--------------|
| **WPT** | Waypoint entry on/off. While lit, a click on the map adds a waypoint (dragging still pans). The last waypoint is the destination. |
| **UNDO** | Remove the last waypoint (or a planned route). |
| **CLR** | Clear the route. |
| **RTE** | Plan a radar-avoiding route from your position to the destination. |
| **NAV** | Fly the route with the A/P. HDG or a heading rocker leaves NAV. |

- **Hand-placed waypoints** are flown leg by leg. Your vertical mode is left alone unless a
  waypoint has a height (below).
- **Editing on the web map** (or loading a plan from the [flight planner](#flight-planner)):
  right-click adds a waypoint (on the route line: inserts one; on a waypoint: deletes it), drag
  moves one, mouse wheel over one sets its leg's **height** and Shift+wheel its **speed**; or type
  them into the route list. A hand-edited planned route is no longer re-planned.
- **Leg height and speed.** A waypoint's height becomes the TF target for the leg flying *to* it
  (`TF RTE` on the MFD); its speed becomes the A/T target as that leg starts (engage the A/T to fly
  it). The rockers override them until the next leg.
- **ETA.** With the A/P on, the MFD's message line shows `ETA TGT h:mm:ss` to the first selected
  target (in NAV: along the route to its point nearest the target, then straight there), or with no
  target in NAV, `ETA WPT` to the route's last waypoint, at the current ground speed.
- **What RTE plans with.** It uses the enemy radars your side knows about, at their known
  positions, and your own RCS (or `RadarEnemyRcs`).
- **How it searches.** It runs Lazy Theta* [[11]](#ref-lazytheta), an any-angle grid search
  [[12]](#ref-theta), over map cells × height bands (`RouteHeightBands`, default 30/150/500 m
  above ground).
- **What a route costs.** A second seen by a radar costs as much as `RouteExposureWeight` seconds
  of extra flying, and low flight costs a little more (`RouteLowFlightPenalty`).
- **The result.** The route goes around radars, or drops into terrain masking where that is
  shorter, and stays higher where it can. This follows recent detection-risk route planners
  [[13]](#ref-riska).
- **Planned legs carry a height.** NAV selects TF and follows the heights leg by leg (`TF RTE`
  on the MFD). It descends early for a lower leg and climbs after the waypoint. Touching the
  vertical rockers or selecting a vertical mode hands the height back to you.
- **Re-planning.** A planned route is re-planned from your position when the known enemy radar
  picture changes: a new, lost or moved site, or a jam-state change. This needs `AutoReplan` and
  happens at most every 10 s.
- **Path following.** NAV uses L1 guidance [[6]](#ref-l1). The L1 distance is kept to at least
  0.6 × the turn radius, so a fast jet's wide turns can't make it circle. The radius is the
  turn the jet can fly now: `MaxBank`, the bank its g budget allows, the wingtip's room over
  the surface (low in TF that can be 20°), and `MaxTurnRate`. It starts the turn onto the next
  leg one turn-lead (plus half the time it takes to roll in, up to the whole next leg) before
  each waypoint and flies it as that arc, so turns roll out on the new leg. L1 alone, starting a
  turn radius out, waited until it was within 0.6 R of the new leg and swung far past it. The
  turn is flown as a turn rate through the same lift vector as HDG, so TF's climbs still come
  first. The A/P MFD shows the waypoint number and the distance to go.
- **Background work.** The planner runs on a worker thread. While a planned route is active, the
  radar page keeps mapping terrain and sweeping enemy radars, even when the page is hidden.
- **Not on rotorcraft.** NAV is for fixed-wing aircraft only.

## Rotorcraft (Chicane, Ibis, Tarantula)

On aircraft whose throttle is a collective, the same MFD and keys fly a helicopter-style AFCS
(automatic flight control system). It works from the hover up to cruise. It flies through the
game's rate-command helicopter fly-by-wire, and also moves the collective and the pedals.

| Aircraft | Configuration | Modelled on | How the A/P flies it |
|----------|---------------|-------------|----------------------|
| SAH-46 Chicane | Single rotor + tail rotor | AH-64 / UH-60 | Collective holds height. Pitch attitude holds speed. |
| UH-90 Ibis | Coaxial rotor + pusher fans | S-97 Raider / X2 | Like a helicopter below ~25 m/s. Above ~40 m/s the fuselage is held level and the pusher fans hold speed. |
| VL-49 Tarantula | Quad tiltwing | CL-84 / V-22 | Blends with the wing angle. Wing up: collective holds height and pitch holds speed. Wing forward: pitch holds height and power holds speed. |

- **Engaging** works at any speed. The aircraft must be at least `MinEngageRadarAlt` up, banked
  less than 45° and pitched less than 30°. Attitude and heading are held from where you engaged.
- **SPD** (the A/T button and key) is a coupled mode here, not an autothrottle. It needs the A/P
  on, and it holds forward *ground* speed.
  - Set it to **0** for **hover hold**: the A/P nulls fore/aft and sideways drift (`HVR HLD`).
  - With SPD off, the A/P holds pitch attitude.
- **Heading.**
  - Below ~12 m/s, the pedals hold the heading bug and the wings stay level (or null drift in
    hover hold).
  - Above ~25 m/s, it makes coordinated banked turns (bank ≤ 30°), with the pedals keeping the
    ball centred.
- **Height.** BALT, RALT (on the TF button) and V/S work as on jets, with two limits:
  - climb and descent are limited to 8 m/s in helicopter mode;
  - sink is limited to 3 m/s at low speed, to stay out of vortex ring state.
- **Near the ground.**
  - The collective is adjusted for rotor tilt, because pitching or banking takes lift away.
  - Within about 15 m of the floor, the speed hold stops tilting the rotor further while the
    aircraft sinks below its commanded climb rate.
  - Sink toward the floor is limited to 0.3 m/s per metre of clearance.
- **Rotor RPM protection** (Chicane, Ibis).
  - The game's engine governor gives at most 110% power, so a big collective demand can drag the
    rotor down.
  - Below 97% rotor RPM, the A/P stops raising the collective and backs it off until the rotor
    recovers. On the Ibis, the pusher fans also stop adding thrust.
  - The MFD shows `ROTOR RPM LIM xx%` while this is active.
- **Ibis Auto Thrust** is switched off while SPD is engaged, and restored afterwards.
- **Tarantula Auto Wing Tilt** stays in charge of the wing. If you switch it off, the A/P flies
  whatever wing angle you set. DATA shows the wing angle as `NAC`.
- **Disconnects.**
  - Stick or pedals past `StickOverrideThreshold`, moving the collective, landing, or auto-hover
    all disconnect the A/P.
  - The collective lever is left where the A/P had it, so the aircraft doesn't drop.

## Second screen (web mirror)

Both MFDs can be mirrored to a browser on another computer on your network. You can click, drag
and scroll them there just like in-game. One difference: holding an arrow on the web page does not
repeat, so each tap is one step (use STEP for 10x steps).

1. In the config file, set `[WebMirror] Enabled = true`.
2. Start the game and get in a cockpit. The server starts the first time you fly.
   - The A/P MFD then shows `WEB MFD http://<ip>:8080`. The address is also in
     `BepInEx\LogOutput.log`.
   - If it shows `WEB MFD OFF - PORT 8080 BUSY`, another program has the port: pick another
     `Port` and set `Enabled = true` again.
   - Allow the game through Windows Firewall on private networks when asked.
3. On the second computer, open that address.

**How it behaves.**
- F6 and F9 still show or hide each MFD on the game screen; the web page shows both regardless.
- Pages are rendered only while a browser is watching them.
- If the picture is upside down, set `FlipVertical = true`.
- Anyone on your network who opens the page can press the MFD buttons.

**The radar map** fills the browser window without its bezel. The browser draws it from data the
game sends (coverage, contacts, route), so it is sharp at any window size and pans and zooms
instantly. The map image and terrain shading come from the flight planner's map file, so the map
picture appears once the game has mapped that map's terrain. Drag to pan, scroll to zoom, click an
enemy to select it, and edit the route as in [Routes and NAV mode](#routes-and-nav-mode). Two
floating windows sit on top; drag them by their title bar, **_** minimises one.

- **A/P**: the autopilot MFD, clickable as in-game. **-** / **+** resize it. Minimising it stops
  the game rendering it.
- **MAP / ROUTE / TARGETS**: everything the map's bezel buttons do (views, target height, EYES,
  SAM, follow jet, whole map, zoom, target RCS typed or **OWN**), then:
  - **Route**: **NAV**, **UNDO**, **CLEAR ROUTE**, **PLAN RTE**, **FLIGHT PLANNER**, the A/P's ETA
    and the planner's state, then each waypoint still ahead with its distance, its leg height and
    speed (type a new one and press Enter; empty = none) and **✕** to delete it.
  - **Target filters**: the game's own target-list filters. Tap one to toggle it; right-click
    or long-press to show only that one. **RESET FILTERS** and **CLEAR SELECTION** do what the
    in-game buttons do.
  - **Selected** lists your targets; tap one to drop it.
  - **Available** lists the enemies the map shows, nearest first. Tap one to add or remove it.

**Mirror settings** (`[WebMirror]`): `Resolution` (A/P MFD, default 1024; capped at the game
window's height), `FramesPerSecond` (default 30, up to 60) and `JpegQuality`. The A/P MFD is
streamed as JPEG frames (MJPEG), encoded off the main thread. `MapAsVideo = true` streams the map
the same way instead of drawing it in the browser (much more CPU); `MapResolution` (default 2560)
is then its long side.

The web server also runs in the game's menus (the MFD images only stream while you fly), so the
flight planner can send a plan to the autopilot before you spawn.

## Flight planner

The flight planner is a map editor for NAV routes. It works with the game closed.
- **Opening it:** double-click `BepInEx\FlightPlanner\planner.html`. With the web mirror on and
  the game running, it's also at `http://<pc>:8080/planner/` (or **FLIGHT PLANNER** on the web
  page).
- **Maps:** each map appears after one flight on it with the mod. The mod maps the terrain in the
  background, then saves the map image and terrain shading to `FlightPlanner\maps\<map>.js`.
- **Airframes:** each jet's fly-by-wire numbers (g limit, corner speed, roll rate) are saved
  after one flight in it. A generic 9 g fighter is always there.

**Editing a plan**
- **Waypoints:** click to add one at the end. Click the route line to insert one, drag a
  waypoint to move it, and right-click one to delete it.
- **Height:** mouse wheel over a waypoint. **Speed:** Shift+wheel over it. Or type either into
  the waypoint table; an empty speed means "keep the current speed".
- **Undo:** **UNDO** or Ctrl+Z.
- **Map:** elsewhere, drag to pan and wheel to zoom. **FIT** shows the whole route.

**Predicted path.** The cyan line is where the chosen jet should fly the route, with a tick
every minute.
- **How it's worked out:** it starts at waypoint 1 heading for waypoint 2, at the start speed,
  and uses the autopilot's own NAV maths: L1 guidance with `L1Period`/`L1Damping`, the turn lead
  at `MaxBank`, the 80% g budget at each leg's speed, and `MaxTurnRate`.
- **The table:** for each waypoint it shows the time it's reached and **MISS**, how close the
  path passes it. Amber means more than 1 km off. NAV cuts corners by about a turn radius, so
  slow down for that leg, or round the corner with more waypoints.
- **What it can't predict:** it is flat (no terrain or climbs), and it treats speed changes as
  8 m/s² either way, whatever the jet.

**Keeping plans**
- **SAVE** keeps a plan in this browser. A page opened from disk and the page served by the game
  are separate browser storage.
- **EXPORT FILE** / **IMPORT FILE** move plans as `.json` files between the two pages, computers
  or players.
- **On the page served by the game:**
  - **SEND TO JET** loads the plan into the autopilot, in flight or from the menus. Then press
    **NAV** in the jet: it flies each leg at its height in TF, and at its speed with the A/T on.
  - **LOAD FROM JET** takes the autopilot's current route into the editor.

## How the control laws work

```
vertical: alt err -> climb rate -> flight-path angle (gamma) -> pitch rate -> stick
lateral:  hdg err -> turn rate  -> bank angle                -> roll rate  -> stick
          (NAV: L1 lateral acceleration / speed -> turn rate)
speed:    speed err (+ energy coupling with the A/P) -> throttle
```

**Inputs** are injected in-process with a Harmony postfix on
`PilotPlayerState.PlayerAxisControls`. It runs after the game reads your stick and before
`Aircraft.FilterInputs()`, the same place your own input lands, with no added latency.

**Outer loops** are proportional, so nothing winds up or stacks.
- Altitude is held through the flight-path angle (from the velocity vector), not pitch attitude.
- Damping on the measured flight-path-angle rate (`FlightPathDamping`) stops the low-speed
  porpoise.
- A coordinated-turn feedforward adds the pitch rate a level turn needs (g·sinφ·tanφ/V).
- Altitude captures follow a square-root profile, so they fly at full rate and round out at the
  end.
- Every command is slew-limited and clamped by g, climb angle, bank and roll rate.

**Rate loops: INDI** (`Tuning.RateLoops`, `Law = Indi`, the default).
- **What INDI does.** The innermost pitch and roll loops use incremental nonlinear dynamic
  inversion [[1]](#ref-sieberling), [[2]](#ref-smeur). Each tick, the stick moves by the increment
  that should produce the missing rate acceleration, based on the measured response.
- **What it removes.** Trim, store loads and errors in the fly-by-wire model then need no
  integrator.
- **Filtering.** The measured acceleration and the applied stick go through the same low-pass
  filter, so their lags match [[2]](#ref-smeur).
- **Safety margin.** Only `IndiIncrementGain` (0.5) of each increment is applied.
- **Tuned to the game's airframe.** Fitted to recorded A/P manoeuvres, the game's fly-by-wire
  answers the stick in about 0.04 s, 1-2 physics ticks late, on both axes. INDI assumes that
  response; it used to assume 0.15 s (pitch) and 0.1 s (roll), believed the stick 3-4x weaker than
  it is, overshot every step and limit-cycled at 7.5 Hz. With the measured response and
  `IndiPitchBandwidth` / `IndiRollBandwidth` 10, a model of it tracks the rate within 0.4°/s
  without chatter from 0.02 to 0.3 s response, 1-4 ticks of delay and 0.5-2x effectiveness.
- **Classic** (`Law = Classic`): feedforward through a model of the game's fly-by-wire (read via
  Harmony), plus PI on rate. It leans on that model, so INDI copes better with damage.

**Learning the airframe** (`AdaptiveEffectiveness`, on by default).
- **What it learns.** For both laws, how much pitch and roll rate the stick really gives on the jet
  you are flying, relative to the fly-by-wire model. If damage or stores change that, the rate loops
  keep their tuning. It learns from your stick inputs as well as the A/P's.
- **How.** A least-squares fit of the measured rate against the model's, both band-passed to the
  slow part of the motion (swings of about 0.4 to 2 s, i.e. manoeuvres). The game answers the stick
  several physics ticks late; an earlier estimate that assumed one tick read that as a weak stick,
  pinned at its floor and made both loops limit-cycle at 7.5 Hz in a recorded flight. Replayed on a
  recorded flight, this one read steady values from 270 to 416 m/s. It remembers about 10 s of
  manoeuvring and forgets only while it has something to learn [[5]](#ref-forgetting), so steady
  flight can't drift it. Online estimation of control
  effectiveness is how adaptive INDI handles model changes [[3]](#ref-delft-ahq), [[4]](#ref-racing-rls).
- **Chatter guard.** Whether learning is on or not, if a loop's stick starts reversing faster than
  6 times a second with real amplitude, that loop is softened (up to 4x) until it calms, then
  released over about 20 s.
- **Checking it.** DATA shows the law and the learned ratios, e.g. `x1.08/0.93` (1 = the
  fly-by-wire model was exact).

**Why the original script oscillated.** The first version of this autopilot was a Python PID
script driving a virtual gamepad: telemetry → UDP → Python PID → ViGEm virtual Xbox pad →
Rewired → game. It stacked integrators on a rate-command airframe: the game's FBW integrator, plus
pitch and vertical-speed integrals in the script. It also added latency and a non-linear stick
curve, and differentiated noisy Euler angles. Moving in-process with rate-based loops removed all
three.

## Tuning

Every gain is in the config file. Turn on **DATA** (on the **SET** page) to see commanded vs
actual bank, flight-path angle, pitch and roll rate, the raw stick outputs, the rate-loop law and
the learned effectiveness. Most low-level settings can also be changed in flight on the **SET** page.

| Symptom | Try |
|---------|-----|
| Altitude bobs, worst when slow | Raise `FlightPathDamping`, or lower `FlightPathGain` |
| Altitude captures feel lazy | Raise `AltitudeCaptureAccel` (round-out) or `MaxClimbRate` (big changes) |
| Final settle onto altitude is slow | Raise `AltitudeGain` |
| Sea skim wanders in height | Raise `FlightPathGain` (with `FlightPathDamping` about 1.25x it) |
| Nose or wings buzz (INDI) | Lower `IndiPitchBandwidth` / `IndiRollBandwidth` (10), then `IndiIncrementGain`; or set `Law = Classic` |
| Rate loops sluggish (INDI) | Raise `IndiPitchBandwidth` / `IndiRollBandwidth` |
| Nose buzzes (Classic) | Lower `PitchRateGain` |
| Wings rock in HDG/ATT (Classic) | Lower `BankGain`, then `RollRateGain` |
| Settles a bit off bank target (Classic) | Raise `RollRateIntegral` |
| TF rides too high over hills | Raise `TerrainFollowing.PullUpG` / `PushOverG`, or fly slower |
| TF ride too violent | Lower `TerrainFollowing.PullUpG` / `PushOverG` |
| TF keeps making sharp corrections | Raise `TerrainFollowing.Smoothing` (0.4 s; 0 = off) |
| Speed hunts with A/T | Raise `SpeedDamping`, or lower `SpeedGain` |
| A/T lights the afterburner | Lower `MaxThrottle` |
| Speed sags in climbs or turns with A/P + A/T | Raise `ClimbFeedforward` / `TurnFeedforward` |
| Speed rises in climbs with A/P + A/T | Lower `ClimbFeedforward` |
| Look-ahead climbs too late / too early | Lower / raise `FloorPullUpG` (keep it below what the jet can pull) |
| Pull-up warning too early / late | `TerrainWarning.ReactionTime`, `PullUpG` |
| NAV wanders across the track / turns too hard | Lower / raise `L1Period` |
| Routes detour too far / go too close to radars | Lower / raise `RouteExposureWeight` |

`ClimbFeedforward` and `TurnFeedforward` are educated guesses that haven't been measured in the
game yet.

## Files and tests

| File | Contents |
|------|----------|
| `src/AutopilotPlugin.cs` | Plugin entry point, aircraft binding, hotkeys, commands |
| `src/Patches.cs` | Harmony hooks into the pilot input path |
| `src/FlightController.cs` | State sampling and the fixed-wing control laws, NAV |
| `src/TerrainFollowing.cs` | The TF mode's look-ahead terrain scan |
| `src/RotorcraftLaws.cs` | Control laws for the helicopters and the tiltwing |
| `src/ApMath.cs` | Pure arithmetic of the control laws (floor, look-ahead, energy coupling, ...) |
| `src/RateLoop.cs` | RLS effectiveness estimator and the INDI rate loop |
| `src/NavGuidance.cs` | Route model and L1 path following |
| `src/RoutePlanner.cs` | Lazy Theta* radar-avoiding route planner |
| `src/CoverageMath.cs` | Masking heights and detection range |
| `src/RadarCoveragePage.cs` | Radar MFD: coverage, MASK view, EYES/SAM, route editing and planning; its data for the web map |
| `src/TerrainHeightGrid.cs` | Terrain height map for the radar page (cached on disk per map) |
| `src/FlightPlanner.cs`, `src/planner.html` | The flight planner (map/airframe export, and the editor app) |
| `src/GunHeatPage.cs` | Small gun-heat display, shown only in aircraft whose gun can overheat |
| `src/Perf.cs` | Where the mod spends its time (DATA page and log) |
| `src/TerrainWarning.cs` | The "Terrain, pull up" warning |
| `src/MfdFrame.cs`, `src/MfdPage.cs` | MFD bezel and drawing; the A/P page and its SET page |
| `src/WebMirror.cs`, `src/TargetPanel.cs` | Web mirror; its target filters and lists |
| `src/ApSettings.cs` | Config entries |
| `src/SoundBank.cs`, `src/DisconnectTone.cs`, `src/Units.cs`, `src/ApCommand.cs` | Helpers |

**Self-check.** `dotnet run --project tests` checks the pure code and needs no game files. It
includes offline models of:
- the helicopter height loop;
- INDI + RLS on a rate-command airframe (response times of 0.1–0.6 s, delays of 0–60 ms,
  effectiveness errors of 0.5–2x);
- a terrain look-ahead ridge crossing;
- terrain following over ridges, a cliff, rolling hills and isolated hills;
- route hand edits (never into flown legs);
- L1 intercepts and reversals;
- route planning around a radar and through a masking valley;
- masking heights against the original line-of-sight sweep.

`node tests/planner-sim.js` checks the flight planner's predicted path (a JavaScript copy of NAV)
against fly-by turn geometry.

These are simplified models, not the game's flight model, so real airframes still need the
[Tuning](#tuning) table.

`PERFORMANCE.md` records where the mod spends frame time and every optimisation made so far.

## References

These papers informed the methods used here. The mod's implementations are simplified and adapted
to the game.

**Rate loops and online estimation**

- <a id="ref-sieberling"></a>**[1]** S. Sieberling, Q. P. Chu, J. A. Mulder, "Robust Flight Control Using Incremental Nonlinear Dynamic Inversion and Angular Acceleration Prediction," *Journal of Guidance, Control, and Dynamics* 33(6), 2010. [doi:10.2514/1.49978](https://doi.org/10.2514/1.49978)
- <a id="ref-smeur"></a>**[2]** E. J. J. Smeur, Q. P. Chu, G. C. H. E. de Croon, "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles," *Journal of Guidance, Control, and Dynamics* 39(3), 2016. [doi:10.2514/1.G001490](https://doi.org/10.2514/1.G001490)
- <a id="ref-delft-ahq"></a>**[3]** "Adaptive Incremental Nonlinear Dynamic Inversion Flight Control for Consistent Handling Qualities," TU Delft. [repository](https://repository.tudelft.nl/record/uuid:d10a6b18-a90d-4936-999d-9c23a9a13f40)
- <a id="ref-racing-rls"></a>**[4]** Online RLS identification of control effectiveness for INDI from a single manoeuvre (racing quadrotor), 2024. [arXiv:2406.11723](https://arxiv.org/abs/2406.11723)
- <a id="ref-forgetting"></a>**[5]** T. R. Fortescue, L. S. Kershenbaum, B. E. Ydstie, "Implementation of self-tuning regulators with variable forgetting factors," *Automatica* 17(6), 1981. [doi:10.1016/0005-1098(81)90070-4](https://doi.org/10.1016/0005-1098(81)90070-4)

**Guidance and energy control**

- <a id="ref-l1"></a>**[6]** S. Park, J. Deyst, J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking," AIAA Guidance, Navigation, and Control Conference, 2004. [doi:10.2514/6.2004-4900](https://doi.org/10.2514/6.2004-4900)
- <a id="ref-tecs"></a>**[7]** A. A. Lambregts, "Vertical Flight Path and Speed Control Autopilot Design Using Total Energy Principles," AIAA Guidance and Control Conference, 1983. [doi:10.2514/6.1983-2239](https://doi.org/10.2514/6.1983-2239)
- <a id="ref-tecsmod"></a>**[8]** P. Bauer et al., modified TECS (TECSMOD) with airspeed priority for UAVs, SZTAKI. [PDF](https://eprints.sztaki.hu/10589/1/Bauer_1_34256645_ny.pdf)

**Ground-collision avoidance**

- <a id="ref-gcas"></a>**[9]** "Nuisance-free Ground Collision Avoidance Design with Adaptive Exponential Control Barrier Functions," 2025. [arXiv:2501.15611](https://arxiv.org/abs/2501.15611)
- <a id="ref-vista"></a>**[10]** "Safety Guardrails in the Sky: Realizing Control Barrier Functions on the VISTA F-16 Jet," 2026. [arXiv:2603.27912](https://arxiv.org/abs/2603.27912)

**Route planning**

- <a id="ref-lazytheta"></a>**[11]** A. Nash, S. Koenig, C. Tovey, "Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D," AAAI 2010. [doi:10.1609/aaai.v24i1.7566](https://doi.org/10.1609/aaai.v24i1.7566)
- <a id="ref-theta"></a>**[12]** K. Daniel, A. Nash, S. Koenig, A. Felner, "Theta*: Any-Angle Path Planning on Grids," *Journal of Artificial Intelligence Research* 39, 2010. [arXiv:1401.3843](https://arxiv.org/abs/1401.3843)
- <a id="ref-riska"></a>**[13]** "Risk-A* and Real-Time MPC for Detection-Risk-Aware Low-Altitude Path Planning of a Fixed-Wing MALE UAV in Mountainous Terrain," *Drones* 10(6), 2026. [doi:10.3390/drones10060469](https://doi.org/10.3390/drones10060469)

**Radar coverage**

- <a id="ref-viewshed"></a>**[14]** "A Comparison of I/O-Efficient Algorithms for Visibility Computation on Massive Grid Terrains," 2018. [arXiv:1810.01946](https://arxiv.org/abs/1810.01946)
- <a id="ref-luciad"></a>**[15]** Luciad, radar line-of-sight (minimum visible altitude) computation. [documentation](https://ldp.luciad.com/portal/productDocumentation/LuciadLightspeed/docs/articles/guide/tea/los.html)
