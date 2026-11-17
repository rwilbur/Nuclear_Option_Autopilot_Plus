# Nuclear Option Autopilot MFD

A BepInEx plugin that adds a fighter-style autopilot to Nuclear Option. It runs entirely
inside the game: the control laws run in the physics tick and you fly it from an A/P page drawn
like an MFD (20 bezel push-buttons, green-phosphor display). No Python, no virtual gamepad,
no ViGEmBus.

```
            [A/P]   [HDG]   [ATT]   [BALT]  [TF]
  [STEP] +--------------------------------------+ [V/S]
  [ ^ ]  |  heading tape + bug                  | [ ^ ]
  [HDG]  |  HDG SEL | ALT HLD | SPD             | [ALT]
  [ v ]  |  attitude indicator (+ FD bank cue)  | [ v ]
  [SET]  |  ALT  SPD  V/S  THR                  | [SYNC]
         +--------------------------------------+
            [A/T]   [ v ]   [SPD]   [ ^ ]   [HIDE]
```

**SET** opens the settings page:
- Left: **NAV FLY-BY / FLY-OVR** (waypoint turn style), **SND** (terrain warning sound), **DATA**
  (debug line), **RESET ALL** (press twice within 3 s: every setting on this page back to its
  default), **BACK**.
- Right: **PREV / NEXT** pick a setting from the list (TF smoothing, look-ahead, pull/push g and
  minimum clearance, ground floor, max bank, turn rate, climb rate, A/T max throttle, terrain
  warning time). **▲ / ▼** change it (hold or STEP for 5x steps). **DFLT** puts that one back.
  Values changed from the default show in amber.
- Every change is saved to the .cfg file straight away.

## Install / build

1. Install BepInEx 5 (Mono) into the game folder. The Thunderstore `BepInExPack` for
   Nuclear Option is the easiest option.
2. Point the build at your game: edit `NuclearOptionDir` in `AutopilotMod.csproj`, or pass
   `-p:NuclearOptionDir="C:\path\to\Nuclear Option"` to `dotnet build`.
3. `dotnet build -c Release`. The build copies `AutopilotMod.dll` into
   `<game>\BepInEx\plugins` automatically. Set `<DeployToGame>false</DeployToGame>` to stop that.
4. Start the game. `BepInEx\LogOutput.log` should show `Autopilot MFD ready.`

## Second screen (web mirror)

Both MFDs can be mirrored to a browser on another computer on your network, and you can click,
drag and scroll them there just like in-game. One difference: holding an arrow on the web page
does not repeat, so each tap is one step (use STEP for 10x steps).

1. In `BepInEx\config\com.yourname.autopilotmod.cfg`, set `[WebMirror] Enabled = true`.
2. Start the game and get in a cockpit: the server starts the first time you fly. The A/P MFD
   then shows `WEB MFD http://<ip>:8080` (or `WEB MFD OFF - PORT 8080 BUSY` if another program
   has the port; pick another `Port` and set `Enabled = true` again), and the
   address is also in `BepInEx\LogOutput.log`. Allow the game through Windows Firewall on private
   networks when asked.
3. On the second computer, open that address.

F6 and F9 still show or hide each MFD on the game screen; the web page shows both regardless.
Pages are only rendered while a browser is watching them. If the picture is upside down, set
`FlipVertical = true`. Anyone on your network who opens the page can press the MFD buttons.

The radar map fills the browser window without its bezel. The browser draws it from data the
game sends (coverage, contacts, route), so it is sharp at any window size and pans and zooms
instantly. The map image and terrain shading come from the flight planner's map file, so the
map picture appears once the game has mapped that map's terrain (the first time, a few seconds
into the flight). Drag to pan, scroll to zoom, click an
enemy to select it, and right-click (long-press on Android) to add a waypoint, insert one on the
route line, or delete one. Two
floating windows sit on top. Drag them by their title bar; **_** minimises one. The browser
remembers where you put them.

- **A/P**: the autopilot MFD, clickable as in-game. **-** / **+** resize it. Minimising it stops
  the game rendering it.
- **MAP / ROUTE / TARGETS**: everything the map's bezel buttons do: which radars to show,
  target height, follow jet, whole map, zoom and target RCS (typed, or **OWN** = your jet's).
  Under that are the route and the targets:
  - **Route**: **NAV** and **CLEAR ROUTE**, the A/P's ETA (as on the MFD), plus each waypoint with its distance, its clearance
    (type a new one and press Enter) and **✕** to delete it.
  - **Target filters**: the game's own target-list filters (follow HUD, lased only, faction, unit
    type, vehicle type). Tap to toggle; right-click or long-press to show only that one.
    **RESET FILTERS** and **CLEAR SELECTION** do what the in-game buttons do.
  - **Selected** lists your current targets; tap one to drop it.
  - **Available** lists the enemies the map shows (after the filters), nearest first. Tap to add or
    remove a target.

Mirror settings in `[WebMirror]`: `Resolution` (A/P MFD, default 1024; fits inside the game window),
`FramesPerSecond` (default 30, up to 60) and `JpegQuality`. The A/P MFD is streamed as JPEG frames
(MJPEG), encoded off the main thread. `MapAsVideo = true` streams the map the same way instead of
drawing it in the browser (much more CPU); `MapResolution` (default 2560) is then its long side.

The web server also runs in the game's menus (the MFD images only stream while you fly), so the
flight planner below can send a plan to the autopilot before you spawn.

## Flight planner

The flight planner is a map editor for NAV routes. It works with the game closed.
- **Opening it:** double-click `BepInEx\FlightPlanner\planner.html`. With the web mirror on and the
  game running, it's also at `http://<pc>:8080/planner/` (or **FLIGHT PLANNER** on the web page).
- **Maps:** each map appears after one flight on it with the mod. The mod maps the terrain in the
  background, then saves the map image and terrain shading to `FlightPlanner\maps\<map>.js`.
  The terrain heights go to `FlightPlanner\terrain\<map>.bin`, so later flights on that map skip
  the mapping (until a game update; delete the file to force a new mapping).
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
  and uses the autopilot's own NAV maths: L1 guidance, the 80% g budget at each leg's speed, and
  your `MaxBank`/`MaxTurnRate`.
- **The table:** for each waypoint it shows the time it's reached and **MISS**, how close the
  path passes it. Amber means more than 1 km off. Slow down for that leg, or round the corner
  with more waypoints.
- **What it can't predict:** it is flat (no terrain or climbs), and it treats speed changes as
  8 m/s² either way, whatever the jet.

**Keeping plans**
- **SAVE** keeps a plan in this browser. Note that a page opened from disk and the page served
  by the game are separate browser storage.
- **EXPORT FILE** / **IMPORT FILE** move plans as `.json` files between the two pages, computers
  or players.
- **On the page served by the game:**
  - **SEND TO JET** loads the plan into the autopilot, in flight or from the menus. Then press
    **NAV** in the jet.
  - **LOAD FROM JET** takes the autopilot's current route into the editor.

## Flying it

| Key (default)   | Config name (`[Keys]`) | What it does |
|-----------------|------------------------|--------------|
| **F5**          | `AutopilotEngage`      | A/P engage / disconnect. Engaging holds what you have: current heading and altitude become the targets. |
| **F8**          | `AutothrottleEngage`   | Autothrottle engage / disconnect (holds current speed). |
| **F6**          | `ShowHideMfd`          | Show / hide the MFD. |
| **F7**          | `MfdCursor`            | MFD cursor on/off, so you can click the push-buttons. Drag the bezel to move an MFD; drag the grip in its bottom-right corner to resize it. |
| **F9**          | `RadarCoveragePage`    | Radar coverage MFD over the game map: off → **ALL** friendly radars → **OWN** radar only → **ENEMY** radars at known positions → off. Green = the game would detect a target at the **TGT** height above ground there (**TGT ALT** = at your own current altitude); red = strong enough signal but hidden by terrain, the earth's curvature (radar horizon) or ground clutter; no colour = signal too weak. Each view has its own target RCS (ALL/OWN default 1, ENEMY defaults to your jet's current RCS), set with the **RCS** arrows (middle button = your own jet's RCS) or typed into the web mirror. With the F7 cursor: scroll to zoom, drag to pan, **MAP** shows the whole map, **FOLW** re-centres on your jet. Enemy units the game map shows (with its target filters applied) are drawn with their map icons and heading; click one to select it as a target, click again to deselect. `RadarCoverageOpacity` (or the web page's slider) sets how strong the green/red is. The terrain is shaded (lit from the north-west) with contour lines every 100 m (bolder every 500 m), so valleys stand out. Zoomed in, a finer height map of the view is raycast in about a second, with 50 m contours. Right-click adds a waypoint (on the route line: inserts one; on a waypoint: deletes it); **NAV** (R1) flies the route, **CLR WPT** (R5) clears it. **EYES** (B1) adds visual spotting: cyan (friendly) / orange (enemy) where a unit would see your jet by eye (its visual range, your jet's visibility × its magnification, line of sight; enemy eyes are skipped while you fly faster than they can track). **SAM** (B2) rings anti-air units at their longest engagement range (ALL: yours, to see the gaps in your air defence; ENEMY: theirs) (solid = missiles, faint = guns; a battery's overlapping rings are drawn as one). In ENEMY, yellow (**RWR**) marks where an enemy radar paints you without detecting you: the game fires your radar warning and map ping on every radar check that reaches you (out to 2× its range, in line of sight), even when the signal is too weak to detect you. **AVOID** (B4) switches to ENEMY and re-plans every leg of the route around radar, visual and SAM coverage at that leg's height, adding waypoints. |
| **PgUp / PgDn** | `VerticalTargetUp` / `VerticalTargetDown` | Vertical target up/down (altitude, radar altitude / TF clearance, or V/S, depending on mode). |
| **] / [**       | `HeadingRight` / `HeadingLeft` | Heading bug right/left (bank target in ATT mode). |
| **= / -**       | `SpeedUp` / `SpeedDown` | Speed target up/down. |
| **Shift** + any rocker | `CoarseStepModifier` | 10x step. The STEP button does the same for the on-screen arrows. |

Hold a rocker key, or hold the mouse on an MFD arrow, and it keeps stepping. After 1.5 s it
switches to the 10x step.

**Rebinding keys.** Edit the `[Keys]` section of `BepInEx\config\com.yourname.autopilotmod.cfg`.
The file is created the first time the game starts with the mod. Changes made while the game is
running take effect when you restart it, or straight away if you use the BepInEx
ConfigurationManager plugin (F1 in-game).

- Values are Unity `KeyCode` names, such as `F5`, `Keypad8`, `PageUp`, `LeftBracket`, `Alpha1`
  or `Equals`. The full list is in the
  [Unity KeyCode reference](https://docs.unity3d.com/ScriptReference/KeyCode.html).
- Add modifiers with `+`, for example `AutopilotEngage = LeftControl + A`. The modifiers must be
  held when you press the key. Holding extra keys, such as Shift for the 10x step, does not block it.
- `None` unbinds a key.
- `CoarseStepModifier` is one key with no modifiers. `LeftShift` or `RightShift` accepts either
  Shift key.

Keys aren't checked for conflicts with each other or with the game's own controls, so pick keys
the game doesn't already use.

**Modes**

- Lateral: **HDG** flies to the heading bug, banking up to `MaxBank` (35°) for at most a
  `MaxTurnRate` turn (standard rate, 3°/s). **ATT** holds a bank angle; below 7° of bank it snaps to wings-level.
- **One lift vector.** On jets in HDG and NAV, the climb or descent the vertical mode wants and the
  turn the lateral mode wants are combined into one direction for the lift. The A/P banks to point
  the lift that way and pulls as hard as that needs.
  - **Push-over in a turn:** when terrain drops away mid-turn, the jet keeps its turn's bank and
    keeps pulling round instead of unloading. With no lift there's no turn.
  - **Never steeper than a level turn:** bank is never steeper than a level turn needs for the
    same sideways pull. A small steering correction over a crest stays a small bank, and straight
    flight is wings level.
- **Climbing comes first, turning gets the rest.** On jets, the A/P uses up to 80% of the g that
  full stick gives the airframe at its current speed. The lift the climb or descent needs is taken
  first, and the turn gets everything left over: a level turn can bank to about 80° (7 g on a 9 g
  jet). Terrain that needs a pull-up takes lift from the turn, so the jet rolls back toward wings
  level. A slow jet, or one with little pitch authority, banks less. `MaxBank` still caps the bank.
- Vertical: **BALT** holds barometric altitude, down to 2 m for sea skimming: below one ALT step
  the rocker moves in TF-sized steps (1 m / 5 ft). **TF** (terrain following) holds a clearance
  above the ground or sea with a forward look, also down to 2 m. See below. **V/S** holds a
  vertical speed. (Rotorcraft have no TF: their button reads **RALT** and holds radar altitude.)
- **NAV** flies a route of waypoints. Place them by right-clicking the radar map (in-game with
  the F7 cursor, or on the web page). Right-click the route line to insert a waypoint
  between two others, or a waypoint to delete it. Drag a waypoint to move it. Then press **NAV** on the radar MFD or the web page. NAV is
  flies the drawn legs with L1 path guidance, the method UAV autopilots use.
  - **Steering:** it commands a turn rate (not a heading) toward a point 8 s of flight (at least
    one turn radius) further along the leg, so the jet joins the line and holds it nearly wings
    level instead of banking hard for small errors. At each turn point it switches to the next leg.
  - **Turn radius:** the turn the jet can sustain level within its g budget.
  - **Corners:** it moves to the next leg where that turn begins (a fly-by), or within 300 m of
    the waypoint. A fly-by can't avoid cutting inside a corner: 0.4 turn radii for 90°, 1.6 for
    135°. At 250 m/s the radius is about 1.8 km, so in tight valleys fly slower (the radius
    shrinks with the square of speed) or add waypoints to round sharp corners. Each flown
  waypoint drops off the route; after the last one the A/P holds that heading in HDG.
  - **Fly-over** (**SET** page, **NAV FLY-OVR**; or `NavFlyOverWaypoints` in `[Tuning.Lateral]`): NAV passes within a few metres
    of each waypoint, then turns as hard as the g budget allows (past `MaxBank`/`MaxTurnRate`; the
    climb and TF still get the g first) straight at the next waypoint, rolls out pointing at it and
    flies there wings level. Short hard turns and long wings-level legs leave TF more pull. The
    flight planner still draws fly-by corners.
  - **Clearance**: each waypoint has one, which is the TF target for the leg flying *to*
    it. It is applied as that leg starts, and again whenever you engage the A/P. The rocker
    still overrides it until the next leg.
  - **Setting a waypoint's height:** scroll the mouse wheel over the waypoint on the map, in-game
    or on the web page (10 m or 50 ft a notch). Or type it into the HEIGHT box in the web page's
    route list and press Enter. The map labels each waypoint with its number and height.
  - **Speed per waypoint:** Shift+wheel over a waypoint, or the SPD box in the web page's route
    list (0 clears it). When NAV starts the leg to that waypoint, its speed becomes the A/T target.
    Engage the A/T to fly it.
  - **New waypoints** copy the previous waypoint's height and speed. The first one takes the current
    TF target, or 60 m if you are in another mode.
  - **Leaving NAV**: turning the heading rocker, or pressing HDG or ATT, goes back to that mode.
  - **ETA**: with the A/P on, the MFD's message line shows `ETA TGT h:mm:ss` to the first selected
    target (in NAV: along the route to the point nearest the target, then straight to it; otherwise
    straight there) at its position as your side knows it (live while tracked, else its last known
    position; a target never tracked gets no ETA), or, with no target, `ETA WPT` to the route's last waypoint. It uses the current
    ground speed, so it will be wrong while you are speeding up or slowing down. Warnings take over the line.
  - **TF and the next leg**: TF's look-ahead sees the turn NAV is making now, not the next leg
    beyond the waypoint, until the turn onto it starts.
- **A/T** is independent of the A/P, as in real jets. It holds speed with the throttle.
- **Heavy jets.** On airframes whose fly-by-wire allows little pitch or roll rate (the heavy
  bomber class), the flight-path and bank loops automatically use lower gains and a lower roll
  rate, so they don't outrun the airframe and oscillate.
- Pressing an already-selected mode, or the target readout beside a rocker, re-syncs that
  target to the current value. **SYNC** re-syncs everything.

**Disconnects.** The A/P drops out with a warble tone and an amber message when:
- you move the stick past `StickOverrideThreshold` (paddle-switch behaviour);
- auto-hover turns on;
- you land;
- you leave the cockpit.

Moving the throttle disconnects the A/T. When the A/T is switched off, the throttle stays where
the A/T left it.

**Low level / sea skimming.**
- **Height source.** The mod runs its own radar altimeter every physics tick. The game's own
  value updates only every 0.1 s, and over shallow water it measures to the seabed.
- **What TF holds.** TF's clearance is the same reference the HUD's `R` shows (the bottom of the
  gear).
- **Floor.** In every vertical mode, the AP keeps the *lowest point of the airframe*
  (wingtips, nose, tail, gear) at least `GroundFloorMeters` (1.5 m) above the surface.
- **Bank limit.** Near the water, bank is limited so the low wingtip can't dip in. The MFD shows
  `BANK LIM - LOW` while this is active. The jet then turns as hard as that bank allows at the height
  it is holding, more slowly, and stays down: it never pulls for more turn than its bank gives, which
  would mostly lift it. To turn harder, set a higher TF target for that leg.
- **No forward look.** The floor and BALT can't see terrain ahead. Over hills, use TF.

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
- **How it decides.** Each physics tick it looks along your velocity vector for terrain, ships,
  or water. It works out whether you could still escape after `ReactionTime` (1.5 s), a
  roll-out to wings level, and a `PullUpG` (4 g) pull.
- **What you get.** If not, Windows text-to-speech says "Terrain, terrain. Pull up!" and keeps
  repeating "Pull up!", with a flashing PULL UP on screen. If no TTS voice is available, a
  whoop siren plays instead.
- **Custom recordings.** To use a real cockpit voice, put WAV, OGG or MP3 files in
  `BepInEx\plugins\AutopilotMod\sounds` and restart the game:
  - `terrain_pull_up.*` plays on the first call.
  - `pull_up.*` plays on repeats.
  - `ap_disconnect.*` replaces the disconnect tone.

  Clips are not bundled with the mod; supply your own and respect the clip's license.
  Text-to-speech and the siren remain as fallbacks.
- **When it stays quiet.** Level sea skimming never triggers it. Shallow descents with the gear
  down (landing approaches) don't either, unless you set `WarnWithGearDown`.

The A/P refuses to engage when the jet is on the ground, below 50 m/s, banked more than 60°, or
pitched more than 45°. Rotorcraft have their own limits; see below.

## Rotorcraft (Chicane, Ibis, Tarantula)

On aircraft whose throttle is a collective, the same MFD and keys fly a helicopter-style AFCS.
The A/P works from the hover up to cruise. It flies through the game's helicopter fly-by-wire,
which is rate-command, and it also moves the collective and the pedals.

| Aircraft | Configuration | Modelled on | How the A/P flies it |
|----------|---------------|-------------|----------------------|
| SAH-46 Chicane | Single rotor + tail rotor | AH-64 / UH-60 | Collective holds height. Pitch attitude holds speed. |
| UH-90 Ibis | Coaxial rotor + pusher fans | S-97 Raider / X2 | Like a helicopter below ~25 m/s. Above ~40 m/s the fuselage is held level and the pusher fans hold speed. |
| VL-49 Tarantula | Quad tiltwing | CL-84 / V-22 | Blends with the wing angle. Wing up: collective holds height and pitch holds speed. Wing forward: pitch holds height and power holds speed. |

- **Engaging** works at any speed. The aircraft must be at least `MinEngageRadarAlt` up, banked
  less than 45° and pitched less than 30°. Attitude and heading are held from where you engaged.
- **SPD** (the A/T button and key) is a coupled mode on rotorcraft, not an autothrottle, and needs
  the A/P on. It holds forward *ground* speed.
  - Set it to **0** for **hover hold**: the A/P nulls fore/aft and sideways drift (`HVR HLD`).
  - With SPD off, the A/P holds pitch attitude, like a real attitude hold.
- **Heading.** Below ~12 m/s the pedals hold the heading bug and the wings stay level (or null
  drift in hover hold). Above ~25 m/s it makes coordinated, banked turns (bank ≤ 30°), with the
  pedals keeping the ball centred.
- **Height.** BALT and V/S as on jets, plus RALT (radar-altitude hold, on the TF button), but climb and descent are limited to 8 m/s in helicopter
  mode. Sink is limited to 3 m/s at low speed to stay out of vortex ring state.
- **Near the ground.** The collective is also adjusted for rotor tilt (pitching or banking takes
  lift away). Within about 15 m of the floor, the speed hold stops tilting the rotor further
  while the aircraft is sinking below its commanded climb rate, and the sink rate toward the
  floor is limited to 0.3 m/s per metre of clearance.
- **Rotor RPM protection** (Chicane, Ibis). The game's engine governor gives at most 110% power,
  so a big collective demand can drag the rotor down. Below 97% rotor RPM the A/P stops raising
  the collective and backs it off until the rotor recovers, like a real Nr/power limiter. The
  climb just goes as fast as the power allows. On the Ibis the pusher fans also stop adding thrust,
  because they share the transmission. The MFD shows `ROTOR RPM LIM xx%` while this is active.
  The Tarantula's props govern their own RPM, so it doesn't need this.
- **Ibis Auto Thrust** is switched off while SPD is engaged, because it fights the speed hold. It
  is restored when SPD turns off.
- **Tarantula Auto Wing Tilt** stays in charge of the wing. If you switch it off, the A/P flies
  whatever wing angle you set. DATA shows the wing angle as `NAC`.
- **Disconnects.** Stick or pedals past `StickOverrideThreshold`, moving the collective, landing,
  or auto-hover all disconnect the A/P. When it lets go, the collective lever is left where the A/P
  had it (keyboard/relative throttles), so the aircraft doesn't drop.

Mouse flyers: while the MFD cursor is on, stick-override detection is paused and the virtual
stick is held centred, so clicking around doesn't fight the A/P. Leave the game's menu
weapon-safety option on, or a left click on a button could also fire the gun.

## Why the old version oscillated, and what changed

The old pipeline was telemetry → UDP → Python PID → ViGEm virtual Xbox pad → Rewired → game.
Three problems combined:

1. **Stacked integrators on a rate-command airframe.** Nuclear Option's fly-by-wire
   (`ControlsFilter.FlyByWire`) treats stick deflection as a pitch-rate (g) demand and has its
   own integrator. The script then added `pitch_pid.ki` (angle → stick) and `vs_pid.ki` on top.
   Three integrators in series, plus the airframe, give a phase lag that makes porpoising
   almost inevitable.
2. **Latency and a non-linear stick.** UDP, Python, ViGEm, Windows, and Rewired add delay, and
   Rewired applies its deadzone and response curve to the virtual pad.
3. **Derivative on noisy angles.** D terms differentiated Euler angles instead of using the
   measured angular rates.

The new control laws (`src/FlightController.cs`):

```
vertical: alt err -> climb rate -> flight-path angle (gamma) -> pitch rate -> stick
lateral:  hdg err -> turn rate  -> bank angle                -> roll rate  -> stick
```

- **Only the innermost rate loops integrate.** Every outer loop is proportional, so there is
  nothing to wind up and stack.
- **Rate commands map onto the FBW.** The pitch feedforward reads the jet's actual FBW
  parameters (g limit, corner speed) through Harmony, so stick ≈ demanded pitch rate.
- **Altitude is held via flight-path angle** (from the velocity vector), not pitch attitude.
  AoA changes no longer feed back into the altitude loop.
- **γ-rate damping** (`FlightPathDamping`) stops the low-speed porpoise. At low speed, flight
  path lags pitch by seconds; plain γ feedback rings, and damping on measured dγ/dt fixes it.
- **Coordinated-turn feedforward** adds the pitch rate a level turn needs (g·sinφ·tanφ/V), so
  the jet doesn't sag in turns. This replaces the old `TURN_COMP_GAIN` guess.
- **Commands are slew-limited** and every loop is clamped by g, climb angle, bank, and roll rate.
- **Altitude uses the game's floating-origin-safe `GlobalY()`**, so the old de-jump hack is gone.
- **Inputs are injected in-process** with a Harmony postfix on
  `PilotPlayerState.PlayerAxisControls`. It runs after the game reads your stick and before
  `Aircraft.FilterInputs()`, the same place your own input lands, with zero latency.

These laws were checked in a simple offline airframe model. On it, a 300 m step settled in
about 21 s with no oscillation, including with the stick-to-rate gain off by 2.5x and a
sluggish low-speed airframe. Real airframes differ, so see Tuning.

## Tuning

Every gain is in `BepInEx\config\com.yourname.autopilotmod.cfg`, with a description for each.
Turn on **DATA** (on the **SET** page) to see commanded vs actual bank, flight-path angle, pitch and roll rate,
and the raw stick outputs.

| Symptom | Try |
|---------|-----|
| Altitude bobs, worst when slow | Raise `FlightPathDamping`, or lower `FlightPathGain` |
| Altitude captures feel lazy | Raise `AltitudeCaptureAccel` (round-out) or `MaxClimbRate` (big changes) |
| Final settle onto altitude is slow | Raise `AltitudeGain` |
| Sea skim wanders in height | Raise `FlightPathGain` (with `FlightPathDamping` about 1.25x it) |
| Pull-up warning too early / late | `TerrainWarning.ReactionTime`, `PullUpG` |
| TF rides too high over hills | Raise `TerrainFollowing.PullUpG` / `PushOverG`, or fly slower |
| TF ride too violent | Lower `TerrainFollowing.PullUpG` / `PushOverG` |
| TF keeps making sharp corrections | Raise `TerrainFollowing.Smoothing` (0.4 s; 0 = off) |
| Nose buzzes / twitches | Lower `PitchRateGain` |
| Wings rock in HDG/ATT | Lower `BankGain`, then `RollRateGain` |
| Settles a bit off bank target | Raise `RollRateIntegral` |
| Speed hunts with A/T | Raise `SpeedDamping`, or lower `SpeedGain` |
| A/T lights the afterburner | Lower `MaxThrottle` |

## Files

- `src/AutopilotPlugin.cs`: plugin entry point, aircraft binding, hotkeys, commands.
- `src/FlightController.cs`: state sampling and control laws.
- `src/RotorcraftLaws.cs`: control laws for the helicopters and the tiltwing.
- `src/TerrainFollowing.cs`: the TF mode's look-ahead terrain scan.
- `src/RouteNav.cs`: the NAV waypoint route.
- `src/MfdFrame.cs`: shared MFD bezel, push-buttons and drawing (IMGUI).
- `src/MfdPage.cs`: the A/P MFD page.
- `src/Patches.cs`: Harmony hooks into the pilot input path.
- `src/ApSettings.cs`: config entries.
- `src/TerrainWarning.cs`: the "Terrain, pull up" warning.
- `src/RadarCoveragePage.cs`: the radar-coverage display (F9).
- `src/TerrainHeightGrid.cs`: one-time terrain height map for the radar page.
- `src/WebMirror.cs`: web page mirror of the MFDs for a second computer.
- `src/TargetPanel.cs`: the web page's target filters and target lists (drives the game's own
  target-list panel).
- `src/SoundBank.cs`: loads the optional custom warning sounds.
- `src/Units.cs`, `src/ApCommand.cs`, `src/DisconnectTone.cs`: helpers.
- `src/ApMath.cs`: the control laws' pure arithmetic (no Unity or game types).
- `src/FlightPlanner.cs`, `src/planner.html`: the flight planner (data export, and the editor app).
- `tests/`: self-check of `ApMath.cs` plus small offline models of the helicopter height loop, of
  terrain following over ridges, a cliff and rolling hills, and of NAV flying a route.
  Run it with `dotnet run --project tests`; it needs no game files.
  `node tests/planner-sim.js` checks that the planner's predicted path agrees with the autopilot's NAV maths.
