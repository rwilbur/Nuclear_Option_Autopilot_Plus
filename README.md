# Nuclear Option Autopilot MFD

A BepInEx plugin that adds a fighter-style autopilot to Nuclear Option. It runs entirely
inside the game: the control laws run in the physics tick and you fly it from an A/P page drawn
like an MFD (20 bezel push-buttons, green-phosphor display). No Python, no virtual gamepad,
no ViGEmBus.

```
            [A/P]   [HDG]   [ATT]   [BALT]  [RALT]
  [STEP] +--------------------------------------+ [V/S]
  [ ^ ]  |  heading tape + bug                  | [ ^ ]
  [HDG]  |  HDG SEL | ALT HLD | SPD             | [ALT]
  [ v ]  |  attitude indicator (+ FD bank cue)  | [ v ]
  [DATA] |  ALT  SPD  V/S  THR                  | [SYNC]
         +--------------------------------------+
            [A/T]   [ v ]   [SPD]   [ ^ ]   [HIDE]
```

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

The page is laid out to fit without scrolling. Next to the MFDs:

- **EXPAND MAP** (top right) fills the window with the radar MFD; **RESTORE** or Esc goes back.
  While expanded, the hidden A/P MFD isn't rendered.
- **Target filters**: the game's own target-list filters (follow HUD, lased only, faction, unit
  type, vehicle type). Tap to toggle; right-click or long-press to show only that one.
  **RESET FILTERS** and **CLEAR SELECTION** do what the in-game buttons do.
- **Selected** lists your current targets; tap one to drop it.
- **Available** lists the enemies the map shows (after the filters), nearest first. Tap to add or
  remove a target.

Mirror settings in `[WebMirror]`: `Resolution` (A/P MFD, default 1024), `MapResolution` (radar
MFD, default 1440; both capped at the game window's height), `FramesPerSecond` (default 30, up
to 60) and `JpegQuality`. Frames are streamed (MJPEG) and encoded off the main thread.

## Flying it

| Key (default)   | Config name (`[Keys]`) | What it does |
|-----------------|------------------------|--------------|
| **F5**          | `AutopilotEngage`      | A/P engage / disconnect. Engaging holds what you have: current heading and altitude become the targets. |
| **F8**          | `AutothrottleEngage`   | Autothrottle engage / disconnect (holds current speed). |
| **F6**          | `ShowHideMfd`          | Show / hide the MFD. |
| **F7**          | `MfdCursor`            | MFD cursor on/off, so you can click the push-buttons. Drag the bezel to move an MFD; drag the grip in its bottom-right corner to resize it. |
| **F9**          | `RadarCoveragePage`    | Radar coverage MFD over the game map: off → **ALL** friendly radars → **OWN** radar only → **ENEMY** radars at known positions → off. Green = the game would detect a target at the **TGT** height above ground there (**TGT ALT** = at your own current altitude); red = strong enough signal but hidden by terrain, the earth's curvature (radar horizon) or ground clutter; no colour = signal too weak. Each view has its own target RCS (ALL/OWN default 1, ENEMY defaults to your jet's current RCS), set with the **RCS** arrows (middle button = your own jet's RCS) or typed into the web mirror. With the F7 cursor: scroll to zoom, drag to pan, **MAP** shows the whole map, **FOLW** re-centres on your jet. Enemy units the game map shows (with its target filters applied) are drawn with their map icons and heading; click one to select it as a target, click again to deselect. |
| **PgUp / PgDn** | `VerticalTargetUp` / `VerticalTargetDown` | Vertical target up/down (altitude, radar altitude, or V/S, depending on mode). |
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
  standard-rate turn. **ATT** holds a bank angle; below 7° of bank it snaps to wings-level.
- Vertical: **BALT** holds barometric altitude. **RALT** holds radar altitude, for low-level
  and sea-skimming flight. **V/S** holds a vertical speed.
  Altitude and radar-altitude targets can't be set below 1 m.
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
- **What RALT holds.** RALT holds the same reference the HUD's `R` shows (the bottom of the
  gear).
- **Floor.** In every vertical mode, the AP keeps the *lowest point of the airframe*
  (wingtips, nose, tail, gear) at least `GroundFloorMeters` (1.5 m) above the surface.
- **Bank limit.** Near the water, bank is limited so the low wingtip can't dip in. The MFD shows
  `BANK LIM - LOW` while this is active, so heading changes are slow down there.
- **Pull-up-limited sink.** Further from the floor, the sink rate is limited to what a
  `FloorPullUpG` (1 g) pull-up can still stop at the floor. Close in, the old linear limit
  (`FloorGain`) applies.
- **Terrain look-ahead** (`TerrainLookahead`, on by default). Every physics tick, 12 rays sample
  terrain, ship decks and the sea along the predicted ground track, including the current turn,
  out to `LookaheadSeconds` (15 s). The A/P starts climbing early enough to clear what it finds.
  It plans with a 1 s reaction, a `FloorPullUpG` pull, and no steeper than `MaxFlightPathAngle`.
  The MFD shows `TERRAIN AHEAD - CLIMB` while the look-ahead is commanding the climb. It works in
  every vertical mode, so BALT over mountains climbs too. The terrain warning below is still
  there as a backstop.

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

**Autothrottle energy coupling** (`EnergyCoupling`). With the A/P and the A/T both engaged, the
A/T adds throttle as soon as the A/P commands a climb or a banked turn, instead of waiting for
the speed to bleed off. The amount is set by `ClimbFeedforward` and `TurnFeedforward`. This is
the energy idea behind TECS, used by PX4 and ArduPilot. **Speed protection** (`SpeedProtection`):
if the A/T is already at `MaxThrottle` and the speed still falls well below target, the A/P climbs
only what the engines can sustain. If needed it descends to regain speed (`SPD PROT` on the MFD).
The terrain floor always wins.

**Rate loops: INDI and in-flight learning.**
- **INDI law** (`Tuning.RateLoops`, `Law = Indi`, the default). The innermost pitch and roll
  loops use incremental nonlinear dynamic inversion. Each tick the A/P moves the stick by the
  increment that should produce the missing rate acceleration, using the measured response.
  Trim, store loads and fly-by-wire model errors need no integrator.
- **Rollback.** `Law = Classic` restores the previous feedforward + PI law.
- **Learning the airframe** (`AdaptiveEffectiveness`). For both laws, a recursive-least-squares
  estimator learns how much pitch and roll rate the stick really gives on the jet you are flying.
  It learns from your own stick inputs as well as the A/P's.
- **Check the estimate.** DATA shows the law and the learned ratios, e.g. `INDI x1.08/0.93`
  (1 = the fly-by-wire model was exact).

The A/P refuses to engage when the jet is on the ground, below 50 m/s, banked more than 60°, or
pitched more than 45°. Rotorcraft have their own limits; see below.

## Routes and NAV mode (radar page)

The radar MFD's bottom row plans and flies routes:

| Button | What it does |
|--------|--------------|
| **WPT** | Toggle waypoint entry: while lit, a click on the map adds a waypoint (dragging still pans). The last waypoint is the destination. |
| **UNDO** | Remove the last waypoint (or a planned route). |
| **CLR** | Clear the route. |
| **RTE** | Plan a radar-avoiding route from your position to the destination. |
| **NAV** | Fly the route with the A/P (lateral mode NAV). HDG or a heading rocker leaves it. |

- **Hand-placed waypoints** are flown leg by leg. Your vertical mode is left alone.
- **How RTE plans.** RTE uses the enemy radars your side knows about, at their known positions.
  It plans for your own RCS (or the `RadarEnemyRcs` you set). It searches map cells × height
  bands (`RouteHeightBands`, default 30/150/500 m above ground) with Lazy Theta*, an any-angle
  grid search. Being seen costs `RouteExposureWeight` seconds per second, and flying low costs a
  little (`RouteLowFlightPenalty`). So the route goes around radars, or drops into terrain
  masking where that is shorter, and stays higher where it can.
- **Planned legs** carry their height. NAV sets RALT and follows it leg by leg (`RALT RTE` on the
  MFD). It descends early for a lower leg and climbs after the waypoint. Touching the vertical
  rockers or a vertical mode takes the height back.
- **Re-planning.** A planned route is re-planned from your position when the known enemy radar
  picture changes: a new or lost site, a moved one, or a jam state (`AutoReplan`, at most every
  10 s).
- **Flying the route.** NAV uses L1 path following, sized so the jet's turn radius at `MaxBank`
  can't make it circle. It turns onto the next leg a turn-radius before each waypoint, so turns
  roll out on the new leg. The A/P MFD shows the waypoint number and the distance to go.
- The planner runs on a worker thread. The radar page keeps mapping terrain and sweeping enemy
  radars in the background while a planned route is active, even when the page is hidden.

**TGT MASK** (the TGT button cycles to it) draws the **masking height**: the lowest height above
the ground that the radars can see at each spot.
- **In ENMY:** fly below it to stay hidden. Red is seen even at 5 m; green is hidden up to 2 km.
- **In ALL/OWN:** an intruder below it slips under your radars.
- **It errs low.** Clutter and the cone's elevation limits are ignored, so the map is
  conservative for hiding.
- **How it's computed.** Each radar sweep is reduced once to a masking height per range bin.
  The normal views are now one comparison per bin, and the self-check confirms the result is
  identical to the old line-of-sight sweep.

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
- **Height.** BALT/RALT/V-S as on jets, but climb and descent are limited to 8 m/s in helicopter
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
Turn on **DATA** (L5) to see commanded vs actual bank, flight-path angle, pitch and roll rate,
and the raw stick outputs.

| Symptom | Try |
|---------|-----|
| Altitude bobs, worst when slow | Raise `FlightPathDamping`, or lower `FlightPathGain` |
| Altitude captures feel lazy | Raise `AltitudeCaptureAccel` (round-out) or `MaxClimbRate` (big changes) |
| Final settle onto altitude is slow | Raise `AltitudeGain` |
| Sea skim wanders in height | Raise `FlightPathGain` (with `FlightPathDamping` about 1.25x it) |
| Pull-up warning too early / late | `TerrainWarning.ReactionTime`, `PullUpG` |
| Nose buzzes / twitches | Lower `PitchRateGain` |
| Wings rock in HDG/ATT | Lower `BankGain`, then `RollRateGain` |
| Settles a bit off bank target | Raise `RollRateIntegral` |
| Speed hunts with A/T | Raise `SpeedDamping`, or lower `SpeedGain` |
| A/T lights the afterburner | Lower `MaxThrottle` |
| Nose or wings buzz on the INDI law | Lower `IndiIncrementGain` or `IndiPitchBandwidth` / `IndiRollBandwidth`, or set `Law = Classic` |
| Rate loops sluggish on INDI | Raise `IndiPitchBandwidth` / `IndiRollBandwidth` |
| Speed sags in climbs or turns with A/P + A/T | Raise `ClimbFeedforward` / `TurnFeedforward` |
| Speed rises in climbs with A/P + A/T | Lower `ClimbFeedforward` |
| Look-ahead climbs too early / too late | Raise / lower `FloorPullUpG` (keep it below what the jet can pull) |
| NAV wanders across the track / turns too hard | Raise / lower `L1Period` |
| Routes detour too far / go too close | Lower / raise `RouteExposureWeight` |

## Files

- `src/AutopilotPlugin.cs`: plugin entry point, aircraft binding, hotkeys, commands.
- `src/FlightController.cs`: state sampling and control laws.
- `src/RotorcraftLaws.cs`: control laws for the helicopters and the tiltwing.
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
- `src/RateLoop.cs`: in-flight effectiveness estimator (RLS) and the INDI rate loop.
- `src/NavGuidance.cs`: the route model and L1 path following.
- `src/RoutePlanner.cs`: the Lazy Theta* radar-avoiding route planner.
- `src/CoverageMath.cs`: masking heights and detection range for the radar page.
- `tests/`: self-check of the pure code (no game files needed). Run it with
  `dotnet run --project tests`. It includes offline models of:
  - the helicopter height loop;
  - INDI + RLS on a rate-command airframe, over response times of 0.1-0.6 s, delays of 0-60 ms
    and effectiveness errors of 0.5-2x;
  - a terrain look-ahead ridge crossing;
  - L1 intercepts and reversals;
  - route planning around a radar and through a masking valley;
  - masking heights against the old line-of-sight sweep.
