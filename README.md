# Nuclear Option Autopilot MFD

A BepInEx plugin that adds a fighter-style autopilot to Nuclear Option. It runs entirely
inside the game: the control laws run in the physics tick and you fly it from an A/P page drawn
like an MFD (20 bezel push-buttons, green-phosphor display). No Python, no virtual gamepad,
no ViGEmBus.

```
            [A/P]   [HDG]   [ATT]   [BALT]  [RALT/TF]
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
  standard-rate turn. **ATT** holds a bank angle; below 7° of bank it snaps to wings-level.
- Vertical: **BALT** holds barometric altitude. **RALT** holds radar altitude, for low-level
  and sea-skimming flight. **V/S** holds a vertical speed.
  Altitude and radar-altitude targets can't be set below 1 m.
- **TF** (terrain following) is RALT with a forward look: press **RALT** a second time to
  switch to it, and again to go back. See below.
- **A/T** is independent of the A/P, as in real jets. It holds speed with the throttle.
- **Heavy jets.** On airframes whose fly-by-wire allows little pitch or roll rate (the heavy
  bomber class), the flight-path and bank loops automatically use lower gains and a lower roll
  rate, so they don't outrun the airframe and oscillate.
- Pressing an already-selected mode, or the target readout beside a rocker, re-syncs that
  target to the current value. **SYNC** re-syncs everything. The exception is **RALT**, which
  swaps between RALT and TF and keeps the target; use the readout or SYNC to re-sync it.

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
- **No forward look.** The floor and RALT can't see terrain ahead. Over hills, use TF.

**Terrain following (TF).** Jets only. Press **RALT** twice (the button then reads **TF** and the
MFD shows `TERR FLW`). The vertical rocker sets the clearance to hold, never less than
`MinClearanceMeters` (20 m).
- **What it does.** It holds the set clearance like RALT, but scans the terrain, ships and sea
  along the track it is about to fly, including the turn HDG or ATT is making, across a corridor
  wider than the wingspan. It starts each climb early enough to cross the high ground at the set
  clearance and level, so it doesn't balloon over the crest, then follows the far side back down.
- **Ride.** `PullUpG` (1.0) and `PushOverG` (0.5) in `[TerrainFollowing]` set how hard it pulls
  and pushes. Higher values hug the ground closer. The faster you fly and the softer the ride,
  the more it rides high over short, sharp hills: the jet can't turn its flight path that fast.
- **Limits.** Climbs and descents go up to `MaxFlightPathAngle` (20°), not `MaxClimbRate`. Use the
  A/T: nothing else holds your speed in a long climb. If terrain ahead is too steep to clear at
  that angle, the MFD shows `TF CLIMB LIM` in amber. Take the jet and turn away. The terrain
  warning below still works in TF.
- **How far it looks.** `LookAheadSeconds` (15 s of flight, 1 to 8 km).
- **What it can miss.** The scan is 40 points, refreshed ten times a second, not a solid picture.
  A thin mast between points, or anything that isn't terrain, a building or a ship, isn't seen.
  Aircraft and missiles aren't either.
- **Cost.** About 400 downward raycasts a second, and only while TF is flying the jet.

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
| TF rides too high over hills | Raise `TerrainFollowing.PullUpG` / `PushOverG`, or fly slower |
| TF ride too violent | Lower `TerrainFollowing.PullUpG` / `PushOverG` |
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
- `tests/`: self-check of `ApMath.cs` plus small offline models of the helicopter height loop and
  of terrain following over ridges, a cliff and rolling hills.
  Run it with `dotnet run --project tests`; it needs no game files.
