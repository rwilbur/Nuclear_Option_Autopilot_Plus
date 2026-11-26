# Fly-by-wire Phase 0: how the game does it

What the game's own code does, read from the decompiled `Assembly-CSharp.dll` (ILSpy 11.2, game build of
25 Nov 2026). These are the facts our fly-by-wire is designed around. File names are the decompiled sources.

## 1. The input path, one physics tick

Unity runs, in each `FixedUpdate`:

1. **`JobManager.FixedUpdateEarly`.** UniTask inserts its FixedUpdate runner first (`injectOnFirst: true`),
   so this runs before every `MonoBehaviour.FixedUpdate`.
   1. `ScheduleJobs`: each `ControlSurface.UpdateJobFields()` copies `controlInputs` into its job, then the
      control-surface job and the aero job are scheduled (the aero job reads transforms after the surface job).
   2. `PilotAeroInputs` → `Pilot.Pilot_OnAeroInputsApplied` (pilot g-damage above 20 g) →
      `PilotPlayerState.FixedUpdateState` → `PlayerAxisControls` (stick → `controlInputs`) →
      `Aircraft.FilterInputs()`.
2. **`JobManager.FixedUpdateLate`**: jobs finished, aero forces applied.
3. **`MonoBehaviour.FixedUpdate`**: `Turbojet` / `Turbofan` read `controlInputs` (nozzles, throttle),
   `Airbrake`, `LandingGear`, ...
4. Physics step.

**Consequence: latency.** What `FilterInputs` writes at tick N reaches the surfaces at tick N+1 (the job
already took its copy at N), so the surface-driven rate change is measured at the start of N+2. Nozzles read
the inputs in the same tick, one tick sooner. If our FCS runs **before `ScheduleJobs`** (a prefix on
`JobManager.FixedUpdateEarly`), with the state left by the last physics step, the surfaces get the command in
the same tick. That is one tick less delay in the feedback loop, without changing the pilot's stick-to-surface
latency (the stick read at N-1 is used at N, which is when stock would apply it too). This is the most
valuable single fact for INDI, whose robustness is limited by delay.

## 2. `Aircraft.FilterInputs` and `ControlsFilter.Filter`

```
FilterInputs: raw = (pitch, yaw, roll)
              relaxedStabilityController?.FilterInput(...)   // some jets: canard AoA feed, before the FBW
              controlsFilter.Filter(controlInputs, raw, CockpitRB(), gForce, flightAssist)
```

`ControlsFilter.Filter` (virtual; only `HeloControlsFilter` overrides it):
- does **nothing** below `minSpeed` (25 m/s) or `minAlt` (1 m radar altitude): the stick goes straight to the surfaces;
- runs **auto-hover** (rewrites pitch/roll/throttle/customAxis1) when active;
- runs **aim assist** when enabled and flight assist is on (small pitch/yaw/roll corrections toward a gun solution);
- runs **`FlyByWire.Filter`** when `flyByWire.Enabled`.

`rb` is the cockpit's rigidbody, not the root. The serialized `AutoTrimmer`, `GLimiter`,
`AngularVelocityDamper`, `SpeedRemap`, `ResponseRateLimiter` and `AoALimiter` classes exist but the base
`Filter` never calls them.

**Clean takeover:** setting `flyByWire.Enabled = false` while our FCS owns the jet keeps hover, aim assist and
the low-speed pass-through running. `controlInputs` after `Filter` is then the pilot's shaped command.
Restore the flag when we let go.

### `FlyByWire.Filter` (the stock law)

Axes are body angular velocity in Unity's frame: x = pitch, y = yaw, z = roll. q̄ ratio = V²ρ / (corner²·1.225).

- **Pitch:** target rate = stick × gLimit·9.81 / max(V, 0.75·corner), × clamp(q̄ ratio, 0.3, 1) below corner.
  The AoA limiter (below corner only) scales the target by 1 − strength·min(AoA − limit, 10°). It blends to
  stick × `maxPitchAngularVel` with Stability Assist off below 1.2× corner q̄ (`limitFactorSmoothed`, about
  1 s). Then a PID on rate error (error clamped ±0.25, I clamped ±0.2), ÷ max(q̄ ratio, 1), is integrated into
  `pitchAdjuster` at up to 2/s. Gear down and slow: the integrator is bled and nose-up is limited on the
  nose wheel.
- **Yaw:** damper. output = −yawTightness × remap × (yaw rate − pedal) − yawWeathervaning × sideslip.
- **Roll:** P loop on roll rate with target = stick × maxRollAngularVel × clamp(q̄ ratio / maxRollSpeed, 0.5, 1).
  q̄ ratio / maxRollSpeed is about 0.005, so the clamp always sits at 0.5: a **fixed 0.5 × max roll rate at
  every speed**. A slow roll-trim integrator runs while the pitch rate is under 0.1 rad/s. Above corner the
  output blends back toward the raw stick.
- **Bug:** input smoothing writes the smoothed yaw into `inputs.roll`, then overwrites it, so yaw is never smoothed.
- Stock **opacity** < 1 blends the output back to the raw stick.

## 3. Control surfaces

`ControlSurface` holds a private **reference** `controlInputs = aircraft.GetInputs()`. `UpdateJobFields`
copies it into the job each tick (zeroed while `locked`). The job (`ControlSurfaceJob_Math`, Burst) does, per
surface:

```
for each channel c in {pitch, roll, yaw}:      // independent servos!
    current_c += clamp(input_c * range_c - current_c, ±servoSpeed*dt)
    current_c  = clamp(current_c, ±|range_c|)
angle = current_pitch + current_roll + current_yaw     // about the visible mesh's local x axis
flap surfaces: driven by gear state only (gear down → -pitchRange), not by inputs
split surfaces: split = (throttle == 0 ? maxSplit : 0) + yawSplitFactor*yaw*maxSplit, clamped, servo-limited
```

- **Travel** is Σ|range_c|, and the **slew rate is up to 3× `servoSpeed`** when all three channels move the
  same way. Collapsing a surface onto one channel (what the existing FCS mods do) loses that.
- **Clean per-surface command:** give each surface its own `ControlInputs` object, swapping the private
  reference. The game's own job then drives it, with stock servos, travel, lock, detach and split behaviour
  untouched. Our allocator picks the three channel inputs per surface. Restore the shared reference to hand back.
- Detached surfaces set `IsDetached`, and the job stops moving them.

## 4. Aerodynamics (`AeroJob_Math`, per `AeroPart`)

Every part is a plate. Per tick:
- v = part velocity − wind; wind = level wind + turbulence × max(|wind|, 10) × Perlin noise of position and
  time (deterministic).
- `airflowChanneling` > 0: v is turned toward another transform's forward.
- Local AoA α = atan2(v_lift.y, v_lift.z) in the part's `liftNormal` frame.
- C_L, C_D from 128-point charts over ±π (index = α·20.372 + 64, linear read). Airfoil −1:
  C_L = 1.8 sin 5α, C_D = 1.5(1 − cos 2α) + 0.02.
- Lift = −n̂ · C_L·½ρV²·S·eff with n̂ = normalize(v × liftRight); drag = C_D·½ρV²·S·eff + parasitic
  ½·½ρV²·(dragArea + (dragArea + 0.1·S)(1 − eff)).
- Transonic drag rise between 0.8 and 1.2 Mach (up to +15 %).
- Torque = F × (−R·centerOfLift) about the part's rigidbody; force applied to the part's own rigidbody.
- Air density from a chart by altitude. Underwater/splash terms.
- No Mach effect on C_L, no downwash, no blanking. A deflected surface only changes its own plate.
- **Damage** (`AeroPart.ApplyDamage`): `wingEffectiveness` = lerp(0.5, 1, hitPoints / 100), so a wrecked part still gives half
  its lift. A detached part gets 0 and leaves the aircraft.

So **the effector model can be the game's own equation, exactly**: a surface's moment at any deflection is
this plate evaluated with its lift frame rotated by that deflection. Airfoil charts come from
`AircraftParameters.airfoils` (`AeroPart.airfoil` indexes it).

## 5. Engines and thrust vectoring

`Turbojet` and `Turbofan` (same logic) read their own private `controlInputs` reference in `FixedUpdate`:
- throttle (+ `splitThrustFactor` × yaw: differential throttle as yaw on some jets) → RPM with spool rate →
  thrust, × altitude curve, cut above `maxSpeed`;
- nozzle pitch target = −clamp(pitch × gain.x, ±1) × vectoring.x − clamp(roll × gain.z, ±1) × vectoring.z;
  yaw target = −clamp(yaw, ±1) × vectoring.y (0 below 3 m radar altitude);
- slew 70°/s per axis; all zero above `thrustVectoringMaxAirspeed`; the pitch angle is clamped ±20°.

**Damage and failure** (`Turbojet`, phase 5):
- Hits on the engine's `criticalParts` lower `condition`, which scales the rpm it can reach above idle by (0.5 + 0.5 × condition).
- Once `condition` reaches 0, the part detaches or the engine goes under water, the engine is inoperable. With no ignition, too
  thin air or no fuel it makes no thrust either. In all those cases `thrustRatio` is 0 at once, not spooled down.
- The afterburners (`JetNozzle.Thrust`) run on the engine's own `controlInputs.throttle` once rpm is over 85%, so a throttle cut
  takes the afterburner thrust off at once and the core thrust as the rpm spools.

**Clean per-engine command:** the same reference swap as the surfaces. A per-engine `ControlInputs` gives
that engine's nozzle angles and throttle (the engine-out throttle cut) without a transpiler. `DuctedFan`,
`PropFan`, `ConstantSpeedProp`, `TurbineEngine` and `RotorShaft` are other `IEngine`s, to read when a jet that
uses them is ported.

### Other thrust and torque sources (found by the phase-2 force probe)

- **`DuctedThrustSystem`** drives `JetNozzle`s from an engine's power: the **VT-7 Vagrant's** thrust comes
  through it, and its `Turbofan` has no nozzles of its own (its `thrustVectoring` reads 0). Its nozzles
  **swivel, Harrier-style**:
  - the angle (0 to `swivelLimit`, 120°) follows `customAxis1`, the custom-axis lever;
  - each nozzle adds `yawSwivel` × yaw input for yaw;
  - each slews at `swivelRate`.

  This is VTOL vectoring, not pitch-control TVC.
- **`ReactionControlSystem`** (+ `Thruster`): reaction-control jets, **on the Vagrant only** of the jets
  logged (the Compass has none; an earlier note of it on the Compass was a misread log). Each thruster's force is thrustMin to thrustMax, set by
  pitch·f.x + yaw·f.y + roll·f.z of the inputs, scaled by the turbofan's thrust ratio. **Only below
  `maxSpeed` (100 m/s)**, for hover and slow flight.
- **`MagicTorqueController`**: adds torque straight from the stick (pitch/yaw/roll × gain) to its acting
  parts, plus sideslip damping, with power draw. Which jets carry it is not known yet.

All three are control effectors: the FBW has to model them (and drive them per channel) on the jets that
have them.

## 6. Other inputs the FCS touches

- `Airbrake`: opens while throttle is exactly 0 (the stock idle airbrake).
- `HighLiftDevice` (slats): scheduled on speed by the game itself.
- `LandingGear`: nose-wheel steering = yaw × `steeringLock`; differential brake from yaw × 0.3 ×
  `differentialBrakeFactor`.

## 7. Multiplayer

- The owner's client is the only one that simulates its aircraft: `LocalSim` = `Player.IsLocalPlayer` for
  players. Remote copies run `SetSimplePhysics` (parts merged, no aero jobs of their own).
- `AircraftNetworkTransform` sends position, rotation and velocity each snapshot, and `ControlInputs` (pitch,
  roll, yaw, throttle, brake, customAxis1, compressed) every 4th snapshot. These are the values **after**
  `FilterInputs`, since the FBW writes into the same object. Remote copies apply them, and their surfaces and
  nozzles animate through stock mixing.
- So the flying is never decided by remote clients. What other players see of our surfaces is stock mixing of
  the synced inputs.
  - **Per-surface mode:** they see an approximation.
  - **Shared-input ("multiplayer") mode:** the FCS writes only the shared `ControlInputs`, and every surface
    and nozzle moves by stock mixing. They see exactly what is flying, as with the stock FBW.
- **Who writes the shared inputs, and when** (for the shared-input mode):
  - `PlayerAxisControls` sets pitch, roll and yaw outright every tick, before `FilterInputs`. It never reads them back.
    The exceptions are a detached cockpit, where it writes nothing, and a pilot too weak (`pilotStrength` < 0.2), where it
    writes zeros.
  - The throttle lever (`PlayerThrottleAxis1Controls`) writes the throttle outright from its own lever state, in Update
    (and in FixedUpdate during auto-hover). A tick with no Update between leaves it untouched.

## 8. Data to collect per jet

`[FlyByWire] ExportAirframes = true` writes `BepInEx\FbwAirframes\<aircraft>.json` 2 s after you start flying
each jet. It contains:
- every rigidbody: mass, CG, inertia and its rotation, in the root frame;
- every `AeroPart`, `ControlSurface`, engine and `RelaxedStabilityController`, with all fields read by
  reflection and the poses of the transforms they reference (lift normal, visible mesh, vectoring transforms,
  nozzles);
- the `ControlsFilter` / `FlyByWire` settings, `AircraftParameters`, and the airfoil charts sampled on the
  game's grid.

Under complex physics every part is unparented and gets its own rigidbody (`AeroPart.CreateRB`), so the
exporter walks `Aircraft.GetAllParts()`, not the root's children. Jet engines (`Turbojet`, `Turbofan`) are not
in `Aircraft.engines` (only the propeller types register there), so engines are found by type.

### What the exports show (25 Nov 2026)

- **Physics runs at 60 Hz** (`fixedDeltaTime` = 1/60 s) on every jet.
- Every part is its own rigidbody: 28–38 per jet. Live mass (stores and fuel included) is what physics uses;
  `definition.mass` is a different, smaller prefab figure.
- **No jet has a `RelaxedStabilityController`**, and **none uses `splitThrustFactor`**.
- **On every surface of every jet, the lift normal sits under the visible mesh**, so it turns with the
  deflection and the game's plate equation models every surface exactly. The only exceptions are the Alkyon's
  wingtips, which are split-only airbrakes with zero deflection range.

| Jet | Stock FBW (g / corner m/s / AoA lim °) | Surfaces: pitch/roll/yaw ranges °, servo °/s | Split surfaces | TVC (pitch, yaw, roll °; max 250 m/s) | Other |
|---|---|---|---|---|---|
| A-19 Brawler | 7.5 / 160 / 14 | elevators −35/±10/0 @30, rudders 0/0/±12 @60, ailerons 0/±35/0 @60 | ailerons 45° | — (2 turboprops) | |
| Alkyon AB-4 | 8 / 200 / 15 | ailerons 0/±30/0 @60, elevators 20/±5/0 @90, rudders 0/0/−15 @60 | wingtips 60° (split only) | — (4 turbofans) | |
| EW-25 Medusa | 6 / 130 / 10 | elevators −25/0/0 @25, rudders 0/0/∓25 @25, ailerons 0/±28/0 @70 | — | — | 2 parts with airflow channeling |
| FS-12 Revoker | 9 / 160 / 27 | canards 35/±5/0, ailerons 0/±30/0, rudder 0/0/−25, all @70 | — | 15, 15, 0 (pitch gain 2) | |
| FS-20 Vortex | 9 / 155 / 25 | elevons −40/±25/0 @70 (×4), canted rudders −40/0/±20 @40 | — | 10, 0, 0 | every surface also pitches |
| KR-67 Ifrit | 9 / 160 / 27 | tails −35/0/±10 @60 (ruddervators), ailerons 0/±30/0 @45, flaps 0/±20/0 @70 | flaps 25° | 15, 0, ±15 per engine (differential: roll) | |
| SFB-81 Darkreach | 4 / 100 / 10 | ailerons 0/±30/0 @50, V-tail −25/0/±10 @50 | ailerons 30° | — | 2 parts with airflow channeling |
| T/A-30 Compass | 8 / 130 / 10 | rudders 0/0/−25 @40, elevators −30/±10/0 @70, ailerons 0/±30/0 @70 | — | — | 4 parts with airflow channeling |
| VT-7 Vagrant | 8 / 160 / 15 | rudder 0/0/25 @50, elevators −25/0/0 @70, ailerons 0/±30/0 @70 | — | — | |
| CI-22 Cricket | 6 / 110 / 10 | ailerons 0/±20/±2 @70 (some yaw too), rudders 0/0/±20 @60, elevator −25/0/0 @70 | — | — (turboprop) | |

What this means for the design:
- **Channel mixing is rich.** Every Vortex surface pitches, the Ifrit and Darkreach tails are ruddervators,
  and most stabilators also roll. Per-surface allocation over all three channels is where Full mode gains
  over stock.
- **Real TVC on three jets**, including **differential (roll) TVC on the Ifrit**: its engines use opposite
  signs on the roll channel. All three cut vectoring above 250 m/s.
- **Split surfaces on four jets.** The Alkyon's wingtips are split-only drag rudders. Each split opens with
  yaw and at idle throttle.
- **60 Hz** sets the tick budget (16.7 ms per tick for everything) and each tick of delay (16.7 ms).
