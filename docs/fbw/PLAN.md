# Fly-by-wire plan

Goal: a fly-by-wire that gets the most out of every fixed-wing jet, built as clean layers, not as a pile of
fixes. Game facts behind it: [PHASE0_FACTS.md](PHASE0_FACTS.md).

## Rules

1. **Controls only.** The FBW may write what a pilot or the stock FBW already moves:
   - control-surface channels;
   - nozzle vectoring;
   - throttle (including the engine-out yaw throttle cut);
   - airbrake / split surfaces;
   - nose-wheel steering and brakes.

   It never changes aero, forces, mass, fuel, fuel burn, weapons or countermeasures. No host permission is
   needed, because nothing about the jet itself changes.
2. **Player aircraft only.** AI jets stay on the stock FBW.
3. **Ideas, not code,** from other FCS mods. Their manuals are used for game facts and pitfalls.
4. **One code path for every jet.** Airframe differences come from the live airframe, plus an optional
   per-jet data file. Never a forked controller.
5. **Only the game adapter touches Unity.** Everything else is plain C# that the offline bench can run.
6. **No special-case devices.** A symptom gets fixed in the model or the law that causes it.

## Modes

| Mode | Surfaces and nozzles | What other players see |
|---|---|---|
| **Full** (single player, or by choice) | Each surface and engine gets its own `ControlInputs`, picked by the allocator | Stock mixing of our shared inputs: an approximation |
| **Multiplayer** (toggle) | The FCS writes only the shared pitch / roll / yaw / throttle, and the game's stock mixing moves everything | Exactly what is flying, the same as the stock FBW |

Both modes use the same laws, INDI and envelope. Only the last layer differs:
- **Full:** allocate across all surfaces.
- **Multiplayer:** solve for the 4 shared inputs through the stock mixing matrix, built from the same surface
  models. It is still stall- and saturation-aware.

Stability Assist doesn't need turning off for this: remote copies never run any FBW, they only replay the
synced inputs.

## Layers

```
 pilot (stick, pedals, throttle) ─┐        A/P (rate / g targets) ─┐
                                  ▼                                ▼
 ┌────────────────────────────────────────────────────────────────────────┐
 │ Laws: g / pitch-rate blend; velocity-vector roll; sideslip loop; modes │
 │ (normal, Stability Assist off, gear down, ground)                      │
 └──────────────┬─────────────────────────────────────────────────────────┘
                ▼ rate / g commands
 ┌────────────────────────────────────────────────────────────────────────┐
 │ Envelope: command governor on the closed-loop model (AoA, g, sideslip,  │◀── capacity
 │ rates); limits from capacity                                           │
 └──────────────┬─────────────────────────────────────────────────────────┘
                ▼ body-rate references
 ┌────────────────────────────────────────────────────────────────────────┐
 │ INDI (3 axes): M_req = M_now + I·(ν − ω̇) with delay compensation         │
 └──────────────┬─────────────────────────────────────────────────────────┘
                ▼ moment increment
 ┌────────────────────────────────────────────────────────────────────────┐
 │ Allocator: box-constrained weighted least squares on the incremental   │──▶ capacity
 │ effectiveness; boxes from travel and servo rate; null-space goals      │    (reachable moments)
 └──────────────┬─────────────────────────────────────────────────────────┘
                ▼ per-surface / per-engine channel inputs
 ┌────────────────────────────────────────────────────────────────────────┐
 │ Effector models: the game's own plate equation per surface at local    │
 │ flow; TVC from live thrust; mass / CG / inertia from live rigidbodies  │
 └──────────────┬─────────────────────────────────────────────────────────┘
                ▼
 Game adapter: runs before JobManager.ScheduleJobs (one tick less loop delay); swaps ControlInputs references;
 disables stock flyByWire.Enabled while engaged and restores it after
```

Choices, and the evidence for them:
- **Effector models = the game's equation.** `AeroJob_Math` is a plain plate model with 128-point airfoil
  charts, so the moment of any deflection can be computed exactly, not fitted. Effectiveness per surface comes
  from finite differences of that equation at the current flow.
- **INDI with almost no filtering.** The game's state is exact; there is no sensor noise. A filter mostly adds
  delay, and delay is INDI's main weakness [Huang 2022, Pollack & van Kampen 2023]. Any filter must be the
  same on measurement and command [Smeur 2016]. The remaining tick is covered by the model: we know this tick's
  commanded moment exactly, so we can predict the acceleration it will cause. This is the hybrid-INDI idea,
  and EINDI's use of modelled deflections to estimate acceleration. Phase 2 checks whether jointed parts
  shake enough to need a filter after all.
- **Allocator: incremental and linear, solved exactly.** INDI makes allocation a linear problem each tick
  [INCA papers; MDPI 2024]. A box-constrained WLS active-set solver (Härkegård) suits 10–20 effectors.
  - Boxes come from what each servo can reach this tick, which is the D-INCA idea [Stam & de Visser 2025]
    and also catches the 3× multi-channel slew.
  - Spare freedom goes to secondary goals: least deflection and drag, nozzles near trim, stalled surfaces
    eased back.
  - Allocation is weighted in the stability axes so roll is scored with its yaw at the real AoA. This is
    the LCDP / Cnβ,dyn idea from the departure literature.
- **Capacity = incremental attainable moment set.** These are the moments reachable this tick from the
  allocator's own boxes and effectiveness, with a margin [Özkol 2024]. The laws and the envelope read it.
  Damage, stores and engine-out become less capacity, with no special-case logic.
- **Envelope = command governor**, as Simon, Härkegård & Löfberg (JGCD) did on Saab's Gripen development
  simulator. They got AoA overshoot from 1.2° to 0.2° and g overshoot from 0.6 to 0.1 g in bleed-off turns.
  What we take from them:
  - **Prediction model:** the desired closed-loop response, a 2nd-order model with this tick's ζ and ω₀,
    stepped with Euler at the controller rate:
    `y(k+1) = a·y(k) + b·y(k−1) + c·r̄(k−1)`, with `a = 2(1 − Ts·ζω₀)`, `b = 2ζω₀Ts − 1 − ω₀²Ts²`,
    `c = ω₀²Ts²`.
  - **Command shape:** `r̄(k+i) = γⁱ·μ + ν`. Only two unknowns, so a nested bisection solves it in
    microseconds, with no QP solver.
  - **Cost:** `β₁μ² + β₂(ν − r)² + Σ slack²`. Their weights were β₁ = 0.01, β₂ = 0.1, with slack weight 1;
    weights at or above the slack weight switch the governor off.
  - **Constraints:** soft (slack), on the predicted AoA and g, over a horizon of about 0.67 s (40 steps at
    60 Hz; longer helped nothing).
  - **Rejected, and why:** static and dynamic reference governors can't lower the command when a
    disturbance arrives. Output-admissible sets don't work when the dynamics change across the envelope.
  - **Their weak point is ours to fix.** The governor reacted only about 0.1 s before an overshoot, because
    the real closed loop didn't match its model (an LQ+PI design tuned at 25 envelope points). A
    constant-offset correction barely helped. INDI makes the real closed loop follow the reference model
    across the whole envelope; that is its point. So the same governor should predict better here.
    Phase 4 logs predicted against actual AoA and g to check this.
  - **Our additions:**
    - limits come from capacity (attainable moments), not fixed numbers;
    - sideslip and roll rate are added as constraints;
    - the g limit also covers the game's pilot damage above 20 g;
    - AoA and g are predicted together, linked by the live lift slope from our surface models (their
      `K_α/nz`).
- **Engine-out yaw.** When yaw capacity can't hold the asymmetric thrust, the live engine's throttle is cut
  through its own `ControlInputs`.
- **Judging it.** Bench step responses are scored against handling-qualities criteria: pitch bandwidth and
  phase delay, Gibson dropback, roll mode time constant (MIL-STD-1797, AGARD AR-279). Scores, not feel, decide
  a regression.

## Performance

- **Hot path:** per tick, about 20 surface evaluations plus one finite difference each, and one small
  active-set solve.
- **Allocation-free:** no garbage per tick; structs of arrays; the Unity read and write done once.
- **Timed per phase in game,** using the existing `Perf` system.
- **Native core, behind an interface from day one,** with the C# as the reference and the fallback, checked
  against each other by tests. Built when the in-game timings show it's worth it.
  - Candidate solver: DAQP, a library-free C dual active-set QP solver for embedded use (check its licence
    first).
  - Plate kernels in C as well.
- **Threading** of effector evaluation, under the same rule: only pure data off the main thread, and the
  same results with it on or off.

## Phases

| # | What | Done when |
|---|---|---|
| 0 | Game facts, airframe exporter | ✅ Facts doc written; all 10 jets exported to `tests/airframes/` (25 Nov 2026) |
| 1 | Adapter + logging + passthrough | ✅ Revoker and Ifrit flights (25 Nov 2026): every surface matched the game's servo model to 0.0001° (`tests/fbw-log-check.js`) |
| 2 | Effector models checked in game | ✅ See "Phase 2 findings" below (Compass, Revoker, Ifrit, Medusa, Vagrant flights, 25 Nov 2026) |
| 3 | INDI + allocator, plain rate laws, Full mode | ✅ Bench passes on the Revoker, Compass, Vortex and Ifrit; flown on the Compass, Vortex, Ifrit and Revoker (25 Nov 2026), Ifrit retest "much better" after splits became yaw effectors |
| 4 | Laws + envelope governor | Bench ✅ (25 Nov 2026): full-stick pulls (200 and 110 m/s), rolling pulls, releases and pushes stay inside AoA and g on four jets, on and off model (see "Phase 4 findings"). In-game check pending |
| 5 | Capacity limits: damage, stores, engine-out (throttle cut), ground steering | Bench damage cases fly without departure |
| 6 | Multiplayer mode | Remote view matches (by construction); same laws pass the HQ checks |
| 7 | Performance: timings, then threading / native if justified | Per-tick cost measured in game, with and without |
| 8 | Autopilot integration: the A/P commands rates and g directly | A/P tests pass; stock FBW copies removed from the A/P |

### Phase 2 findings

- **The plate model is the game's.** Our plate, evaluated on each part's own job inputs, matches the aero
  job's force to within 0.001% and the total aero moment to within 0.01% on the Revoker, Ifrit and Vagrant.
  On the Compass and Medusa it is within 0.1% for 95% of ticks, with rare spikes up to 17%; those are the
  jets with flow-channeling parts, so our `RotateTowards` probably differs in some edge case (to fix).
- **The jet is not a perfect rigid body, and INDI doesn't need it to be.**
  - With every scripted force captured (ForceProbe) and every body counted (inventory: all dynamic, no
    bodies missed), yaw is predicted almost exactly (R² 0.97–0.999).
  - But the absolute pitch moment can be off by a load-dependent amount: on the Compass, about 8°/s² per g
    nose-up that never happens. Neither scripts nor masses explain it. It is most likely the joints:
    stretched joints apply their equal and opposite forces at different points and leave a net torque.
  - INDI only uses the change in acceleration a change in deflection causes, and in increments the
    prediction holds: tick-to-tick gain 0.88–1.07 on every jet and axis.
  - Roll comes out about 10% lower than the rigid prediction, which the online effectiveness estimate absorbs.
- **The measured angular acceleration carries joint vibration** on the multi-part jets (the stiff Revoker
  and Ifrit less so). INDI's filtering of measurement and command is needed, kept the same on both.
- **More effectors than surfaces and nozzles:** `DuctedThrustSystem` (the Vagrant's nozzles),
  `ReactionControlSystem` thrusters, and possibly `MagicTorqueController` (docs/fbw/PHASE0_FACTS.md
  section 5).

### Phase 4 findings (bench, 25 Nov 2026)

- **AoA limits come from the airframe's own lift curve,** swept with the game's plates on the clean airframe (surfaces
  neutral, the CG's air on every part) every half second. Lift, not body-normal force: at high AoA drag adds a large
  part along body up, so the normal force keeps rising long after the stall. On the fighters the lift curve is a long
  plateau (Ifrit: lift slope 10 g/rad at 3 deg, about 1 g/rad from 20 deg, nominal peak at 42 deg). So the limit is the
  lowest AoA giving 97% of peak lift: Revoker 33, Vortex 31, Ifrit 32, Compass 16, Vagrant 15, Alkyon 15 deg (stock
  limiters 27, 25, 27, 10, 15, 15).
- **The governor's model must include the effectors' capacity.** At low speed the nose takes about a second to start
  and as long to stop. An ideal-loop model let the Ifrit run 16 deg past its limit at 110 m/s. The prediction now limits
  each axis's acceleration to what the effectors reach at full travel, changing no faster than their servos allow, with
  the same hedge the law applies. With it, 0.5 s AoA predictions are within 1–2 deg in pulls.
- **Static pitch stiffness** (from the same plate sweep) shifts that capacity as AoA grows; on the unstable Revoker it
  removes the nose-down authority past some AoA. Horizon: 1.5 s (the Gripen's 0.67 s was too short to see a slow stop).
- **Rolling pulls: inertia coupling.** Rolling about the flight path pitches the nose up by
  ½ k pₛ² sin 2α, k = (I_yaw − I_roll)/I_pitch (NASA TP-1689), which no surface removes. A capped roll rate was rejected.
  Instead the governor shapes the pitch and roll commands together (Ye et al. 2015: a predictive governor on
  stability-axis rolls beats a static limiter), with the coupling in its prediction. The trade between roll and pull
  comes out of the cost each tick.
- **Allocation moved to stability axes,** so "keep the sideslip, give up roll" holds at every AoA. Rolling-pull
  sideslip on the Ifrit went from 35 to about 6 deg.
- **Tried and taken out: a lateral (sideslip) prediction.** It used plate-derived Lp, Np, Lr, Nr, Lβ, Nβ, Yβ (all with
  sensible signs on all ten jets) and a sideslip limit, aiming to replace the 180 deg/s roll cap. Its sideslip
  predictions ran 5–8 deg off at the stock 286 deg/s Ifrit roll. Acting on them, the governor swung its commands
  and upset the Vortex and Revoker. Move suppression steadied it but cost pushes their g. So the roll cap stays for now.
  The next attempt needs the lateral model checked against flight logs first.
- **First flights (Ifrit, Revoker, 25 Nov 2026).**
  - The envelope held in pulls: the Ifrit was past its AoA limit for 0.55 s in 5 minutes, by at most 0.8 deg. In flight the
    0.5 s AoA prediction was 1.5–2 deg rms, as on the bench.
  - Two excursions came with full pedal held against or into a roll: the Revoker at −4.5 g, the Ifrit at 10 g. A roll with
    sideslip trades AoA for sideslip at pₛ tan β (33 deg/s on the Revoker), which the model lacked. It was added, with
    sideslip held at its measured value.
  - The coordinate search over the paper's two pitch unknowns (ν and the decaying μ) did not settle: the pitch command
    jumped between ticks, and the roll was sometimes reversed or raised past the pilot's. Now there is one pitch unknown,
    and the roll can only be taken toward zero, never reversed or added to.
  - Still open: sideslip. A full roll at 89 m/s and 25 deg AoA builds 9 deg with no pedal; on the bench, a Vortex rolling
    pull at 150 m/s builds 23 deg. The governor holds AoA and g only, and the 180 deg/s roll cap stays. The logged rolls are
    the data for validating a lateral model before it is tried again.
- Known: with a deliberate extra tick of delay, the Revoker at 110 m/s still overshoots its AoA limit by 4.7 deg (under
  its lift peak). The game's timing has no such tick (phase 1). Solve cost: up to ~200 simulations of 90 ticks, only on
  ticks where the pilot's command would break a limit. To be timed in phase 7.

Bench: a rigid-body simulator built on the effector models and each jet's export. It adds deliberate model
error, delay, servo lag and part flex, because a plant that matches the model exactly makes everything look
perfect. It lives in `tests/` next to SelfCheck.

## References

- INDI: Sieberling 2010; Smeur 2016; Grondman 2018 (Citation flight test); Huang et al., JGCD 2022 (delay
  margin); Pollack & van Kampen 2023 (robustness).
- Allocation: Härkegård 2002 (WLS active set); Matamoros 2017 / de Heer & de Visser 2023 (INCA); Stam & de
  Visser 2025 (D-INCA); Arnström et al. 2022 (DAQP).
- Envelope: Simon, Härkegård & Löfberg, "Command Governor Approach to Maneuver Limiting in Fighter
  Aircraft", JGCD (preprint; Saab ARES / Gripen simulator); Bemporad, Casavola & Mosca 1997 (command
  governor); Kolmanovsky, Garone & Di Cairano 2014 (governor tutorial); Özkol 2024 (incremental attainable moment set, arXiv 2406.01246); Autenrieb
  2025 (control barrier functions).
- High AoA: NASA high-alpha control-law work (stability-axis yaw damper, aileron-rudder interconnect);
  Cnβ,dyn / LCDP departure criteria.
- Handling qualities: MIL-STD-1797; AGARD AR-279; Gibson dropback / phase-rate criteria.
