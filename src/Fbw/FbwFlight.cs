using System;
using System.Collections.Generic;
using AutopilotMod.Fbw.Control;
using AutopilotMod.Fbw.Model;
using HarmonyLib;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    // inside the namespace: the autopilot's own FlightState and RateLaw in AutopilotMod would win otherwise
    using FlightState = Control.FlightState;
    using RateLaw = Control.RateLaw;

    /// <summary>
    /// Full or multiplayer mode on the held aircraft: each tick, fills the <see cref="FlightComputer"/>'s effector slots from
    /// the game (positions, this tick's reach, effectiveness from the game's own plate at the live flow) and writes its
    /// commands back. Full mode: each surface's and jet engine's own inputs; slots are the control surfaces with travel, the
    /// splits, then two per vectoring engine (nozzle pitch, nozzle yaw). Multiplayer mode: the three shared inputs through the
    /// stock mixing (<see cref="SharedMix"/>), so other players see exactly what flies. The same work the offline bench does
    /// (tests/FbwBench.cs), with game objects.
    /// </summary>
    internal sealed class FbwFlight
    {
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> StockG = AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>("gLimitPositive");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> StockQ = AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>("maxPitchAngularVel");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> StockP = AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>("maxRollAngularVel");

        private const float NozzleSlew = 70f; // deg/s, Turbojet/Turbofan.FixedUpdate

        private readonly AircraftLink link;
        private readonly List<SurfaceLink> surfaces = new List<SurfaceLink>(); // deflection slots [0, S)
        private readonly List<bool> pitchRole = new List<bool>();              // per deflection slot: stock uses it for pitch
        private readonly List<SurfaceLink> splits = new List<SurfaceLink>();   // split (drag) slots [S, S + P)
        private readonly List<SurfaceLink> passive = new List<SurfaceLink>();  // flaps and the like: stock behaviour
        private readonly List<EngineLink> vectoring = new List<EngineLink>();  // two slots each, from NozzleBase
        private readonly float[] range = new float[3], current = new float[3], inputs = new float[3];
        private readonly LiftCurve curve;                                       // the envelope's airframe numbers
        private readonly Dictionary<AeroPart, SurfaceLink> surfaceOf = new Dictionary<AeroPart, SurfaceLink>();

        public readonly FlightComputer Computer;

        /// <summary>Multiplayer mode: the shared inputs only.</summary>
        public readonly bool Shared;
        private readonly SharedMix mix;
        private readonly ControlInputs pilot = new ControlInputs(); // multiplayer mode: the pilot's inputs, saved before they are overwritten
        private readonly float[] written = new float[3];            // multiplayer mode: the shared pitch, roll, yaw written last
        private float writtenThrottle = -1f;

        public FbwFlight(AircraftLink link, ControlsFilter.FlyByWire stock, bool shared)
        {
            this.link = link;
            Shared = shared;
            // Sorted from each surface's own settings, so every surface gets its slot for the whole flight; each tick a slot
            // whose surface can't act right now (shot off, locked, no flow) sits that tick out and comes back when it can.
            foreach (SurfaceLink s in link.Surfaces)
            {
                bool travel = s.StockPitch != 0f || s.StockRoll != 0f || s.StockYaw != 0f;
                if (s.IsFlap || !travel || s.Part == null || s.Hinge == null) passive.Add(s);
                else
                {
                    surfaces.Add(s);
                    pitchRole.Add(s.StockPitch != 0f);
                }
                // a split that opens with the yaw input, on a surface whose yaw channel moves nothing: a drag-yaw effector
                if (s.Part != null && s.StockMaxSplit > 0f && s.StockYawSplit != 0f && s.StockSplitDrag > 0f && s.StockYaw == 0f) splits.Add(s);
            }
            foreach (EngineLink e in link.Engines)
            {
                Vector3 v = e.Vectoring;
                if ((v.x > 0f || v.y > 0f) && e.VectoringTransform != null) vectoring.Add(e);
            }
            Computer = new FlightComputer(shared ? SharedMix.Slots : NozzleBase + 2 * vectoring.Count, link.Engines.Count);
            mix = shared ? new SharedMix(surfaces.Count + vectoring.Count) : null;
            curve = new LiftCurve(link.AeroParts.Count);
            foreach (SurfaceLink s in link.Surfaces)
                if (s.Part != null && s.Hinge != null) surfaceOf[s.Part] = s;
            for (int k = 0; k < splits.Count && !shared; k++) Computer.CostScale[surfaces.Count + k] = 3f; // drag costs speed: closed when not needed
            for (int k = 0; k < vectoring.Count && !shared; k++)
            {
                Computer.CostScale[NozzleBase + 2 * k] = 10f; // nozzles back to centre with spare freedom: thrust goes forward
                Computer.CostScale[NozzleBase + 2 * k + 1] = 10f;
            }
            if (stock != null)
            {
                Computer.Law.NzMax = StockG(stock);
                Computer.Law.QMax = StockQ(stock);
                Computer.Law.PMax = 0.5f * StockP(stock); // what the yaw can't coordinate, FlightComputer.CoordinatedRoll takes off
            }
        }

        public int Surfaces => surfaces.Count;
        public int Splits => splits.Count;
        public int Nozzles => vectoring.Count;
        private int NozzleBase => surfaces.Count + splits.Count;

        public void Engage(in AirframeState s)
        {
            if (Shared)
            {
                // the mixing follows what the stock fly-by-wire wrote last; the pilot's inputs are those until the next filter
                ControlInputs sh = link.Shared;
                written[0] = sh.pitch;
                written[1] = sh.roll;
                written[2] = sh.yaw;
                writtenThrottle = -1f;
                Copy(sh, pilot);
            }
            Computer.Engage(State(s));
        }

        /// <summary>
        /// Standby (the stock fly-by-wire flies): the envelope only, for the display. Slower than this the air says little (on the ground,
        /// any wind from any side), so the lift curve is swept along the nose instead: its lift per airspeed squared doesn't depend on the speed.
        /// </summary>
        public void Observe(in AirframeState state, float dt)
        {
            const float SweepSpeed = 100f, Slow = 30f; // m/s
            AirframeState s = state;
            if (s.Speed < Slow)
            {
                s.VCg = link.Aircraft.rb.transform.forward * SweepSpeed;
                s.Alpha = s.Beta = 0f;
            }
            Q4 rot = GameAero.Q(link.Aircraft.rb.rotation);
            Envelope(s, rot, link.Mass, link.Mass.Cg, link.Aero, dt);
            Computer.Last = State(state); // the readouts: the jet as it is
        }

        /// <summary>
        /// Multiplayer mode, after Aircraft.FilterInputs: the pilot's stick (and hover, aim assist, the autopilot) just landed in the
        /// shared inputs. Keep it for the next tick and put the command back, for the engines' FixedUpdate and the network.
        /// </summary>
        public void AfterFilter()
        {
            ControlInputs sh = link.Shared;
            float throttle = pilot.throttle;
            Copy(sh, pilot);
            // the throttle lever is written in Update, not every tick: unchanged since our write means the lever didn't move
            if (sh.throttle == writtenThrottle) pilot.throttle = throttle;
            Apply();
        }

        private void Apply()
        {
            ControlInputs sh = link.Shared;
            sh.pitch = written[0];
            sh.roll = written[1];
            sh.yaw = written[2];
            if (writtenThrottle >= 0f) sh.throttle = writtenThrottle;
        }

        public FlightState State(in AirframeState s)
        {
            FlightComputer.Attitude(GameAero.Q(link.Aircraft.rb.rotation), out float bank, out float pitch);
            return new FlightState
            {
                P = s.P, Q = s.Q, R = s.R, PDot = s.PDot, QDot = s.QDot, RDot = s.RDot,
                Speed = s.Speed, Alpha = s.Alpha, Beta = s.Beta, Bank = bank, Pitch = pitch, Nz = s.Nz,
            };
        }

        /// <summary>One tick, before the game's jobs: effectors in, laws + INDI + allocation, inputs out.</summary>
        public void Tick(in AirframeState state, float dt)
        {
            FlightComputer fc = Computer;
            Q4 rot = GameAero.Q(link.Aircraft.rb.rotation);
            MassProps mass = link.Mass;
            V3 cg = mass.Cg;
            GameAero aero = link.Aero;
            Envelope(state, rot, mass, cg, aero, dt);
            ControlInputs pilot = Shared ? this.pilot : link.Shared;
            bool idle = pilot.throttle == 0f;
            if (Shared) FillShared(state, rot, mass, cg, aero, idle, dt);
            else FillFull(state, rot, mass, cg, aero, idle, dt);

            // engine-out yaw: each jet engine's thrust yaw in stability axes, along the body axis (a swivelled nozzle's yaw is the allocator's)
            float ca = (float)Math.Cos(state.Alpha), sa = (float)Math.Sin(state.Alpha);
            V3 bodyFwd = rot * new V3(0f, 0f, 1f);
            for (int i = 0; i < link.Engines.Count; i++)
            {
                EngineLink e = link.Engines[i];
                V3 m = V3.Zero;
                JetNozzle[] nozzles = e.Alive ? e.Nozzles : null;
                if (nozzles != null)
                    foreach (JetNozzle n in nozzles)
                        if (EngineLink.Pushing(n, out Transform tt, out float thrust) && thrust > 0f)
                            m += V3.Cross(GameAero.V(tt.position) - cg, bodyFwd * thrust);
                V3 acc = FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, m));
                fc.Cut.Yaw[i] = acc.Z * ca - acc.X * sa;
            }
            fc.Cut.Enabled = !LimitOverride; // the override gives the pilot all the thrust there is, sideslip or not

            fc.Step(pilot.pitch, pilot.roll, pilot.yaw, State(state), dt);
            if (Shared) WriteShared(pilot);
            else Write(pilot, idle);
        }

        private void FillFull(in AirframeState state, Q4 rot, MassProps mass, V3 cg, GameAero aero, bool idle, float dt)
        {
            FlightComputer fc = Computer;
            for (int j = 0; j < surfaces.Count; j++)
            {
                SurfaceLink s = surfaces[j];
                if (!s.JobReady)
                {
                    fc.Disable(j, 0f);
                    continue;
                }
                s.ReadChannels(range, current);
                float angle = current[0] + current[1] + current[2];
                if (s.Locked || s.Part.rb == null || !aero.LiveInput(s.Part, out Plate.Input input))
                {
                    fc.Disable(j, angle);
                    continue;
                }
                V3 hinge = GameAero.V(s.Hinge.right);
                V3 m = Effectiveness.Surface(input, hinge, GameAero.V(s.Part.rb.worldCenterOfMass) - cg, aero.Lift, aero.Drag);
                V3 acc = FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, m));
                // A surface the stock mixing never pitches with (ailerons, wing flaps) is not used for pitch: otherwise the
                // allocator parks the wing's roll surfaces deflected together for pitch trim, and rolls start slowly (Ifrit
                // log, 25 Nov). Its real pitch side effect still shows in the measured acceleration, which INDI takes out.
                if (!pitchRole[j]) acc = new V3(acc.X, 0f, acc.Z);
                fc.SetEffect(j, acc);
                fc.Position[j] = angle;
                ChannelMix.Reach(range, current, s.Servo, dt * FlightComputer.ReachTicks, out float rLo, out float rHi);
                Effectiveness.AttachedRange(input, hinge, angle, aero.Peaks, 1f, out float aLo, out float aHi);
                float travel = Math.Abs(range[0]) + Math.Abs(range[1]) + Math.Abs(range[2]);
                fc.SetBox(j, rLo, rHi, Math.Max(aLo, -travel), Math.Min(aHi, travel));
            }

            // split surfaces: drag on one side, yaw. At idle throttle the game opens them all (the airbrake): left to stock then.
            Vector3 wind = link.Aircraft.GetWindVelocity();
            for (int k = 0; k < splits.Count; k++)
            {
                SurfaceLink s = splits[k];
                int j = surfaces.Count + k;
                float now = s.JobReady ? s.Split : 0f;
                if (idle || !s.JobReady || s.Locked || s.Part.rb == null)
                {
                    fc.Disable(j, now);
                    continue;
                }
                V3 v = GameAero.V(s.Part.rb.velocity - wind);
                V3 m = Effectiveness.Split(v, link.Aircraft.airDensity, s.SplitDrag, GameAero.V(s.Part.rb.worldCenterOfMass) - cg);
                fc.SetEffect(j, FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, m)));
                fc.Position[j] = now;
                float step = s.Servo * dt * FlightComputer.ReachTicks, max = s.MaxSplit;
                fc.SetBox(j, Math.Max(0f, now - step), Math.Min(max, now + step), 0f, max);
            }

            for (int k = 0; k < vectoring.Count; k++)
            {
                EngineLink e = vectoring[k];
                int jx = NozzleBase + 2 * k, jy = jx + 1;
                Vector3 now = e.Alive ? e.Nozzle : Vector3.zero;
                bool vt = NozzleMoments(e, state.Speed, cg, out V3 mx, out V3 my);
                Vector3 vec = e.Vectoring;
                float step = NozzleSlew * dt * FlightComputer.ReachTicks;
                Nozzle(fc, jx, vt && vec.x > 0f && mx.SqrMagnitude > 0f, now.x, vec.x, step, mx, mass, rot);
                Nozzle(fc, jy, vt && vec.y > 0f && my.SqrMagnitude > 0f && link.Aircraft.radarAlt >= 3f, now.y, vec.y, step, my, mass, rot);
            }
        }

        /// <summary>Multiplayer mode: the same effectors, read the same way, as the three shared inputs through the stock mixing.</summary>
        private void FillShared(in AirframeState state, Q4 rot, MassProps mass, V3 cg, GameAero aero, bool idle, float dt)
        {
            mix.Begin(written[0], written[1], written[2]);
            foreach (SurfaceLink s in surfaces)
            {
                if (!s.JobReady || s.Locked || s.Part.rb == null || !aero.LiveInput(s.Part, out Plate.Input input)) continue;
                s.ReadChannels(range, current);
                float angle = current[0] + current[1] + current[2];
                V3 hinge = GameAero.V(s.Hinge.right);
                V3 m = Effectiveness.Surface(input, hinge, GameAero.V(s.Part.rb.worldCenterOfMass) - cg, aero.Lift, aero.Drag);
                Effectiveness.AttachedRange(input, hinge, angle, aero.Peaks, 1f, out float aLo, out float aHi);
                float travel = Math.Abs(range[0]) + Math.Abs(range[1]) + Math.Abs(range[2]);
                // every channel the stock mixing drives, the ailerons' pitch included (it cancels left to right): this is what moves
                mix.Surface(FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, m)), range, s.Servo, Math.Max(aLo, -travel), Math.Min(aHi, travel), dt);
            }
            Vector3 wind = link.Aircraft.GetWindVelocity();
            foreach (SurfaceLink s in splits)
            {
                if (!s.JobReady || s.Locked || s.Part.rb == null) continue;
                V3 v = GameAero.V(s.Part.rb.velocity - wind);
                V3 m = Effectiveness.Split(v, link.Aircraft.airDensity, s.SplitDrag, GameAero.V(s.Part.rb.worldCenterOfMass) - cg);
                mix.Split(FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, m)), s.YawSplitFactor, s.MaxSplit, idle);
            }
            foreach (EngineLink e in vectoring)
            {
                if (!NozzleMoments(e, state.Speed, cg, out V3 mx, out V3 my)) continue;
                Vector3 vec = e.Vectoring, gain = e.Gain;
                mix.Nozzle(FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, mx)), FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, my)),
                           mx.SqrMagnitude > 0f, my.SqrMagnitude > 0f && link.Aircraft.radarAlt >= 3f, vec.x, vec.y, vec.z, gain.x, gain.z, NozzleSlew, dt);
            }
            mix.Fill(Computer);
        }

        // a vectoring engine's moment per degree of nozzle pitch and yaw (world), from its pushing nozzles; false when it can't vector now
        private static bool NozzleMoments(EngineLink e, float speed, V3 cg, out V3 mx, out V3 my)
        {
            mx = my = V3.Zero;
            Transform vt = e.Alive && speed <= e.VectoringMaxAirspeed ? e.VectoringTransform : null;
            if (vt == null) return false;
            V3 pitchAxis = GameAero.V(vt.right), yawAxis = GameAero.V(vt.parent != null ? vt.parent.up : vt.up);
            foreach (JetNozzle n in e.Nozzles)
            {
                if (!EngineLink.Pushing(n, out Transform tt, out float thrust) || thrust <= 0f) continue;
                V3 arm = GameAero.V(tt.position) - cg, fwd = GameAero.V(tt.forward);
                mx += Effectiveness.Nozzle(pitchAxis, fwd, thrust, arm);
                my += Effectiveness.Nozzle(yawAxis, fwd, thrust, arm);
            }
            return true;
        }

        /// <summary>Multiplayer mode: the command as the shared inputs (now, for the surface job; again after the filter).</summary>
        private void WriteShared(ControlInputs pilot)
        {
            float[] cmd = Computer.Command;
            for (int c = 0; c < 3; c++) written[c] = Math.Max(-1f, Math.Min(1f, cmd[c]));
            // one throttle for every engine: the engine-out cut takes them all back by the most any one needs (with one dead, just the
            // live one), never to exactly 0, which would open the idle airbrake
            float cut = 1f;
            for (int i = 0; i < link.Engines.Count; i++) cut = Math.Min(cut, Computer.Cut.Scale[i]);
            writtenThrottle = pilot.throttle > 0f ? Math.Max(1e-3f, pilot.throttle * cut) : 0f;
            Apply();
        }

        /// <summary>
        /// The envelope's inputs: the airframe's lift slope, AoA limits and pitch stiffness from every aero part's plate at the live
        /// flow (<see cref="LiftCurve"/>), the inertia-coupling gain from the live inertia, and Stability Assist.
        /// </summary>
        private void Envelope(in AirframeState state, Q4 rot, MassProps mass, V3 cg, GameAero aero, float dt)
        {
            curve.Count = 0;
            foreach (AeroPart p in link.AeroParts)
            {
                if (!aero.LiveInput(p, out Plate.Input input)) continue;
                int i = curve.Count++;
                curve.Parts[i] = input;
                curve.Arm[i] = GameAero.V(p.rb.worldCenterOfMass) - cg;
                bool moves = surfaceOf.TryGetValue(p, out SurfaceLink s) && s.JobReady;
                curve.Deflection[i] = moves ? s.Angle : 0f;
                curve.Hinge[i] = moves ? GameAero.V(s.Hinge.right) : V3.Zero;
            }
            V3 right = rot * new V3(1f, 0f, 0f), up = rot * new V3(0f, 1f, 0f), fwd = rot * new V3(0f, 0f, 1f);
            curve.Update(state.Alpha, GameAero.V(state.VCg), right, up, mass.Mass * 9.81f, aero.Lift, aero.Drag, dt);

            FlightComputer fc = Computer;
            V3 flow = GameAero.V(state.VCg);
            float speed = flow.Magnitude;
            if (speed > 1f)
            {
                V3 rollAxis = flow / -speed; // right roll about the flight path, in Unity's left-handed sign
                V3 omega = GameAero.V(link.Aircraft.rb.angularVelocity);
                fc.RollAccel = FlightComputer.ToAero(rot, FlightComputer.RollResponse(mass.Inertia, curve.RateMoment(rollAxis), omega, rollAxis));
            }
            else fc.RollAccel = V3.Zero;
            fc.LiftSlope = curve.Known ? curve.Slope : 0f;
            fc.AlphaMax = curve.AlphaMax;
            fc.AlphaMin = curve.AlphaMin;
            fc.PitchStiffness = FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, curve.MomentSlope)).Y;
            float iPitch = V3.Dot(right, MassProps.Mul(mass.Inertia, right)), iYaw = V3.Dot(up, MassProps.Mul(mass.Inertia, up));
            float iRoll = V3.Dot(fwd, MassProps.Mul(mass.Inertia, fwd));
            fc.CouplingGain = iPitch > 0f ? (iYaw - iRoll) / iPitch : 0f;
            // the limit override: no AoA limit (as Stability Assist off: full pitch rate) or the lift peak's, and the g limit raised
            bool peak = LimitOverride && OverridePeak;
            fc.Assist = link.Aircraft.flightAssist && !(LimitOverride && !OverridePeak);
            if (peak)
            {
                fc.AlphaMax = Math.Max(curve.AlphaMax, curve.PeakPos);
                fc.AlphaMin = Math.Min(curve.AlphaMin, curve.PeakNeg);
            }
            fc.Law.GScale = LimitOverride ? OverrideG : 1f;
            fc.SlipAllowance = SlipAllowance;
        }

        /// <summary>The pilot holds the limit-override key; set by the controller each tick with the settings below.</summary>
        public bool LimitOverride;

        /// <summary>Override settings: AoA limit raised to the lift peak (else none), and the g limit's multiplier.</summary>
        public bool OverridePeak;
        public float OverrideG = 1f;

        /// <summary>Sideslip (rad) a roll may build before the coordinated-roll limit takes over.</summary>
        public float SlipAllowance;

        /// <summary>The envelope's airframe numbers, for the logs and the display.</summary>
        public LiftCurve Curve => curve;

        private static void Nozzle(FlightComputer fc, int j, bool on, float now, float limit, float step, V3 moment, MassProps mass, Q4 rot)
        {
            if (!on)
            {
                fc.Disable(j, now);
                return;
            }
            fc.Position[j] = now;
            fc.SetEffect(j, FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, moment)));
            fc.SetBox(j, Math.Max(-limit, now - step), Math.Min(limit, now + step), -limit, limit);
        }

        /// <summary>The commands as each surface's and engine's own inputs; the rest of the shared inputs passes through.</summary>
        private void Write(ControlInputs pilot, bool idle)
        {
            float[] cmd = Computer.Command;
            for (int j = 0; j < surfaces.Count; j++)
            {
                SurfaceLink s = surfaces[j];
                ControlInputs i = s.Inputs;
                Pass(pilot, i);
                if (!s.JobReady) continue;
                s.ReadChannels(range, current);
                ChannelMix.Surface(cmd[j], range, current, inputs);
                i.pitch = inputs[0];
                i.roll = inputs[1];
                i.yaw = inputs[2];
            }
            foreach (SurfaceLink s in passive) Copy(pilot, s.Inputs);
            // splits: the yaw input sets them (split = YawSplitFactor x yaw x MaxSplit); at idle, the pilot's pedal and the airbrake as stock
            for (int k = 0; k < splits.Count; k++)
            {
                SurfaceLink s = splits[k];
                float span = s.YawSplitFactor * s.MaxSplit;
                s.Inputs.yaw = idle || span == 0f ? pilot.yaw : Math.Max(-1f, Math.Min(1f, cmd[surfaces.Count + k] / span));
            }

            for (int i = 0; i < link.Engines.Count; i++)
            {
                EngineLink e = link.Engines[i];
                Pass(pilot, e.Inputs);
                e.Inputs.throttle = pilot.throttle * Computer.Cut.Scale[i]; // the engine-out cut (1 = the pilot's throttle)
                e.Inputs.pitch = e.Inputs.roll = e.Inputs.yaw = 0f;
            }
            for (int k = 0; k < vectoring.Count; k++)
            {
                EngineLink e = vectoring[k];
                Vector3 vec = e.Vectoring, gain = e.Gain;
                int jx = NozzleBase + 2 * k;
                e.Inputs.pitch = ChannelMix.NozzlePitchInput(cmd[jx], vec.x, gain.x);
                e.Inputs.yaw = ChannelMix.NozzleYawInput(cmd[jx + 1], vec.y);
            }
        }

        // what the FBW does not move: throttle (and the idle airbrake / split it opens; the engines' own throttle takes the engine-out cut), brake, the custom axis
        private static void Pass(ControlInputs from, ControlInputs to)
        {
            to.throttle = from.throttle;
            to.brake = from.brake;
            to.customAxis1 = from.customAxis1;
        }

        private static void Copy(ControlInputs from, ControlInputs to)
        {
            Pass(from, to);
            to.pitch = from.pitch;
            to.roll = from.roll;
            to.yaw = from.yaw;
        }
    }
}
