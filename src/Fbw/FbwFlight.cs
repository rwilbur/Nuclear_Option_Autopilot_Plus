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
    /// Full mode on the held aircraft: each tick, fills the <see cref="FlightComputer"/>'s effector slots from
    /// the game (positions, this tick's reach, effectiveness from the game's own plate at the live flow) and
    /// writes its commands back as each surface's and jet engine's own inputs. Slots: the control surfaces with
    /// travel, then two per vectoring engine (nozzle pitch, nozzle yaw). The same work the offline bench does
    /// (tests/FbwBench.cs), with game objects.
    /// </summary>
    internal sealed class FbwFlight
    {
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> StockG = AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>("gLimitPositive");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> StockQ = AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>("maxPitchAngularVel");
        private static readonly AccessTools.FieldRef<ControlsFilter.FlyByWire, float> StockP = AccessTools.FieldRefAccess<ControlsFilter.FlyByWire, float>("maxRollAngularVel");

        private const float NozzleSlew = 70f; // deg/s, Turbojet/Turbofan.FixedUpdate

        private readonly AircraftLink link;
        private readonly List<SurfaceLink> surfaces = new List<SurfaceLink>(); // effector slots 0..n-1
        private readonly List<SurfaceLink> passive = new List<SurfaceLink>();  // flaps and split-only: stock behaviour
        private readonly List<EngineLink> vectoring = new List<EngineLink>();  // two slots each
        private readonly float[] range = new float[3], current = new float[3], inputs = new float[3];

        public readonly FlightComputer Computer;

        public FbwFlight(AircraftLink link, ControlsFilter.FlyByWire stock)
        {
            this.link = link;
            foreach (SurfaceLink s in link.Surfaces)
            {
                if (!s.JobReady) continue;
                s.ReadChannels(range, current);
                bool travel = range[0] != 0f || range[1] != 0f || range[2] != 0f;
                if (s.Flap || !travel || s.Part == null || s.Hinge == null) passive.Add(s);
                else surfaces.Add(s);
            }
            foreach (EngineLink e in link.Engines)
            {
                Vector3 v = e.Vectoring;
                if ((v.x > 0f || v.y > 0f) && e.VectoringTransform != null) vectoring.Add(e);
            }
            Computer = new FlightComputer(surfaces.Count + 2 * vectoring.Count);
            for (int k = 0; k < vectoring.Count; k++)
            {
                Computer.CostScale[surfaces.Count + 2 * k] = 10f; // nozzles back to centre with spare freedom: thrust goes forward
                Computer.CostScale[surfaces.Count + 2 * k + 1] = 10f;
            }
            if (stock != null)
            {
                Computer.Law.NzMax = StockG(stock);
                Computer.Law.QMax = StockQ(stock);
                Computer.Law.PMax = Math.Min(0.5f * StockP(stock), RateLaw.RollCap);
            }
        }

        public int Surfaces => surfaces.Count;
        public int Nozzles => vectoring.Count;

        public void Engage(in AirframeState s) => Computer.Engage(State(s));

        public FlightState State(in AirframeState s)
        {
            FlightComputer.Attitude(GameAero.Q(link.Aircraft.rb.rotation), out float bank, out float pitch);
            return new FlightState
            {
                P = s.P, Q = s.Q, R = s.R, PDot = s.PDot, QDot = s.QDot, RDot = s.RDot,
                Speed = s.Speed, Alpha = s.Alpha, Beta = s.Beta, Bank = bank, Pitch = pitch,
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
                fc.SetEffect(j, FlightComputer.ToAero(rot, MassProps.Solve(mass.Inertia, m)));
                fc.Position[j] = angle;
                ChannelMix.Reach(range, current, s.Servo, dt * FlightComputer.ReachTicks, out float rLo, out float rHi);
                Effectiveness.AttachedRange(input, hinge, angle, aero.Peaks, 1f, out float aLo, out float aHi);
                fc.SetBox(j, rLo, rHi, aLo, aHi);
            }

            for (int k = 0; k < vectoring.Count; k++)
            {
                EngineLink e = vectoring[k];
                int jx = surfaces.Count + 2 * k, jy = jx + 1;
                Vector3 now = e.Alive ? e.Nozzle : Vector3.zero;
                bool on = e.Alive && state.Speed <= e.VectoringMaxAirspeed;
                V3 mx = V3.Zero, my = V3.Zero;
                Transform vt = on ? e.VectoringTransform : null;
                if (vt != null)
                {
                    V3 pitchAxis = GameAero.V(vt.right), yawAxis = GameAero.V(vt.parent != null ? vt.parent.up : vt.up);
                    foreach (JetNozzle n in e.Nozzles)
                    {
                        if (!EngineLink.Pushing(n, out Transform tt, out float thrust) || thrust <= 0f) continue;
                        V3 arm = GameAero.V(tt.position) - cg, fwd = GameAero.V(tt.forward);
                        mx += Effectiveness.Nozzle(pitchAxis, fwd, thrust, arm);
                        my += Effectiveness.Nozzle(yawAxis, fwd, thrust, arm);
                    }
                }
                Vector3 vec = e.Vectoring;
                float step = NozzleSlew * dt * FlightComputer.ReachTicks;
                Nozzle(fc, jx, vt != null && vec.x > 0f && mx.SqrMagnitude > 0f, now.x, vec.x, step, mx, mass, rot);
                Nozzle(fc, jy, vt != null && vec.y > 0f && my.SqrMagnitude > 0f && link.Aircraft.radarAlt >= 3f, now.y, vec.y, step, my, mass, rot);
            }

            ControlInputs pilot = link.Shared;
            fc.Step(pilot.pitch, pilot.roll, pilot.yaw, State(state), dt);
            Write(pilot);
        }

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
        private void Write(ControlInputs pilot)
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
                // a split surface opens with the yaw input (and at idle): with no yaw travel to use, the pedal keeps it stock
                i.yaw = range[2] == 0f && s.MaxSplit > 0f ? pilot.yaw : inputs[2];
            }
            foreach (SurfaceLink s in passive) Copy(pilot, s.Inputs);

            foreach (EngineLink e in link.Engines)
            {
                Pass(pilot, e.Inputs);
                e.Inputs.pitch = e.Inputs.roll = e.Inputs.yaw = 0f;
            }
            for (int k = 0; k < vectoring.Count; k++)
            {
                EngineLink e = vectoring[k];
                Vector3 vec = e.Vectoring, gain = e.Gain;
                int jx = surfaces.Count + 2 * k;
                e.Inputs.pitch = ChannelMix.NozzlePitchInput(cmd[jx], vec.x, gain.x);
                e.Inputs.yaw = ChannelMix.NozzleYawInput(cmd[jx + 1], vec.y);
            }
        }

        // what the FBW does not move: throttle (and the idle airbrake / split it opens), brake, the custom axis
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
