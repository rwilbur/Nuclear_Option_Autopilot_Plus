using System.Collections.Generic;
using AutopilotMod.Fbw.Model;
using BepInEx.Logging;
using NuclearOption.Jobs;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// Phase-2 diagnostics, run with telemetry on. After the game's aero job (JobManager.FixedUpdateLate,
    /// before the physics step) it:
    /// 1. evaluates our <see cref="Plate"/> on each part's own job inputs and compares it with the force and
    ///    torque the job produced (is the effector model the game's?);
    /// 2. sums the game's aero moments, Unity's drag on each rigidbody and, after each engine's FixedUpdate,
    ///    the nozzle thrust, about the CG, and predicts the rigid-body angular acceleration of the coming
    ///    physics step (does the jointed jet behave as one rigid body?). The next tick's measured
    ///    acceleration is the answer.
    /// </summary>
    internal sealed class AeroCheck
    {
        private readonly AircraftLink link;
        private readonly ManualLogSource log;
        private readonly List<AeroPart> parts;
        private readonly HashSet<Rigidbody> bodies = new HashSet<Rigidbody>();
        private readonly MassProps mass = new MassProps();
        private readonly GameAero aero;

        // the pass of the current tick, waiting for the engines and then the next tick's measurement
        private bool pending;
        private V3 mAero, mThrust, mDamp, omega, cg;
        private Quaternion rootRot;
        private Vector3 rootPos;
        private int ticks;

        /// <summary>Last pass: Σ|F_model − F_game| / Σ|F_game| over the parts.</summary>
        public float ForceError { get; private set; }
        /// <summary>Last pass: |M_model − M_game| / |M_game| for the total aero moment about the CG.</summary>
        public float MomentError { get; private set; }
        /// <summary>Last pass: the worst single part's relative force error (parts over 50 N).</summary>
        public float WorstPart { get; private set; }
        /// <summary>Last pass: parts left out (touching water).</summary>
        public int Skipped { get; private set; }

        public AeroCheck(AircraftLink link, ManualLogSource log)
        {
            this.link = link;
            this.log = log;
            parts = link.AeroParts;
            aero = link.Aero;
            ForceProbe.Start(bodies);
            ForceProbe.Reset(link.Aircraft.rb.position);
            LogInventory();
        }

        /// <summary>
        /// Once per check: every rigidbody of the jet with the flags that decide whether physics counts it,
        /// and any other rigidbody near the jet (pilot, mounted stores) that the mass sum would miss.
        /// </summary>
        private void LogInventory()
        {
            var own = new HashSet<Rigidbody>();
            var sb = new System.Text.StringBuilder("FBW bodies of " + link.Aircraft.definition.unitName + ":");
            foreach (AeroPart p in parts)
            {
                if (p == null || p.rb == null || !own.Add(p.rb)) continue;
                Rigidbody rb = p.rb;
                sb.Append($"\n  {rb.name}: {rb.mass:F1} kg, kinematic {rb.isKinematic}, gravity {rb.useGravity}, detached {p.IsDetached()}, drag {rb.drag:F2}/{rb.angularDrag:F2}, joints {rb.GetComponents<Joint>().Length}");
            }
            Vector3 at = link.Aircraft.rb.position;
            foreach (Rigidbody rb in Object.FindObjectsOfType<Rigidbody>())
            {
                if (own.Contains(rb) || (rb.position - at).sqrMagnitude > 40f * 40f) continue;
                Joint j = rb.GetComponent<Joint>();
                string to = j != null && j.connectedBody != null ? j.connectedBody.name : "-";
                sb.Append($"\n  NOT COUNTED {rb.name} ({rb.gameObject.GetComponents<Component>().Length} components): {rb.mass:F1} kg, kinematic {rb.isKinematic}, gravity {rb.useGravity}, joint to {to}, {(rb.position - at).magnitude:F1} m away");
            }
            log.LogInfo(sb.ToString());
        }

        /// <summary>After the aero job finished, before the physics step.</summary>
        public void AfterAero()
        {
            mass.Clear();
            bodies.Clear();
            foreach (AeroPart p in parts)
            {
                if (p == null || p.IsDetached() || p.rb == null || !bodies.Add(p.rb)) continue;
                Rigidbody rb = p.rb;
                mass.Add(rb.mass, V(rb.worldCenterOfMass), V(rb.inertiaTensor), Q(rb.rotation * rb.inertiaTensorRotation));
            }
            mass.Finish();
            cg = mass.Cg;

            float errSum = 0f, forceSum = 0f, worst = 0f;
            int skipped = 0;
            V3 mGame = V3.Zero, mModel = V3.Zero;
            foreach (AeroPart p in parts)
            {
                if (p == null || p.IsDetached() || p.rb == null) continue;
                if (!aero.JobInput(p, out Plate.Input input, out Vector3 force, out Vector3 torque, out JobForceType kind))
                {
                    skipped++;
                    continue;
                }
                V3 fm = Plate.Force(input, aero.Lift, aero.Drag, out _);
                V3 tm = input.CenterOfLift.SqrMagnitude > 0f ? Plate.Torque(input, fm) : V3.Zero;
                V3 fg = kind == JobForceType.NoForce ? V3.Zero : V(force);
                V3 tg = kind == JobForceType.ForceAndTorque ? V(torque) : V3.Zero;

                float e = (fm - fg).Magnitude, g = fg.Magnitude;
                errSum += e;
                forceSum += g;
                if (g > 50f && e / g > worst) worst = e / g;
                V3 r = V(p.rb.worldCenterOfMass) - cg;
                mGame += V3.Cross(r, fg) + tg;
                mModel += V3.Cross(r, fm) + tm;
            }
            ForceError = forceSum > 1f ? errSum / forceSum : 0f;
            MomentError = mGame.Magnitude > 1f ? (mModel - mGame).Magnitude / mGame.Magnitude : 0f;
            WorstPart = worst;
            Skipped = skipped;

            // Unity's per-rigidbody damping in the coming step: v *= 1 / (1 + dt c), likewise w with angularDrag
            float dt = Time.fixedDeltaTime;
            Rigidbody root = link.Aircraft.rb;
            V3 damp = V3.Zero;
            foreach (Rigidbody rb in bodies)
            {
                float k = rb.drag / (1f + dt * rb.drag);
                damp += V3.Cross(V(rb.worldCenterOfMass) - cg, -k * rb.mass * V(rb.velocity));
                float ka = rb.angularDrag / (1f + dt * rb.angularDrag);
                Quaternion axes = rb.rotation * rb.inertiaTensorRotation;
                Vector3 local = Quaternion.Inverse(axes) * rb.angularVelocity;
                damp -= ka * V(axes * Vector3.Scale(rb.inertiaTensor, local));
            }
            mAero = mGame;
            mDamp = damp;
            mThrust = V3.Zero;
            omega = V(root.angularVelocity);
            rootRot = root.rotation;
            rootPos = root.position;
            pending = true;
        }

        /// <summary>After an engine's FixedUpdate: add the thrust its nozzles just applied.</summary>
        public void AfterEngine(MonoBehaviour engine)
        {
            if (!pending) return;
            JetNozzle[] nozzles = EngineLink.NozzlesOf(engine);
            if (nozzles == null) return;
            foreach (JetNozzle n in nozzles)
            {
                if (!EngineLink.Pushing(n, out Transform tt, out float thrust)) continue;
                mThrust += V3.Cross(V(tt.position) - cg, V(tt.forward * thrust));
            }
        }

        /// <summary>
        /// Start of the next tick: what the last tick's forces predict for the physics step just taken, to set
        /// against the measured acceleration. False if there was no pass. Then starts the probe's next tick.
        /// </summary>
        public bool Predict(out Prediction p)
        {
            p = default;
            bool had = pending;
            if (pending)
            {
                pending = false;
                // itemised: the game's aero forces, the nozzle thrust, Unity's damping
                p.Itemised = Body(mass.AngularAccel(mAero + mThrust + mDamp, omega));
                // everything any script applied (the probe), with the same damping
                V3 probe = V(ForceProbe.MomentAbout(new Vector3(cg.X, cg.Y, cg.Z)));
                p.Probe = Body(mass.AngularAccel(probe + mDamp, omega));
                p.Unexplained = Body(probe - mAero - mThrust); // N m: scripted moment the itemised sum lacks
                p.Thrust = Body(mThrust);
                Vector3 cgRel = Quaternion.Inverse(rootRot) * (new Vector3(cg.X, cg.Y, cg.Z) - rootPos);
                p.CgFromRoot = new V3(cgRel.x, cgRel.y, cgRel.z); // Unity body axes (x right, y up, z forward)
            }
            ForceProbe.Reset(link.Aircraft.rb.position);
            if (++ticks % 600 == 0) log.LogInfo("FBW force sources (by moment, so far): " + ForceProbe.Callers());
            return had;
        }

        /// <summary>Stop the probe and report its sources.</summary>
        public void Dispose()
        {
            log.LogInfo("FBW force sources (by moment): " + ForceProbe.Callers());
            ForceProbe.ClearCallers();
            ForceProbe.Stop();
        }

        /// <summary>A world vector in aerospace body axes (roll, pitch, yaw): see <see cref="AirData"/>.</summary>
        private V3 Body(V3 w)
        {
            Vector3 b = Quaternion.Inverse(rootRot) * new Vector3(w.X, w.Y, w.Z);
            return new V3(AirData.P(b.z), AirData.Q(b.x), AirData.R(b.y));
        }

        internal struct Prediction
        {
            public V3 Itemised, Probe;  // rad/s^2 (p, q, r)
            public V3 Unexplained;      // N m (roll, pitch, yaw)
            public V3 Thrust;           // N m (roll, pitch, yaw)
            public V3 CgFromRoot;       // m, Unity body axes
        }

        private static V3 V(Vector3 v) => GameAero.V(v);
        private static Q4 Q(Quaternion q) => GameAero.Q(q);
    }
}
