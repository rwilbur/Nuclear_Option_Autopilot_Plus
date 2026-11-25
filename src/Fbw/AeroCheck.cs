using System.Collections.Generic;
using AutopilotMod.Fbw.Model;
using HarmonyLib;
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
        private static readonly AccessTools.FieldRef<JobManager, AeroJobSettings> AeroJob =
            AccessTools.FieldRefAccess<JobManager, AeroJobSettings>("aeroJob");
        private static readonly AccessTools.FieldRef<AeroPart, PtrAllocation<AeroPartFields>> PartJob =
            AccessTools.FieldRefAccess<AeroPart, PtrAllocation<AeroPartFields>>("JobFields");
        private static readonly AccessTools.FieldRef<Turbojet, JetNozzle[]> JetNozzles = AccessTools.FieldRefAccess<Turbojet, JetNozzle[]>("nozzles");
        private static readonly AccessTools.FieldRef<Turbofan, JetNozzle[]> FanNozzles = AccessTools.FieldRefAccess<Turbofan, JetNozzle[]>("nozzles");
        private static readonly AccessTools.FieldRef<JetNozzle, float> NozzleThrust = AccessTools.FieldRefAccess<JetNozzle, float>("totalThrust");
        private static readonly AccessTools.FieldRef<JetNozzle, float> NozzleShare = AccessTools.FieldRefAccess<JetNozzle, float>("thrustProportion");
        private static readonly AccessTools.FieldRef<JetNozzle, Transform> NozzleTransform = AccessTools.FieldRefAccess<JetNozzle, Transform>("thrustTransform");

        private readonly AircraftLink link;
        private readonly List<AeroPart> parts;
        private readonly HashSet<Rigidbody> bodies = new HashSet<Rigidbody>();
        private readonly MassProps mass = new MassProps();
        private readonly float[] lift, drag, density;

        // the pass of the current tick, waiting for the engines and then the next tick's measurement
        private bool pending;
        private V3 moment, omega, cg;
        private Quaternion rootRot;

        /// <summary>Last pass: Σ|F_model − F_game| / Σ|F_game| over the parts.</summary>
        public float ForceError { get; private set; }
        /// <summary>Last pass: |M_model − M_game| / |M_game| for the total aero moment about the CG.</summary>
        public float MomentError { get; private set; }
        /// <summary>Last pass: the worst single part's relative force error (parts over 50 N).</summary>
        public float WorstPart { get; private set; }
        /// <summary>Last pass: parts left out (touching water).</summary>
        public int Skipped { get; private set; }

        public AeroCheck(AircraftLink link)
        {
            this.link = link;
            parts = AircraftLink.Find<AeroPart>(link.Aircraft);
            AeroJobSettings settings = AeroJob(JobManager.i);
            lift = settings.liftCharts.ToArray();
            drag = settings.dragCharts.ToArray();
            density = LevelInfo.airDensityChart.ToArray();
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

            Vector3 wind = LevelInfo.i != null ? LevelInfo.i.GetWind() : Vector3.zero;
            float turbulence = LevelInfo.i != null ? LevelInfo.i.GetTurbulence() : 0f;
            float t = Time.timeSinceLevelLoad;
            float errSum = 0f, forceSum = 0f, worst = 0f;
            int skipped = 0;
            V3 mGame = V3.Zero, mModel = V3.Zero;
            foreach (AeroPart p in parts)
            {
                if (p == null || p.IsDetached() || p.rb == null || !PartJob(p).IsCreated) continue;
                ref AeroPartFields f = ref PartJob(p).Ref();
                Transform lt = p.LiftNormal;
                Vector3 gp = lt.GlobalPosition().AsVector3();
                if (Mathf.Max(f.submergedAmount, -(gp.y - f.collisionSize.y)) > 0f)
                {
                    skipped++;
                    continue;
                }
                var input = new Plate.Input
                {
                    VAir = V(f.velocity - Wind(gp, wind, turbulence, t)),
                    LiftRot = Q(lt.rotation),
                    ChannelForward = V(p.transform.forward),
                    Channeling = f.airflowChanneling,
                    Foil = f.airfoilID,
                    Area = f.wingArea,
                    DragArea = f.dragArea,
                    Eff = f.wingEffectiveness,
                    Rho = Plate.Read(density, 0, gp.y * 0.0021f, density.Length),
                    SpeedOfSound = LevelInfo.GetSpeedOfSound(gp.y),
                    CenterOfLift = V(f.centerOfLift),
                };
                V3 fm = Plate.Force(input, lift, drag, out _);
                V3 tm = f.centerOfLift != Vector3.zero ? Plate.Torque(input, fm) : V3.Zero;
                V3 fg = f.hasForce == JobForceType.NoForce ? V3.Zero : V(f.force);
                V3 tg = f.hasForce == JobForceType.ForceAndTorque ? V(f.torque) : V3.Zero;

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
            V3 w = V(root.angularVelocity);
            foreach (Rigidbody rb in bodies)
            {
                float k = rb.drag / (1f + dt * rb.drag);
                mGame += V3.Cross(V(rb.worldCenterOfMass) - cg, -k * rb.mass * V(rb.velocity));
                float ka = rb.angularDrag / (1f + dt * rb.angularDrag);
                Quaternion axes = rb.rotation * rb.inertiaTensorRotation;
                Vector3 local = Quaternion.Inverse(axes) * rb.angularVelocity;
                mGame -= ka * V(axes * Vector3.Scale(rb.inertiaTensor, local));
            }
            moment = mGame;
            omega = w;
            rootRot = root.rotation;
            pending = true;
        }

        /// <summary>After an engine's FixedUpdate: add the thrust its nozzles just applied.</summary>
        public void AfterEngine(MonoBehaviour engine)
        {
            if (!pending) return;
            JetNozzle[] nozzles = engine is Turbojet j ? JetNozzles(j) : engine is Turbofan f ? FanNozzles(f) : null;
            if (nozzles == null) return;
            foreach (JetNozzle n in nozzles)
            {
                if (n == null || NozzleShare(n) <= 0f) continue;
                Transform tt = NozzleTransform(n);
                moment += V3.Cross(V(tt.position) - cg, V(tt.forward * NozzleThrust(n)));
            }
        }

        /// <summary>
        /// Start of the next tick: the rigid-body angular acceleration (aerospace body axes, rad/s^2) the last
        /// tick's forces predict, to set against the measured one. False if there was no pass.
        /// </summary>
        public bool Predict(out float pDot, out float qDot, out float rDot)
        {
            pDot = qDot = rDot = 0f;
            if (!pending) return false;
            pending = false;
            V3 a = mass.AngularAccel(moment, omega);
            Vector3 body = Quaternion.Inverse(rootRot) * new Vector3(a.X, a.Y, a.Z);
            pDot = AirData.P(body.z);
            qDot = AirData.Q(body.x);
            rDot = AirData.R(body.y);
            return true;
        }

        /// <summary>The aero job's wind at a point (AeroJob_Math.CalcWind): level wind plus Perlin turbulence.</summary>
        private static Vector3 Wind(Vector3 gp, Vector3 windVelocity, float turbulence, float time)
        {
            Vector3 n = windVelocity.normalized;
            gp += time * Vector3.one * 10f;
            Vector3 gust = (-0.75f + Mathf.PerlinNoise1D(gp.z * 0.02f) + 0.5f * Mathf.PerlinNoise1D(gp.z * 0.1f)) * n
                         + 0.5f * (-0.75f + Mathf.PerlinNoise1D(gp.x * 0.02f) + 0.5f * Mathf.PerlinNoise1D(gp.x * 0.1f)) * Vector3.Cross(n, Vector3.up)
                         + 0.3f * (-0.75f + Mathf.PerlinNoise1D(gp.y * 0.02f) + 0.5f * Mathf.PerlinNoise1D(gp.y * 0.1f)) * Vector3.up;
            return windVelocity + turbulence * Mathf.Max(windVelocity.magnitude, 10f) * gust;
        }

        private static V3 V(Vector3 v) => new V3(v.x, v.y, v.z);
        private static Q4 Q(Quaternion q) => new Q4(q.x, q.y, q.z, q.w);
    }
}
