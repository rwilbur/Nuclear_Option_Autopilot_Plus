using AutopilotMod.Fbw.Control;
using AutopilotMod.Fbw.Model;
using HarmonyLib;
using NuclearOption.Jobs;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// The game's aerodynamic data for the FBW's models: the airfoil charts and air-density chart the aero job
    /// uses, each foil's stall angles, and a part's plate input now (its rigidbody's velocity, its lift frame,
    /// the air at its lift point: wind and turbulence as the job computes them). Loaded once per bind.
    /// </summary>
    internal sealed class GameAero
    {
        private static readonly AccessTools.FieldRef<JobManager, AeroJobSettings> AeroJob =
            AccessTools.FieldRefAccess<JobManager, AeroJobSettings>("aeroJob");
        private static readonly AccessTools.FieldRef<AeroPart, PtrAllocation<AeroPartFields>> PartJob =
            AccessTools.FieldRefAccess<AeroPart, PtrAllocation<AeroPartFields>>("JobFields");

        public readonly float[] Lift, Drag, Density;
        public readonly FoilPeaks Peaks;

        public GameAero()
        {
            AeroJobSettings settings = AeroJob(JobManager.i);
            Lift = settings.liftCharts.ToArray();
            Drag = settings.dragCharts.ToArray();
            Density = LevelInfo.airDensityChart.ToArray();
            Peaks = new FoilPeaks(Lift);
        }

        /// <summary>Air density at a global altitude, read as the aero job reads it.</summary>
        public float Rho(float globalY) => Plate.Read(Density, 0, globalY * 0.0021f, Density.Length);

        /// <summary>
        /// The part's plate input now: what the aero job will see this tick, but for the deflection, which the caller
        /// varies. False before the part's job exists or with the part touching water.
        /// </summary>
        public bool LiveInput(AeroPart p, out Plate.Input input)
        {
            input = default;
            if (p == null || p.IsDetached() || p.rb == null || !PartJob(p).IsCreated) return false;
            ref AeroPartFields f = ref PartJob(p).Ref();
            Transform lt = p.LiftNormal;
            Vector3 gp = lt.GlobalPosition().AsVector3();
            if (Mathf.Max(f.submergedAmount, -(gp.y - f.collisionSize.y)) > 0f) return false;
            LevelInfo level = LevelInfo.i;
            Vector3 wind = level != null ? Wind(gp, level.GetWind(), level.GetTurbulence(), Time.timeSinceLevelLoad) : Vector3.zero;
            input = Fill(f, p, p.rb.velocity - wind, lt.rotation, gp.y);
            return true;
        }

        /// <summary>The plate input the aero job used this tick (its own captured velocity): for the phase-2 check.</summary>
        public bool JobInput(AeroPart p, out Plate.Input input, out Vector3 force, out Vector3 torque, out JobForceType kind)
        {
            input = default;
            force = torque = Vector3.zero;
            kind = JobForceType.NoForce;
            if (p == null || p.IsDetached() || p.rb == null || !PartJob(p).IsCreated) return false;
            ref AeroPartFields f = ref PartJob(p).Ref();
            Transform lt = p.LiftNormal;
            Vector3 gp = lt.GlobalPosition().AsVector3();
            if (Mathf.Max(f.submergedAmount, -(gp.y - f.collisionSize.y)) > 0f) return false;
            LevelInfo level = LevelInfo.i;
            Vector3 wind = level != null ? Wind(gp, level.GetWind(), level.GetTurbulence(), Time.timeSinceLevelLoad) : Vector3.zero;
            input = Fill(f, p, f.velocity - wind, lt.rotation, gp.y);
            force = f.force;
            torque = f.torque;
            kind = f.hasForce;
            return true;
        }

        private Plate.Input Fill(in AeroPartFields f, AeroPart p, Vector3 vAir, Quaternion liftRot, float globalY) => new Plate.Input
        {
            VAir = V(vAir),
            LiftRot = Q(liftRot),
            ChannelForward = V(p.transform.forward),
            Channeling = f.airflowChanneling,
            Foil = f.airfoilID,
            Area = f.wingArea,
            DragArea = f.dragArea,
            Eff = f.wingEffectiveness,
            Rho = Rho(globalY),
            SpeedOfSound = LevelInfo.GetSpeedOfSound(globalY),
            CenterOfLift = V(f.centerOfLift),
        };

        /// <summary>The aero job's wind at a point (AeroJob_Math.CalcWind): level wind plus Perlin turbulence.</summary>
        public static Vector3 Wind(Vector3 gp, Vector3 windVelocity, float turbulence, float time)
        {
            Vector3 n = windVelocity.normalized;
            gp += time * Vector3.one * 10f;
            Vector3 gust = (-0.75f + Mathf.PerlinNoise1D(gp.z * 0.02f) + 0.5f * Mathf.PerlinNoise1D(gp.z * 0.1f)) * n
                         + 0.5f * (-0.75f + Mathf.PerlinNoise1D(gp.x * 0.02f) + 0.5f * Mathf.PerlinNoise1D(gp.x * 0.1f)) * Vector3.Cross(n, Vector3.up)
                         + 0.3f * (-0.75f + Mathf.PerlinNoise1D(gp.y * 0.02f) + 0.5f * Mathf.PerlinNoise1D(gp.y * 0.1f)) * Vector3.up;
            return windVelocity + turbulence * Mathf.Max(windVelocity.magnitude, 10f) * gust;
        }

        public static V3 V(Vector3 v) => new V3(v.x, v.y, v.z);
        public static Q4 Q(Quaternion q) => new Q4(q.x, q.y, q.z, q.w);
        public static Vector3 U(V3 v) => new Vector3(v.X, v.Y, v.Z);
    }
}
