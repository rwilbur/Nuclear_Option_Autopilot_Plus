using System.Collections.Generic;
using HarmonyLib;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>Flight state at the start of a tick, from the last physics step. Aerospace signs (see <see cref="AirData"/>).</summary>
    internal struct AirframeState
    {
        public float Time, Dt;
        public float Speed, Rho;          // m/s (the game's airspeed), kg/m^3
        public float Mass;                // kg, attached parts only
        public Vector3 Cg, VCg;           // world, at the centre of mass; VCg is air-relative
        public float Alpha, Beta;         // rad, at the CG
        public float P, Q, R;             // rad/s, body rates (root rigidbody)
        public float PDot, QDot, RDot;    // rad/s^2, over the last tick
        public float Nz;                  // g along body up, gravity removed (1 in level flight)
    }

    /// <summary>
    /// The FBW's hold on one aircraft: its control surfaces and jet engines, and the flight state each tick.
    /// Unity objects are only touched here and in the surface and engine links.
    /// </summary>
    internal sealed class AircraftLink
    {
        public readonly Aircraft Aircraft;
        public readonly List<SurfaceLink> Surfaces = new List<SurfaceLink>();
        public readonly List<EngineLink> Engines = new List<EngineLink>();
        public readonly List<AeroPart> AeroParts;

        /// <summary>The game's aero data (charts, air), loaded at bind.</summary>
        public readonly GameAero Aero;

        private readonly ControlInputs shared;
        private readonly HashSet<Rigidbody> seenBodies = new HashSet<Rigidbody>();

        /// <summary>Mass, CG and inertia (world axes, about the CG) of the attached parts, as of the last <see cref="Sample"/>.</summary>
        public readonly Model.MassProps Mass = new Model.MassProps();
        private bool havePrev;
        private Vector3 omegaPrev, vCgPrev;

        public AircraftLink(Aircraft aircraft)
        {
            Aircraft = aircraft;
            shared = aircraft.GetInputs();
            foreach (ControlSurface cs in Find<ControlSurface>(aircraft))
                Surfaces.Add(new SurfaceLink(cs, shared));
            foreach (MonoBehaviour m in Find<MonoBehaviour>(aircraft))
            {
                EngineLink e = EngineLink.TryCreate(m, shared);
                if (e != null) Engines.Add(e);
            }
            AeroParts = Find<AeroPart>(aircraft);
            Aero = new GameAero();
            gears = Find<LandingGear>(aircraft);
        }

        private static readonly AccessTools.FieldRef<LandingGear, AeroPart> GearPart = AccessTools.FieldRefAccess<LandingGear, AeroPart>("attachedPart");
        private readonly List<LandingGear> gears;

        /// <summary>
        /// A wheel still on the aircraft carries weight (the stock fly-by-wire's own test: LandingGear.WeightOnWheel(0.05)). Retracted
        /// gear and water carry none, so sea-skimming is airborne.
        /// </summary>
        public bool OnWheels
        {
            get
            {
                foreach (LandingGear g in gears)
                {
                    if (g == null) continue;
                    AeroPart p = GearPart(g);
                    if (p != null && p.IsDetached()) continue; // a leg shot off, lying on the ground
                    if (g.WeightOnWheel(0.05f)) return true;
                }
                return false;
            }
        }

        /// <summary>The aircraft's shared inputs: the pilot's (filtered) command, and what other players' copies replay.</summary>
        public ControlInputs Shared => shared;

        public void HoldSurfaces()
        {
            foreach (SurfaceLink s in Surfaces) s.Hold();
        }

        public void HoldEngines()
        {
            foreach (EngineLink e in Engines) e.Hold();
        }

        public bool Owns(MonoBehaviour engine)
        {
            foreach (EngineLink e in Engines)
                if (e.Is(engine)) return true;
            return false;
        }

        /// <summary>Hand every surface and engine back to the shared inputs.</summary>
        public void Release()
        {
            foreach (SurfaceLink s in Surfaces) s.Release();
            foreach (EngineLink e in Engines) e.Release();
        }

        /// <summary>The state left by the last physics step.</summary>
        public AirframeState Sample(float dt)
        {
            var s = new AirframeState { Time = UnityEngine.Time.time, Dt = dt, Speed = Aircraft.speed, Rho = Aircraft.airDensity };

            // mass, CG, inertia and CG velocity over the attached parts (each is its own rigidbody under complex physics)
            Mass.Clear();
            Vector3 mv = Vector3.zero;
            seenBodies.Clear();
            foreach (UnitPart p in Aircraft.GetAllParts())
            {
                if (p == null || p.IsDetached()) continue;
                Rigidbody rb = p.rb;
                if (rb == null || !seenBodies.Add(rb)) continue;
                Mass.Add(rb.mass, GameAero.V(rb.worldCenterOfMass), GameAero.V(rb.inertiaTensor), GameAero.Q(rb.rotation * rb.inertiaTensorRotation));
                mv += rb.mass * rb.velocity;
            }
            Rigidbody root = Aircraft.rb;
            if (seenBodies.Count == 0)
            {
                Mass.Add(root.mass, GameAero.V(root.worldCenterOfMass), GameAero.V(root.inertiaTensor), GameAero.Q(root.rotation * root.inertiaTensorRotation));
                mv = root.mass * root.velocity;
            }
            Mass.Finish();
            s.Mass = Mass.Mass;
            s.Cg = GameAero.U(Mass.Cg);
            Vector3 vCg = mv / Mass.Mass;
            s.VCg = vCg - Aircraft.GetWindVelocity();

            Transform t = root.transform;
            Vector3 vb = t.InverseTransformDirection(s.VCg);
            s.Alpha = AirData.Alpha(vb.x, vb.y, vb.z);
            s.Beta = AirData.Beta(vb.x, vb.y, vb.z);

            Vector3 w = t.InverseTransformDirection(root.angularVelocity);
            s.P = AirData.P(w.z);
            s.Q = AirData.Q(w.x);
            s.R = AirData.R(w.y);
            if (havePrev && dt > 0f)
            {
                Vector3 wd = (w - omegaPrev) / dt;
                s.PDot = AirData.P(wd.z);
                s.QDot = AirData.Q(wd.x);
                s.RDot = AirData.R(wd.y);
                Vector3 a = (vCg - vCgPrev) / dt - Physics.gravity;
                s.Nz = Vector3.Dot(a, t.up) / 9.81f;
            }
            omegaPrev = w;
            vCgPrev = vCg;
            havePrev = true;
            return s;
        }

        /// <summary>
        /// Every <typeparamref name="T"/> on the aircraft. Under complex physics each part is unparented
        /// (AeroPart.CreateRB), so the root's children alone miss most of the jet: search every part.
        /// </summary>
        public static List<T> Find<T>(Aircraft ac) where T : Component
        {
            var seen = new HashSet<T>();
            var list = new List<T>();
            foreach (T c in ac.GetComponentsInChildren<T>(true))
                if (seen.Add(c)) list.Add(c);
            foreach (UnitPart p in ac.GetAllParts())
                if (p != null)
                    foreach (T c in p.GetComponentsInChildren<T>(true))
                        if (seen.Add(c)) list.Add(c);
            return list;
        }
    }
}
