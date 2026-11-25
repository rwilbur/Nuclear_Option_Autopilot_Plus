using HarmonyLib;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// One jet engine (Turbojet or Turbofan, which share the same vectoring code). Each reads its private
    /// <c>controlInputs</c> reference in its own FixedUpdate, after the pilot's inputs are filtered. While
    /// held, it reads <see cref="Inputs"/>: this engine's own nozzle pitch/yaw/roll channels and throttle.
    /// Other engine types (propellers) are not linked and keep the shared inputs.
    /// </summary>
    internal sealed class EngineLink
    {
        private static readonly AccessTools.FieldRef<Turbojet, ControlInputs> JetInputs =
            AccessTools.FieldRefAccess<Turbojet, ControlInputs>("controlInputs");
        private static readonly AccessTools.FieldRef<Turbofan, ControlInputs> FanInputs =
            AccessTools.FieldRefAccess<Turbofan, ControlInputs>("controlInputs");
        private static readonly AccessTools.FieldRef<Turbojet, Vector3> JetNozzle =
            AccessTools.FieldRefAccess<Turbojet, Vector3>("nozzleAngles");
        private static readonly AccessTools.FieldRef<Turbofan, Vector3> FanNozzle =
            AccessTools.FieldRefAccess<Turbofan, Vector3>("nozzleAngles");
        private static readonly AccessTools.FieldRef<Turbojet, Vector3> JetVec = AccessTools.FieldRefAccess<Turbojet, Vector3>("thrustVectoring");
        private static readonly AccessTools.FieldRef<Turbofan, Vector3> FanVec = AccessTools.FieldRefAccess<Turbofan, Vector3>("thrustVectoring");
        private static readonly AccessTools.FieldRef<Turbojet, Vector3> JetGain = AccessTools.FieldRefAccess<Turbojet, Vector3>("thrustVectoringGain");
        private static readonly AccessTools.FieldRef<Turbofan, Vector3> FanGain = AccessTools.FieldRefAccess<Turbofan, Vector3>("thrustVectoringGain");
        private static readonly AccessTools.FieldRef<Turbojet, float> JetMaxV = AccessTools.FieldRefAccess<Turbojet, float>("thrustVectoringMaxAirspeed");
        private static readonly AccessTools.FieldRef<Turbofan, float> FanMaxV = AccessTools.FieldRefAccess<Turbofan, float>("thrustVectoringMaxAirspeed");
        private static readonly AccessTools.FieldRef<Turbojet, Transform[]> JetVt = AccessTools.FieldRefAccess<Turbojet, Transform[]>("vectoringTransforms");
        private static readonly AccessTools.FieldRef<Turbofan, Transform[]> FanVt = AccessTools.FieldRefAccess<Turbofan, Transform[]>("vectoringTransforms");
        private static readonly AccessTools.FieldRef<Turbojet, JetNozzle[]> JetNozzles = AccessTools.FieldRefAccess<Turbojet, JetNozzle[]>("nozzles");
        private static readonly AccessTools.FieldRef<Turbofan, JetNozzle[]> FanNozzles = AccessTools.FieldRefAccess<Turbofan, JetNozzle[]>("nozzles");
        private static readonly AccessTools.FieldRef<JetNozzle, float> NozzleThrust = AccessTools.FieldRefAccess<JetNozzle, float>("totalThrust");
        private static readonly AccessTools.FieldRef<JetNozzle, float> NozzleShare = AccessTools.FieldRefAccess<JetNozzle, float>("thrustProportion");
        private static readonly AccessTools.FieldRef<JetNozzle, Transform> NozzleTransform = AccessTools.FieldRefAccess<JetNozzle, Transform>("thrustTransform");

        private readonly Turbojet jet;
        private readonly Turbofan fan;
        private readonly ControlInputs shared;

        public readonly string Name;

        /// <summary>This engine's own inputs, read in its FixedUpdate while <see cref="Held"/>.</summary>
        public readonly ControlInputs Inputs = new ControlInputs();

        public bool Held { get; private set; }

        private EngineLink(Turbojet jet, Turbofan fan, string name, ControlInputs shared)
        {
            this.jet = jet;
            this.fan = fan;
            Name = name;
            this.shared = shared;
        }

        /// <summary>A link for a Turbojet or Turbofan, else null.</summary>
        public static EngineLink TryCreate(MonoBehaviour engine, ControlInputs shared) => engine switch
        {
            Turbojet j => new EngineLink(j, null, j.name, shared),
            Turbofan f => new EngineLink(null, f, f.name, shared),
            _ => null,
        };

        public bool Alive => jet != null || fan != null;

        /// <summary>True if this link is for <paramref name="engine"/>.</summary>
        public bool Is(MonoBehaviour engine) => engine != null && (engine == jet || engine == fan);

        public ControlInputs Active => Held ? Inputs : shared;

        public void Hold()
        {
            if (Held || !Alive) return;
            InputRef = Inputs;
            Held = true;
        }

        public void Release()
        {
            if (Held && Alive) InputRef = shared;
            Held = false;
        }

        /// <summary>Nozzle angles now (deg): x pitch, y yaw (z unused by the game).</summary>
        public Vector3 Nozzle => jet != null ? JetNozzle(jet) : FanNozzle(fan);

        /// <summary>Thrust now (N).</summary>
        public float Thrust => jet != null ? jet.GetThrust() : fan.GetThrust();

        /// <summary>Max nozzle angles per channel (deg): x pitch, y yaw, z roll (added into pitch). Zero: no vectoring.</summary>
        public Vector3 Vectoring => jet != null ? JetVec(jet) : FanVec(fan);

        /// <summary>Input gains per channel (the input is multiplied by this before clamping to ±1).</summary>
        public Vector3 Gain => jet != null ? JetGain(jet) : FanGain(fan);

        /// <summary>Above this airspeed the game zeroes the nozzles.</summary>
        public float VectoringMaxAirspeed => jet != null ? JetMaxV(jet) : FanMaxV(fan);

        /// <summary>The first vectoring transform: its right axis is the pitch hinge, its parent's up the yaw hinge. Null: none.</summary>
        public Transform VectoringTransform
        {
            get
            {
                Transform[] t = jet != null ? JetVt(jet) : FanVt(fan);
                return t != null && t.Length > 0 ? t[0] : null;
            }
        }

        public JetNozzle[] Nozzles => jet != null ? JetNozzles(jet) : FanNozzles(fan);

        /// <summary>The nozzles of a Turbojet or Turbofan, else null.</summary>
        public static JetNozzle[] NozzlesOf(MonoBehaviour engine) =>
            engine is Turbojet j ? JetNozzles(j) : engine is Turbofan f ? FanNozzles(f) : null;

        /// <summary>A nozzle that applies thrust: its thrust transform (force along forward at its position) and thrust (N).</summary>
        public static bool Pushing(JetNozzle n, out Transform at, out float thrust)
        {
            at = null;
            thrust = 0f;
            if (n == null || NozzleShare(n) <= 0f) return false;
            at = NozzleTransform(n);
            thrust = NozzleThrust(n);
            return at != null;
        }

        private ControlInputs InputRef
        {
            set
            {
                if (jet != null) JetInputs(jet) = value;
                else FanInputs(fan) = value;
            }
        }
    }
}
