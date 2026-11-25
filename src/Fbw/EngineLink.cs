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
