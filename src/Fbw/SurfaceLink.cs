using HarmonyLib;
using NuclearOption.Jobs;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// One control surface. The game's surface job reads the surface's private <c>controlInputs</c>
    /// reference (normally the aircraft's shared inputs) once per tick, in JobManager.ScheduleJobs. While
    /// held, that reference points at <see cref="Inputs"/>, so this surface gets its own pitch/roll/yaw
    /// channel inputs. The game's job still does the servo, travel, lock, detach and split logic.
    /// </summary>
    internal sealed class SurfaceLink
    {
        private static readonly AccessTools.FieldRef<ControlSurface, ControlInputs> InputsRef =
            AccessTools.FieldRefAccess<ControlSurface, ControlInputs>("controlInputs");
        private static readonly AccessTools.FieldRef<ControlSurface, PtrAllocation<ControlSurfaceFields>> JobRef =
            AccessTools.FieldRefAccess<ControlSurface, PtrAllocation<ControlSurfaceFields>>("JobFields");

        public readonly ControlSurface Surface;
        public readonly string Name;

        /// <summary>This surface's own inputs, read by its job while <see cref="Held"/>.</summary>
        public readonly ControlInputs Inputs = new ControlInputs();

        private readonly ControlInputs shared;

        public bool Held { get; private set; }

        public SurfaceLink(ControlSurface surface, ControlInputs shared)
        {
            Surface = surface;
            Name = surface.name;
            this.shared = shared;
        }

        /// <summary>False once Unity has destroyed the surface.</summary>
        public bool Alive => Surface != null;

        /// <summary>The inputs the surface's job reads this tick.</summary>
        public ControlInputs Active => Held ? Inputs : shared;

        public void Hold()
        {
            if (Held || !Alive) return;
            InputsRef(Surface) = Inputs;
            Held = true;
        }

        public void Release()
        {
            if (Held && Alive) InputsRef(Surface) = shared;
            Held = false;
        }

        // ---- the job's state. Only read between jobs (JobManager.FixedUpdateEarly, before ScheduleJobs):
        // ---- the Burst job writes these fields while it runs.

        /// <summary>The job's fields exist (created in the surface's Awake) and the surface is attached.</summary>
        public bool JobReady => Alive && JobRef(Surface).IsCreated && !Job.IsDetached;

        private ref ControlSurfaceFields Job => ref JobRef(Surface).Ref();

        /// <summary>Deflection now (deg): the sum of the three channels' servo positions.</summary>
        public float Angle
        {
            get
            {
                ref ControlSurfaceFields f = ref Job;
                return f.currentPitch + f.currentRoll + f.currentYaw;
            }
        }

        /// <summary>Deflection the job is moving toward with the active inputs (deg), each channel clamped to its range.</summary>
        public float Target
        {
            get
            {
                ref ControlSurfaceFields f = ref Job;
                if (f.flap) return f.currentPitch; // flaps follow the gear, not the inputs
                ControlInputs i = Active;
                return Channel(i.pitch, f.pitchRange) + Channel(i.roll, f.rollRange) + Channel(i.yaw, f.yawRange);
            }
        }

        public float Split => Job.splitAmount;

        private static float Channel(float input, float range)
        {
            float r = System.Math.Abs(range);
            return System.Math.Max(-r, System.Math.Min(r, input * range));
        }
    }
}
