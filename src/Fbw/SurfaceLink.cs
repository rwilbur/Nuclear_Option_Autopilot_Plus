using HarmonyLib;
using NuclearOption.Jobs;
using UnityEngine;

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
        private static readonly AccessTools.FieldRef<ControlSurface, UnitPart> AttachedRef =
            AccessTools.FieldRefAccess<ControlSurface, UnitPart>("attachedSurface");
        private static readonly AccessTools.FieldRef<ControlSurface, GameObject> MeshRef =
            AccessTools.FieldRefAccess<ControlSurface, GameObject>("visibleMesh");
        private static readonly AccessTools.FieldRef<ControlSurface, bool> LockedRef =
            AccessTools.FieldRefAccess<ControlSurface, bool>("locked");

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
            Part = AttachedRef(surface) as AeroPart;
            GameObject mesh = MeshRef(surface);
            Hinge = mesh != null ? mesh.transform : null;
        }

        /// <summary>The aero part whose plate this surface turns.</summary>
        public readonly AeroPart Part;

        /// <summary>The visible mesh the job rotates: its right axis is the hinge (positive deflection turns about +x).</summary>
        public readonly Transform Hinge;

        /// <summary>The game has locked the surface (its job then reads zero inputs).</summary>
        public bool Locked => Alive && LockedRef(Surface);

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

        public float MaxSplit => Job.maxSplit;

        /// <summary>A flap: driven by the gear, not the inputs.</summary>
        public bool Flap => Job.flap;

        public float Servo => Job.servoSpeed;

        /// <summary>Per channel (pitch, roll, yaw): range and position (deg), into caller-owned arrays.</summary>
        public void ReadChannels(float[] range, float[] current)
        {
            ref ControlSurfaceFields f = ref Job;
            range[0] = f.pitchRange; range[1] = f.rollRange; range[2] = f.yawRange;
            current[0] = f.currentPitch; current[1] = f.currentRoll; current[2] = f.currentYaw;
        }

        /// <summary>The job's three channel positions (deg), for a surface with no link (the exporter). False before the job exists.</summary>
        public static bool Channels(ControlSurface cs, out float pitch, out float roll, out float yaw)
        {
            pitch = roll = yaw = 0f;
            if (cs == null || !JobRef(cs).IsCreated) return false;
            ref ControlSurfaceFields f = ref JobRef(cs).Ref();
            pitch = f.currentPitch;
            roll = f.currentRoll;
            yaw = f.currentYaw;
            return true;
        }

        private static float Channel(float input, float range)
        {
            float r = System.Math.Abs(range);
            return System.Math.Max(-r, System.Math.Min(r, input * range));
        }
    }
}
