using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using HarmonyLib;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// Phase-2 diagnostic: every force and torque any script applies to the held aircraft's rigidbodies,
    /// through the five Rigidbody methods all the AddForce/AddTorque overloads end in. Patched only while
    /// the aero check runs. Gives the complete scripted moment per tick (everything but gravity, contacts
    /// and Unity's own damping), and which game classes applied it, so nothing that moves the jet can be
    /// left out of the rigid-body check unnoticed.
    /// </summary>
    internal static class ForceProbe
    {
        private const int SampleEvery = 16; // stack-trace one call in this many, to name the callers cheaply

        private static Harmony harmony;
        private static HashSet<Rigidbody> watched;
        private static Vector3 reference;
        private static double fx, fy, fz, mx, my, mz; // this tick, moment about the reference point
        private static int calls;
        private static readonly Dictionary<string, double> byCaller = new Dictionary<string, double>();

        public static bool Active => harmony != null;

        public static void Start(HashSet<Rigidbody> bodies)
        {
            watched = bodies;
            if (harmony != null) return;
            harmony = new Harmony(AutopilotPlugin.Guid + ".fbwprobe");
            Type rb = typeof(Rigidbody);
            Patch(rb, nameof(Rigidbody.AddForce), new[] { typeof(Vector3), typeof(ForceMode) }, nameof(OnForce));
            Patch(rb, nameof(Rigidbody.AddRelativeForce), new[] { typeof(Vector3), typeof(ForceMode) }, nameof(OnRelativeForce));
            Patch(rb, nameof(Rigidbody.AddTorque), new[] { typeof(Vector3), typeof(ForceMode) }, nameof(OnTorque));
            Patch(rb, nameof(Rigidbody.AddRelativeTorque), new[] { typeof(Vector3), typeof(ForceMode) }, nameof(OnRelativeTorque));
            Patch(rb, nameof(Rigidbody.AddForceAtPosition), new[] { typeof(Vector3), typeof(Vector3), typeof(ForceMode) }, nameof(OnForceAtPosition));
        }

        public static void Stop()
        {
            harmony?.UnpatchSelf();
            harmony = null;
            watched = null;
        }

        /// <summary>Start a tick's sums, with moments taken about <paramref name="point"/>.</summary>
        public static void Reset(Vector3 point)
        {
            reference = point;
            fx = fy = fz = mx = my = mz = 0;
        }

        public static Vector3 Force => new Vector3((float)fx, (float)fy, (float)fz);

        /// <summary>The tick's scripted moment about <paramref name="cg"/> (N m, world).</summary>
        public static Vector3 MomentAbout(Vector3 cg)
        {
            Vector3 d = cg - reference;
            Vector3 f = Force;
            return new Vector3((float)mx, (float)my, (float)mz) - Vector3.Cross(d, f);
        }

        /// <summary>Who applied what, largest first: share of the summed |moment| per game class.</summary>
        public static string Callers()
        {
            double total = byCaller.Values.Sum();
            if (total <= 0) return "none";
            var sb = new StringBuilder();
            foreach (var kv in byCaller.OrderByDescending(k => k.Value))
                sb.Append(kv.Key).Append(' ').Append((100 * kv.Value / total).ToString("F1")).Append("%, ");
            return sb.ToString().TrimEnd(',', ' ');
        }

        public static void ClearCallers() => byCaller.Clear();

        private static void Patch(Type t, string name, Type[] args, string prefix)
        {
            harmony.Patch(AccessTools.Method(t, name, args), new HarmonyMethod(typeof(ForceProbe), prefix));
        }

        // ---- prefixes: convert to a world force at a point, or a world torque ----

        private static void OnForce(Rigidbody __instance, Vector3 force, ForceMode mode)
        {
            if (Watched(__instance)) AddForce(__instance, Scale(__instance, force, mode), __instance.worldCenterOfMass);
        }

        private static void OnRelativeForce(Rigidbody __instance, Vector3 force, ForceMode mode)
        {
            if (Watched(__instance)) AddForce(__instance, Scale(__instance, __instance.rotation * force, mode), __instance.worldCenterOfMass);
        }

        private static void OnForceAtPosition(Rigidbody __instance, Vector3 force, Vector3 position, ForceMode mode)
        {
            if (Watched(__instance)) AddForce(__instance, Scale(__instance, force, mode), position);
        }

        private static void OnTorque(Rigidbody __instance, Vector3 torque, ForceMode mode)
        {
            if (Watched(__instance)) AddTorque(TorqueScale(__instance, torque, mode));
        }

        private static void OnRelativeTorque(Rigidbody __instance, Vector3 torque, ForceMode mode)
        {
            if (Watched(__instance)) AddTorque(TorqueScale(__instance, __instance.rotation * torque, mode));
        }

        private static bool Watched(Rigidbody rb) => watched != null && rb != null && watched.Contains(rb);

        private static void AddForce(Rigidbody _, Vector3 f, Vector3 at)
        {
            Vector3 m = Vector3.Cross(at - reference, f);
            fx += f.x; fy += f.y; fz += f.z;
            mx += m.x; my += m.y; mz += m.z;
            Attribute(m.magnitude); // about the reference (the root, near the CG): ranks the sources by their pitch/roll/yaw effect
        }

        private static void AddTorque(Vector3 t)
        {
            mx += t.x; my += t.y; mz += t.z;
            Attribute(t.magnitude);
        }

        private static void Attribute(double size)
        {
            if (++calls % SampleEvery != 0) return;
            string who = "?";
            var st = new StackTrace(2, false);
            for (int i = 0; i < st.FrameCount; i++)
            {
                Type t = st.GetFrame(i)?.GetMethod()?.DeclaringType;
                if (t == null || t == typeof(ForceProbe) || t == typeof(Rigidbody) || t.Namespace == "HarmonyLib") continue;
                who = t.FullName;
                break;
            }
            byCaller.TryGetValue(who, out double v);
            byCaller[who] = v + size;
        }

        private static Vector3 Scale(Rigidbody rb, Vector3 f, ForceMode mode) => mode switch
        {
            ForceMode.Impulse => f / Time.fixedDeltaTime,
            ForceMode.Acceleration => f * rb.mass,
            ForceMode.VelocityChange => f * rb.mass / Time.fixedDeltaTime,
            _ => f,
        };

        private static Vector3 TorqueScale(Rigidbody rb, Vector3 t, ForceMode mode)
        {
            if (mode == ForceMode.Force) return t;
            if (mode == ForceMode.Impulse) return t / Time.fixedDeltaTime;
            // acceleration modes ignore the inertia: torque = I a
            Quaternion axes = rb.rotation * rb.inertiaTensorRotation;
            Vector3 a = Quaternion.Inverse(axes) * t;
            Vector3 iw = axes * Vector3.Scale(rb.inertiaTensor, a);
            return mode == ForceMode.VelocityChange ? iw / Time.fixedDeltaTime : iw;
        }
    }
}
