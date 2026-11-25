using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using BepInEx;
using BepInEx.Configuration;
using BepInEx.Logging;
using UnityEngine;

namespace AutopilotMod.Fbw
{
    /// <summary>
    /// Phase-0 tool for the fly-by-wire work: writes everything the game knows about the player's jet
    /// (rigidbodies, aero parts, control surfaces, engines, the stock FBW and the airfoil charts) to
    /// BepInEx/FbwAirframes/&lt;unit&gt;.json, once per spawned aircraft. Fields are read by reflection, so
    /// no jet's layout is assumed and a game update that adds fields just adds lines. Poses are in the
    /// aircraft root's frame (Unity axes: x right, y up, z forward). JSON is written by hand: JsonUtility
    /// silently drops nested classes from a mod assembly.
    /// </summary>
    internal sealed class AirframeExport
    {
        private const float SettleTime = 2f; // s in the seat before reading: joints and stores settled
        private const BindingFlags Own = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly;

        private readonly ConfigEntry<bool> enabled;
        private readonly ManualLogSource log;
        private Aircraft pending, done;
        private float pendingSince;

        public AirframeExport(ConfigEntry<bool> enabled, ManualLogSource log)
        {
            this.enabled = enabled;
            this.log = log;
        }

        /// <summary>Frame update while the local player is in the seat of <paramref name="ac"/>.</summary>
        public void Tick(Aircraft ac)
        {
            if (!enabled.Value || ac == done) return;
            if (ac != pending)
            {
                pending = ac;
                pendingSince = Time.time;
                return;
            }
            if (Time.time - pendingSince < SettleTime) return;
            done = ac;
            try
            {
                string path = Write(ac);
                log.LogInfo("FBW airframe export: " + path);
            }
            catch (Exception e)
            {
                log.LogWarning("FBW airframe export failed: " + e);
            }
        }

        private static string Write(Aircraft ac)
        {
            Transform root = ac.transform;
            var j = new Json();
            j.Begin('{');
            j.Key("unit").Str(ac.definition.unitName);
            j.Key("exportedUtc").Str(DateTime.UtcNow.ToString("o"));
            j.Key("fixedDeltaTime").Num(Time.fixedDeltaTime);
            j.Key("localSim").Bool(ac.LocalSim);
            j.Key("airDensity").Num(ac.airDensity);
            j.Key("speed").Num(ac.speed);

            j.Key("definitionMass").Num(ac.definition.mass); // the prefab's mass, to check the bodies sum against
            j.Key("parts").Int(ac.GetAllParts().Count);
            j.Key("bodies").Begin('[');
            foreach (Rigidbody rb in Find<Rigidbody>(ac))
            {
                j.Begin('{');
                j.Key("name").Str(rb.name);
                j.Key("isRoot").Bool(rb == ac.rb);
                j.Key("mass").Num(rb.mass);
                j.Key("com").Vec(root.InverseTransformPoint(rb.worldCenterOfMass));
                j.Key("inertia").Vec(rb.inertiaTensor);
                j.Key("inertiaRotation").Quat(Quaternion.Inverse(root.rotation) * rb.rotation * rb.inertiaTensorRotation);
                j.End('}');
            }
            j.End(']');

            Components(j, "aeroParts", Find<AeroPart>(ac), root);
            Components(j, "surfaces", Find<ControlSurface>(ac), root);
            // not ac.engines: jet engines (Turbojet) never register there, only the propeller types do
            Components(j, "engines", Find<MonoBehaviour>(ac).FindAll(m => m is IEngine), root);
            Components(j, "other", Find<RelaxedStabilityController>(ac), root);
            ControlsFilter cf = ac.GetControlsFilter();
            if (cf != null)
            {
                j.Key("controlsFilter");
                Describe(j, cf, root);
            }

            if (ac.definition is AircraftDefinition def && def.aircraftParameters != null)
            {
                j.Key("aircraftParameters").Begin('[');
                foreach (string line in Fields(def.aircraftParameters, root, null, "", 0)) j.Str(line);
                j.End(']');
                j.Key("airfoils").Begin('[');
                Airfoil[] foils = def.aircraftParameters.airfoils ?? new Airfoil[0];
                for (int k = 0; k < foils.Length; k++)
                {
                    j.Begin('{');
                    j.Key("index").Int(k);
                    j.Key("name").Str(foils[k].name);
                    j.Key("lift").Begin('[');
                    for (int i = 0; i < 128; i++) j.Num(foils[k].liftCoef.Evaluate(ChartAlpha(i)));
                    j.End(']');
                    j.Key("drag").Begin('[');
                    for (int i = 0; i < 128; i++) j.Num(foils[k].dragCoef.Evaluate(ChartAlpha(i)));
                    j.End(']');
                    j.End('}');
                }
                j.End(']');
            }
            j.End('}');

            string dir = Path.Combine(Paths.BepInExRootPath, "FbwAirframes");
            Directory.CreateDirectory(dir);
            string file = Path.Combine(dir, Sanitize(ac.definition.unitName) + ".json");
            File.WriteAllText(file, j.ToString());
            return file;
        }

        private static List<T> Find<T>(Aircraft ac) where T : Component => AircraftLink.Find<T>(ac);

        /// <summary>The game's airfoil chart grid (AeroJob_Math): 128 points over +-pi.</summary>
        private static float ChartAlpha(int i) => (i - 64) * 0.04908734f;

        private static void Components<T>(Json j, string key, IEnumerable<T> items, Transform root) where T : Component
        {
            j.Key(key).Begin('[');
            foreach (T c in items) Describe(j, c, root);
            j.End(']');
        }

        private static void Describe(Json j, Component c, Transform root)
        {
            var poses = new List<(string name, Transform t)>();
            List<string> fields = Fields(c, root, poses, "", 0);
            j.Begin('{');
            j.Key("type").Str(c.GetType().Name);
            j.Key("path").Str(PathOf(c.transform, root));
            j.Key("pose");
            Pose(j, c.transform, root);
            j.Key("fields").Begin('[');
            foreach (string line in fields) j.Str(line);
            j.End(']');
            j.Key("transforms").Begin('{');
            foreach ((string name, Transform t) in poses)
            {
                j.Key(name);
                Pose(j, t, root);
            }
            j.End('}');
            j.End('}');
        }

        /// <summary>Every field of <paramref name="o"/> down its type chain: values as text, transforms into <paramref name="poses"/>, nested serializable classes two levels deep.</summary>
        private static List<string> Fields(object o, Transform root, List<(string, Transform)> poses, string prefix, int depth)
        {
            var lines = new List<string>();
            for (Type t = o.GetType(); t != null && t != typeof(MonoBehaviour) && t != typeof(object); t = t.BaseType)
            {
                foreach (FieldInfo f in t.GetFields(Own))
                {
                    object v;
                    try { v = f.GetValue(o); }
                    catch { continue; }
                    string key = prefix + f.Name;
                    switch (v)
                    {
                        case null:
                            break;
                        case float or int or bool or double or Enum or Vector2 or Vector3 or Vector4 or Quaternion or string:
                            lines.Add(key + "=" + Format(v));
                            break;
                        case Transform tr:
                            poses?.Add((key, tr));
                            break;
                        case GameObject go: // e.g. ControlSurface.visibleMesh
                            poses?.Add((key, go.transform));
                            break;
                        case Component comp:
                            lines.Add(key + "->" + comp.GetType().Name + ":" + PathOf(comp.transform, root));
                            poses?.Add((key, comp.transform));
                            break;
                        case Component[] arr: // also Transform[]
                            for (int i = 0; i < arr.Length; i++)
                                if (arr[i] != null) poses?.Add((key + "[" + i + "]", arr[i].transform));
                            break;
                        case AnimationCurve curve:
                            lines.Add(key + "=curve(" + curve.length + " keys)");
                            break;
                        default:
                            Type vt = v.GetType();
                            if (depth < 2 && vt.IsClass && !vt.IsArray && !(v is UnityEngine.Object) && vt.IsDefined(typeof(SerializableAttribute), false))
                                lines.AddRange(Fields(v, root, poses, key + ".", depth + 1));
                            break;
                    }
                }
            }
            return lines;
        }

        private static void Pose(Json j, Transform t, Transform root)
        {
            Quaternion q = Quaternion.Inverse(root.rotation) * t.rotation;
            j.Begin('{');
            j.Key("path").Str(PathOf(t, root));
            j.Key("position").Vec(root.InverseTransformPoint(t.position));
            j.Key("right").Vec(q * Vector3.right);
            j.Key("up").Vec(q * Vector3.up);
            j.Key("forward").Vec(q * Vector3.forward);
            j.End('}');
        }

        private static string PathOf(Transform t, Transform root)
        {
            if (t == root) return "";
            string p = t.name;
            for (Transform x = t.parent; x != null && x != root; x = x.parent) p = x.name + "/" + p;
            return p;
        }

        private static string Format(object v) => v switch
        {
            float f => F(f),
            double d => d.ToString("R", CultureInfo.InvariantCulture),
            Vector2 a => $"({F(a.x)},{F(a.y)})",
            Vector3 a => $"({F(a.x)},{F(a.y)},{F(a.z)})",
            Vector4 a => $"({F(a.x)},{F(a.y)},{F(a.z)},{F(a.w)})",
            Quaternion a => $"({F(a.x)},{F(a.y)},{F(a.z)},{F(a.w)})",
            _ => Convert.ToString(v, CultureInfo.InvariantCulture),
        };

        private static string F(float f) => f.ToString("R", CultureInfo.InvariantCulture);

        private static string Sanitize(string s)
        {
            foreach (char c in Path.GetInvalidFileNameChars()) s = s.Replace(c, '_');
            return s;
        }

        /// <summary>Minimal streaming JSON writer: commas and nesting handled, one value per line.</summary>
        private sealed class Json
        {
            private readonly StringBuilder sb = new StringBuilder(1 << 16);
            private readonly Stack<bool> first = new Stack<bool>();
            private bool afterKey;

            public Json Begin(char open)
            {
                Sep();
                sb.Append(open);
                first.Push(true);
                return this;
            }

            public Json End(char close)
            {
                first.Pop();
                sb.Append('\n').Append(' ', first.Count * 2).Append(close);
                return this;
            }

            public Json Key(string k)
            {
                Sep();
                Quote(k);
                sb.Append(": ");
                afterKey = true;
                return this;
            }

            public Json Str(string s) { Sep(); Quote(s ?? ""); return this; }
            public Json Num(float f) { Sep(); sb.Append(float.IsNaN(f) || float.IsInfinity(f) ? "null" : F(f)); return this; }
            public Json Int(int i) { Sep(); sb.Append(i.ToString(CultureInfo.InvariantCulture)); return this; }
            public Json Bool(bool b) { Sep(); sb.Append(b ? "true" : "false"); return this; }
            public Json Vec(Vector3 v) { Begin('[').Num(v.x).Num(v.y).Num(v.z); return End(']'); }
            public Json Quat(Quaternion q) { Begin('[').Num(q.x).Num(q.y).Num(q.z).Num(q.w); return End(']'); }

            public override string ToString() => sb.ToString();

            private void Sep()
            {
                if (afterKey) { afterKey = false; return; }
                if (first.Count == 0) return;
                if (!first.Peek()) sb.Append(',');
                first.Pop();
                first.Push(false);
                sb.Append('\n').Append(' ', first.Count * 2);
            }

            private void Quote(string s)
            {
                sb.Append('"');
                foreach (char c in s)
                {
                    if (c == '"' || c == '\\') sb.Append('\\').Append(c);
                    else if (c < ' ') sb.Append("\\u").Append(((int)c).ToString("x4"));
                    else sb.Append(c);
                }
                sb.Append('"');
            }
        }
    }
}
