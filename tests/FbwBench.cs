using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using AutopilotMod.Fbw;
using AutopilotMod.Fbw.Control;
using AutopilotMod.Fbw.Model;

/// <summary>
/// Offline bench for the fly-by-wire: one jet from its export (tests/airframes) as a rigid body, every part
/// a game plate (Model.Plate), surfaces on the game's per-channel servos, vectoring nozzles; flown by the
/// same FlightComputer the game runs. Simplified on purpose: rigid (no joints), still air, constant
/// density, and a speed-hold force at the CG in place of the engines' thrust (the vectoring nozzles still
/// push, at half their maximum thrust). The plant can be made deliberately different from the model the
/// controller uses: surface/nozzle effect scaled, extra ticks of delay, noise on the measured acceleration.
/// </summary>
internal sealed class FbwBench
{
    private const float Dt = 1f / 60f, G = 9.81f, Rho = 1.0f, SoundSpeed = 330f;

    private sealed class Part
    {
        public V3 Com, ChannelFwd, Col;
        public Q4 Lift0;
        public float Area, DragArea, Eff, Channeling;
        public int Foil;
        public Surface Surface;
    }

    private sealed class Surface
    {
        public string Name;
        public Part Part;
        public V3 Hinge;
        public readonly float[] Range = new float[3], Cur = new float[3], In = new float[3];
        public float Servo;
        public float Angle => Cur[0] + Cur[1] + Cur[2];
        public bool PitchRole => Range[0] != 0f; // stock pitches with it (FbwFlight: no pitch use otherwise)
        // split (drag) surface: force = -split x SplitDrag x rho |v| v on the part
        public float MaxSplit, SplitDrag, YawSplitFactor, Split;
        public bool Splits => MaxSplit > 0f && SplitDrag > 0f && YawSplitFactor != 0f && Range[2] == 0f;
    }

    private sealed class Nozzle
    {
        public V3 Pos, Fwd0, PitchAxis, YawAxis;
        public float VecX, VecY, MaxV, Thrust, Ax, Ay, CmdX, CmdY;
    }

    public readonly string Unit;
    private readonly List<Part> parts = new List<Part>();
    private readonly List<Surface> surfaces = new List<Surface>();
    private readonly List<Surface> splits = new List<Surface>(); // slots after the surfaces, before the nozzles
    private readonly List<Nozzle> nozzles = new List<Nozzle>();
    private readonly MassProps mass = new MassProps(); // body frame, about the CG
    private readonly float[] lift, drag;
    private readonly FoilPeaks peaks;
    public readonly float GLimit, QMax, PMax, StockAlphaLimit;
    public readonly LiftCurve Curve;
    public bool Assist = true; // Stability Assist

    // plant state (world translation, body rotation)
    private V3 vel, omega; // omega in body axes (Unity)
    private Q4 rot;
    private V3 omegaDot;   // measured over the last step, body axes
    private float nz = 1f; // load factor over the last step (g along body up, gravity removed)

    // plant vs model
    public float EffectScale = 1f;
    public int Delay;
    public float Noise; // deg/s^2 rms on the measured angular acceleration
    private readonly Random random = new Random(1);
    private readonly Queue<float[]> delayed = new Queue<float[]>();

    public FlightComputer Fc;

    public FbwBench(string jsonPath)
    {
        var d = (Dictionary<string, object>)MiniJson.Parse(File.ReadAllText(jsonPath));
        Unit = (string)d["unit"];

        var bodies = new Dictionary<string, V3>();
        foreach (Dictionary<string, object> b in L(d["bodies"]))
        {
            V3 com = Vec(b["com"]);
            mass.Add((float)(double)b["mass"], com, Vec(b["inertia"]), Quat(b["inertiaRotation"]));
            bodies[(bool)b["isRoot"] ? "" : (string)b["name"]] = com;
        }
        mass.Finish();

        var foils = L(d["airfoils"]).Cast<Dictionary<string, object>>().ToList();
        lift = new float[Math.Max(1, foils.Count) * Plate.ChartPoints];
        drag = new float[lift.Length];
        foreach (var f in foils)
        {
            int k = (int)(double)f["index"];
            var l = L(f["lift"]); var dr = L(f["drag"]);
            for (int i = 0; i < Plate.ChartPoints; i++)
            {
                lift[k * Plate.ChartPoints + i] = (float)(double)l[i];
                drag[k * Plate.ChartPoints + i] = (float)(double)dr[i];
            }
        }

        var byPath = new Dictionary<string, Part>();
        foreach (Dictionary<string, object> a in L(d["aeroParts"]))
        {
            var t = (Dictionary<string, object>)a["transforms"];
            var ln = (Dictionary<string, object>)t["liftNormal"];
            string rbPath = t.ContainsKey("rb") ? (string)((Dictionary<string, object>)t["rb"])["path"] : (string)a["path"];
            string rbName = rbPath.Split('/').Last();
            var p = new Part
            {
                Com = bodies.TryGetValue(rbPath == "" ? "" : rbName, out V3 c) ? c : Vec(ln["position"]),
                Lift0 = Basis(ln),
                ChannelFwd = Vec(((Dictionary<string, object>)a["pose"])["forward"]),
                Area = F(a, "wingArea"), DragArea = F(a, "dragArea"), Eff = F(a, "wingEffectiveness"),
                Channeling = F(a, "airflowChanneling"), Foil = (int)F(a, "airfoil"), Col = V(Field(a, "centerOfLift")),
            };
            parts.Add(p);
            byPath[(string)a["path"]] = p;
        }

        foreach (Dictionary<string, object> s in L(d["surfaces"]))
        {
            if (Field(s, "flap") == "True") continue;
            string att = Fields(s).First(x => x.StartsWith("attachedSurface->")).Split(new[] { ':' }, 2)[1];
            if (!byPath.TryGetValue(att, out Part p)) continue;
            var t = (Dictionary<string, object>)s["transforms"];
            var sf = new Surface
            {
                Name = ((string)s["path"]).Split('/').Last(), Part = p, Servo = F(s, "servoSpeed"),
                MaxSplit = F(s, "maxSplit"), SplitDrag = F(s, "splitDrag"), YawSplitFactor = F(s, "yawSplitFactor"),
            };
            sf.Range[0] = F(s, "pitchRange"); sf.Range[1] = F(s, "rollRange"); sf.Range[2] = F(s, "yawRange");
            if (sf.Range.All(r => r == 0f)) continue; // split-only (wingtip brakes)
            // the hinge is the visible mesh's x axis; older exports lack it: the lift normal's x is the same axis
            sf.Hinge = Vec(((Dictionary<string, object>)(t.ContainsKey("visibleMesh") ? t["visibleMesh"] : FindLiftNormal(d, att)))["right"]);
            // the export caught the surface wherever it was: turn its lift frame back to zero deflection
            string jp = Field(s, "job.currentPitch");
            if (jp != null)
            {
                HasDeflections = true;
                float defl = F(s, "job.currentPitch") + F(s, "job.currentRoll") + F(s, "job.currentYaw");
                p.Lift0 = Q4.AngleAxis(-defl, sf.Hinge) * p.Lift0;
            }
            p.Surface = sf;
            surfaces.Add(sf);
            if (sf.Splits) splits.Add(sf);
        }

        // left/right mirror mismatch of the lift frames (deg): how far the export's "neutral" is from neutral
        foreach (var kv in byPath)
        {
            if (!kv.Key.EndsWith("_L") || !byPath.TryGetValue(kv.Key.Substring(0, kv.Key.Length - 2) + "_R", out Part r)) continue;
            V3 ul = kv.Value.Lift0 * new V3(0f, 1f, 0f), ur = r.Lift0 * new V3(0f, 1f, 0f);
            float dot = Math.Abs(-ul.X * ur.X + ul.Y * ur.Y + ul.Z * ur.Z); // abs: a flipped plate normal is the same plate
            Asymmetry = Math.Max(Asymmetry, (float)(Math.Acos(Math.Min(1f, dot)) * 57.29578));
        }

        foreach (Dictionary<string, object> e in L(d["engines"]))
        {
            var t = (Dictionary<string, object>)e["transforms"];
            V3 tv = V(Field(e, "thrustVectoring") ?? "(0,0,0)"), gain = V(Field(e, "thrustVectoringGain") ?? "(1,1,1)");
            if ((tv.X == 0f && tv.Y == 0f) || !t.ContainsKey("nozzles[0]") || !t.ContainsKey("vectoringTransforms[0]")) continue;
            var n = (Dictionary<string, object>)t["nozzles[0]"];
            var vt = (Dictionary<string, object>)t["vectoringTransforms[0]"];
            nozzles.Add(new Nozzle
            {
                Pos = Vec(n["position"]), Fwd0 = Vec(n["forward"]), PitchAxis = Vec(vt["right"]), YawAxis = Vec(vt["up"]),
                VecX = tv.X, VecY = tv.Y, MaxV = F(e, "thrustVectoringMaxAirspeed"),
                Thrust = 0.5f * (Field(e, "maxThrust") != null ? F(e, "maxThrust") : 60000f),
            });
        }

        peaks = new FoilPeaks(lift);

        var cf = (Dictionary<string, object>)d["controlsFilter"];
        GLimit = F(cf, "flyByWire.gLimitPositive");
        QMax = F(cf, "flyByWire.maxPitchAngularVel");
        PMax = 0.5f * F(cf, "flyByWire.maxRollAngularVel"); // what stock's roll schedule always gives (PHASE0_FACTS 2)
        StockAlphaLimit = F(cf, "flyByWire.alphaLimiter");
        Curve = new LiftCurve(parts.Count);
    }

    /// <summary>The export recorded each surface's deflection, so the bench's neutral is the real one.</summary>
    public bool HasDeflections { get; }

    /// <summary>Largest left/right mismatch of mirrored lift frames, deg (after removing recorded deflections).</summary>
    public float Asymmetry { get; }

    public int Surfaces => surfaces.Count;
    public int Nozzles => nozzles.Count;
    private int NozzleBase => surfaces.Count + splits.Count;
    public int Slots => NozzleBase + 2 * nozzles.Count;

    /// <summary>Level flight at <paramref name="speed"/>, wings level, at the AoA whose lift carries the weight, surfaces neutral.</summary>
    public void Start(float speed)
    {
        float lo = -0.1f, hi = 0.35f;
        for (int i = 0; i < 40; i++)
        {
            float a = 0.5f * (lo + hi);
            SetFlight(speed, a);
            Aero(out V3 f, out _);
            if (V3.Dot(f, new V3(0f, 1f, 0f)) < mass.Mass * G) lo = a; else hi = a;
        }
        SetFlight(speed, 0.5f * (lo + hi));
        Fc = new FlightComputer(Slots);
        Fc.Law.PMax = Math.Min(PMax, RateLaw.RollCap);
        Fc.Law.QMax = QMax;
        Fc.Law.NzMax = GLimit;
        omegaDot = V3.Zero;
        Fc.Engage(State());
    }

    /// <summary>The airframe's AoA limits and lift peaks at the current flight (deg), as the envelope finds them.</summary>
    public (float max, float min, float peakPos, float peakNeg) Limits()
    {
        FillCurve(rot.Inverse * vel, State().Alpha);
        const float D = 57.29578f;
        return (Curve.AlphaMax * D, Curve.AlphaMin * D, Curve.PeakPos * D, Curve.PeakNeg * D);
    }

    // the envelope's airframe numbers, from the same plates (body frame here)
    public void FillCurve(V3 vb, float alpha)
    {
        Curve.Count = 0;
        foreach (Part p in parts)
        {
            int i = Curve.Count++;
            Curve.Parts[i] = Input(p, vb, p.Surface != null ? p.Surface.Angle : 0f);
            Curve.Deflection[i] = p.Surface != null ? p.Surface.Angle : 0f;
            Curve.Hinge[i] = p.Surface != null ? p.Surface.Hinge : V3.Zero;
            Curve.Arm[i] = p.Com - mass.Cg;
        }
        Curve.Update(alpha, vb, new V3(1f, 0f, 0f), new V3(0f, 1f, 0f), mass.Mass * G, lift, drag, Dt);
    }

    private void SetFlight(float speed, float alpha)
    {
        rot = Q4.AngleAxis(-alpha * 57.29578f, new V3(1f, 0f, 0f)); // nose up by alpha
        vel = new V3(0f, 0f, speed);
        omega = V3.Zero;
        foreach (Surface s in surfaces) { Array.Clear(s.Cur, 0, 3); s.Split = 0f; }
        foreach (Nozzle n in nozzles) n.Ax = n.Ay = n.CmdX = n.CmdY = 0f;
    }

    public FlightState State()
    {
        V3 vb = rot.Inverse * vel;
        FlightComputer.Attitude(rot, out float bank, out float pitch);
        V3 wd = omegaDot;
        if (Noise > 0f) wd += new V3(Gauss(), Gauss(), Gauss()) * (Noise / 57.29578f);
        return new FlightState
        {
            P = AirData.P(omega.Z), Q = AirData.Q(omega.X), R = AirData.R(omega.Y),
            PDot = AirData.P(wd.Z), QDot = AirData.Q(wd.X), RDot = AirData.R(wd.Y),
            Speed = vel.Magnitude, Alpha = AirData.Alpha(vb.X, vb.Y, vb.Z), Beta = AirData.Beta(vb.X, vb.Y, vb.Z),
            Bank = bank, Pitch = pitch, Nz = nz,
        };
    }

    /// <summary>One tick: the FBW reads the state and commands, the servos move, the physics steps.</summary>
    public void Step(float stickPitch, float stickRoll, float stickYaw)
    {
        FlightState s = State();
        Controller(s);
        Fc.Step(stickPitch, stickRoll, stickYaw, s, Dt);
        float[] cmd = (float[])Fc.Command.Clone();
        delayed.Enqueue(cmd);
        float[] use = delayed.Count > Delay ? delayed.Dequeue() : null;
        if (use != null) Actuate(use);
        Physics();
    }

    /// <summary>What the game adapter does each tick: positions, reach and effectiveness of every effector slot.</summary>
    private void Controller(in FlightState s)
    {
        V3 vb = rot.Inverse * vel, cg = mass.Cg;
        FillCurve(vb, s.Alpha);
        Fc.LiftSlope = Curve.Slope;
        Fc.PitchStiffness = Body(MassProps.Solve(mass.Inertia, Curve.MomentSlope)).Y;
        Fc.CouplingGain = (float)((mass.Inertia[4] - mass.Inertia[8]) / mass.Inertia[0]); // body frame: x pitch, y yaw, z roll
        Fc.AlphaMax = Curve.AlphaMax;
        Fc.AlphaMin = Curve.AlphaMin;
        Fc.Assist = Assist;
        for (int j = 0; j < surfaces.Count; j++)
        {
            Surface sf = surfaces[j];
            Fc.Position[j] = sf.Angle;
            ChannelMix.Reach(sf.Range, sf.Cur, sf.Servo, Dt * FlightComputer.ReachTicks, out float rLo, out float rHi);
            Plate.Input input = Input(sf.Part, vb, sf.Angle);
            Effectiveness.AttachedRange(input, sf.Hinge, sf.Angle, peaks, 1f, out float aLo, out float aHi);
            float travel = Math.Abs(sf.Range[0]) + Math.Abs(sf.Range[1]) + Math.Abs(sf.Range[2]);
            Fc.SetBox(j, rLo, rHi, Math.Max(aLo, -travel), Math.Min(aHi, travel));
            V3 m = Effectiveness.Surface(input, sf.Hinge, sf.Part.Com - cg, lift, drag);
            V3 acc = Body(MassProps.Solve(mass.Inertia, m));
            Fc.SetEffect(j, sf.PitchRole ? acc : new V3(acc.X, 0f, acc.Z));
        }
        for (int k = 0; k < splits.Count; k++)
        {
            Surface sf = splits[k];
            int j = surfaces.Count + k;
            V3 v = vb + V3.Cross(omega, sf.Part.Com - cg);
            Fc.Position[j] = sf.Split;
            float step = sf.Servo * Dt * FlightComputer.ReachTicks;
            Fc.SetBox(j, Math.Max(0f, sf.Split - step), Math.Min(sf.MaxSplit, sf.Split + step), 0f, sf.MaxSplit);
            Fc.SetEffect(j, Body(MassProps.Solve(mass.Inertia, Effectiveness.Split(v, Rho, sf.SplitDrag, sf.Part.Com - cg))));
            Fc.CostScale[j] = 3f;
        }
        for (int k = 0; k < nozzles.Count; k++)
        {
            Nozzle n = nozzles[k];
            int jx = NozzleBase + 2 * k, jy = jx + 1;
            bool on = s.Speed <= n.MaxV;
            V3 fwd = NozzleForward(n, n.Ax, n.Ay), arm = n.Pos - cg;
            float step = 70f * Dt * FlightComputer.ReachTicks;
            if (on && n.VecX > 0f)
            {
                Fc.Position[jx] = n.Ax;
                Fc.SetBox(jx, n.Ax - step, n.Ax + step, -n.VecX, n.VecX);
                Fc.SetEffect(jx, Body(MassProps.Solve(mass.Inertia, Effectiveness.Nozzle(n.PitchAxis, fwd, n.Thrust, arm))));
                Fc.CostScale[jx] = 10f;
            }
            else Fc.Disable(jx, n.Ax);
            if (on && n.VecY > 0f)
            {
                Fc.Position[jy] = n.Ay;
                Fc.SetBox(jy, n.Ay - step, n.Ay + step, -n.VecY, n.VecY);
                Fc.SetEffect(jy, Body(MassProps.Solve(mass.Inertia, Effectiveness.Nozzle(n.YawAxis, fwd, n.Thrust, arm))));
                Fc.CostScale[jy] = 10f;
            }
            else Fc.Disable(jy, n.Ay);
        }
    }

    private void Actuate(float[] cmd)
    {
        for (int j = 0; j < surfaces.Count; j++)
        {
            Surface sf = surfaces[j];
            ChannelMix.Surface(cmd[j], sf.Range, sf.Cur, sf.In);
            for (int c = 0; c < 3; c++) // the game's job, per channel
            {
                float r = sf.Range[c];
                sf.Cur[c] += Clamp(sf.In[c] * r - sf.Cur[c], -sf.Servo * Dt, sf.Servo * Dt);
                sf.Cur[c] = Clamp(sf.Cur[c], -Math.Abs(r), Math.Abs(r));
            }
        }
        for (int k = 0; k < splits.Count; k++)
        {
            Surface sf = splits[k];
            // through the game's split mapping (split = YawSplitFactor x yaw x MaxSplit, clamped) and servo
            float yaw = Clamp(cmd[surfaces.Count + k] / (sf.YawSplitFactor * sf.MaxSplit), -1f, 1f);
            float target = Clamp(sf.YawSplitFactor * yaw * sf.MaxSplit, 0f, sf.MaxSplit);
            sf.Split += Clamp(target - sf.Split, -sf.Servo * Dt, sf.Servo * Dt);
        }
        for (int k = 0; k < nozzles.Count; k++)
        {
            Nozzle n = nozzles[k];
            // through the game's input mapping and slew (Turbojet.FixedUpdate)
            float px = -Clamp(ChannelMix.NozzlePitchInput(cmd[NozzleBase + 2 * k], n.VecX, 2f) * 2f, -1f, 1f) * n.VecX;
            float py = Clamp(-ChannelMix.NozzleYawInput(cmd[NozzleBase + 2 * k + 1], n.VecY), -1f, 1f) * n.VecY;
            n.Ax += Clamp(px - n.Ax, -70f * Dt, 70f * Dt);
            n.Ay += Clamp(py - n.Ay, -70f * Dt, 70f * Dt);
            if (vel.Magnitude > n.MaxV) n.Ax = n.Ay = 0f;
        }
    }

    private void Physics()
    {
        Aero(out V3 fBody, out V3 mBody);
        // keep the speed: cancel the force along the flight path (the engines' job in this bench)
        V3 fWorld = rot * fBody;
        V3 vHat = vel.Normalized;
        fWorld -= vHat * V3.Dot(fWorld, vHat);
        V3 acc = fWorld / mass.Mass + new V3(0f, -G, 0f);
        V3 wd = mass.AngularAccel(mBody, omega);
        vel += acc * Dt;
        omega += wd * Dt;
        omegaDot = wd;
        nz = V3.Dot(rot.Inverse * fWorld, new V3(0f, 1f, 0f)) / (mass.Mass * G);
        V3 ww = rot * omega;
        float wm = ww.Magnitude;
        if (wm > 1e-9f) rot = Normalize(Q4.AngleAxis(wm * Dt * 57.29578f, ww / wm) * rot);
    }

    /// <summary>Total aero + nozzle force and moment about the CG, body axes.</summary>
    private void Aero(out V3 force, out V3 moment)
    {
        V3 vb = rot.Inverse * vel, cg = mass.Cg;
        force = moment = V3.Zero;
        foreach (Part p in parts)
        {
            float defl = p.Surface != null ? p.Surface.Angle * EffectScale : 0f;
            Plate.Input input = Input(p, vb, defl);
            V3 f = Plate.Force(input, lift, drag, out _);
            V3 m = V3.Cross(p.Com - cg, f);
            if (p.Col.SqrMagnitude > 0f) m += Plate.Torque(input, f);
            force += f;
            moment += m;
        }
        foreach (Surface sf in splits)
        {
            V3 v = vb + V3.Cross(omega, sf.Part.Com - cg);
            V3 f = v * (-sf.Split * EffectScale * sf.SplitDrag * Rho * v.Magnitude);
            force += f;
            moment += V3.Cross(sf.Part.Com - cg, f);
        }
        foreach (Nozzle n in nozzles)
        {
            V3 f = NozzleForward(n, n.Ax * EffectScale, n.Ay * EffectScale) * n.Thrust;
            force += f;
            moment += V3.Cross(n.Pos - cg, f);
        }
    }

    private Plate.Input Input(Part p, V3 vBodyCg, float deflection)
    {
        V3 v = vBodyCg + V3.Cross(omega, p.Com - mass.Cg);
        Q4 lr = p.Surface != null ? Q4.AngleAxis(deflection, p.Surface.Hinge) * p.Lift0 : p.Lift0;
        return new Plate.Input
        {
            VAir = v, LiftRot = lr, ChannelForward = p.ChannelFwd, Channeling = p.Channeling, Foil = p.Foil,
            Area = p.Area, DragArea = p.DragArea, Eff = p.Eff, Rho = Rho, SpeedOfSound = SoundSpeed, CenterOfLift = p.Col,
        };
    }

    private static V3 NozzleForward(Nozzle n, float ax, float ay) =>
        Q4.AngleAxis(ay, n.YawAxis) * (Q4.AngleAxis(ax, n.PitchAxis) * n.Fwd0);

    private static V3 Body(V3 unityBodyAccel) => new V3(-unityBodyAccel.Z, -unityBodyAccel.X, unityBodyAccel.Y);

    private float Gauss()
    {
        double u = 1.0 - random.NextDouble(), v = random.NextDouble();
        return (float)(Math.Sqrt(-2.0 * Math.Log(u)) * Math.Cos(2.0 * Math.PI * v));
    }

    // ---- export helpers ----

    private static object FindLiftNormal(Dictionary<string, object> d, string partPath)
    {
        foreach (Dictionary<string, object> a in L(d["aeroParts"]))
            if ((string)a["path"] == partPath) return ((Dictionary<string, object>)a["transforms"])["liftNormal"];
        throw new InvalidDataException("no lift normal for " + partPath);
    }

    private static List<object> L(object o) => (List<object>)o;
    private static IEnumerable<string> Fields(Dictionary<string, object> c) => L(c["fields"]).Cast<string>();
    private static string Field(Dictionary<string, object> c, string key) =>
        Fields(c).Where(x => x.StartsWith(key + "=")).Select(x => x.Substring(key.Length + 1)).FirstOrDefault();
    private static float F(Dictionary<string, object> c, string key) =>
        float.TryParse(Field(c, key), NumberStyles.Float, CultureInfo.InvariantCulture, out float v) ? v : 0f;
    private static V3 V(string s)
    {
        if (s == null) return V3.Zero;
        float[] x = s.Trim('(', ')').Split(',').Select(t => float.Parse(t, CultureInfo.InvariantCulture)).ToArray();
        return new V3(x[0], x[1], x[2]);
    }
    private static V3 Vec(object o) { var l = L(o); return new V3((float)(double)l[0], (float)(double)l[1], (float)(double)l[2]); }
    private static Q4 Quat(object o) { var l = L(o); return new Q4((float)(double)l[0], (float)(double)l[1], (float)(double)l[2], (float)(double)l[3]); }

    /// <summary>A rotation from a pose's right/up/forward axes (columns of its matrix).</summary>
    private static Q4 Basis(object poseObj)
    {
        var pose = (Dictionary<string, object>)poseObj;
        V3 r = Vec(pose["right"]), u = Vec(pose["up"]), f = Vec(pose["forward"]);
        double m00 = r.X, m10 = r.Y, m20 = r.Z, m01 = u.X, m11 = u.Y, m21 = u.Z, m02 = f.X, m12 = f.Y, m22 = f.Z;
        double t = m00 + m11 + m22, x, y, z, w;
        if (t > 0) { double s = Math.Sqrt(t + 1) * 2; w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s; }
        else if (m00 > m11 && m00 > m22) { double s = Math.Sqrt(1 + m00 - m11 - m22) * 2; w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s; }
        else if (m11 > m22) { double s = Math.Sqrt(1 + m11 - m00 - m22) * 2; w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s; }
        else { double s = Math.Sqrt(1 + m22 - m00 - m11) * 2; w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s; }
        return Normalize(new Q4((float)x, (float)y, (float)z, (float)w));
    }

    private static Q4 Normalize(Q4 q)
    {
        float n = (float)Math.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
        return new Q4(q.X / n, q.Y / n, q.Z / n, q.W / n);
    }

    private static float Clamp(float v, float lo, float hi) => v < lo ? lo : v > hi ? hi : v;
}
