using System;

namespace AutopilotMod.Fbw.Control
{
    /// <summary>
    /// What one axis's effectors can give (rad/s²; rad/s³): the acceleration they reach at full usable travel, and how
    /// fast it can change (one tick of servo travel). Filled by the caller each tick.
    /// </summary>
    internal struct AxisCapacity
    {
        public float Lo, Hi, JerkLo, JerkHi;

        public static AxisCapacity None => new AxisCapacity
        {
            Lo = float.NegativeInfinity, Hi = float.PositiveInfinity, JerkLo = float.NegativeInfinity, JerkHi = float.PositiveInfinity,
        };

        /// <summary>The acceleration a tick on from <paramref name="now"/>, heading for <paramref name="want"/>.</summary>
        public float Next(float now, float want, float dt)
        {
            float reach = Math.Max(Lo, Math.Min(Hi, want));
            now += Math.Max(JerkLo * dt, Math.Min(JerkHi * dt, reach - now));
            return Math.Max(Lo, Math.Min(Hi, now));
        }
    }

    /// <summary>
    /// The envelope: a command governor on the pitch and roll commands together. Each tick it picks the pitch-rate and
    /// stability-axis roll-rate targets handed to the rate loops so that the predicted AoA and load factor stay inside their
    /// limits over the next <see cref="Horizon"/> ticks, changing the pilot's commands as little as it can:
    ///   q̄(k+i) = γⁱ μ + ν,  p̄ₛ(k+i) = ρ,
    ///   min  β₁ μ² + β₂ (ν − q_pilot)² + β_p (ρ − p_pilot)² + Σᵢ slackᵢ²,   r̄(k) applied.
    /// Pitch follows Simon, Härkegård and Löfberg (JGCD; Saab's Gripen simulator); governing a stability-axis roll at high
    /// AoA with a predictive (dynamic) reference governor follows Ye, Chen and Wu (2015), who found it far better than a static
    /// limiter. There are no fixed caps: when the jet can't give the whole roll and the whole pull together, the trade between
    /// them comes out of the cost each tick, and each comes back as soon as the prediction allows.
    ///
    /// The prediction is the closed loop this FBW is built to have, not a fitted model:
    /// - each axis: the law's pre-filter, the INDI rate loop as designed (wanted acceleration K (cmd − rate) + cmd rate), the
    ///   effectors' <see cref="AxisCapacity"/>, and the hedge of what they can't give, as the law does it;
    /// - pitch: the airframe's static stiffness (<see cref="Stiffness"/>), and the inertia coupling of the roll: rolling at pₛ
    ///   about the flight path at AoA α turns the body at p = pₛ cos α, r = pₛ sin α, which pitches the nose by
    ///   k p r = ½ k pₛ² sin 2α, k = (I_yaw − I_roll)/I_pitch. Nose up at positive AoA, no surface removes it, and in a hard
    ///   rolling pull it outruns the nose-down authority (NASA TP-1689; bench: the Revoker past 50 deg at 150 m/s);
    /// - the short-period kinematics α̇ = q − (g/V)(n_z − n_z,grav), n_z = n_z0 + (dn_z/dα)(α − α0), lift slope from the
    ///   game's own plates (<see cref="LiftCurve"/>).
    /// Solved by golden-section searches on each unknown in turn, each cost one simulation of the horizon; only when the
    /// pilot's own commands are predicted to break a limit.
    /// Units in the cost: AoA slack in degrees, load-factor slack in g, rates relative to the full-stick rates.
    /// </summary>
    internal sealed class CommandGovernor
    {
        // ticks: 1.5 s at 60 Hz. The Gripen's 0.67 s was enough for its quick loop; here a capacity-limited jet at low speed
        // takes over a second to stop its nose, and the governor must see that before it starts (bench, Revoker at 110 m/s).
        public const int Horizon = 90;
        public const int PredictStep = 30;      // the prediction logged for checking against what happened (0.5 s)
        private const float G = 9.81f, Rad2Deg = 57.29578f;
        private const int Rounds = 3, Golden = 24; // searches: rounds of (ν, μ, ρ), golden-section steps each

        public float Gamma = 0.95f;             // per tick: decay of the transient part μ
        public float Beta1 = 0.01f, Beta2 = 0.1f, SlackWeight = 1f; // the paper's weights
        public float BetaRoll = 0.1f;           // giving up roll costs as much as giving up pitch, per full-stick unit

        /// <summary>Limits: AoA (rad), load factor (g). <see cref="AlphaLimited"/> false: the AoA limits are off (Stability Assist off).</summary>
        public float AlphaMax = 0.35f, AlphaMin = -0.26f, NzMax = 9f, NzMin = -3f;
        public bool AlphaLimited = true;

        /// <summary>The effectors' capacity on pitch and on stability-axis roll, filled by the caller each tick.</summary>
        public AxisCapacity Pitch = AxisCapacity.None, Roll = AxisCapacity.None;

        /// <summary>
        /// The airframe, filled by the caller: static pitch stiffness (rad/s² of pitch acceleration per rad of AoA; + = pitches up as
        /// AoA grows) and the inertia-coupling gain (I_yaw − I_roll)/I_pitch.
        /// </summary>
        public float Stiffness, CouplingGain;

        /// <summary>The loops being predicted, filled by the caller: the pre-filter time constant (s) and the rate-loop gains (1/s).</summary>
        public float Tau = 0.08f, KPitch = 8f, KRoll = 10f;

        /// <summary>Last solve: what the pilot asked for and what is handed on (rad/s), and the prediction <see cref="PredictStep"/> ahead.</summary>
        public float Wanted, Governed, WantedRoll, GovernedRoll, PredAlpha, PredNz;

        // this tick's model
        private float q0, qf0, qAcc0, p0, pf0, pAcc0, alpha0, nz0, excess, slope, gv, kf, dt, hiA, loA;
        private float qScale, pScale, qWant, pWant, coupling0;
        private float predA, predN;

        /// <summary>The measured state and the law's commands for one solve (aerospace axes; roll in stability axes).</summary>
        internal struct Now
        {
            public float PitchTarget, RollTarget;  // the pilot's targets (rad/s)
            public float PitchFilter, RollFilter;  // the law's pre-filter states (rad/s)
            public float Q, QDot, Roll, RollDot;   // measured rates and (filtered) accelerations
            public float Alpha, Nz, NzGravity, Speed, LiftSlope;
            public float FullPitch, FullRoll;      // full-stick rates (rad/s): the cost's units
            public float PitchLimit, RollLimit;    // the largest commands (rad/s)
        }

        /// <summary>One solve. Returns through <see cref="Governed"/> and <see cref="GovernedRoll"/>.</summary>
        public void Govern(in Now n, float step)
        {
            Wanted = qWant = n.PitchTarget;
            WantedRoll = pWant = n.RollTarget;
            q0 = n.Q; qf0 = n.PitchFilter; qAcc0 = Math.Max(Pitch.Lo, Math.Min(Pitch.Hi, n.QDot));
            p0 = n.Roll; pf0 = n.RollFilter; pAcc0 = Math.Max(Roll.Lo, Math.Min(Roll.Hi, n.RollDot));
            alpha0 = n.Alpha; nz0 = n.Nz; excess = n.Nz - n.NzGravity; slope = n.LiftSlope;
            dt = step; kf = step / (Tau + step); gv = G / Math.Max(n.Speed, 30f);
            qScale = Math.Max(n.FullPitch, 0.05f);
            pScale = Math.Max(n.FullRoll, 0.05f);
            hiA = AlphaLimited ? AlphaMax * Rad2Deg : float.PositiveInfinity;
            loA = AlphaLimited ? AlphaMin * Rad2Deg : float.NegativeInfinity;
            coupling0 = Coupling(p0, alpha0); // in the measured pitch acceleration already

            float mu = 0f, nu = qWant, rho = pWant;
            if (Slack(0f, nu, rho) > 0f) // the pilot's own commands stay inside the envelope: nothing to do (and no search)
            {
                for (int round = 0; round < Rounds; round++)
                {
                    nu = Search(0, mu, nu, rho, -n.PitchLimit, n.PitchLimit);
                    mu = Search(1, mu, nu, rho, -2f * n.PitchLimit, 2f * n.PitchLimit);
                    rho = Search(2, mu, nu, rho, -n.RollLimit, n.RollLimit);
                }
                mu = Math.Max(-n.PitchLimit - nu, Math.Min(n.PitchLimit - nu, mu));
            }
            Slack(mu, nu, rho); // leaves the prediction of the chosen commands in predA, predN
            PredAlpha = predA / Rad2Deg;
            PredNz = predN;
            Governed = mu + nu;
            GovernedRoll = rho;
        }

        /// <summary>The roll's inertia-coupling pitch acceleration (rad/s²) at stability-axis roll rate <paramref name="ps"/> and AoA <paramref name="alpha"/>.</summary>
        private float Coupling(float ps, float alpha) => 0.5f * CouplingGain * ps * ps * (float)Math.Sin(2f * alpha);

        private float Cost(float mu, float nu, float rho)
        {
            float dq = (nu - qWant) / qScale, m = mu / qScale, dp = (rho - pWant) / pScale;
            return Beta1 * m * m + Beta2 * dq * dq + BetaRoll * dp * dp + SlackWeight * Slack(mu, nu, rho);
        }

        /// <summary>The commands' predicted limit violations, summed over the horizon (deg² and g²); the prediction <see cref="PredictStep"/> ahead in predA, predN.</summary>
        private float Slack(float mu, float nu, float rho)
        {
            float slack = 0f;
            // a: AoA change from now (rad). eq, ep: each axis's acceleration as the effectors set it (rad/s²); the airframe's stiffness
            // and the roll's inertia coupling add to pitch on top, counted from their values now (in the measured acceleration already).
            float qf = qf0, q = q0, eq = qAcc0, pf = pf0, p = p0, ep = pAcc0, a = 0f, u = 1f;
            for (int i = 0; i < Horizon; i++)
            {
                // roll about the flight path
                float pfn = pf + kf * (rho - pf);
                float pWantAcc = KRoll * (pfn - p) + (pfn - pf) / dt;
                ep = Roll.Next(ep, pWantAcc, dt);
                pfn = Toward(pfn, p, (pWantAcc - ep) * dt);

                // pitch
                float target = u * mu + nu;
                u *= Gamma;
                float qfn = qf + kf * (target - qf);
                float qWantAcc = KPitch * (qfn - q) + (qfn - qf) / dt;
                float natural = Stiffness * a + Coupling(p, alpha0 + a) - coupling0;
                eq = Pitch.Next(eq, qWantAcc - natural, dt);
                float acc = eq + natural;
                qfn = Toward(qfn, q, (qWantAcc - acc) * dt);

                a += dt * (q - gv * (excess + slope * a));
                q += acc * dt;
                p += ep * dt;
                qf = qfn;
                pf = pfn;
                float ya = (alpha0 + a) * Rad2Deg, yn = nz0 + slope * a;
                slack += Over(ya, loA, hiA) + Over(yn, NzMin, NzMax);
                if (i == PredictStep) { predA = ya; predN = yn; }
            }
            return slack;
        }

        private static float Over(float y, float lo, float hi)
        {
            float s = y > hi ? y - hi : y < lo ? lo - y : 0f;
            return s * s;
        }

        // RateLaw.Toward: move cmd by -shortfall, not past the measured rate
        private static float Toward(float cmd, float measured, float shortfall)
        {
            float next = cmd - shortfall;
            if ((cmd - measured) * (next - measured) < 0f) return measured;
            return Math.Abs(next - measured) < Math.Abs(cmd - measured) ? next : cmd;
        }

        // golden-section minimum of the cost over one unknown (0: ν, 1: μ, 2: ρ) on [lo, hi], the others held; unimodal along each
        private float Search(int which, float mu, float nu, float rho, float lo, float hi)
        {
            const float R = 0.618034f;
            float x1 = hi - R * (hi - lo), x2 = lo + R * (hi - lo);
            float f1 = At(which, x1, mu, nu, rho), f2 = At(which, x2, mu, nu, rho);
            for (int i = 0; i < Golden; i++)
            {
                if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - R * (hi - lo); f1 = At(which, x1, mu, nu, rho); }
                else { lo = x1; x1 = x2; f1 = f2; x2 = lo + R * (hi - lo); f2 = At(which, x2, mu, nu, rho); }
            }
            return 0.5f * (lo + hi);
        }

        private float At(int which, float x, float mu, float nu, float rho) =>
            which == 0 ? Cost(mu, x, rho) : which == 1 ? Cost(x, nu, rho) : Cost(mu, nu, x);
    }
}
