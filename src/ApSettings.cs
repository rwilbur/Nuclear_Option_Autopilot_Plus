using BepInEx.Configuration;
using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// All user-tunable values, stored in BepInEx\config\com.yourname.autopilotmod.cfg.
    /// Edit the .cfg while the game is closed (or use a BepInEx config manager in-game);
    /// values are read live every physics tick, so in-game edits apply immediately.
    /// </summary>
    internal sealed class ApSettings
    {
        // ---- keys ----
        public ConfigEntry<KeyboardShortcut> KeyApToggle;
        public ConfigEntry<KeyboardShortcut> KeyAtToggle;
        public ConfigEntry<KeyboardShortcut> KeyMfdToggle;
        public ConfigEntry<KeyboardShortcut> KeyCursorToggle;
        public ConfigEntry<KeyboardShortcut> KeyVertUp;
        public ConfigEntry<KeyboardShortcut> KeyVertDown;
        public ConfigEntry<KeyboardShortcut> KeyHdgLeft;
        public ConfigEntry<KeyboardShortcut> KeyHdgRight;
        public ConfigEntry<KeyboardShortcut> KeySpdUp;
        public ConfigEntry<KeyboardShortcut> KeySpdDown;
        public ConfigEntry<KeyboardShortcut> KeyRadarPage;
        public ConfigEntry<KeyCode> KeyCoarseStep;

        // ---- vertical ----
        public ConfigEntry<float> AltitudeGain;
        public ConfigEntry<float> MaxClimbRate;
        public ConfigEntry<float> MaxFlightPathAngle;
        public ConfigEntry<float> AltitudeCaptureAccel;
        public ConfigEntry<float> FloorGain;
        public ConfigEntry<float> FlightPathGain;
        public ConfigEntry<float> FlightPathDamping;
        public ConfigEntry<float> MaxPullG;
        public ConfigEntry<float> MaxPushG;
        public ConfigEntry<float> PitchRateGain;
        public ConfigEntry<float> PitchRateIntegralFbw;
        public ConfigEntry<float> PitchRateIntegralNoFbw;
        public ConfigEntry<float> GroundFloor;

        // ---- terrain following ----
        public ConfigEntry<float> TfLookAhead;
        public ConfigEntry<float> TfPullG;
        public ConfigEntry<float> TfPushG;
        public ConfigEntry<float> TfMinClearance;
        public ConfigEntry<float> TfSmoothing;

        // ---- lateral ----
        public ConfigEntry<float> HeadingGain;
        public ConfigEntry<float> MaxTurnRate;
        public ConfigEntry<bool> NavFlyOver;
        public ConfigEntry<float> MaxBank;
        public ConfigEntry<float> BankGain;
        public ConfigEntry<float> MaxRollRate;
        public ConfigEntry<float> RollRateGain;
        public ConfigEntry<float> RollRateIntegral;

        // ---- autothrottle ----
        public ConfigEntry<float> SpeedGain;
        public ConfigEntry<float> SpeedIntegral;
        public ConfigEntry<float> SpeedDamping;
        public ConfigEntry<float> AtMinThrottle;
        public ConfigEntry<float> AtMaxThrottle;

        // ---- safety ----
        public ConfigEntry<float> StickOverride;
        public ConfigEntry<float> MinEngageSpeed;
        public ConfigEntry<float> MinEngageRadarAlt;
        public ConfigEntry<bool> DisconnectTone;

        // ---- display ----
        public ConfigEntry<bool> MfdVisible;
        public ConfigEntry<float> MfdSize;
        public ConfigEntry<float> MfdX;
        public ConfigEntry<float> MfdY;
        public ConfigEntry<bool> ShowDebug;
        public ConfigEntry<float> RadarSize;
        public ConfigEntry<float> RadarX;
        public ConfigEntry<float> RadarY;
        public ConfigEntry<float> RadarTargetAgl;
        public ConfigEntry<float> RadarEnemyRcs, RadarFriendlyRcs;
        public ConfigEntry<float> RadarOpacity;
        public ConfigEntry<bool> RadarEyes, RadarSams;
        public ConfigEntry<float> GunHeatSize, GunHeatX, GunHeatY;

        // ---- web mirror ----
        public ConfigEntry<bool> MirrorEnabled;
        public ConfigEntry<int> MirrorPort;
        public ConfigEntry<int> MirrorResolution;
        public ConfigEntry<int> MirrorMapResolution;
        public ConfigEntry<int> MirrorFps;
        public ConfigEntry<int> MirrorJpegQuality;
        public ConfigEntry<bool> MirrorFlipVertical;
        public ConfigEntry<bool> MirrorMapJpeg;

        // ---- terrain warning ----
        public ConfigEntry<bool> GpwsEnabled;
        public ConfigEntry<bool> GpwsVoice;
        public ConfigEntry<int> GpwsVoiceVolume;
        public ConfigEntry<int> GpwsVoiceRate;
        public ConfigEntry<float> GpwsReactionTime;
        public ConfigEntry<float> GpwsPullG;
        public ConfigEntry<bool> GpwsWithGearDown;

        public ApSettings(ConfigFile cfg)
        {
            const string K = "Keys";
            KeyApToggle = cfg.Bind(K, "AutopilotEngage", new KeyboardShortcut(KeyCode.F5), "Engage / disconnect the autopilot (A/P master).");
            KeyAtToggle = cfg.Bind(K, "AutothrottleEngage", new KeyboardShortcut(KeyCode.F8), "Engage / disconnect the autothrottle.");
            KeyMfdToggle = cfg.Bind(K, "ShowHideMfd", new KeyboardShortcut(KeyCode.F6), "Show / hide the A/P MFD page.");
            KeyRadarPage = cfg.Bind(K, "RadarCoveragePage", new KeyboardShortcut(KeyCode.F9), "Cycle the radar-coverage display: off -> ALL friendly radars -> OWN radar only -> ENEMY radars at known positions -> off.");
            KeyCursorToggle = cfg.Bind(K, "MfdCursor", new KeyboardShortcut(KeyCode.F7), "Toggle a mouse cursor so the MFD push-buttons can be clicked. While it is on, the bezel can also be dragged to move the MFD.");
            KeyVertUp = cfg.Bind(K, "VerticalTargetUp", new KeyboardShortcut(KeyCode.PageUp), "Raise the selected altitude / vertical-speed target (hold Shift for a 10x step).");
            KeyVertDown = cfg.Bind(K, "VerticalTargetDown", new KeyboardShortcut(KeyCode.PageDown), "Lower the selected altitude / vertical-speed target (hold Shift for a 10x step).");
            KeyHdgLeft = cfg.Bind(K, "HeadingLeft", new KeyboardShortcut(KeyCode.LeftBracket), "Heading bug left (or bank target left in ATT mode). Shift = 10x.");
            KeyHdgRight = cfg.Bind(K, "HeadingRight", new KeyboardShortcut(KeyCode.RightBracket), "Heading bug right (or bank target right in ATT mode). Shift = 10x.");
            KeySpdUp = cfg.Bind(K, "SpeedUp", new KeyboardShortcut(KeyCode.Equals), "Raise the autothrottle speed target. Shift = 10x.");
            KeySpdDown = cfg.Bind(K, "SpeedDown", new KeyboardShortcut(KeyCode.Minus), "Lower the autothrottle speed target. Shift = 10x.");
            KeyCoarseStep = cfg.Bind(K, "CoarseStepModifier", KeyCode.LeftShift, "Hold with any rocker key for a 10x step. LeftShift or RightShift accepts either Shift key.");

            const string V = "Tuning.Vertical";
            AltitudeGain = cfg.Bind(V, "AltitudeGain", 0.3f, "Altitude error (m) -> climb rate (m/s) close to the target (the final round-out). Softened automatically below 150 m/s. Lower = gentler, slower captures.");
            AltitudeCaptureAccel = cfg.Bind(V, "AltitudeCaptureAccel", 2.5f, "Vertical deceleration (m/s^2) used to round out onto a new altitude. Higher = keeps climbing/descending at full rate longer before levelling off.");
            MaxClimbRate = cfg.Bind(V, "MaxClimbRate", 40f, "Largest climb/descent rate the altitude modes will command (m/s). 40 m/s ~= 7900 fpm.");
            MaxFlightPathAngle = cfg.Bind(V, "MaxFlightPathAngle", 20f, "Largest climb/dive angle the autopilot will command (deg).");
            FlightPathGain = cfg.Bind(V, "FlightPathGain", 2.0f, "Flight-path-angle error (rad) -> pitch rate (rad/s), scheduled with speed. Raise for tighter height-keeping; lower if the jet porpoises.");
            FlightPathDamping = cfg.Bind(V, "FlightPathDamping", 2.5f, "Flight-path-angle RATE (rad/s) -> pitch rate (rad/s). The anti-porpoise term: raise it if altitude hold bobs, especially at low speed.");
            MaxPullG = cfg.Bind(V, "MaxPullG", 1.5f, "Extra g above the 1g baseline the autopilot may pull for pitch-up corrections.");
            MaxPushG = cfg.Bind(V, "MaxPushG", 1.0f, "g below the 1g baseline the autopilot may push for pitch-down corrections.");
            PitchRateGain = cfg.Bind(V, "PitchRateGain", 0.6f, "Proportional feedback on pitch-rate error, on top of the feedforward. Lower if the nose buzzes.");
            PitchRateIntegralFbw = cfg.Bind(V, "PitchRateIntegralFbw", 0.25f, "Pitch-rate integral gain on jets WITH fly-by-wire (the FBW already integrates, so keep this small).");
            PitchRateIntegralNoFbw = cfg.Bind(V, "PitchRateIntegralNoFbw", 1.5f, "Pitch-rate integral gain on aircraft WITHOUT fly-by-wire (this is what trims out the 1g stick force).");
            GroundFloor = cfg.Bind(V, "GroundFloorMeters", 1.5f, "Minimum clearance between the airframe's LOWEST point (wingtips, nose, tail, gear) and the water/ground. In every A/P mode the sink rate is limited near it, and bank is limited so a wingtip can't dip below it.");
            FloorGain = cfg.Bind(V, "FloorGain", 1.0f, "How hard the floor pushes back: allowed sink rate (m/s) per metre of clearance above the floor. Below the floor it commands a climb at the same rate.");

            const string F = "TerrainFollowing";
            TfLookAhead = cfg.Bind(F, "LookAheadSeconds", 15f, new ConfigDescription("How far ahead TF scans the terrain, in seconds of flight (never less than 1 km or more than 8 km). Longer sees tall terrain sooner.", new AcceptableValueRange<float>(5f, 30f)));
            TfPullG = cfg.Bind(F, "PullUpG", 3.0f, "TF pull-up limit: extra g above 1g (3.0 = 4 g on the G meter). Not capped by MaxPullG, but never more than 80% of what the airframe's fly-by-wire gives at the current speed. Higher = hugs the terrain closer and starts climbs later.");
            TfPushG = cfg.Bind(F, "PushOverG", 3.0f, "TF push-over limit across crests and into valleys: g below 1g (1.0 = weightless, 3.0 = -2 g on the G meter). Not capped by MaxPushG; same airframe cap as PullUpG. Higher = follows downslopes closer.");
            TfMinClearance = cfg.Bind(F, "MinClearanceMeters", 2f, "Lowest clearance target TF accepts (m), never less than 2. The GroundFloorMeters floor still keeps the airframe's lowest point clear. The terrain scan is a row of points, not a solid picture, and TF can dip several metres below the target over sharp crests, so a low target is a real risk.");
            TfSmoothing = cfg.Bind(F, "Smoothing", 0.4f, new ConfigDescription("Seconds over which TF eases into a change of flight path. Higher = calmer ride with fewer sharp corrections, but it follows the terrain less tightly and dips further below the target over crests. 0 = off.", new AcceptableValueRange<float>(0f, 2f)));

            const string L = "Tuning.Lateral";
            HeadingGain = cfg.Bind(L, "HeadingGain", 0.35f, "Heading error (deg) -> commanded turn rate (deg/s).");
            MaxTurnRate = cfg.Bind(L, "MaxTurnRate", 3f, "Largest turn rate HDG mode will command (deg/s). 3 deg/s = standard-rate turn.");
            NavFlyOver = cfg.Bind(L, "NavFlyOverWaypoints", false, "NAV turn style. false = fly-by: smooth arcs that cut each corner, within MaxTurnRate/MaxBank. true = fly-over: pass over each waypoint, then turn as hard as the g budget allows (ignoring MaxTurnRate/MaxBank; climb and TF still get the g first) straight at the next one and fly to it wings level. More time wings level = more pull left for terrain following.");
            MaxBank = cfg.Bind(L, "MaxBank", 35f, "Largest bank angle the autopilot will command (deg).");
            BankGain = cfg.Bind(L, "BankGain", 1.2f, "Bank error -> roll rate (1/s). Lower if the wings rock back and forth.");
            MaxRollRate = cfg.Bind(L, "MaxRollRate", 25f, "Largest roll rate the autopilot will command (deg/s). Jets never get more than 80% of the airframe's own full-stick roll rate, so a high value means 'as fast as the jet allows'.");
            RollRateGain = cfg.Bind(L, "RollRateGain", 0.5f, "Proportional feedback on roll-rate error.");
            RollRateIntegral = cfg.Bind(L, "RollRateIntegral", 0.6f, "Roll-rate integral gain (trims out asymmetric loads).");

            const string T = "Tuning.Autothrottle";
            SpeedGain = cfg.Bind(T, "SpeedGain", 0.04f, "Throttle per m/s of speed error.");
            SpeedIntegral = cfg.Bind(T, "SpeedIntegral", 0.012f, "Throttle integral per (m/s * s) of speed error.");
            SpeedDamping = cfg.Bind(T, "SpeedDamping", 0.06f, "Throttle per m/s^2 of acceleration (damps hunting).");
            AtMinThrottle = cfg.Bind(T, "MinThrottle", 0.05f, "Lowest throttle the autothrottle will set.");
            AtMaxThrottle = cfg.Bind(T, "MaxThrottle", 1.0f, "Highest throttle the autothrottle will set. Lower it to keep the A/T out of afterburner.");

            const string S = "Safety";
            StickOverride = cfg.Bind(S, "StickOverrideThreshold", 0.35f, "Pitch/roll stick deflection (0-1) that disconnects the autopilot, like a paddle switch. 1.01 disables stick override.");
            MinEngageSpeed = cfg.Bind(S, "MinEngageSpeed", 50f, "Autopilot will not engage below this speed (m/s).");
            MinEngageRadarAlt = cfg.Bind(S, "MinEngageRadarAlt", 2f, "Autopilot will not engage below this radar altitude (m).");
            DisconnectTone = cfg.Bind(S, "DisconnectTone", true, "Play a warble tone whenever the autopilot disconnects.");

            const string D = "Display";
            MfdVisible = cfg.Bind(D, "Visible", true, "Show the A/P MFD page when flying.");
            MfdSize = cfg.Bind(D, "Size", 0.40f, "MFD size as a fraction of screen height.");
            MfdX = cfg.Bind(D, "PositionX", 0.985f, "MFD horizontal position (0 = left edge, 1 = right edge). Updated when you drag it.");
            MfdY = cfg.Bind(D, "PositionY", 0.97f, "MFD vertical position (0 = top, 1 = bottom). Updated when you drag it.");
            RadarSize = cfg.Bind(D, "RadarPageSize", 0.40f, "Radar-coverage display size as a fraction of screen height.");
            RadarX = cfg.Bind(D, "RadarPagePositionX", 0.015f, "Radar-coverage display horizontal position (0 = left edge, 1 = right edge).");
            RadarY = cfg.Bind(D, "RadarPagePositionY", 0.97f, "Radar-coverage display vertical position (0 = top, 1 = bottom).");
            RadarTargetAgl = cfg.Bind(D, "RadarTargetHeight", 150f, "Height above the ground (m) of the target the coverage map is drawn for: 5, 30, 150, 500 or 2000, or -1 = ALT: your own altitude above sea level. The TGT button cycles it.");
            RadarEnemyRcs = cfg.Bind(D, "RadarEnemyRcs", 0f, new ConfigDescription("Radar cross section the ENMY coverage map is drawn for. 0 = your own aircraft's current RCS (it grows with external stores). The RCS buttons and the web mirror set it.", new AcceptableValueRange<float>(0f, 1000f)));
            RadarOpacity = cfg.Bind(D, "RadarCoverageOpacity", 0.5f, new ConfigDescription("Opacity of the coverage colours on the radar map (0 = invisible, 1 = solid). The web mirror's slider sets it.", new AcceptableValueRange<float>(0f, 1f)));
            RadarFriendlyRcs = cfg.Bind(D, "RadarFriendlyRcs", 1f, new ConfigDescription("Radar cross section the ALL/OWN coverage maps are drawn for (e.g. a stealthy enemy's). 0 = your own aircraft's current RCS. The RCS buttons and the web mirror set it.", new AcceptableValueRange<float>(0f, 1000f)));
            RadarEyes = cfg.Bind(D, "RadarShowVisual", true, "Radar map: also show where units spot the target by eye (EYES button).");
            RadarSams = cfg.Bind(D, "RadarShowSamRings", true, "Radar map: draw anti-air weapon engagement rings (SAM button).");
            GunHeatSize = cfg.Bind(D, "GunHeatSize", 0.22f, "Gun-heat display size as a fraction of screen height. It shows only in aircraft whose gun can overheat.");
            GunHeatX = cfg.Bind(D, "GunHeatPositionX", 0.985f, "Gun-heat display horizontal position (0 = left edge, 1 = right edge). Updated when you drag it.");
            GunHeatY = cfg.Bind(D, "GunHeatPositionY", 0.45f, "Gun-heat display vertical position (0 = top, 1 = bottom). Updated when you drag it.");
            ShowDebug = cfg.Bind(D, "ShowDebugLine", false, "Show commanded vs actual rates and raw stick outputs on the MFD (useful when tuning).");

            const string M = "WebMirror";
            MirrorEnabled = cfg.Bind(M, "Enabled", false, "Serve the MFDs as a web page on your local network (open http://<this PC's IP>:<Port> on another computer). Anyone on the network who opens it can press the MFD buttons. Windows Firewall asks once to allow it.");
            MirrorPort = cfg.Bind(M, "Port", 8080, new ConfigDescription("TCP port for the web page.", new AcceptableValueRange<int>(1024, 65535)));
            MirrorResolution = cfg.Bind(M, "Resolution", 1024, new ConfigDescription("Size (px) of the mirrored autopilot MFD image. Capped at the game window's height.", new AcceptableValueRange<int>(256, 2048)));
            MirrorMapResolution = cfg.Bind(M, "MapResolution", 2560, new ConfigDescription("Long side (px) of the mirrored radar map, which takes the browser window's shape. Fits inside the game window.", new AcceptableValueRange<int>(256, 3840)));
            MirrorFps = cfg.Bind(M, "FramesPerSecond", 30, new ConfigDescription("Mirror update rate. Higher costs more CPU and network.", new AcceptableValueRange<int>(1, 60)));
            MirrorJpegQuality = cfg.Bind(M, "JpegQuality", 80, new ConfigDescription("JPEG quality of the mirrored images.", new AcceptableValueRange<int>(10, 100)));
            MirrorFlipVertical = cfg.Bind(M, "FlipVertical", false, "Set to true if the web page shows the MFDs upside down (depends on the graphics API).");
            MirrorMapJpeg = cfg.Bind(M, "MapAsVideo", false, "Stream the radar map as JPEG frames rendered by the game (the old way) instead of sending its data for the browser to draw. Costs much more CPU; MapResolution, FramesPerSecond and JpegQuality then apply to the map too. Reload the web page after changing it.");

            const string W = "TerrainWarning";
            GpwsEnabled = cfg.Bind(W, "Enabled", true, "\"Terrain, pull up\" ground-collision warning. Works with or without the autopilot.");
            GpwsVoice = cfg.Bind(W, "Voice", true, "Use a voice for the warning: your own recording from BepInEx\\plugins\\AutopilotMod\\sounds if present, otherwise Windows text-to-speech. If false, or if neither is available, a whoop siren plays instead.");
            GpwsVoiceVolume = cfg.Bind(W, "VoiceVolume", 100, new ConfigDescription("Voice volume (recording and text-to-speech).", new AcceptableValueRange<int>(0, 100)));
            GpwsVoiceRate = cfg.Bind(W, "VoiceRate", 3, new ConfigDescription("Text-to-speech speed (-10 slow .. 10 fast).", new AcceptableValueRange<int>(-10, 10)));
            GpwsReactionTime = cfg.Bind(W, "ReactionTime", 1.5f, "Seconds of warning you get before a pull-up must begin. Raise for earlier warnings.");
            GpwsPullG = cfg.Bind(W, "PullUpG", 4f, "Load factor (g) assumed for the escape pull-up. Lower = earlier warnings.");
            GpwsWithGearDown = cfg.Bind(W, "WarnWithGearDown", false, "Also warn on shallow descents with the gear down (off = no nuisance warnings on landing approaches; steep dives still warn).");

            MigrateDefaults(cfg);
        }

        /// <summary>
        /// BepInEx keeps whatever is in the .cfg, so changed defaults never reach existing installs.
        /// Bump values that still sit on an old default; anything the user tuned is left alone.
        /// </summary>
        private void MigrateDefaults(ConfigFile cfg)
        {
            ConfigEntry<int> version = cfg.Bind("Internal", "ConfigVersion", 1, "Used to update defaults between mod versions. Don't edit.");
            if (version.Value < 2)
            {
                Bump(AltitudeGain, 0.12f, 0.3f);
                Bump(FlightPathGain, 0.8f, 2.0f);
                Bump(FlightPathDamping, 1.5f, 2.5f);
                Bump(GroundFloor, 3f, 1.5f);
                version.Value = 2;
            }
            if (version.Value < 3)
            {
                Bump(TfPullG, 1.0f, 3.0f);
                Bump(TfPushG, 0.5f, 2.0f);
                version.Value = 3;
            }
            if (version.Value < 4)
            {
                Bump(TfMinClearance, 20f, 5f);
                version.Value = 4;
            }
            if (version.Value < 5)
            {
                Bump(TfPushG, 2.0f, 3.0f);
                version.Value = 5;
            }
            if (version.Value < 6)
            {
                if (MirrorMapResolution.Value == 1440) MirrorMapResolution.Value = 2560;
                version.Value = 6;
            }
            if (version.Value < 7)
            {
                Bump(TfMinClearance, 5f, 2f);
                version.Value = 7;
            }
        }

        private static void Bump(ConfigEntry<float> entry, float oldDefault, float newDefault)
        {
            if (Mathf.Abs(entry.Value - oldDefault) < 1e-4f)
                entry.Value = newDefault;
        }
    }
}
