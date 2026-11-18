using System;
using System.Reflection;
using BepInEx.Logging;
using HarmonyLib;
using UnityEngine;

namespace AutopilotMod
{
    /// <summary>
    /// Tiny buttonless MFD for guns the game models heat on (Gun.heatEnabled, e.g. the Tarantula's
    /// cannon). The game's rules (Gun.Heat):
    ///   each shot: heat += heatPerShot (and above 2x maxHeat each shot adds even more: runaway + barrel damage FX)
    ///   cooling:   heat -= (coolingPerSecond + coolingPer100kph * speed_kph / 100) * dt, so flying faster cools faster
    ///   above maxHeat: rate of fire, accuracy and muzzle velocity drop in proportion to heat/maxHeat - 1.
    /// Hidden when the aircraft has no such gun. Drag the bezel to move it, the corner to resize.
    /// </summary>
    internal sealed class GunHeatPage : MfdFrame
    {
        private static readonly Color Red = new Color(1f, 0.25f, 0.2f, 1f);

        // Gun.Heat is a private nested class: read it by reflection.
        private static readonly FieldInfo HeatEnabledF = AccessTools.Field(typeof(Gun), "heatEnabled");
        private static readonly FieldInfo HeatObjF = AccessTools.Field(typeof(Gun), "heat");
        private static readonly Type HeatT = AccessTools.Inner(typeof(Gun), "Heat");
        private static readonly FieldInfo HeatF = AccessTools.Field(HeatT, "heat");
        private static readonly FieldInfo MaxF = AccessTools.Field(HeatT, "maxHeat");
        private static readonly FieldInfo PerShotF = AccessTools.Field(HeatT, "heatPerShot");
        private static readonly FieldInfo CoolF = AccessTools.Field(HeatT, "coolingPerSecond");
        private static readonly FieldInfo CoolSpdF = AccessTools.Field(HeatT, "coolingPer100kph");

        private readonly ManualLogSource log;
        private string logged;

        // hottest heat-modelled gun on the current aircraft, refreshed by Tick()
        private bool has;
        private float heat, max, perShot, cool, coolSpd, speed;

        public GunHeatPage(ApSettings cfg, ManualLogSource log) : base(cfg.GunHeatSize, cfg.GunHeatX, cfg.GunHeatY)
        {
            this.log = log;
        }

        public bool Visible => has;

        protected override Osb[] Buttons => Array.Empty<Osb>();

        public void Tick(Aircraft ac)
        {
            has = false;
            if (ac == null || ac.weaponStations == null) return;
            speed = ac.speed;
            foreach (WeaponStation st in ac.weaponStations)
            foreach (Weapon w in st.Weapons)
            {
                if (!(w is Gun g) || !(bool)HeatEnabledF.GetValue(g)) continue;
                object h = HeatObjF.GetValue(g);
                float m = (float)MaxF.GetValue(h), x = (float)HeatF.GetValue(h);
                if (m <= 0f || (has && x / m <= heat / max)) continue;
                has = true;
                heat = x;
                max = m;
                perShot = (float)PerShotF.GetValue(h);
                cool = (float)CoolF.GetValue(h);
                coolSpd = (float)CoolSpdF.GetValue(h);
            }
            if (has && logged != ac.definition.unitName)
            {
                logged = ac.definition.unitName;
                log.LogInfo($"{logged} gun heat: max {max}, +{perShot}/shot, cools {cool}/s + {coolSpd} per 100 km/h /s");
            }
        }

        protected override void DrawScreen()
        {
            if (!has) // the web mirror renders it out of the jet too
            {
                Text(Sc(0.05f, 0.4f, 0.9f, 0.2f), "NO GUN HEAT", 0.07f, GreenDim, TextAnchor.MiddleCenter, false);
                return;
            }
            float n = heat / max;
            // the game's cooling rate, in heat units per second
            float rate = cool + coolSpd * speed * 0.036f;
            Color c = n > 2f ? Red : n > 1f ? Amber : Green;
            bool blink = n > 2f && Mathf.Repeat(Time.unscaledTime, 0.5f) > 0.3f;

            Text(Sc(0.05f, 0.04f, 0.9f, 0.12f), "GUN HEAT", 0.07f, GreenDim, TextAnchor.MiddleCenter, false);

            // bar 0..200% of maxHeat; penalties start at 100%, barrel damage at 200%
            Rect bar = Sc(0.08f, 0.2f, 0.84f, 0.17f);
            Border(bar, GreenDim, Mathf.Max(1f, 0.006f * S), 0f);
            Rect fill = new Rect(bar.x + 2f, bar.y + 2f, (bar.width - 4f) * Mathf.Clamp01(n / 2f), bar.height - 4f);
            if (!blink) Fill(fill, c, 0f);
            float mid = bar.center.x;
            Line(new Vector2(mid, bar.y - 0.03f * S), new Vector2(mid, bar.yMax + 0.03f * S), Mathf.Max(1f, 0.008f * S), Amber);

            Text(Sc(0.05f, 0.4f, 0.9f, 0.2f), $"{n * 100f:0}%", 0.14f, c, TextAnchor.MiddleCenter);
            string state = n > 2f ? "BARREL DMG" : n > 1f ? "HOT - ROF/ACC LOSS" : "OK";
            Text(Sc(0.05f, 0.6f, 0.9f, 0.1f), state, 0.06f, c, TextAnchor.MiddleCenter);

            // rounds before it gets hot (ignores cooling during the burst), and cooling
            string left = n < 1f && perShot > 0f ? $"{Mathf.FloorToInt((max - heat) / perShot)} RDS" : "-- RDS";
            Text(Sc(0.06f, 0.73f, 0.44f, 0.1f), left, 0.06f, Green, TextAnchor.MiddleLeft);
            Text(Sc(0.5f, 0.73f, 0.44f, 0.1f), $"-{rate / max * 100f:0}%/S", 0.06f, Green, TextAnchor.MiddleRight);
            if (n > 1f && rate > 0f)
                Text(Sc(0.05f, 0.85f, 0.9f, 0.1f), $"OK IN {(heat - max) / rate:0.0}S", 0.06f, Amber, TextAnchor.MiddleCenter);
        }
    }
}
