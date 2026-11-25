using HarmonyLib;

namespace AutopilotMod
{
    /// <summary>
    /// PilotPlayerState.FixedUpdateState() does:  PlayerAxisControls();  aircraft.FilterInputs();
    /// A postfix on PlayerAxisControls therefore sees the pilot's stick (for override detection) and
    /// can replace it before the fly-by-wire runs - the same place the game's own input lands, so
    /// there is no race with other scripts and no OS/gamepad round-trip.
    /// </summary>
    [HarmonyPatch(typeof(PilotPlayerState), "PlayerAxisControls")]
    internal static class PlayerAxisControlsPatch
    {
        private static void Postfix(PilotPlayerState __instance)
        {
            AutopilotPlugin.Instance?.OnPlayerAxisControls(__instance);
        }
    }

    /// <summary>
    /// Start of every physics tick, before ScheduleJobs copies each surface's inputs into its job: the
    /// fly-by-wire's control step runs here, with the state the last physics step left.
    /// </summary>
    [HarmonyPatch(typeof(NuclearOption.Jobs.JobManager), "FixedUpdateEarly")]
    internal static class FbwPreJobsPatch
    {
        private static void Prefix() => Fbw.FbwController.Current?.PreJobs();
    }

    /// <summary>The aero job's forces are applied; the physics step comes after the MonoBehaviours' FixedUpdate.</summary>
    [HarmonyPatch(typeof(NuclearOption.Jobs.JobManager), "FixedUpdateLate")]
    internal static class FbwAfterAeroPatch
    {
        private static void Postfix() => Fbw.FbwController.Current?.AfterAero();
    }

    [HarmonyPatch(typeof(Turbojet), "FixedUpdate")]
    internal static class FbwAfterTurbojetPatch
    {
        private static void Postfix(Turbojet __instance) => Fbw.FbwController.Current?.AfterEngine(__instance);
    }

    [HarmonyPatch(typeof(Turbofan), "FixedUpdate")]
    internal static class FbwAfterTurbofanPatch
    {
        private static void Postfix(Turbofan __instance) => Fbw.FbwController.Current?.AfterEngine(__instance);
    }

    /// <summary>After the pilot's inputs are filtered, before the engines' FixedUpdate reads them.</summary>
    [HarmonyPatch(typeof(Aircraft), nameof(Aircraft.FilterInputs))]
    internal static class FbwAfterFilterPatch
    {
        private static void Postfix(Aircraft __instance) => Fbw.FbwController.Current?.AfterFilter(__instance);
    }

    /// <summary>The throttle lever is read in Update (and in FixedUpdate during auto-hover).</summary>
    [HarmonyPatch(typeof(PilotPlayerState), "PlayerThrottleAxis1Controls")]
    internal static class PlayerThrottlePatch
    {
        private static void Postfix(PilotPlayerState __instance)
        {
            AutopilotPlugin.Instance?.OnPlayerThrottle(__instance);
        }
    }
}
