namespace AutopilotMod
{
    /// <summary>Everything the pilot can ask of the autopilot - shared by hotkeys and MFD push-buttons.</summary>
    internal enum ApCommand
    {
        None,
        ApToggle,
        AtToggle,
        ModeHdg,
        ModeAtt,
        ModeBalt,
        ModeTf,
        ModeVs,
        LatUp,
        LatDown,
        LatSync,
        VertUp,
        VertDown,
        VertSync,
        SpdUp,
        SpdDown,
        SpdSync,
        SyncAll,
        StepToggle,
        DataToggle,
        SoundToggle,
        Hide,

        // radar coverage page
        RdrAll,
        RdrOwn,
        RdrTgt,
        RdrHide,
        RdrZoomIn,
        RdrZoomOut,
        RdrFollow,
        RdrMap,
        RdrEnemy,
        RdrRcsUp,
        RdrRcsDown,
        RdrRcsOwn,
        RdrEyes,
        RdrSam,

        // navigation / routing (radar page bottom row)
        RteWpt,
        RteUndo,
        RteClear,
        RtePlan,
        NavToggle,

        // A/P MFD settings page
        SetPage,
        SetPrev,
        SetNext,
        SetValUp,
        SetValDown,
        SetDefault,
        SetResetAll,
    }
}
