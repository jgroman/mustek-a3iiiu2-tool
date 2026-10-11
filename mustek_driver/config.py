import configparser

DEFAULTS = {
    "timing": {
        "AFE_ADCCLK_Timing": 4042321920,
        "AFE_ADCVS_Timing": 66846720,
        "AFE_ADCRS_Timing": 66846720,
        "AFE_ChannelA_LatchPos": 3080,
        "AFE_ChannelB_LatchPos": 520,
        "AFE_ChannelC_LatchPos": 3088,
        "AFE_ChannelD_LatchPos": 5120,
        "AFE_Secondary_FF_LatchPos": 10,
        "CCD_DummyCycleTiming": 4294901760,
        "CCD_PH1_Timing": 268369920,
        "PHTG_PluseWidth": 2,
        "PHTG_WaitWidth": 1,
        "PHTG_TimingAdj": 0,
        "PHTG_TimingSetup": 6,
    },
    "current": {
        "50_A": 100, "50_B": 100,
        "75_A": 90, "75_B": 90,
        "150_A": 80, "150_B": 80,
        "300_A": 70, "300_B": 70,
        "600_A": 45, "600_B": 45,
        "1200_A": 40, "1200_B": 40,
        "2400_A": 60, "2400_B": 60,
    },
    "setting": {
        "ModelName": "ScanExpress A3 USB 600/1200 Pro",
        "BackSpeed": 2631,
        "PID_SCANNER": "pid_040b",
        "VID_SCANNER": "vid_055f",
        "Gap_Blur": 1,
        "xStart": 88,
    },
    "fastscan": {"fast_scan": 0},
}


def load_ini(path):
    """Parse an INI file, returning DEFAULTS merged with overrides."""
    merged = {k: dict(v) for k, v in DEFAULTS.items()}
    cp = configparser.ConfigParser()
    cp.optionxform = str
    cp.read(path)
    section_map = {
        "A3IIIU2_TIMING": "timing",
        "A3IIIU2_CURRENT": "current",
        "A3IIIU2_SETTING": "setting",
        "A3IIIU2_FASTSCAN": "fastscan",
    }
    for ini_section, key in section_map.items():
        if not cp.has_section(ini_section):
            continue
        for name, value in cp.items(ini_section):
            if name in merged[key]:
                if isinstance(merged[key][name], int):
                    merged[key][name] = int(value)
                else:
                    merged[key][name] = value
    return merged
