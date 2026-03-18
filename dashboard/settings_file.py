"""
settings_file.py — JSON load/save for PC-side settings profiles.
"""

import json
from pathlib import Path
from car_config import DEFAULTS


def save_settings(params: dict, filepath: str):
    """Save settings dict to a JSON file."""
    data = {
        "umbreon_settings": True,
        "version": 1,
        "params": params,
    }
    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)


VALIDATION_RANGES = {
    "FOD": (0, 20000), "SOD": (0, 20000), "ACD": (0, 20000), "CFD": (0, 20000),
    "KP": (0.0, 10000.0), "KI": (0.0, 10000.0), "KD": (0.0, 10000.0),
    "MSP": (1500, 2000), "XSP": (1500, 2000), "BSP": (1000, 1500),
    "MNP": (0, 180), "XNP": (0, 180), "NTP": (0, 180),
    "ENH": (1, 1000), "WDM": (0.001, 1.0),
    "LMS": (10, 1000),
    "SPD1": (0.0, 20.0), "SPD2": (0.0, 20.0),
    "COE1": (0.0, 5.0), "COE2": (0.0, 5.0),
    "WDD": (0.0, 360.0), "STK": (1, 1000),
    "BML": (0.1, 100.0), "BLV": (0.0, 50.0),
}


def load_settings(filepath: str) -> dict:
    """
    Load settings from a JSON file.
    Returns a dict of param key→value.
    Raises FileNotFoundError or ValueError on problems.
    """
    with open(filepath, "r") as f:
        data = json.load(f)
    if not isinstance(data, dict) or "params" not in data:
        raise ValueError("Invalid settings file format")
    params = data["params"]
    # Validate loaded values against safe ranges
    for key, val in params.items():
        if key in VALIDATION_RANGES:
            lo, hi = VALIDATION_RANGES[key]
            try:
                num = float(val)
                if num < lo or num > hi:
                    raise ValueError(
                        f"Parameter {key}={val} out of range [{lo}, {hi}]"
                    )
            except (TypeError, ValueError) as e:
                if "out of range" in str(e):
                    raise
    return params


def merge_with_defaults(params: dict) -> dict:
    """Return a complete settings dict, filling missing keys from DEFAULTS."""
    merged = dict(DEFAULTS)
    merged.update(params)
    return merged
