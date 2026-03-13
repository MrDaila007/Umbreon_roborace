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
    return data["params"]


def merge_with_defaults(params: dict) -> dict:
    """Return a complete settings dict, filling missing keys from DEFAULTS."""
    merged = dict(DEFAULTS)
    merged.update(params)
    return merged
