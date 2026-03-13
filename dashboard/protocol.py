"""
protocol.py — Telemetry CSV parser and command protocol encoder/decoder.

Telemetry lines start with digits (timestamp), commands start with '$',
headers start with '#'.

CSV telemetry format:
  No IMU (8 fields): ms, s0, s1, s2, s3, steer, speed, target
  With IMU (10 fields): ms, s0, s1, s2, s3, steer, speed, target, yaw, heading

Command protocol:
  $PING        → $PONG
  $GET         → $CFG:FOD=1200,SOD=1000,...
  $SET:KP=5.0  → $ACK  (or $NAK:reason)
  $SAVE        → $ACK
  $LOAD        → $ACK  (or $NAK:no_saved_config)
  $RST         → $ACK  (reset to compile-time defaults)
"""

from dataclasses import dataclass, field
from typing import Optional

from car_config import FLOAT_KEYS


@dataclass
class TelemetryFrame:
    """One parsed telemetry line."""
    ms: int = 0
    s0: int = 0
    s1: int = 0
    s2: int = 0
    s3: int = 0
    steer: int = 0
    speed: float = 0.0
    target: float = 0.0
    yaw: Optional[float] = None
    heading: Optional[float] = None
    has_imu: bool = False


def parse_telemetry(line: str) -> Optional[TelemetryFrame]:
    """Parse a CSV telemetry line. Returns None if unparseable."""
    line = line.strip()
    if not line or not line[0].isdigit():
        return None
    parts = line.split(",")
    if len(parts) < 8:
        return None
    try:
        f = TelemetryFrame(
            ms=int(parts[0]),
            s0=int(parts[1]),
            s1=int(parts[2]),
            s2=int(parts[3]),
            s3=int(parts[4]),
            steer=int(parts[5]),
            speed=float(parts[6]),
            target=float(parts[7]),
        )
        if len(parts) >= 10:
            f.yaw = float(parts[8])
            f.heading = float(parts[9])
            f.has_imu = True
        return f
    except (ValueError, IndexError):
        return None


def parse_header(line: str) -> Optional[list]:
    """Parse a '#'-prefixed header line into column names."""
    line = line.strip()
    if not line.startswith("#"):
        return None
    return [c.strip() for c in line[1:].split(",")]


# ─── Command encoding ────────────────────────────────────────────────────────

def encode_ping() -> str:
    return "$PING\n"

def encode_get() -> str:
    return "$GET\n"

def encode_set(params: dict) -> str:
    """Encode a $SET command. params is {key: value}."""
    pairs = ",".join(f"{k}={v}" for k, v in params.items())
    return f"$SET:{pairs}\n"

def encode_save() -> str:
    return "$SAVE\n"

def encode_load() -> str:
    return "$LOAD\n"

def encode_reset() -> str:
    return "$RST\n"


# ─── Response parsing ────────────────────────────────────────────────────────

def parse_response(line: str) -> dict:
    """
    Parse a firmware response line.
    Returns dict with 'type' key: 'pong', 'cfg', 'ack', 'nak', or None.
    """
    line = line.strip()
    if not line.startswith("$"):
        return {"type": None, "raw": line}

    if line == "$PONG":
        return {"type": "pong"}
    elif line == "$ACK":
        return {"type": "ack"}
    elif line.startswith("$NAK:"):
        return {"type": "nak", "reason": line[5:]}
    elif line.startswith("$CFG:"):
        params = {}
        for pair in line[5:].split(","):
            if "=" in pair:
                k, v = pair.split("=", 1)
                k = k.strip()
                try:
                    if k in FLOAT_KEYS:
                        params[k] = float(v)
                    else:
                        params[k] = int(v)
                except ValueError:
                    params[k] = v
        return {"type": "cfg", "params": params}
    else:
        return {"type": None, "raw": line}
