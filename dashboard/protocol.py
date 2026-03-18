"""
protocol.py — Telemetry CSV parser and command protocol encoder/decoder.

Telemetry lines start with digits (timestamp), commands start with '$',
headers start with '#'.

CSV telemetry format (dynamic sensor count):
  4 sensors, no IMU  (8 fields):  ms, s0, s1, s2, s3, steer, speed, target
  4 sensors, with IMU (10 fields): ms, s0, s1, s2, s3, steer, speed, target, yaw, heading
  6 sensors, no IMU  (10 fields): ms, s0, s1, s2, s3, s4, s5, steer, speed, target
  6 sensors, with IMU (12 fields): ms, s0, s1, s2, s3, s4, s5, steer, speed, target, yaw, heading

Command protocol:
  $PING        → $PONG
  $GET         → $CFG:FOD=1200,SOD=1000,...
  $SET:KP=5.0  → $ACK  (or $NAK:reason)
  $SAVE        → $ACK
  $LOAD        → $ACK  (or $NAK:no_saved_config)
  $RST         → $ACK  (reset to compile-time defaults)
"""

import threading
from dataclasses import dataclass, field
from typing import Optional, List

from car_config import FLOAT_KEYS


@dataclass
class TelemetryFrame:
    """One parsed telemetry line."""
    ms: int = 0
    sensors: List[int] = field(default_factory=lambda: [0, 0, 0, 0])
    steer: int = 0
    speed: float = 0.0
    target: float = 0.0
    yaw: Optional[float] = None
    heading: Optional[float] = None
    has_imu: bool = False
    sensor_count: int = 4

    # Backward-compatible accessors for s0–s5
    @property
    def s0(self): return self.sensors[0] if len(self.sensors) > 0 else 0
    @property
    def s1(self): return self.sensors[1] if len(self.sensors) > 1 else 0
    @property
    def s2(self): return self.sensors[2] if len(self.sensors) > 2 else 0
    @property
    def s3(self): return self.sensors[3] if len(self.sensors) > 3 else 0
    @property
    def s4(self): return self.sensors[4] if len(self.sensors) > 4 else 0
    @property
    def s5(self): return self.sensors[5] if len(self.sensors) > 5 else 0


# Module-level detected sensor count (updated by parse_header or auto-detect)
_detected_sensor_count = 0  # 0 = not yet detected
_sensor_count_lock = threading.Lock()


def set_sensor_count(n: int):
    """Explicitly set the expected sensor count (called after $GET or header parse)."""
    global _detected_sensor_count
    with _sensor_count_lock:
        _detected_sensor_count = n


def get_sensor_count() -> int:
    """Return the detected sensor count, or 4 as default."""
    with _sensor_count_lock:
        return _detected_sensor_count if _detected_sensor_count > 0 else 4


def parse_telemetry(line: str) -> Optional[TelemetryFrame]:
    """Parse a CSV telemetry line. Returns None if unparseable."""
    line = line.strip()
    if not line or not line[0].isdigit():
        return None
    parts = line.split(",")
    n_fields = len(parts)
    if n_fields < 8:
        return None

    # Determine sensor count from field count:
    # Format: ms, [sensors...], steer, speed, target [, yaw, heading]
    # So n_fields = 1 + sensor_count + 3 (no IMU) or 1 + sensor_count + 5 (IMU)
    with _sensor_count_lock:
        sc = _detected_sensor_count
    if sc > 0:
        # Use known sensor count
        has_imu = n_fields >= (1 + sc + 5)
    else:
        # Auto-detect: 12 fields → 6 sensors + IMU, 10 → 4 + IMU, 8 → 4 no IMU
        if n_fields >= 12:
            sc = 6
            has_imu = True
        elif n_fields >= 10:
            sc = 4
            has_imu = True
        else:
            sc = 4
            has_imu = False

    try:
        sensors = [int(parts[1 + i]) for i in range(sc)]
        si = 1 + sc  # index of steer field
        f = TelemetryFrame(
            ms=int(parts[0]),
            sensors=sensors,
            steer=int(parts[si]),
            speed=float(parts[si + 1]),
            target=float(parts[si + 2]),
            sensor_count=sc,
        )
        if has_imu and n_fields >= si + 5:
            f.yaw = float(parts[si + 3])
            f.heading = float(parts[si + 4])
            f.has_imu = True
        return f
    except (ValueError, IndexError):
        return None


def parse_header(line: str) -> Optional[list]:
    """Parse a '#'-prefixed header line into column names.
    Also auto-detects sensor count from sensor column names (s0, s1, ...).
    """
    line = line.strip()
    if not line.startswith("#"):
        return None
    cols = [c.strip() for c in line[1:].split(",")]
    # Count sensor columns (s0, s1, s2, ...)
    sc = sum(1 for c in cols if c.startswith("s") and c[1:].isdigit())
    if sc > 0:
        set_sensor_count(sc)
    return cols


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

def encode_start() -> str:
    return "$START\n"

def encode_stop() -> str:
    return "$STOP\n"

def encode_status() -> str:
    return "$STATUS\n"

def encode_test(name: str) -> str:
    return f"$TEST:{name}\n"

def encode_drv(steer: int, speed: float) -> str:
    """Encode a $DRV manual drive command (fire-and-forget, no response)."""
    return f"$DRV:{steer},{speed:.2f}\n"


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
        # Auto-detect sensor count from SNS parameter
        if "SNS" in params:
            set_sensor_count(int(params["SNS"]))
        return {"type": "cfg", "params": params}
    elif line.startswith("$STS:"):
        state = line[5:]
        return {"type": "car_status", "running": state == "RUN"}
    elif line.startswith("$TDONE:"):
        return {"type": "test_done", "test": line[7:]}
    elif line.startswith("$TR:"):
        parts = line[4:].split(",", 1)
        method = parts[0]
        params = {}
        if len(parts) > 1:
            for pair in parts[1].split(","):
                if "=" in pair:
                    k, v = pair.split("=", 1)
                    try:
                        params[k] = float(v)
                    except ValueError:
                        params[k] = v
        return {"type": "test_result", "method": method, "params": params}
    elif line.startswith("$T:"):
        return {"type": "test_log", "data": line[3:]}
    else:
        return {"type": None, "raw": line}
