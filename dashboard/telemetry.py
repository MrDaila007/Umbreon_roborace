"""
telemetry.py — Thread-safe ring buffers for all telemetry channels.
"""

import threading
from collections import deque

from protocol import TelemetryFrame

# Maximum samples kept in each ring buffer
RING_SIZE = 2500


MAX_SENSORS = 6


class TelemetryStore:
    """Thread-safe storage for telemetry data with per-channel ring buffers."""

    def __init__(self, maxlen: int = RING_SIZE):
        self._lock = threading.Lock()
        self._maxlen = maxlen
        # Per-channel deques
        self.ms      = deque(maxlen=maxlen)
        self.sensors = [deque(maxlen=maxlen) for _ in range(MAX_SENSORS)]
        self.steer   = deque(maxlen=maxlen)
        self.speed   = deque(maxlen=maxlen)
        self.target  = deque(maxlen=maxlen)
        self.yaw     = deque(maxlen=maxlen)
        self.heading = deque(maxlen=maxlen)
        self._count = 0
        self._sensor_count = 4

    def push(self, frame: TelemetryFrame):
        """Append a telemetry frame to all channel buffers."""
        with self._lock:
            self.ms.append(frame.ms)
            self._sensor_count = max(self._sensor_count, frame.sensor_count)
            for i in range(MAX_SENSORS):
                val = frame.sensors[i] if i < len(frame.sensors) else 0
                self.sensors[i].append(val)
            self.steer.append(frame.steer)
            self.speed.append(frame.speed)
            self.target.append(frame.target)
            self.yaw.append(frame.yaw if frame.yaw is not None else 0.0)
            self.heading.append(frame.heading if frame.heading is not None else 0.0)
            self._count += 1

    @property
    def sensor_count(self) -> int:
        with self._lock:
            return self._sensor_count

    def get_recent(self, n: int) -> dict:
        """
        Return the most recent n samples as a dict of lists.
        Thread-safe snapshot.
        """
        with self._lock:
            def tail(dq):
                if n >= len(dq):
                    return list(dq)
                return list(dq)[-n:]

            result = {
                "ms":      tail(self.ms),
                "steer":   tail(self.steer),
                "speed":   tail(self.speed),
                "target":  tail(self.target),
                "yaw":     tail(self.yaw),
                "heading": tail(self.heading),
            }
            # Backward-compatible s0–s5 keys
            for i in range(MAX_SENSORS):
                result[f"s{i}"] = tail(self.sensors[i])
            return result

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def clear(self):
        with self._lock:
            self.ms.clear()
            for dq in self.sensors:
                dq.clear()
            self.steer.clear()
            self.speed.clear()
            self.target.clear()
            self.yaw.clear()
            self.heading.clear()
            self._count = 0
