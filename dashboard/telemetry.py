"""
telemetry.py — Thread-safe ring buffers for all telemetry channels.
"""

import threading
from collections import deque

from protocol import TelemetryFrame

# Maximum samples kept in each ring buffer
RING_SIZE = 2500


class TelemetryStore:
    """Thread-safe storage for telemetry data with per-channel ring buffers."""

    def __init__(self, maxlen: int = RING_SIZE):
        self._lock = threading.Lock()
        self._maxlen = maxlen
        # Per-channel deques
        self.ms      = deque(maxlen=maxlen)
        self.s0      = deque(maxlen=maxlen)
        self.s1      = deque(maxlen=maxlen)
        self.s2      = deque(maxlen=maxlen)
        self.s3      = deque(maxlen=maxlen)
        self.steer   = deque(maxlen=maxlen)
        self.speed   = deque(maxlen=maxlen)
        self.target  = deque(maxlen=maxlen)
        self.yaw     = deque(maxlen=maxlen)
        self.heading = deque(maxlen=maxlen)
        self._count = 0

    def push(self, frame: TelemetryFrame):
        """Append a telemetry frame to all channel buffers."""
        with self._lock:
            self.ms.append(frame.ms)
            self.s0.append(frame.s0)
            self.s1.append(frame.s1)
            self.s2.append(frame.s2)
            self.s3.append(frame.s3)
            self.steer.append(frame.steer)
            self.speed.append(frame.speed)
            self.target.append(frame.target)
            self.yaw.append(frame.yaw if frame.yaw is not None else 0.0)
            self.heading.append(frame.heading if frame.heading is not None else 0.0)
            self._count += 1

    def get_recent(self, n: int) -> dict:
        """
        Return the most recent n samples as a dict of lists.
        Thread-safe snapshot.
        """
        with self._lock:
            def tail(dq):
                if n >= len(dq):
                    return list(dq)
                return list(dq)[-n:]  # slicing a deque isn't O(1), but n≤250 is fine

            return {
                "ms":      tail(self.ms),
                "s0":      tail(self.s0),
                "s1":      tail(self.s1),
                "s2":      tail(self.s2),
                "s3":      tail(self.s3),
                "steer":   tail(self.steer),
                "speed":   tail(self.speed),
                "target":  tail(self.target),
                "yaw":     tail(self.yaw),
                "heading": tail(self.heading),
            }

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    def clear(self):
        with self._lock:
            for dq in (self.ms, self.s0, self.s1, self.s2, self.s3,
                       self.steer, self.speed, self.target,
                       self.yaw, self.heading):
                dq.clear()
            self._count = 0
