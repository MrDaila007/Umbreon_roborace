"""
tcp_client.py — TCP client for Umbreon WiFi bridge, adapted for ROS2.

Background thread reads lines, dispatches telemetry frames and command
responses to separate queues.
"""

import socket
import threading
import queue
import time
from dataclasses import dataclass, field
from typing import Optional

from umbreon_bridge.protocol import parse_telemetry, parse_response, TelemetryFrame


class TcpClient:
    """Manages TCP connection to the Umbreon car / simulator bridge."""

    def __init__(self):
        self.telemetry_queue: queue.Queue = queue.Queue(maxsize=200)
        self.response_queue: queue.Queue = queue.Queue(maxsize=50)
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self, host: str, port: int, timeout: float = 3.0):
        """Connect to the car. Raises socket errors on failure."""
        if self._connected:
            self.disconnect()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(timeout)
        self._sock.connect((host, port))
        self._sock.settimeout(0.1)
        self._connected = True
        self._running = True
        self._thread = threading.Thread(target=self._io_loop, daemon=True)
        self._thread.start()

    def disconnect(self):
        """Disconnect and stop background thread."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._connected = False

    def send_command(self, cmd: str):
        """Send a command string immediately. Must end with \\n."""
        if not self._connected or not self._sock:
            return
        if not cmd.endswith("\n"):
            cmd += "\n"
        try:
            self._sock.sendall(cmd.encode("ascii", errors="replace"))
        except OSError:
            self._connected = False
            self._running = False

    def _io_loop(self):
        """Background thread: read lines, dispatch to queues."""
        buf = b""
        while self._running:
            try:
                data = self._sock.recv(4096)
                if not data:
                    self._connected = False
                    self._running = False
                    break
                buf += data
            except socket.timeout:
                pass
            except OSError:
                self._connected = False
                self._running = False
                break

            while b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                if line[0].isdigit():
                    frame = parse_telemetry(line)
                    if frame:
                        # Drop oldest if full
                        if self.telemetry_queue.full():
                            try:
                                self.telemetry_queue.get_nowait()
                            except queue.Empty:
                                pass
                        self.telemetry_queue.put(frame)
                elif line[0] == "$":
                    resp = parse_response(line)
                    if self.response_queue.full():
                        try:
                            self.response_queue.get_nowait()
                        except queue.Empty:
                            pass
                    self.response_queue.put(resp)
