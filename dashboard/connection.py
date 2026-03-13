"""
connection.py — TCP client for Umbreon WiFi telemetry bridge (DT-06).

Background thread reads lines from 192.168.4.1:23, puts parsed frames
on rx_queue. Main thread puts commands on tx_queue.
Handles partial recv buffering.
"""

import socket
import threading
import queue
from typing import Optional

from protocol import parse_telemetry, parse_response, TelemetryFrame

DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 23


class Connection:
    """Manages TCP connection to the Umbreon WiFi bridge."""

    def __init__(self):
        self.host: str = DEFAULT_HOST
        self.port: int = DEFAULT_PORT
        self.rx_queue: queue.Queue = queue.Queue()   # TelemetryFrame or response dict
        self.tx_queue: queue.Queue = queue.Queue()   # command strings
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self, host: str = None, port: int = None, timeout: float = 3.0):
        """Connect to the car. Raises socket errors on failure."""
        if self._connected:
            self.disconnect()
        if host:
            self.host = host
        if port:
            self.port = port

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(timeout)
        self._sock.connect((self.host, self.port))
        self._sock.settimeout(0.1)  # non-blocking-ish for recv
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
        """Queue a command string for sending. Must end with \\n."""
        if not cmd.endswith("\n"):
            cmd += "\n"
        self.tx_queue.put(cmd)

    def _io_loop(self):
        """Background thread: read lines, send queued commands."""
        buf = b""
        while self._running:
            # ── Send queued commands ─────────────────────────────────────
            while not self.tx_queue.empty():
                try:
                    cmd = self.tx_queue.get_nowait()
                    self._sock.sendall(cmd.encode("ascii", errors="replace"))
                except (OSError, queue.Empty):
                    break

            # ── Receive data ─────────────────────────────────────────────
            try:
                data = self._sock.recv(4096)
                if not data:
                    # Connection closed by remote
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

            # ── Process complete lines ───────────────────────────────────
            while b"\n" in buf:
                line_bytes, buf = buf.split(b"\n", 1)
                line = line_bytes.decode("ascii", errors="replace").strip()
                if not line:
                    continue

                # Try telemetry first (starts with digit)
                if line[0].isdigit():
                    frame = parse_telemetry(line)
                    if frame:
                        self.rx_queue.put(frame)
                # Then try command response (starts with $)
                elif line[0] == "$":
                    resp = parse_response(line)
                    self.rx_queue.put(resp)
                # Header lines (#) — ignore silently
                # else: unknown — drop
