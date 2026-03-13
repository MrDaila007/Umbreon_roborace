#!/usr/bin/env python3
"""
app.py — Umbreon Dashboard: live telemetry, track map, and remote settings.

    cd dashboard && python app.py

Connect to Umbreon WiFi → 192.168.4.1:23 → live plots + map.
"""

import sys
import os
import queue
import tkinter as tk
from tkinter import ttk, messagebox

# Ensure dashboard package is on path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from connection import Connection
from protocol import TelemetryFrame
from telemetry import TelemetryStore
from plots import TelemetryPlots
from track_map import TrackMap
from settings_panel import SettingsPanel

POLL_MS = 20          # how often to drain rx_queue (ms)
PLOT_MS = 100         # plot refresh interval (ms)
MAP_MS = 200          # map redraw interval (ms)


class UmbreonDashboard:
    """Main application window."""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Umbreon Dashboard")
        self.root.geometry("1280x800")
        self.root.minsize(800, 500)

        self.conn = Connection()
        self.store = TelemetryStore()

        self._recording = False
        self._rec_file = None
        self._frame_count = 0

        self._build_ui()
        self._schedule_polling()

    def _build_ui(self):
        """Build the main window layout."""
        # ── Toolbar ─────────────────────────────────────────────────────
        toolbar = ttk.Frame(self.root)
        toolbar.pack(fill="x", padx=4, pady=2)

        ttk.Label(toolbar, text="Host:").pack(side="left")
        self._host_var = tk.StringVar(value="192.168.4.1")
        ttk.Entry(toolbar, textvariable=self._host_var,
                   width=14).pack(side="left", padx=2)

        ttk.Label(toolbar, text="Port:").pack(side="left")
        self._port_var = tk.StringVar(value="23")
        ttk.Entry(toolbar, textvariable=self._port_var,
                   width=5).pack(side="left", padx=2)

        self._connect_btn = ttk.Button(toolbar, text="Connect",
                                         command=self._toggle_connect)
        self._connect_btn.pack(side="left", padx=4)

        self._status_label = ttk.Label(toolbar, text="Disconnected",
                                         foreground="red")
        self._status_label.pack(side="left", padx=4)

        ttk.Separator(toolbar, orient="vertical").pack(side="left",
                                                        fill="y", padx=4)

        self._rec_btn = ttk.Button(toolbar, text="Record",
                                     command=self._toggle_record)
        self._rec_btn.pack(side="left", padx=4)

        self._ping_btn = ttk.Button(toolbar, text="Ping",
                                      command=self._send_ping)
        self._ping_btn.pack(side="left", padx=2)

        self._count_label = ttk.Label(toolbar, text="Frames: 0")
        self._count_label.pack(side="right", padx=4)

        # ── Main paned layout ───────────────────────────────────────────
        # Vertical: top (plots+settings) | bottom (map)
        vpane = ttk.PanedWindow(self.root, orient="vertical")
        vpane.pack(fill="both", expand=True, padx=2, pady=2)

        # Top: horizontal pane — left (plots) | right (settings)
        hpane = ttk.PanedWindow(vpane, orient="horizontal")
        vpane.add(hpane, weight=3)

        # Left pane: plots
        plot_frame = ttk.Frame(hpane)
        hpane.add(plot_frame, weight=3)
        self.plots = TelemetryPlots(plot_frame, self.store)
        self.plots.widget.pack(fill="both", expand=True)

        # Right pane: settings
        settings_frame = ttk.Frame(hpane)
        hpane.add(settings_frame, weight=1)
        self.settings = SettingsPanel(settings_frame, self.conn)
        self.settings.frame.pack(fill="both", expand=True)

        # Bottom pane: track map
        map_frame = ttk.LabelFrame(vpane, text="Track Map")
        vpane.add(map_frame, weight=2)
        self.track_map = TrackMap(map_frame)
        self.track_map.canvas.pack(fill="both", expand=True)

    def _schedule_polling(self):
        """Set up periodic callbacks."""
        self.root.after(POLL_MS, self._poll_rx)
        self.root.after(PLOT_MS, self._update_plots)
        self.root.after(MAP_MS, self._update_map)

    def _poll_rx(self):
        """Drain rx_queue: telemetry frames → store, responses → handle."""
        try:
            for _ in range(200):  # process up to 200 items per tick
                try:
                    item = self.conn.rx_queue.get_nowait()
                except queue.Empty:
                    break

                if isinstance(item, TelemetryFrame):
                    self.store.push(item)
                    self.track_map.push_frame(item)
                    self._frame_count += 1
                    if self._recording and self._rec_file:
                        self._write_record(item)
                elif isinstance(item, dict):
                    self._handle_response(item)
        finally:
            # Update status
            if self.conn.connected:
                self._status_label.configure(text="Connected", foreground="green")
                self._connect_btn.configure(text="Disconnect")
            else:
                self._status_label.configure(text="Disconnected", foreground="red")
                self._connect_btn.configure(text="Connect")
            self._count_label.configure(text=f"Frames: {self._frame_count}")
            self.root.after(POLL_MS, self._poll_rx)

    def _update_plots(self):
        """Refresh plots at lower frequency."""
        self.plots.update()
        self.root.after(PLOT_MS, self._update_plots)

    def _update_map(self):
        """Redraw track map."""
        self.track_map.redraw()
        self.root.after(MAP_MS, self._update_map)

    def _handle_response(self, resp: dict):
        """Handle a parsed command response."""
        rtype = resp.get("type")
        if rtype == "pong":
            self._status_label.configure(text="Connected (PONG)",
                                          foreground="green")
        elif rtype == "cfg":
            self.settings.handle_cfg_response(resp["params"])
        elif rtype == "ack":
            pass  # silent success
        elif rtype == "nak":
            messagebox.showwarning("NAK from car", resp.get("reason", "unknown"))

    # ── Toolbar actions ──────────────────────────────────────────────────

    def _toggle_connect(self):
        if self.conn.connected:
            self.conn.disconnect()
        else:
            host = self._host_var.get().strip()
            try:
                port = int(self._port_var.get().strip())
            except ValueError:
                port = 23
            try:
                self.conn.connect(host, port)
            except Exception as e:
                messagebox.showerror("Connection Error", str(e))

    def _send_ping(self):
        from protocol import encode_ping
        if self.conn.connected:
            self.conn.send_command(encode_ping())

    def _toggle_record(self):
        if not self._recording:
            import datetime
            fname = datetime.datetime.now().strftime("umbreon_%Y%m%d_%H%M%S.csv")
            try:
                self._rec_file = open(fname, "w")
                self._rec_file.write("#ms,s0,s1,s2,s3,steer,speed,target,yaw,heading\n")
                self._recording = True
                self._rec_btn.configure(text="Stop Rec")
            except OSError as e:
                messagebox.showerror("Record Error", str(e))
        else:
            self._recording = False
            self._rec_btn.configure(text="Record")
            if self._rec_file:
                self._rec_file.close()
                self._rec_file = None

    def _write_record(self, frame: TelemetryFrame):
        yaw = frame.yaw if frame.yaw is not None else ""
        heading = frame.heading if frame.heading is not None else ""
        self._rec_file.write(
            f"{frame.ms},{frame.s0},{frame.s1},{frame.s2},{frame.s3},"
            f"{frame.steer},{frame.speed:.2f},{frame.target:.1f},"
            f"{yaw},{heading}\n"
        )

    def run(self):
        """Start the main event loop."""
        try:
            self.root.mainloop()
        finally:
            self.conn.disconnect()
            if self._rec_file:
                self._rec_file.close()


def main():
    app = UmbreonDashboard()
    app.run()


if __name__ == "__main__":
    main()
