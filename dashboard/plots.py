"""
plots.py — Live telemetry plots using matplotlib embedded in tkinter.

4 stacked subplots:
  1. LiDAR distances (s0-s3)
  2. Speed + target
  3. Steering
  4. IMU yaw rate + heading

Updates at ~10 Hz via set_ydata/draw_idle, rolling 250-sample window.
"""

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from telemetry import TelemetryStore
from car_config import SENSOR_NAMES, SENSOR_COLORS

WINDOW = 250  # samples shown on plots


class TelemetryPlots:
    """Matplotlib figure with 4 stacked subplots, embedded in a tkinter parent."""

    def __init__(self, parent, store: TelemetryStore):
        self.store = store
        self._x = np.arange(WINDOW)

        self.fig = Figure(figsize=(8, 7), dpi=90)
        self.fig.set_facecolor("#f0f0f0")
        self.fig.subplots_adjust(hspace=0.35, left=0.10, right=0.96,
                                  top=0.97, bottom=0.04)

        # ── Subplot 1: LiDAR distances ──────────────────────────────────
        self.ax_lidar = self.fig.add_subplot(4, 1, 1)
        self.ax_lidar.set_ylabel("LiDAR (cm×10)")
        self.ax_lidar.set_xlim(0, WINDOW)
        self.ax_lidar.set_ylim(0, 3000)
        self.ax_lidar.grid(True, alpha=0.3)
        zeros = np.zeros(WINDOW)
        self.lines_lidar = []
        for i, (name, color) in enumerate(zip(SENSOR_NAMES, SENSOR_COLORS)):
            ln, = self.ax_lidar.plot(self._x, zeros, color=color,
                                      linewidth=1, label=name)
            self.lines_lidar.append(ln)
        self.ax_lidar.legend(loc="upper right", fontsize=7, ncol=4)

        # ── Subplot 2: Speed + target ───────────────────────────────────
        self.ax_speed = self.fig.add_subplot(4, 1, 2)
        self.ax_speed.set_ylabel("Speed (m/s)")
        self.ax_speed.set_xlim(0, WINDOW)
        self.ax_speed.set_ylim(0, 4.0)
        self.ax_speed.grid(True, alpha=0.3)
        self.line_speed, = self.ax_speed.plot(
            self._x, zeros, color="tab:blue", linewidth=1, label="actual")
        self.line_target, = self.ax_speed.plot(
            self._x, zeros, color="tab:red", linewidth=1,
            linestyle="--", label="target")
        self.ax_speed.legend(loc="upper right", fontsize=7, ncol=2)

        # ── Subplot 3: Steering ─────────────────────────────────────────
        self.ax_steer = self.fig.add_subplot(4, 1, 3)
        self.ax_steer.set_ylabel("Steering")
        self.ax_steer.set_xlim(0, WINDOW)
        self.ax_steer.set_ylim(-800, 800)
        self.ax_steer.axhline(0, color="gray", linewidth=0.5)
        self.ax_steer.grid(True, alpha=0.3)
        self.line_steer, = self.ax_steer.plot(
            self._x, zeros, color="tab:purple", linewidth=1)

        # ── Subplot 4: IMU yaw + heading ────────────────────────────────
        self.ax_imu = self.fig.add_subplot(4, 1, 4)
        self.ax_imu.set_ylabel("IMU (°)")
        self.ax_imu.set_xlim(0, WINDOW)
        self.ax_imu.set_ylim(-200, 200)
        self.ax_imu.axhline(0, color="gray", linewidth=0.5)
        self.ax_imu.grid(True, alpha=0.3)
        self.line_yaw, = self.ax_imu.plot(
            self._x, zeros, color="tab:cyan", linewidth=1, label="yaw rate")
        self.line_heading, = self.ax_imu.plot(
            self._x, zeros, color="tab:brown", linewidth=1, label="heading")
        self.ax_imu.legend(loc="upper right", fontsize=7, ncol=2)

        # ── Embed in tkinter ────────────────────────────────────────────
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.widget = self.canvas.get_tk_widget()

    def update(self):
        """Pull latest data from store and refresh plot lines."""
        data = self.store.get_recent(WINDOW)
        n = len(data["ms"])
        if n == 0:
            return

        def _pad(arr):
            """Left-pad with zeros to WINDOW length."""
            a = np.array(arr, dtype=float)
            if len(a) < WINDOW:
                a = np.concatenate([np.zeros(WINDOW - len(a)), a])
            return a

        # LiDAR
        for i, key in enumerate(["s0", "s1", "s2", "s3"]):
            self.lines_lidar[i].set_ydata(_pad(data[key]))

        # Speed
        self.line_speed.set_ydata(_pad(data["speed"]))
        self.line_target.set_ydata(_pad(data["target"]))

        # Steering
        self.line_steer.set_ydata(_pad(data["steer"]))

        # IMU
        self.line_yaw.set_ydata(_pad(data["yaw"]))
        self.line_heading.set_ydata(_pad(data["heading"]))

        self.canvas.draw_idle()
