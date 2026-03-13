"""
track_map.py — Live track map builder on a tkinter Canvas.

Dead reckoning from telemetry:
  - Heading from IMU gyro (preferred) or kinematic model
  - Position from speed × dt
  - Sensor ray endpoints → wall points (color-coded by sensor)
  - Car trail as polyline
  - Pan/zoom with mouse
  - Grid consolidation every 500 points (20mm cells)
"""

import math
import tkinter as tk

from car_config import (
    WHEELBASE, SENSOR_FWD, SENSOR_DEG, SENSOR_LAT,
    SENSOR_COLORS, MAX_LIDAR_M, MAX_STEER_RAD,
)

CELL_SIZE = 0.020   # 20mm grid cells for consolidation
CONSOLIDATE_EVERY = 500  # consolidate wall points every N new points


class TrackMap:
    """Canvas-based live map builder using dead reckoning + LiDAR."""

    def __init__(self, parent):
        self.canvas = tk.Canvas(parent, bg="black", highlightthickness=0)

        # ── Car state (dead reckoning) ───────────────────────────────────
        self.car_x = 0.0       # metres
        self.car_y = 0.0       # metres
        self.car_heading = 0.0 # radians (0 = right, CCW positive)
        self._prev_ms = None

        # ── View transform ──────────────────────────────────────────────
        self._scale = 200.0     # pixels per metre
        self._offset_x = 0.0   # canvas pixel offset
        self._offset_y = 0.0
        self._drag_start = None

        # ── Data ─────────────────────────────────────────────────────────
        self.trail = []                     # [(x, y), ...] car positions
        self.wall_points = [[] for _ in range(4)]  # per-sensor [(x, y), ...]
        self._new_points_since_consolidate = 0
        self._grid = {}   # (gx, gy, sensor_idx) → True for consolidated

        # ── Mouse bindings ───────────────────────────────────────────────
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<MouseWheel>", self._on_scroll)
        # Linux scroll
        self.canvas.bind("<Button-4>", lambda e: self._on_scroll_linux(e, 1))
        self.canvas.bind("<Button-5>", lambda e: self._on_scroll_linux(e, -1))

    def reset(self):
        """Clear all map data and reset car position."""
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_heading = 0.0
        self._prev_ms = None
        self.trail.clear()
        self.wall_points = [[] for _ in range(4)]
        self._new_points_since_consolidate = 0
        self._grid.clear()

    def push_frame(self, frame):
        """
        Update dead-reckoned position from a telemetry frame.
        frame has: ms, s0-s3, steer, speed, target, yaw, heading, has_imu
        """
        if self._prev_ms is None:
            self._prev_ms = frame.ms
            return

        dt = (frame.ms - self._prev_ms) / 1000.0
        self._prev_ms = frame.ms
        if dt <= 0 or dt > 1.0:
            return

        # ── Update heading ──────────────────────────────────────────────
        if frame.has_imu and frame.yaw is not None:
            # Use gyro yaw rate (°/s → rad/s), integrate
            yaw_rad = math.radians(frame.yaw)
            self.car_heading += yaw_rad * dt
        else:
            # Kinematic: steer is -1000..+1000, map to steering angle
            steer_frac = frame.steer / 1000.0
            steer_angle = steer_frac * math.radians(MAX_STEER_RAD)
            if abs(steer_angle) > 0.001:
                turn_radius = WHEELBASE / math.tan(steer_angle)
                self.car_heading += (frame.speed * dt) / turn_radius

        # ── Update position ─────────────────────────────────────────────
        dx = frame.speed * dt * math.cos(self.car_heading)
        dy = frame.speed * dt * math.sin(self.car_heading)
        self.car_x += dx
        self.car_y += dy

        self.trail.append((self.car_x, self.car_y))

        # ── Compute wall points from LiDAR readings ─────────────────────
        sensor_dists = [frame.s0, frame.s1, frame.s2, frame.s3]
        for i, dist_cm10 in enumerate(sensor_dists):
            dist_m = dist_cm10 / 10000.0  # cm×10 → metres
            if dist_m <= 0 or dist_m >= MAX_LIDAR_M:
                continue

            # Sensor position relative to rear axle
            s_fwd = SENSOR_FWD
            s_lat = SENSOR_LAT[i]
            s_angle = math.radians(SENSOR_DEG[i])

            # Sensor world position
            cos_h = math.cos(self.car_heading)
            sin_h = math.sin(self.car_heading)
            sx = self.car_x + s_fwd * cos_h - s_lat * sin_h
            sy = self.car_y + s_fwd * sin_h + s_lat * cos_h

            # Wall point at sensor origin + dist along sensor direction
            ray_angle = self.car_heading + s_angle
            wx = sx + dist_m * math.cos(ray_angle)
            wy = sy + dist_m * math.sin(ray_angle)

            # Grid dedup
            gx = int(wx / CELL_SIZE)
            gy = int(wy / CELL_SIZE)
            key = (gx, gy, i)
            if key not in self._grid:
                self._grid[key] = True
                self.wall_points[i].append((wx, wy))
                self._new_points_since_consolidate += 1

    def _world_to_canvas(self, wx, wy):
        """Convert world coords (metres) to canvas pixels."""
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        cx = cw / 2 + (wx * self._scale) + self._offset_x
        cy = ch / 2 - (wy * self._scale) + self._offset_y  # Y flipped
        return cx, cy

    def redraw(self):
        """Full redraw of the map on canvas."""
        self.canvas.delete("all")

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 2 or ch < 2:
            return

        # ── Wall points ──────────────────────────────────────────────────
        for i, pts in enumerate(self.wall_points):
            color = SENSOR_COLORS[i]
            # Convert color names to hex for canvas
            color_hex = _color_name_to_hex(color)
            for wx, wy in pts:
                cx, cy = self._world_to_canvas(wx, wy)
                if 0 <= cx <= cw and 0 <= cy <= ch:
                    self.canvas.create_rectangle(
                        cx - 1, cy - 1, cx + 1, cy + 1,
                        fill=color_hex, outline="")

        # ── Car trail ────────────────────────────────────────────────────
        if len(self.trail) >= 2:
            # Draw trail as thin polyline (downsample for performance)
            step = max(1, len(self.trail) // 1000)
            coords = []
            for j in range(0, len(self.trail), step):
                cx, cy = self._world_to_canvas(*self.trail[j])
                coords.extend([cx, cy])
            if len(coords) >= 4:
                self.canvas.create_line(*coords, fill="yellow",
                                         width=1, smooth=False)

        # ── Car position marker ──────────────────────────────────────────
        if self.trail:
            cx, cy = self._world_to_canvas(self.car_x, self.car_y)
            r = 5
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r,
                                     fill="white", outline="yellow")
            # Heading indicator
            hx = cx + 12 * math.cos(-self.car_heading)  # flip for canvas Y
            hy = cy + 12 * math.sin(-self.car_heading)
            self.canvas.create_line(cx, cy, hx, hy, fill="white", width=2)

    # ── Mouse handlers ──────────────────────────────────────────────────────
    def _on_drag_start(self, event):
        self._drag_start = (event.x, event.y)

    def _on_drag(self, event):
        if self._drag_start:
            dx = event.x - self._drag_start[0]
            dy = event.y - self._drag_start[1]
            self._offset_x += dx
            self._offset_y += dy
            self._drag_start = (event.x, event.y)
            self.redraw()

    def _on_scroll(self, event):
        factor = 1.15 if event.delta > 0 else 1 / 1.15
        self._scale *= factor
        self._scale = max(10, min(5000, self._scale))
        self.redraw()

    def _on_scroll_linux(self, event, direction):
        factor = 1.15 if direction > 0 else 1 / 1.15
        self._scale *= factor
        self._scale = max(10, min(5000, self._scale))
        self.redraw()


def _color_name_to_hex(name: str) -> str:
    """Convert matplotlib color names to hex for tkinter canvas."""
    _MAP = {
        "tab:green":  "#2ca02c",
        "tab:blue":   "#1f77b4",
        "tab:orange": "#ff7f0e",
        "tab:red":    "#d62728",
        "tab:purple": "#9467bd",
        "tab:cyan":   "#17becf",
        "tab:brown":  "#8c564b",
    }
    return _MAP.get(name, name)
