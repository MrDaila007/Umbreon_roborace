#!/usr/bin/env python3
"""
sim.py — Umbreon roborace algorithm simulator
2-D top-down environment, kinematic bicycle model, 4× LiDAR rays.

    pip install numpy matplotlib
    python sim.py              # live animation
    python sim.py --fast       # pre-compute full run and plot path + graphs
    python sim.py --ticks N    # control how many 40-ms ticks to run (default 1500)
    python sim.py --bridge     # run sim + TCP server for dashboard testing

Car dimensions (measured):
    Length: 290 mm   Wheelbase: 173 mm   Track: 120 mm   Wheel ⌀: 60 mm

Sensor layout (bumper-mounted, 80 mm ahead of front axle):
    s[0] L-Out  45° left  of heading, 90 mm left  of centre
    s[1] L-Fwd  0°  (straight),       40 mm left  of centre
    s[2] R-Fwd  0°  (straight),       40 mm right of centre
    s[3] R-Out  45° right of heading,  90 mm right of centre

Steering convention (matches write_steer() inversion):
    steer_cmd > 0  →  RIGHT turn
    steer_cmd < 0  →  LEFT  turn
"""

import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

# ─── Algorithm constants (mirrors Umbreon_roborace.ino / luna_car.h) ─────────
FRONT_OBSTACLE_DIST =  600   # cm×10  =  60 cm  (was 1200; tuned for 45-deg sensors)
SIDE_OPEN_DIST      = 1000   # cm×10  = 100 cm
ALL_CLOSE_DIST      =  800   # cm×10  =  80 cm
CLOSE_FRONT_DIST    =  201   # cm×10  = ~20 cm
LOOP_MS             =   40   # ms — control period

# ─── Car physics (measured from real Umbreon) ────────────────────────────────
CAR_LENGTH   = 0.290           # m  (total body length)
WHEELBASE    = 0.173           # m  (front-rear axle centre-to-centre)
TRACK_WIDTH  = 0.120           # m  (rear axle, wheel-to-wheel min)
WHEEL_DIAM   = 0.060           # m  (60 mm)
FRONT_OH     = 0.080           # m  (front axle to front bumper ≈ sensor line)
REAR_OH      = CAR_LENGTH - WHEELBASE - FRONT_OH  # 37 mm
MAX_STR_RAD  = np.radians(28)  # physical max steering angle
SPEED_TC     = 5.0             # speed time-constant (Hz) — first-order response

# ─── Sensor layout ────────────────────────────────────────────────────────────
# All 4 TF-Luna sensors on front bumper, 80 mm ahead of front axle centre.
# Lateral offsets from centreline (measured on car):
#   Central pair (FL, FR): 80 mm apart  → ±40 mm from centre
#   Outer pair  (L, R):   50 mm further → ±90 mm from centre
SENSOR_FWD_M  = WHEELBASE + 0.080          # 253 mm ahead of rear axle
SENSOR_DEG    = [45.0,  0.0,  0.0, -45.0]  # relative to car heading
SENSOR_LAT_M  = [0.09,  0.04, -0.04, -0.09]  # lateral offset (+ = left)
SENSOR_NAMES  = ["L-Out", "L-Fwd", "R-Fwd", "R-Out"]
SENSOR_COLORS = ["tab:green", "tab:blue", "tab:orange", "tab:red"]
MAX_LIDAR_M   = 8.0    # max LiDAR range

# ─── Track builder ────────────────────────────────────────────────────────────
CORRIDOR_W = 1.05   # corridor width (m), mid of 0.95–1.15 regulation range

def _arc_segments(cx, cy, r, a_start, a_end, n=12):
    """Generate line segments approximating an arc (angles in radians)."""
    segs = []
    angles = np.linspace(a_start, a_end, n + 1)
    for i in range(n):
        segs.append((cx + r * np.cos(angles[i]),   cy + r * np.sin(angles[i]),
                      cx + r * np.cos(angles[i+1]), cy + r * np.sin(angles[i+1])))
    return segs

def _rounded_rect(cx, cy, half_w, half_h, r, n_arc=10):
    """Rectangle with rounded corners, returned as list of (x1,y1,x2,y2) segments."""
    segs = []
    # Clamp radius
    r = min(r, half_w, half_h)
    # Straight edges (between corners)
    # bottom: left→right
    segs.append((cx - half_w + r, cy - half_h,     cx + half_w - r, cy - half_h))
    # right:  bottom→top
    segs.append((cx + half_w,     cy - half_h + r,  cx + half_w,     cy + half_h - r))
    # top:    right→left
    segs.append((cx + half_w - r, cy + half_h,     cx - half_w + r, cy + half_h))
    # left:   top→bottom
    segs.append((cx - half_w,     cy + half_h - r,  cx - half_w,     cy - half_h + r))
    # Corner arcs (bottom-right, top-right, top-left, bottom-left)
    segs += _arc_segments(cx + half_w - r, cy - half_h + r, r, -np.pi/2, 0,       n_arc)
    segs += _arc_segments(cx + half_w - r, cy + half_h - r, r, 0,        np.pi/2,  n_arc)
    segs += _arc_segments(cx - half_w + r, cy + half_h - r, r, np.pi/2,  np.pi,    n_arc)
    segs += _arc_segments(cx - half_w + r, cy - half_h + r, r, np.pi,    3*np.pi/2, n_arc)
    return segs

def build_track():
    """
    Complex track with rounded corners, chicane, and narrowing section.

    Layout (~8 × 5 m):
      - Main oval loop with rounded corners (r=0.8m outer, r-corridor inner)
      - Chicane barrier in top straight
      - Narrowing on right side

    Car starts in bottom straight, heading east.
    """
    W = CORRIDOR_W
    # ── Main oval ────────────────────────────────────────────────────────────
    # Outer boundary: 8 × 5 m, corner radius 0.9
    out_hw, out_hh, out_r = 4.0, 2.5, 0.9
    cx, cy = 4.0, 2.5
    outer = _rounded_rect(cx, cy, out_hw, out_hh, out_r)

    # Inner boundary: shrunk by corridor width
    in_hw  = out_hw - W
    in_hh  = out_hh - W
    in_r   = max(0.1, out_r - W)
    inner  = _rounded_rect(cx, cy, in_hw, in_hh, in_r)

    walls = outer + inner

    # ── Chicane in top straight ──────────────────────────────────────────────
    # Two small barriers staggered, forcing S-turn
    chic_y_out = cy + out_hh           # top outer wall y = 5.0
    chic_y_in  = cy + out_hh - W       # top inner wall y = 3.95
    chic_mid   = (chic_y_out + chic_y_in) / 2

    # Barrier 1: sticks out from outer wall toward centre
    b1x = cx - 0.8
    walls.append((b1x, chic_y_out, b1x, chic_mid + 0.05))
    walls.append((b1x - 0.15, chic_mid + 0.05, b1x + 0.15, chic_mid + 0.05))

    # Barrier 2: sticks out from inner wall toward outer
    b2x = cx + 0.8
    walls.append((b2x, chic_y_in, b2x, chic_mid - 0.05))
    walls.append((b2x - 0.15, chic_mid - 0.05, b2x + 0.15, chic_mid - 0.05))

    # ── Narrowing on right side ──────────────────────────────────────────────
    # Bump inner wall outward on right straight, narrowing corridor to ~0.7 m
    narrow_x = cx + out_hw - W   # inner wall right side x ≈ 6.95
    bump = 0.30
    walls.append((narrow_x + bump, cy - 0.6, narrow_x + bump, cy + 0.6))
    # Close off the bump top and bottom
    walls.append((narrow_x, cy - 0.6,  narrow_x + bump, cy - 0.6))
    walls.append((narrow_x, cy + 0.6,  narrow_x + bump, cy + 0.6))
    # Remove the inner wall segment behind the bump by covering it
    # (the bump creates a parallel wall closer to outer)

    # Start position: bottom straight, heading east
    return walls, 2.5, 0.35, 0.0

# ─── Sensor raycast ───────────────────────────────────────────────────────────
def _ray_seg(ox, oy, dx, dy, x1, y1, x2, y2):
    """Return t of first ray–wall intersection, or None."""
    wx = x2 - x1;  wy = y2 - y1
    den = dx * wy - dy * wx
    if abs(den) < 1e-10:
        return None
    t = ((x1 - ox) * wy - (y1 - oy) * wx) / den
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    if t > 1e-6 and 0.0 <= u <= 1.0:
        return t
    return None

def sense(x, y, h, walls):
    """Return [s0..s3] in cm×10 (9999 if beyond range)."""
    readings = []
    ch, sh = np.cos(h), np.sin(h)  # heading unit vector
    perp = h + np.pi / 2           # unit vector pointing left of heading
    cp, sp = np.cos(perp), np.sin(perp)
    for deg, lat in zip(SENSOR_DEG, SENSOR_LAT_M):
        ox = x + SENSOR_FWD_M * ch + lat * cp   # forward + lateral offset
        oy = y + SENSOR_FWD_M * sh + lat * sp
        ang = h + np.radians(deg)
        dx, dy = np.cos(ang), np.sin(ang)
        t_min = MAX_LIDAR_M
        for seg in walls:
            r = _ray_seg(ox, oy, dx, dy, *seg)
            if r is not None and r < t_min:
                t_min = r
        readings.append(min(int(t_min * 1000.0), 9999))   # m → cm×10
    return readings

# ─── Control algorithm (direct port of work() in Umbreon_roborace.ino) ───────
def work(state, s):
    """
    state : mutable dict — 'stuck_time', 'turns', 'v' (current speed m/s)
    s     : sensor list [s0..s3] in cm×10
    Returns (steer_cmd, target_speed_ms)
    """
    f_l = s[1] < FRONT_OBSTACLE_DIST
    f_r = s[2] < FRONT_OBSTACLE_DIST

    # ── Steering diff ─────────────────────────────────────────────────────────
    if s[0] > SIDE_OPEN_DIST and s[3] > SIDE_OPEN_DIST:
        diff = 800                 # open corridor — bias toward right wall
    else:
        diff = s[3] - s[0]         # balance between walls

    if all(x < ALL_CLOSE_DIST for x in s):
        diff = 800                 # boxed in — hard right

    # Diagonal correction (mirrors Umbreon_roborace.ino fix):
    # blends FR-FL into steering to smooth corner transitions.
    diff += (s[2] - s[1]) // 3

    # ── Speed ─────────────────────────────────────────────────────────────────
    how_clear = int(f_l) + int(f_r)
    if how_clear == 0:
        coef, spd = 0.3, 2.7
    else:
        coef, spd = 0.7, 0.8

    steer = int(np.clip(diff * coef, -1000, 1000))

    # ── Stuck detection ───────────────────────────────────────────────────────
    c_fl = s[1] < CLOSE_FRONT_DIST
    c_fr = s[2] < CLOSE_FRONT_DIST
    if c_fl or c_fr or state['v'] < 0.1:
        state['stuck_time'] += 1
    else:
        state['stuck_time'] = 0

    if state['stuck_time'] > 25:
        steer = 0
        spd   = -0.5           # simplified reverse (no go_back() in sim)
        state['stuck_time'] = 0

    # ── U-turn detection ──────────────────────────────────────────────────────
    state['turns'] += diff * state['v'] / -1000.0
    state['turns']  = float(np.clip(state['turns'], -1500.0, 50.0))

    if state['turns'] < -18.0:
        steer            = 1000
        spd              = -0.5
        state['turns']   = 0.0

    return steer, spd

# ─── Physics step ─────────────────────────────────────────────────────────────
DT         = 0.005                              # physics timestep (s)
CTRL_STEPS = max(1, int((LOOP_MS / 1000.0) / DT))   # physics steps per tick

def step_physics(x, y, h, v, steer_cmd, target_v, dt=DT):
    """One physics sub-step. Returns (x, y, h, v)."""
    # steer_cmd > 0 → right turn (matches write_steer() inversion)
    norm  = np.clip(steer_cmd / 1000.0, -1.0, 1.0)
    angle = -norm * MAX_STR_RAD          # negative so positive cmd = right
    if abs(v) > 0.01 and abs(angle) > 1e-4:
        omega = v / (WHEELBASE / np.tan(angle))
    else:
        omega = 0.0
    h += omega * dt
    x += v * np.cos(h) * dt
    y += v * np.sin(h) * dt
    v += (target_v - v) * dt * SPEED_TC
    if target_v >= 0.0:
        v = max(0.0, v)
    return x, y, h, v

# ─── Pre-compute mode (--fast) ────────────────────────────────────────────────
def simulate(walls, x0, y0, h0, n_ticks=1500):
    x, y, h, v = x0, y0, h0, 0.0
    steer_cmd, target_v = 0, 0.0
    state = {'stuck_time': 0, 'turns': 0.0, 'v': 0.0}

    xs, ys, vs_log, steers_log, sens_log = [x0], [y0], [0.0], [], []

    for _ in range(n_ticks):
        s = sense(x, y, h, walls)
        state['v'] = v
        steer_cmd, target_v = work(state, s)
        sens_log.append(s[:])
        steers_log.append(steer_cmd)

        for _ in range(CTRL_STEPS):
            x, y, h, v = step_physics(x, y, h, v, steer_cmd, target_v)

        xs.append(x); ys.append(y); vs_log.append(v)

    return xs, ys, vs_log, steers_log, sens_log

def run_fast(walls, x0, y0, h0, n_ticks):
    xs, ys, vs_log, steers_log, sens_log = simulate(walls, x0, y0, h0, n_ticks)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Umbreon Roborace — Pre-computed run", fontsize=13)

    # ── Left: track + colour-coded path ──────────────────────────────────────
    ax = axes[0]
    ax.set_aspect('equal')
    ax.set_title("Path  (colour = speed)")
    for seg in walls:
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]], 'k-', lw=2.5)
    sc = ax.scatter(xs, ys, c=vs_log, cmap='plasma', s=5, vmin=0, vmax=3.0, zorder=3)
    plt.colorbar(sc, ax=ax, label='speed (m/s)')
    ax.plot(x0, y0, 'g^', ms=10, label='start', zorder=5)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(fontsize=9)

    # ── Right: speed + steering + sensor distances over time ─────────────────
    ax2 = axes[1]
    t = np.arange(len(vs_log)) * (LOOP_MS / 1000.0)
    ax2.plot(t, vs_log,  label='speed (m/s)', lw=1.5)
    ax2.plot(t[:len(steers_log)],
             [s / 1000.0 for s in steers_log],
             label='steer (norm)', lw=1, alpha=0.7)
    for i, (name, col) in enumerate(zip(SENSOR_NAMES, SENSOR_COLORS)):
        d = [r[i] / 10.0 for r in sens_log]   # cm
        ax2.plot(t[:len(d)], d, label=f"{name} (cm)", lw=0.8, alpha=0.5, color=col)
    ax2.axhline(FRONT_OBSTACLE_DIST / 10, color='gray', ls='--', lw=0.8,
                label=f'FRONT_OBSTACLE ({FRONT_OBSTACLE_DIST//10} cm)')
    ax2.set_xlabel("time (s)"); ax2.set_title("Signals over time")
    ax2.legend(fontsize=7, ncol=2); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

# ─── Live animation mode ──────────────────────────────────────────────────────
def run_live(walls, x0, y0, h0):
    sim = dict(x=x0, y=y0, h=h0, v=0.0,
               steer=0, target_v=0.0,
               ctrl={'stuck_time': 0, 'turns': 0.0, 'v': 0.0},
               trail_x=[x0], trail_y=[y0],
               sensors=[9999]*4)

    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect('equal')
    ax.set_xlim(-0.5, 8.5); ax.set_ylim(-0.5, 5.5)
    ax.set_title("Umbreon Roborace — Live simulation  (close window to stop)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    # Static walls
    for seg in walls:
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]], 'k-', lw=2.5)

    trail_line, = ax.plot([], [], 'b-', lw=1, alpha=0.35, label='path')
    ray_lines   = [ax.plot([], [], '-', color=c, alpha=0.75, lw=1.5,
                            label=n)[0]
                   for c, n in zip(SENSOR_COLORS, SENSOR_NAMES)]
    car_patches = [None, None]   # [body_rect, heading_arrow]
    info_box    = ax.text(0.015, 0.97, '', transform=ax.transAxes,
                          va='top', fontsize=8, fontfamily='monospace',
                          bbox=dict(facecolor='white', alpha=0.7, pad=3))

    ax.legend(loc='upper right', fontsize=8, ncol=2)

    def _redraw_car():
        for p in car_patches:
            if p is not None:
                p.remove()
        # Car body rectangle (rear-axle is reference point)
        h = sim['h']
        cx = sim['x'] + (WHEELBASE / 2 + FRONT_OH - REAR_OH) / 2 * np.cos(h)
        cy = sim['y'] + (WHEELBASE / 2 + FRONT_OH - REAR_OH) / 2 * np.sin(h)
        angle_deg = np.degrees(h)
        rect = mpatches.FancyBboxPatch(
            (-CAR_LENGTH / 2, -TRACK_WIDTH / 2), CAR_LENGTH, TRACK_WIDTH,
            boxstyle="round,pad=0.005",
            facecolor='steelblue', alpha=0.6, edgecolor='navy', lw=1.2, zorder=5)
        t = matplotlib.transforms.Affine2D().rotate(h).translate(cx, cy) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)
        car_patches[0] = rect
        # Heading arrow (small, at front)
        fx = sim['x'] + (WHEELBASE + FRONT_OH) * np.cos(h)
        fy = sim['y'] + (WHEELBASE + FRONT_OH) * np.sin(h)
        arr = mpatches.FancyArrow(
            fx, fy, 0.04 * np.cos(h), 0.04 * np.sin(h),
            width=0.02, head_width=0.04,
            color='navy', zorder=7, length_includes_head=True)
        ax.add_patch(arr)
        car_patches[1] = arr

    def frame(_):
        # One control tick ───────────────────────────────────────────────────
        s = sense(sim['x'], sim['y'], sim['h'], walls)
        sim['sensors']      = s
        sim['ctrl']['v']    = sim['v']
        steer, tgt_v        = work(sim['ctrl'], s)
        sim['steer']        = steer
        sim['target_v']     = tgt_v

        for _ in range(CTRL_STEPS):
            sim['x'], sim['y'], sim['h'], sim['v'] = step_physics(
                sim['x'], sim['y'], sim['h'], sim['v'], steer, tgt_v)

        sim['trail_x'].append(sim['x'])
        sim['trail_y'].append(sim['y'])

        # Graphics ────────────────────────────────────────────────────────────
        trail_line.set_data(sim['trail_x'][-800:], sim['trail_y'][-800:])
        _redraw_car()

        ch, sh = np.cos(sim['h']), np.sin(sim['h'])
        perp = sim['h'] + np.pi / 2
        cp, sp = np.cos(perp), np.sin(perp)
        for i, (deg, lat, rl) in enumerate(zip(SENSOR_DEG, SENSOR_LAT_M, ray_lines)):
            ox = sim['x'] + SENSOR_FWD_M * ch + lat * cp
            oy = sim['y'] + SENSOR_FWD_M * sh + lat * sp
            ang = sim['h'] + np.radians(deg)
            d   = min(s[i] / 1000.0, MAX_LIDAR_M)
            rl.set_data([ox, ox + d * np.cos(ang)],
                        [oy, oy + d * np.sin(ang)])

        info_box.set_text(
            f"v = {sim['v']:.2f} m/s    steer = {sim['steer']:+5d}\n"
            f"Lo={s[0]/10:5.0f}cm  Lf={s[1]/10:5.0f}cm  "
            f"Rf={s[2]/10:5.0f}cm  Ro={s[3]/10:5.0f}cm\n"
            f"stuck = {sim['ctrl']['stuck_time']:2d}    "
            f"turns = {sim['ctrl']['turns']:+6.1f}")

        return [trail_line, *ray_lines, info_box]

    ani = FuncAnimation(fig, frame, interval=20, blit=False,
                        cache_frame_data=False)
    plt.tight_layout()
    plt.show()

# ─── Dashboard bridge (TCP server mimicking DT-06 output) ────────────────────
import socket
import threading
import time

class TelemetryBridge:
    """TCP server that mimics the DT-06 WiFi bridge for dashboard testing."""

    def __init__(self, port=8023):
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self._server = None
        self._cfg = dict(
            FOD=FRONT_OBSTACLE_DIST, SOD=SIDE_OPEN_DIST,
            ACD=ALL_CLOSE_DIST, CFD=CLOSE_FRONT_DIST,
            KP=4.18, KI=2.93, KD=0.43,
            MSP=96, XSP=110, BSP=85,
            MNP=40, XNP=140, NTP=90,
            ENH=62, WDM=0.060, LMS=LOOP_MS,
            SPD1=2.7, SPD2=0.8, COE1=0.3, COE2=0.7,
            WDD=120.0, RCW=1, STK=25, IMU=1, DBG=1,
        )

    def start(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(('0.0.0.0', self.port))
        self._server.listen(2)
        self._server.settimeout(0.5)
        t = threading.Thread(target=self._accept_loop, daemon=True)
        t.start()
        print(f"  Bridge TCP server on port {self.port}")
        print(f"  Dashboard: connect to localhost:{self.port}")

    def _accept_loop(self):
        while True:
            try:
                conn, addr = self._server.accept()
                conn.setblocking(False)
                with self.lock:
                    self.clients.append(conn)
                try:
                    conn.sendall(b"#ms,s0,s1,s2,s3,steer,speed,target,yaw,heading\n")
                except OSError:
                    pass
                print(f"  Dashboard client connected: {addr}")
            except socket.timeout:
                continue
            except OSError:
                break

    def send_telemetry(self, ms, s, steer, speed, target, yaw=0.0, heading=0.0):
        line = (f"{ms},{s[0]},{s[1]},{s[2]},{s[3]},"
                f"{steer},{speed:.2f},{target:.1f},"
                f"{yaw:.1f},{heading:.1f}\n")
        data = line.encode('ascii')
        with self.lock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
                # Check for incoming commands
                try:
                    cmd_data = c.recv(256)
                    if cmd_data:
                        self._handle_commands(c, cmd_data.decode('ascii', errors='replace'))
                except BlockingIOError:
                    pass
                except OSError:
                    dead.append(c)
            for c in dead:
                try:
                    c.close()
                except OSError:
                    pass
                if c in self.clients:
                    self.clients.remove(c)

    def _handle_commands(self, conn, raw):
        for line in raw.strip().split('\n'):
            line = line.strip()
            if not line.startswith('$'):
                continue
            try:
                if line == '$PING':
                    conn.sendall(b'$PONG\n')
                elif line == '$GET':
                    pairs = ','.join(f'{k}={v}' for k, v in self._cfg.items())
                    conn.sendall(f'$CFG:{pairs}\n'.encode())
                elif line.startswith('$SET:'):
                    for pair in line[5:].split(','):
                        if '=' in pair:
                            k, v = pair.split('=', 1)
                            k = k.strip()
                            if k in self._cfg:
                                try:
                                    self._cfg[k] = float(v) if '.' in v else int(v)
                                except ValueError:
                                    pass
                    conn.sendall(b'$ACK\n')
                elif line in ('$SAVE', '$LOAD', '$RST'):
                    conn.sendall(b'$ACK\n')
            except OSError:
                pass


def _bridge_step(sim, walls):
    """One simulation tick for bridge mode. Returns (s, steer, tgt_v)."""
    s = sense(sim['x'], sim['y'], sim['h'], walls)
    sim['sensors']   = s
    sim['ctrl']['v'] = sim['v']
    steer, tgt_v     = work(sim['ctrl'], s)
    sim['steer']     = steer
    sim['target_v']  = tgt_v

    # Simulated IMU
    dt = LOOP_MS / 1000.0
    norm = np.clip(steer / 1000.0, -1, 1)
    sa = -norm * MAX_STR_RAD
    if abs(sim['v']) > 0.01 and abs(sa) > 1e-4:
        omega = sim['v'] / (WHEELBASE / np.tan(sa))
    else:
        omega = 0.0
    sim['yaw_rate'] = np.degrees(omega)
    sim['heading'] += sim['yaw_rate'] * dt

    for _ in range(CTRL_STEPS):
        sim['x'], sim['y'], sim['h'], sim['v'] = step_physics(
            sim['x'], sim['y'], sim['h'], sim['v'], steer, tgt_v)

    sim['trail_x'].append(sim['x'])
    sim['trail_y'].append(sim['y'])
    sim['tick'] += 1
    return s, steer, tgt_v


def _make_bridge_sim(x0, y0, h0):
    return dict(x=x0, y=y0, h=h0, v=0.0,
                steer=0, target_v=0.0,
                ctrl={'stuck_time': 0, 'turns': 0.0, 'v': 0.0},
                trail_x=[x0], trail_y=[y0],
                sensors=[9999]*4,
                heading=0.0, yaw_rate=0.0,
                tick=0)


def run_bridge(walls, x0, y0, h0, headless=False):
    """Run simulation with TCP bridge for dashboard.
    headless=True: no matplotlib, runs indefinitely (Ctrl+C to stop).
    headless=False: matplotlib animation + bridge.
    """
    bridge = TelemetryBridge()
    bridge.start()
    sim = _make_bridge_sim(x0, y0, h0)

    def _send(s, steer, tgt_v):
        ms = int(sim['tick'] * LOOP_MS)
        bridge.send_telemetry(ms, s, steer, sim['v'], tgt_v,
                              sim['yaw_rate'], sim['heading'])

    if headless:
        print("  Headless mode -- Ctrl+C to stop")
        try:
            while True:
                s, steer, tgt_v = _bridge_step(sim, walls)
                _send(s, steer, tgt_v)
                time.sleep(LOOP_MS / 1000.0)
        except KeyboardInterrupt:
            print("\n  Bridge stopped.")
        return

    # ── GUI mode ─────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect('equal')
    ax.set_xlim(-0.5, 8.5); ax.set_ylim(-0.5, 5.5)
    ax.set_title("Umbreon Roborace -- Bridge mode  (dashboard on port 8023)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    for seg in walls:
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]], 'k-', lw=2.5)

    trail_line, = ax.plot([], [], 'b-', lw=1, alpha=0.35, label='path')
    ray_lines   = [ax.plot([], [], '-', color=c, alpha=0.75, lw=1.5,
                            label=n)[0]
                   for c, n in zip(SENSOR_COLORS, SENSOR_NAMES)]
    car_patches = [None, None]
    info_box    = ax.text(0.015, 0.97, '', transform=ax.transAxes,
                          va='top', fontsize=8, fontfamily='monospace',
                          bbox=dict(facecolor='white', alpha=0.7, pad=3))
    ax.legend(loc='upper right', fontsize=8, ncol=2)

    def _redraw_car():
        for p in car_patches:
            if p is not None:
                p.remove()
        h = sim['h']
        cx = sim['x'] + (WHEELBASE / 2 + FRONT_OH - REAR_OH) / 2 * np.cos(h)
        cy = sim['y'] + (WHEELBASE / 2 + FRONT_OH - REAR_OH) / 2 * np.sin(h)
        rect = mpatches.FancyBboxPatch(
            (-CAR_LENGTH / 2, -TRACK_WIDTH / 2), CAR_LENGTH, TRACK_WIDTH,
            boxstyle="round,pad=0.005",
            facecolor='steelblue', alpha=0.6, edgecolor='navy', lw=1.2, zorder=5)
        t = matplotlib.transforms.Affine2D().rotate(h).translate(cx, cy) + ax.transData
        rect.set_transform(t)
        ax.add_patch(rect)
        car_patches[0] = rect
        fx = sim['x'] + (WHEELBASE + FRONT_OH) * np.cos(h)
        fy = sim['y'] + (WHEELBASE + FRONT_OH) * np.sin(h)
        arr = mpatches.FancyArrow(
            fx, fy, 0.04 * np.cos(h), 0.04 * np.sin(h),
            width=0.02, head_width=0.04,
            color='navy', zorder=7, length_includes_head=True)
        ax.add_patch(arr)
        car_patches[1] = arr

    def frame(_):
        s, steer, tgt_v = _bridge_step(sim, walls)
        _send(s, steer, tgt_v)

        trail_line.set_data(sim['trail_x'][-800:], sim['trail_y'][-800:])
        _redraw_car()

        ch, sh = np.cos(sim['h']), np.sin(sim['h'])
        perp = sim['h'] + np.pi / 2
        cp, sp = np.cos(perp), np.sin(perp)
        for i, (deg, lat, rl) in enumerate(zip(SENSOR_DEG, SENSOR_LAT_M, ray_lines)):
            ox = sim['x'] + SENSOR_FWD_M * ch + lat * cp
            oy = sim['y'] + SENSOR_FWD_M * sh + lat * sp
            ang = sim['h'] + np.radians(deg)
            d   = min(s[i] / 1000.0, MAX_LIDAR_M)
            rl.set_data([ox, ox + d * np.cos(ang)],
                        [oy, oy + d * np.sin(ang)])

        info_box.set_text(
            f"v = {sim['v']:.2f} m/s    steer = {sim['steer']:+5d}\n"
            f"Lo={s[0]/10:5.0f}cm  Lf={s[1]/10:5.0f}cm  "
            f"Rf={s[2]/10:5.0f}cm  Ro={s[3]/10:5.0f}cm\n"
            f"stuck = {sim['ctrl']['stuck_time']:2d}    "
            f"turns = {sim['ctrl']['turns']:+6.1f}    "
            f"yaw = {sim['yaw_rate']:+6.1f}")

        return [trail_line, *ray_lines, info_box]

    ani = FuncAnimation(fig, frame, interval=LOOP_MS, blit=False,
                        cache_frame_data=False)
    plt.tight_layout()
    plt.show()


# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Umbreon roborace simulator")
    parser.add_argument("--fast",     action="store_true",
                        help="Pre-compute and plot (no animation)")
    parser.add_argument("--bridge",   action="store_true",
                        help="Run sim + TCP bridge for dashboard testing")
    parser.add_argument("--headless", action="store_true",
                        help="With --bridge: no matplotlib window, runs indefinitely")
    parser.add_argument("--ticks",    type=int, default=1500,
                        help="Control ticks for --fast mode (default 1500 = 60 s)")
    args = parser.parse_args()

    walls, x0, y0, h0 = build_track()

    if args.fast:
        run_fast(walls, x0, y0, h0, args.ticks)
    elif args.bridge:
        run_bridge(walls, x0, y0, h0, headless=args.headless)
    else:
        run_live(walls, x0, y0, h0)
