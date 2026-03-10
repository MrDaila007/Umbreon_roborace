#!/usr/bin/env python3
"""
sim.py — Umbreon roborace algorithm simulator
2-D top-down environment, kinematic bicycle model, 4× LiDAR rays.

    pip install numpy matplotlib
    python sim.py            # live animation
    python sim.py --fast     # pre-compute full run and plot path + graphs
    python sim.py --ticks N  # control how many 40-ms ticks to run (default 1500)

Sensor layout (bumper-mounted, same as firmware):
    s[0] L-Out  45° left  of heading, offset 7 cm left
    s[1] L-Fwd  0°  (straight), offset 3 cm left
    s[2] R-Fwd  0°  (straight), offset 3 cm right
    s[3] R-Out  45° right of heading, offset 7 cm right

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

# ─── Car physics ──────────────────────────────────────────────────────────────
WHEELBASE    = 0.18            # m  (typical 1:10 RC car)
MAX_STR_RAD  = np.radians(28)  # physical max steering angle
SPEED_TC     = 5.0             # speed time-constant (Hz) — first-order response

# ─── Sensor layout ────────────────────────────────────────────────────────────
# Physical layout (all sensors on front bumper):
#   s[0] Left-Out  — outer left,  45° left  of heading
#   s[1] Left-Fwd  — center-left, straight (0°), offset 3 cm left  of centreline
#   s[2] Right-Fwd — center-right,straight (0°), offset 3 cm right of centreline
#   s[3] Right-Out — outer right, 45° right of heading
SENSOR_DEG    = [45.0,  0.0,  0.0, -45.0]   # relative to car heading
SENSOR_LAT_M  = [0.07,  0.03, -0.03, -0.07]  # lateral offset (+ = left)
SENSOR_NAMES  = ["L-Out", "L-Fwd", "R-Fwd", "R-Out"]
SENSOR_COLORS = ["tab:green", "tab:blue", "tab:orange", "tab:red"]
MAX_LIDAR_M   = 8.0    # max LiDAR range

# ─── Track builder ────────────────────────────────────────────────────────────
# Corridor width ~1.05 m (mid of 0.95–1.15 m regulation range)
def build_track():
    """
    Rectangular loop:
      Outer wall: 6 m × 3.5 m
      Inner wall: 4 m × 1.5 m  (corridor ~1 m on all 4 sides)

    Car starts in bottom straight, heading east.
    """
    outer = [
        (0.0, 0.0,  6.0, 0.0),
        (6.0, 0.0,  6.0, 3.5),
        (6.0, 3.5,  0.0, 3.5),
        (0.0, 3.5,  0.0, 0.0),
    ]
    inner = [
        (1.0, 1.0,  5.0, 1.0),
        (5.0, 1.0,  5.0, 2.5),
        (5.0, 2.5,  1.0, 2.5),
        (1.0, 2.5,  1.0, 1.0),
    ]
    return outer + inner, 1.5, 0.5, 0.0   # walls, x0, y0, heading_rad

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
    perp = h + np.pi / 2          # unit vector pointing left of heading
    cp, sp = np.cos(perp), np.sin(perp)
    for deg, lat in zip(SENSOR_DEG, SENSOR_LAT_M):
        ox = x + lat * cp          # sensor origin with lateral offset
        oy = y + lat * sp
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
    ax.set_xlim(-0.2, 6.2); ax.set_ylim(-0.2, 3.7)
    ax.set_title("Umbreon Roborace — Live simulation  (close window to stop)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

    # Static walls
    for seg in walls:
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]], 'k-', lw=2.5)

    trail_line, = ax.plot([], [], 'b-', lw=1, alpha=0.35, label='path')
    ray_lines   = [ax.plot([], [], '-', color=c, alpha=0.75, lw=1.5,
                            label=n)[0]
                   for c, n in zip(SENSOR_COLORS, SENSOR_NAMES)]
    arrow_ref   = [None]
    info_box    = ax.text(0.015, 0.97, '', transform=ax.transAxes,
                          va='top', fontsize=8, fontfamily='monospace',
                          bbox=dict(facecolor='white', alpha=0.7, pad=3))

    ax.legend(loc='upper right', fontsize=8, ncol=2)

    def _redraw_arrow():
        if arrow_ref[0] is not None:
            arrow_ref[0].remove()
        arrow_ref[0] = mpatches.FancyArrow(
            sim['x'], sim['y'],
            0.13 * np.cos(sim['h']),
            0.13 * np.sin(sim['h']),
            width=0.06, head_width=0.10,
            color='steelblue', zorder=6, length_includes_head=True)
        ax.add_patch(arrow_ref[0])

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
        _redraw_arrow()

        perp = sim['h'] + np.pi / 2
        cp, sp = np.cos(perp), np.sin(perp)
        for i, (deg, lat, rl) in enumerate(zip(SENSOR_DEG, SENSOR_LAT_M, ray_lines)):
            ox = sim['x'] + lat * cp
            oy = sim['y'] + lat * sp
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

# ─── Entry point ──────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Umbreon roborace simulator")
    parser.add_argument("--fast",  action="store_true",
                        help="Pre-compute and plot (no animation)")
    parser.add_argument("--ticks", type=int, default=1500,
                        help="Control ticks for --fast mode (default 1500 = 60 s)")
    args = parser.parse_args()

    walls, x0, y0, h0 = build_track()

    if args.fast:
        run_fast(walls, x0, y0, h0, args.ticks)
    else:
        run_live(walls, x0, y0, h0)
