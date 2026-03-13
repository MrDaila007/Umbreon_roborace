"""
car_config.py — Constants and key mappings for the Umbreon dashboard.

Sensor geometry comes from sim.py (measured on the real car).
Default parameter values mirror the compile-time #defines in luna_car.h
and Umbreon_roborace.ino.
"""

# ─── Sensor geometry (from sim.py lines 54-59) ──────────────────────────────
WHEELBASE = 0.173          # m
SENSOR_FWD = 0.253         # m ahead of rear axle (WHEELBASE + 0.080)
MAX_STEER_RAD = 28.0       # degrees (physical max steering angle)
SENSOR_DEG = [45.0, 0.0, 0.0, -45.0]         # relative to car heading
SENSOR_LAT = [0.09, 0.04, -0.04, -0.09]      # lateral offset (+ = left)
SENSOR_NAMES = ["L-Out", "L-Fwd", "R-Fwd", "R-Out"]
SENSOR_COLORS = ["tab:green", "tab:blue", "tab:orange", "tab:red"]
MAX_LIDAR_M = 8.0          # max LiDAR range in metres

# ─── Kinematic parameters ────────────────────────────────────────────────────
CAR_LENGTH = 0.290          # m (total body)
TRACK_WIDTH = 0.120         # m
WHEEL_DIAM = 0.060          # m

# ─── Default parameter values (compile-time defaults) ────────────────────────
DEFAULTS = {
    "FOD":  1200,     # FRONT_OBSTACLE_DIST (cm×10)
    "SOD":  1000,     # SIDE_OPEN_DIST
    "ACD":   800,     # ALL_CLOSE_DIST
    "CFD":   201,     # CLOSE_FRONT_DIST
    "KP":    4.18,    # PID_KP
    "KI":    2.93,    # PID_KI
    "KD":    0.43,    # PID_KD
    "MSP":    96,     # MIN_SPEED
    "XSP":   110,     # MAX_SPEED
    "BSP":    85,     # MIN_BSPEED
    "MNP":    40,     # MIN_POINT (steering)
    "XNP":   140,     # MAX_POINT
    "NTP":    90,     # NEUTRAL_POINT
    "ENH":    62,     # ENCODER_HOLES
    "WDM":  0.060,    # WHEEL_DIAM_M
    "LMS":    40,     # LOOP_MS
    "SPD1": 2.7,      # speed when clear
    "SPD2": 0.8,      # speed when blocked
    "COE1": 0.3,      # steer coef when clear
    "COE2": 0.7,      # steer coef when blocked
    "WDD": 120.0,     # WRONG_DIR_DEG
    "RCW":     1,     # RACE_CW (bool: 1=CW, 0=CCW)
    "STK":    25,     # stuck_time threshold
    "IMU":     1,     # USE_IMU (read-only)
    "DBG":     1,     # USE_WIFI_DEBUG (read-only)
}

# ─── Key abbreviation ↔ full name mapping ────────────────────────────────────
KEY_NAMES = {
    "FOD":  "Front Obstacle Dist",
    "SOD":  "Side Open Dist",
    "ACD":  "All Close Dist",
    "CFD":  "Close Front Dist",
    "KP":   "PID Kp",
    "KI":   "PID Ki",
    "KD":   "PID Kd",
    "MSP":  "Min Speed (ESC)",
    "XSP":  "Max Speed (ESC)",
    "BSP":  "Min Reverse Speed",
    "MNP":  "Min Steer Point",
    "XNP":  "Max Steer Point",
    "NTP":  "Neutral Steer Point",
    "ENH":  "Encoder Holes",
    "WDM":  "Wheel Diameter (m)",
    "LMS":  "Loop Period (ms)",
    "SPD1": "Speed Clear (m/s)",
    "SPD2": "Speed Blocked (m/s)",
    "COE1": "Steer Coef Clear",
    "COE2": "Steer Coef Blocked",
    "WDD":  "Wrong Dir Degrees",
    "RCW":  "Race Clockwise",
    "STK":  "Stuck Threshold",
    "IMU":  "IMU Enabled (r/o)",
    "DBG":  "WiFi Debug (r/o)",
}

# Keys that are read-only (compile-time flags)
READONLY_KEYS = {"IMU", "DBG"}

# ─── Parameter grouping for UI ───────────────────────────────────────────────
PARAM_GROUPS = {
    "Obstacle Distances": ["FOD", "SOD", "ACD", "CFD"],
    "PID Tuning":         ["KP", "KI", "KD"],
    "Speed / ESC":        ["MSP", "XSP", "BSP"],
    "Steering":           ["MNP", "XNP", "NTP"],
    "Tachometer":         ["ENH", "WDM"],
    "Control Loop":       ["LMS", "SPD1", "SPD2", "COE1", "COE2"],
    "Navigation":         ["WDD", "RCW", "STK"],
    "Flags (read-only)":  ["IMU", "DBG"],
}

# ─── Float keys (need float parsing/display, rest are int) ───────────────────
FLOAT_KEYS = {"KP", "KI", "KD", "WDM", "SPD1", "SPD2", "COE1", "COE2", "WDD"}
