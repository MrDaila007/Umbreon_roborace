"""
car_config.py — Constants and key mappings for the Umbreon dashboard.

Sensor geometry comes from sim.py (measured on the real car).
Default parameter values mirror the compile-time #defines in luna_car.h
and Umbreon_roborace.ino.
"""

# ─── Sensor geometry configurations ─────────────────────────────────────────
SENSOR_CONFIGS = {
    4: {
        "name": "4x TF-Luna",
        "angles_deg": [45.0, 0.0, 0.0, -45.0],
        "lateral_m": [0.09, 0.04, -0.04, -0.09],
        "names": ["L-Out", "L-Fwd", "R-Fwd", "R-Out"],
        "colors": ["tab:green", "tab:blue", "tab:orange", "tab:red"],
        "max_range_m": 8.0,
    },
    6: {
        "name": "6x VL53L0X",
        "angles_deg": [-90.0, 0.0, -45.0, 45.0, 0.0, 90.0],
        "lateral_m": [-0.12, -0.04, -0.09, 0.09, 0.04, 0.12],
        "names": ["H-Right", "R-Fwd", "R-Out", "L-Out", "L-Fwd", "H-Left"],
        "colors": [
            "tab:pink", "tab:orange", "tab:red",
            "tab:green", "tab:blue", "tab:purple",
        ],
        "max_range_m": 2.0,
    },
}

# ─── Backward-compatible aliases (default to 4-sensor config) ────────────────
WHEELBASE = 0.173          # m
SENSOR_FWD = 0.253         # m ahead of rear axle (WHEELBASE + 0.080)
MAX_STEER_RAD = 28.0       # degrees (physical max steering angle)
SENSOR_DEG = SENSOR_CONFIGS[4]["angles_deg"]
SENSOR_LAT = SENSOR_CONFIGS[4]["lateral_m"]
SENSOR_NAMES = SENSOR_CONFIGS[4]["names"]
SENSOR_COLORS = SENSOR_CONFIGS[4]["colors"]
MAX_LIDAR_M = SENSOR_CONFIGS[4]["max_range_m"]

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
    "KP":   60.0,     # PID_KP (scaled for µs ESC output)
    "KI":   40.0,     # PID_KI
    "KD":    6.0,     # PID_KD
    "MSP":  1540,     # MIN_SPEED (µs)
    "XSP":  1700,     # MAX_SPEED (µs)
    "BSP":  1460,     # MIN_BSPEED (µs)
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
    "IMR":     1,     # IMU rotated 180° (negate yaw)
    "SVR":     0,     # Servo reverse (negate steering)
    "CAL":     0,     # Calibrated flag (auto-set after ESC+servo cal)
    "BEN":     0,     # Battery monitoring enabled
    "BML":   2.8,     # Battery divider multiplier (R1+R2)/R2
    "BLV":   6.0,     # Battery low voltage cutoff (V)
    "IMU":     1,     # USE_IMU (read-only)
    "DBG":     1,     # USE_WIFI_DEBUG (read-only)
    "SNS":     4,     # Sensor count (read-only)
    "SMX":  8000,     # Max sensor range cm×10 (read-only)
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
    "MSP":  "Min Speed (µs)",
    "XSP":  "Max Speed (µs)",
    "BSP":  "Min Reverse (µs)",
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
    "IMR":  "IMU Rotated 180°",
    "SVR":  "Servo Reverse",
    "CAL":  "Calibrated",
    "BEN":  "Battery Monitor",
    "BML":  "Battery Multiplier",
    "BLV":  "Battery Low (V)",
    "IMU":  "IMU Enabled (r/o)",
    "DBG":  "WiFi Debug (r/o)",
    "SNS":  "Sensor Count (r/o)",
    "SMX":  "Max Range (r/o)",
}

# Keys that are read-only (compile-time flags)
READONLY_KEYS = {"IMU", "DBG", "SNS", "SMX"}

# ─── Parameter grouping for UI ───────────────────────────────────────────────
PARAM_GROUPS = {
    "Obstacle Distances": ["FOD", "SOD", "ACD", "CFD"],
    "PID Tuning":         ["KP", "KI", "KD"],
    "Speed / ESC":        ["MSP", "XSP", "BSP"],
    "Steering":           ["MNP", "XNP", "NTP"],
    "Tachometer":         ["ENH", "WDM"],
    "Control Loop":       ["LMS", "SPD1", "SPD2", "COE1", "COE2"],
    "Navigation":         ["WDD", "RCW", "STK"],
    "Hardware":           ["IMR", "SVR", "CAL", "BEN", "BML", "BLV"],
    "Flags (read-only)":  ["IMU", "DBG", "SNS", "SMX"],
}

# ─── Float keys (need float parsing/display, rest are int) ───────────────────
FLOAT_KEYS = {"KP", "KI", "KD", "WDM", "SPD1", "SPD2", "COE1", "COE2", "WDD", "BML", "BLV"}
