#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// config.h — Runtime-configurable parameters and state globals
//
// All cfg_* variables have compile-time defaults and can be changed at runtime
// via the $SET/$GET command protocol.  They are persisted to EEPROM with $SAVE.
//
// Hardware layers (luna_car.h, hw_esp32s3.h) declare these as `extern`.
// ═════════════════════════════════════════════════════════════════════════════

// ─── Obstacle thresholds (cm×10) ─────────────────────────────────────────────
int   cfg_front_obstacle_dist = 1200;   // 120 cm — start steering around
int   cfg_side_open_dist      = 1000;   // 100 cm — side is "open"
int   cfg_all_close_dist      =  800;   // 80 cm  — boxed in
int   cfg_close_front_dist    =  201;   // 20.1 cm — very close (stuck trigger)

// ─── PID gains ───────────────────────────────────────────────────────────────
float cfg_pid_kp = 60.0f;
float cfg_pid_ki = 40.0f;
float cfg_pid_kd = 6.0f;

// ─── ESC limits (µs, 1000–2000) ─────────────────────────────────────────────
int   cfg_min_speed   = 1540;    // slowest forward
int   cfg_max_speed   = 1700;    // fastest forward
int   cfg_min_bspeed  = 1460;    // slowest reverse

// ─── Steering limits (servo degrees) ─────────────────────────────────────────
int   cfg_min_point     = 40;
int   cfg_max_point     = 140;
int   cfg_neutral_point = 90;

// ─── Tachometer / encoder ────────────────────────────────────────────────────
int   cfg_encoder_holes = 62;           // holes per revolution
float cfg_wheel_diam_m  = 0.060f;      // 60 mm wheel

// ─── Control loop ────────────────────────────────────────────────────────────
int   cfg_loop_ms      = 40;           // control tick interval (ms)
float cfg_spd_clear    = 2.7f;         // speed when path clear (m/s)
float cfg_spd_blocked  = 0.8f;         // speed when path blocked (m/s)
float cfg_coe_clear    = 0.3f;         // steering coefficient (clear)
float cfg_coe_blocked  = 0.7f;         // steering coefficient (blocked)

// ─── Navigation ──────────────────────────────────────────────────────────────
float cfg_wrong_dir_deg = 120.0f;      // heading threshold for wrong-direction (°)
bool  cfg_race_cw       = true;        // true = clockwise race direction
int   cfg_stuck_thresh  = 25;          // stuck ticks before recovery (~1s at 40ms)

// ─── Hardware flags ──────────────────────────────────────────────────────────
bool  cfg_imu_rotate    = true;        // negate yaw for 180°-rotated IMU
bool  cfg_servo_reverse = true;        // invert servo direction
bool  cfg_calibrated    = false;       // ESC calibrated flag (set after calibration)

// ─── Battery monitoring ──────────────────────────────────────────────────────
bool  cfg_bat_enabled    = false;      // enable battery ADC
float cfg_bat_multiplier = 2.8f;       // (R1+R2)/R2 — default for R1=18k, R2=10k
float cfg_bat_low        = 6.0f;       // low voltage cutoff (V) — stop after 10s

// ─── Start/Stop state ────────────────────────────────────────────────────────
// #define COMPETITION_MODE     // uncomment to start driving immediately
#ifdef COMPETITION_MODE
bool car_running = true;
#else
bool car_running = false;
#endif

// ─── Manual drive mode ($DRV command) ────────────────────────────────────────
bool manual_mode = false;
bool drv_enabled = false;       // $DRVEN/$DRVOFF — allows $DRV without $START
unsigned long last_drv_ms = 0;
int manual_steer = 0;
float manual_speed = 0.0f;
