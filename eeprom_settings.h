#pragma once
// ═════════════════════════════════════════════════════════════════════════════
// eeprom_settings.h — EEPROM persistence for runtime configuration
//
// Packed struct at address 0 (~60 bytes).  Magic + version + checksum guard
// against loading stale/corrupt data from a different firmware version.
//
// Depends on: config.h globals, <EEPROM.h>
// ═════════════════════════════════════════════════════════════════════════════

#define SETTINGS_MAGIC   0x554D4252   // "UMBR"
#define SETTINGS_VERSION 5
#define SETTINGS_ADDR    0

struct __attribute__((packed)) CarSettings {
    uint32_t magic;
    uint8_t  version;
    // Obstacle thresholds
    int16_t  front_obstacle_dist;
    int16_t  side_open_dist;
    int16_t  all_close_dist;
    int16_t  close_front_dist;
    // PID
    float    pid_kp;
    float    pid_ki;
    float    pid_kd;
    // ESC (µs, 1000–2000)
    int16_t  min_speed;
    int16_t  max_speed;
    int16_t  min_bspeed;
    // Steering
    int8_t   min_point;
    int8_t   max_point;
    int8_t   neutral_point;
    // Tachometer
    int8_t   encoder_holes;
    float    wheel_diam_m;
    // Control
    int8_t   loop_ms;
    float    spd_clear;
    float    spd_blocked;
    float    coe_clear;
    float    coe_blocked;
    // Navigation
    float    wrong_dir_deg;
    uint8_t  race_cw;
    int8_t   stuck_thresh;
    // IMU
    uint8_t  imu_rotate;
    // Servo
    uint8_t  servo_reverse;
    // Calibration
    uint8_t  calibrated;
    // Battery
    uint8_t  bat_enabled;
    float    bat_multiplier;
    float    bat_low;
    // Checksum (sum of all preceding bytes)
    uint8_t  checksum;
};

static uint8_t compute_checksum(const CarSettings& s) {
    uint8_t sum = 0;
    const uint8_t* p = (const uint8_t*)&s;
    size_t len = sizeof(CarSettings) - 1;  // exclude checksum byte
    for (size_t i = 0; i < len; i++) sum += p[i];
    return sum;
}

static void populate_struct(CarSettings& s) {
    s.magic   = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;
    s.front_obstacle_dist = (int16_t)cfg_front_obstacle_dist;
    s.side_open_dist      = (int16_t)cfg_side_open_dist;
    s.all_close_dist      = (int16_t)cfg_all_close_dist;
    s.close_front_dist    = (int16_t)cfg_close_front_dist;
    s.pid_kp       = cfg_pid_kp;
    s.pid_ki       = cfg_pid_ki;
    s.pid_kd       = cfg_pid_kd;
    s.min_speed    = (int16_t)cfg_min_speed;
    s.max_speed    = (int16_t)cfg_max_speed;
    s.min_bspeed   = (int16_t)cfg_min_bspeed;
    s.min_point    = (int8_t)cfg_min_point;
    s.max_point    = (int8_t)cfg_max_point;
    s.neutral_point = (int8_t)cfg_neutral_point;
    s.encoder_holes = (int8_t)cfg_encoder_holes;
    s.wheel_diam_m  = cfg_wheel_diam_m;
    s.loop_ms       = (int8_t)cfg_loop_ms;
    s.spd_clear     = cfg_spd_clear;
    s.spd_blocked   = cfg_spd_blocked;
    s.coe_clear     = cfg_coe_clear;
    s.coe_blocked   = cfg_coe_blocked;
    s.wrong_dir_deg = cfg_wrong_dir_deg;
    s.race_cw       = cfg_race_cw ? 1 : 0;
    s.stuck_thresh  = (int8_t)cfg_stuck_thresh;
    s.imu_rotate    = cfg_imu_rotate ? 1 : 0;
    s.servo_reverse = cfg_servo_reverse ? 1 : 0;
    s.calibrated    = cfg_calibrated ? 1 : 0;
    s.bat_enabled    = cfg_bat_enabled ? 1 : 0;
    s.bat_multiplier = cfg_bat_multiplier;
    s.bat_low        = cfg_bat_low;
    s.checksum      = compute_checksum(s);
}

static void apply_struct(const CarSettings& s) {
    cfg_front_obstacle_dist = s.front_obstacle_dist;
    cfg_side_open_dist      = s.side_open_dist;
    cfg_all_close_dist      = s.all_close_dist;
    cfg_close_front_dist    = s.close_front_dist;
    cfg_pid_kp       = s.pid_kp;
    cfg_pid_ki       = s.pid_ki;
    cfg_pid_kd       = s.pid_kd;
    cfg_min_speed    = s.min_speed;
    cfg_max_speed    = s.max_speed;
    cfg_min_bspeed   = s.min_bspeed;
    cfg_min_point    = s.min_point;
    cfg_max_point    = s.max_point;
    cfg_neutral_point = s.neutral_point;
    cfg_encoder_holes = s.encoder_holes;
    cfg_wheel_diam_m  = s.wheel_diam_m;
    cfg_loop_ms       = s.loop_ms;
    cfg_spd_clear     = s.spd_clear;
    cfg_spd_blocked   = s.spd_blocked;
    cfg_coe_clear     = s.coe_clear;
    cfg_coe_blocked   = s.coe_blocked;
    cfg_wrong_dir_deg = s.wrong_dir_deg;
    cfg_race_cw       = s.race_cw != 0;
    cfg_stuck_thresh  = s.stuck_thresh;
    cfg_imu_rotate    = s.imu_rotate != 0;
    cfg_servo_reverse = s.servo_reverse != 0;
    cfg_calibrated    = s.calibrated != 0;
    cfg_bat_enabled    = s.bat_enabled != 0;
    cfg_bat_multiplier = s.bat_multiplier;
    cfg_bat_low        = s.bat_low;
}

bool load_settings() {
    CarSettings s;
    EEPROM.get(SETTINGS_ADDR, s);
    if (s.magic != SETTINGS_MAGIC || s.version != SETTINGS_VERSION)
        return false;
    if (compute_checksum(s) != s.checksum)
        return false;
    apply_struct(s);
    return true;
}

bool save_settings() {
    CarSettings s;
    populate_struct(s);
    EEPROM.put(SETTINGS_ADDR, s);
    EEPROM.commit();
    return true;
}
