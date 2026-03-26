#pragma once

#include <stdint.h>

// ─── EEPROM settings ────────────────────────────────────────────────────────
#define SETTINGS_MAGIC   0x554D4252   // "UMBR"
#define SETTINGS_VERSION 6
#define SETTINGS_ADDR    0

struct __attribute__((packed)) CarSettings {
    uint32_t magic;
    uint8_t  version;
    uint8_t  sensor_config;       // SENSOR_CONFIG value — reject if mismatch
    // Obstacle thresholds
    int16_t  front_obstacle_dist;
    int16_t  side_open_dist;
    int16_t  all_close_dist;
    int16_t  close_front_dist;
    // PID
    float    pid_kp;
    float    pid_ki;
    float    pid_kd;
    // ESC (us, 1000-2000)
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
