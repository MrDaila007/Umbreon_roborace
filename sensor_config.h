#pragma once
// ─── Sensor configuration ────────────────────────────────────────────────────
// Select sensor hardware via SENSOR_CONFIG in the main .ino file.
// All downstream code uses SENSOR_COUNT and role indices (IDX_*) instead of
// hardcoded s[0]–s[3].

#ifndef SENSOR_4X_LUNA
#define SENSOR_4X_LUNA      1
#endif
#ifndef SENSOR_6X_VL53L0X
#define SENSOR_6X_VL53L0X   2
#endif

#ifndef SENSOR_CONFIG
#error "Define SENSOR_CONFIG in your .ino before including sensor_config.h"
#endif

// ─── 4× TF-Luna (SerialPIO, UART) ──────────────────────────────────────────
#if SENSOR_CONFIG == SENSOR_4X_LUNA

#define SENSOR_COUNT        4
#define IDX_LEFT            0
#define IDX_FRONT_LEFT      1
#define IDX_FRONT_RIGHT     2
#define IDX_RIGHT           3
#define HAS_HARD_SIDES      0
#define MAX_SENSOR_RANGE    8000   // cm×10  (800 cm)

// Default thresholds (cm×10)
#define DEFAULT_FOD         1200
#define DEFAULT_SOD         1000
#define DEFAULT_ACD          800
#define DEFAULT_CFD          201

// ─── 6× VL53L0X (I2C, ToF) ─────────────────────────────────────────────────
#elif SENSOR_CONFIG == SENSOR_6X_VL53L0X

#define SENSOR_COUNT        6
#define IDX_HARD_LEFT       0
#define IDX_LEFT            1
#define IDX_FRONT_LEFT      2
#define IDX_FRONT_RIGHT     3
#define IDX_RIGHT           4
#define IDX_HARD_RIGHT      5
#define HAS_HARD_SIDES      1
#define MAX_SENSOR_RANGE    2000   // cm×10  (200 cm)

// Default thresholds — tighter for short-range sensors
#define DEFAULT_FOD          800
#define DEFAULT_SOD          600
#define DEFAULT_ACD          400
#define DEFAULT_CFD          150

// I2C bus (Wire1 on RP2350)
#define VL53_SDA            20
#define VL53_SCL            21

// XSHUT pins — one per sensor, active-low
#define VL53_XSHUT_PINS     {6, 7, 8, 9, 14, 15}

// Base I2C address — sensors get 0x30, 0x31, …, 0x35
#define VL53_BASE_ADDR      0x30

#else
#error "Unknown SENSOR_CONFIG value"
#endif
