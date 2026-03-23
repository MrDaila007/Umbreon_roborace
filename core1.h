#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// core1.h — Dual-core infrastructure for RP2350
//
// Core 0: control loop, sensors, PID, WiFi commands, Servo/ESC, watchdog
// Core 1: OLED menu rendering, rotary encoder polling
//
// Communication: Core 1 → Core 0 via rp2040.fifo (hardware 32-entry FIFO)
// Shared data:   sensor_snapshot[] (written Core 0, read Core 1)
// I2C0 mutex:    protects Wire bus shared between IMU (Core 0) and OLED (Core 1)
// ─────────────────────────────────────────────────────────────────────────────

#include <pico/mutex.h>
#include "sensor_config.h"

// ─── I2C0 mutex (IMU on Core 0 ↔ OLED on Core 1) ───────────────────────────
auto_init_mutex(i2c0_mutex);

// ─── Inter-core FIFO messages (Core 1 → Core 0) ────────────────────────────
enum Core1Msg : uint32_t {
    MSG_NONE           = 0,
    MSG_STOP_CAR       = 1,
    MSG_START_CAR      = 2,
    MSG_SAVE_EEPROM    = 3,
    MSG_LOAD_EEPROM    = 4,
    MSG_RESET_DEFAULTS = 5,
    MSG_RUN_TEST_BASE  = 0x100,  // + test index (0–7)
};

// ─── Shared state (Core 0 writes, Core 1 reads) ─────────────────────────────
volatile bool core1_test_active = false;   // true while Core 0 runs a test from menu
volatile int  sensor_snapshot[SENSOR_COUNT] = {0};  // latest sensor values for OLED dashboard
