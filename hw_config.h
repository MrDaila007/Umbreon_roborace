#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// hw_config.h — Platform auto-detection for Umbreon roborace firmware
//
// Supported platforms:
//   RP2350 (Pico 2)   — FQBN rp2040:rp2040:rpipico2 (arduino-pico)
//   ESP32-S3           — FQBN esp32:esp32:esp32s3 (arduino-esp32)
//
// This header detects the target at compile time and sets:
//   PLATFORM_RP2350   = 1 or 0
//   PLATFORM_ESP32S3  = 1 or 0
// ─────────────────────────────────────────────────────────────────────────────

#if defined(ARDUINO_ARCH_RP2040)
  // arduino-pico uses ARDUINO_ARCH_RP2040 for both RP2040 and RP2350
  #define PLATFORM_RP2350    1
  #define PLATFORM_ESP32S3   0

#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)
  #define PLATFORM_RP2350    0
  #define PLATFORM_ESP32S3   1

#else
  #error "Unsupported platform — expected RP2350 (arduino-pico) or ESP32-S3 (arduino-esp32)"
#endif
