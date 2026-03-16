// WiFi configuration for Umbreon ESP32-S3 (example).
// Copy this file to `wifi_config.h` and edit the values.
// `wifi_config.h` is git-ignored so your real credentials stay local.
//
// If wifi_config.h does not exist, the firmware uses safe defaults
// (AP mode, SSID "Umbreon", password "12345678").

#pragma once

// ─── WiFi mode ──────────────────────────────────────────────────────────────
// WIFI_AP  — create own hotspot (default, best for competition/field)
// WIFI_STA — connect to existing router (best for lab/development)
// In STA mode, falls back to AP if connection fails after STA_TIMEOUT_S.
#define WIFI_MODE_SETTING  WIFI_AP

// ─── STA credentials (your home/lab WiFi) ───────────────────────────────────
#define STA_SSID       "YourWiFi"
#define STA_PASS       "YourPassword"
#define STA_TIMEOUT_S  15

// ─── AP credentials (fallback / competition) ────────────────────────────────
#define AP_SSID        "Umbreon"
#define AP_PASS        "12345678"   // min 8 chars for WPA2
