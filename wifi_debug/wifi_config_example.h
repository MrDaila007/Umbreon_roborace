// WiFi configuration for Umbreon WiFi bridge (example).
// Copy this file to `wifi_config.h` and edit the values.
// `wifi_config.h` is git-ignored so your real credentials stay local.

#pragma once

// ─── WiFi mode ──────────────────────────────────────────────────────────────
// Set to WIFI_STA to connect to your router, WIFI_AP to create own hotspot.
// In STA mode, falls back to AP if connection fails after STA_TIMEOUT_S.
// You can override WIFI_MODE here if you want a different default.
// Example:
//   #define WIFI_MODE WIFI_STA
// or:
//   #define WIFI_MODE WIFI_AP

// ─── STA credentials (your home/lab WiFi) ───────────────────────────────────
// Replace with your real network name and password.
#define STA_SSID  "YourWiFi"
#define STA_PASS  "YourPassword"

// Seconds to wait before falling back from STA to AP mode.
#define STA_TIMEOUT_S 30

// ─── AP credentials (fallback / competition) ────────────────────────────────
// These are used when running as an access point.
#define AP_SSID   "Umbreon"
#define AP_PASS   "12345678"   // min 8 chars for WPA2
