/**
 * ______      _
 * | ___ \    | |
 * | |_/ /___ | |__   ___  _ __ __ _  ___ ___
 * |    // _ \| '_ \ / _ \| '__/ _` |/ __/ _ \
 * | |\ \ (_) | |_) | (_) | | | (_| | (_|  __/
 * \_| \_\___/|_.__/ \___/|_|  \__,_|\___\___|
 *
 * Umbreon — Autonomous roborace car
 *
 * Two hardware platforms from one codebase:
 *   Platform A: RP2350 (Pico 2) + ESP8266 WiFi bridge  (luna_car.h)
 *   Platform B: ESP32-S3 single-chip                    (hw_esp32s3.h)
 *
 * Distance units: cm×10  (e.g. 1200 = 120 cm, 200 = 20 cm)
 *
 * File layout:
 *   hw_config.h       — platform auto-detection
 *   config.h          — runtime-configurable parameters + state globals
 *   luna_car.h        — RP2350 hardware layer (SerialPIO LiDAR, Servo)
 *   hw_esp32s3.h      — ESP32-S3 hardware layer (VL53L0X I2C, WiFi, ESP32Servo)
 *   tests.h           — serial hardware tests (USB console)
 *   eeprom_settings.h — EEPROM persistence (load/save config)
 *   control.h         — autonomous driving logic (work, go_back, stuck detection)
 *   wifi_tests.h      — WiFi remote test routines (lidar, servo, esc, autotune...)
 *   commands.h        — command protocol, idle telemetry, ESC calibration
 */

#include "hw_config.h"

#pragma GCC optimize("Ofast")

// ─── Feature flags ──────────────────────────────────────────────────────────
#define USE_IMU         1       // 1 = enable MPU-6050 gyro, 0 = disable

#if PLATFORM_RP2350
#define USE_WIFI_DEBUG  1       // 1 = enable Wemos D1 Mini WiFi telemetry, 0 = disable
#endif

// Unified telemetry flag — true when any output channel is available
#if (PLATFORM_RP2350 && defined(USE_WIFI_DEBUG) && USE_WIFI_DEBUG) || PLATFORM_ESP32S3
  #define HAS_TELEM 1
#else
  #define HAS_TELEM 0
#endif

#if PLATFORM_RP2350 && HAS_TELEM
#define DEBUG_TX_PIN  16         // GP16 = UART0 TX → D1 Mini RX
#define DEBUG_RX_PIN  17         // GP17 = UART0 RX ← D1 Mini TX
#endif

// ─── Configuration & state ──────────────────────────────────────────────────
#include "config.h"

// ─── Platform-specific hardware layer ───────────────────────────────────────
#if PLATFORM_RP2350
  #include "luna_car.h"
  #define telem Serial1      // telemetry via UART to ESP8266 bridge
#elif PLATFORM_ESP32S3
  #include "hw_esp32s3.h"
  #define telem _telem_stream // telemetry via built-in WiFi
#endif

#include "tests.h"             // serial hardware tests (USB console)
#include <EEPROM.h>

Car car;

// ─── Firmware modules ───────────────────────────────────────────────────────
#include "eeprom_settings.h"   // EEPROM load/save (needs car, config.h, EEPROM.h)
#include "control.h"           // driving logic: work(), go_back() (needs car, config.h)
#include "wifi_tests.h"        // WiFi remote tests (needs car, telem, HAS_TELEM)
#include "commands.h"          // command protocol, calibration (needs everything above)

// ─── Setup ──────────────────────────────────────────────────────────────────
unsigned long next_loop = 0;

void setup() {
    Serial.begin(115200);

    // Try loading saved settings from EEPROM (falls back to compile-time defaults)
    EEPROM.begin(256);
    load_settings();

    car.init();
#if USE_IMU
    car.imu_init();
    car.imu_calibrate();  // sample gyro bias while stationary (~1s)
#endif
#if PLATFORM_RP2350 && HAS_TELEM
    Serial1.setTX(DEBUG_TX_PIN);
    Serial1.setRX(DEBUG_RX_PIN);
    Serial1.begin(115200);
#elif PLATFORM_ESP32S3
    wifi_setup();
#endif
#if HAS_TELEM
    telem.println("#ms,s0,s1,s2,s3,"
#if PLATFORM_ESP32S3
                    "s4,s5,"
#endif
                    "steer,speed,target"
#if USE_IMU
                    ",yaw,heading"
#endif
                    );
#endif

    // Run ESC+servo calibration on first boot (or after $RST + reboot)
    if (!cfg_calibrated) {
        run_calibration();
    } else {
        delay(3700);   // allow ESC to arm and sensors to start streaming
    }
}

// ─── Main loop ──────────────────────────────────────────────────────────────
void loop() {
    // Drain sensor bytes even between control ticks
    car.poll_lidars();
    if (cfg_bat_enabled) car.bat_update();  // battery ADC (self-throttles to 500ms)

#if PLATFORM_ESP32S3
    // Handle WiFi servers (HTTP, WebSocket, TCP clients)
    wifi_loop();
#endif

#if HAS_TELEM
    // Process incoming dashboard commands
    process_commands();
#endif

    unsigned long now = millis();
    if (now >= next_loop) {
        next_loop = max(now, next_loop + (unsigned long)cfg_loop_ms);

        // Manual drive ($DRVEN + $DRV) — works without $START
        bool drv_active = drv_enabled && manual_mode && (millis() - last_drv_ms < 500);

        if (car_running && !drv_active) {
            // Autonomous mode
            manual_mode = false;
            work();
        } else if (drv_active) {
            // Manual drive mode — apply $DRV commands directly
            car.poll_lidars();
#if USE_IMU
            car.imu_update();
#endif
            int* s = car.read_sensors();
            car.write_steer(manual_steer);
            car.write_speed_ms(manual_speed);
            car.pid_control_motor();
#if HAS_TELEM
            telem.print(millis());              telem.print(',');
            telem.print(s[0]);                  telem.print(',');
            telem.print(s[1]);                  telem.print(',');
            telem.print(s[2]);                  telem.print(',');
            telem.print(s[3]);                  telem.print(',');
#if PLATFORM_ESP32S3
            telem.print(s[4]);                  telem.print(',');
            telem.print(s[5]);                  telem.print(',');
#endif
            telem.print(manual_steer);          telem.print(',');
            telem.print(get_speed(), 2);        telem.print(',');
            telem.print(manual_speed, 1);
#if USE_IMU
            telem.print(',');
            telem.print(car.yaw_rate, 1);       telem.print(',');
            telem.print(car.heading, 1);
#endif
            telem.println();
#endif
        }
#if HAS_TELEM
        else {
            send_idle_telemetry();
        }
#endif

        // ── Low-voltage safety cutoff ────────────────────────────────────
        static unsigned long bat_low_since = 0;
        if (cfg_bat_enabled && car.bat_voltage > 0.5f && car.bat_voltage < cfg_bat_low) {
            if (bat_low_since == 0) bat_low_since = now;
            else if (now - bat_low_since > 10000) {
                // Low voltage for >10 seconds — emergency stop
                if (car_running || drv_enabled) {
                    car_running = false;
                    drv_enabled = false;
                    manual_mode = false;
                    car.write_speed(0);
                    car.write_steer(0);
#if HAS_TELEM
                    telem.println("$STS:STOP");
                    telem.println("$T:BAT,phase=LOW_VOLTAGE_CUTOFF");
#endif
                }
            }
        } else {
            bat_low_since = 0;  // voltage OK — reset timer
        }
    }
}
