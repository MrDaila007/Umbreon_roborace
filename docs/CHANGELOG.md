# Changelog

All notable changes to the Umbreon Roborace project are documented in this file.

## [Unreleased] — fix/codebase-audit

### Safety (P0)
- Add 2s timeout to busy-wait loops in `go_back`/`go_back_long` to prevent infinite hangs
- Guard division by zero when `cfg_encoder_holes=0` in `get_speed`/PID
- Enable RP2350 hardware watchdog with 8s timeout

### Correctness (P1)
- Fix XSS vulnerability in web dashboard test results table — use `textContent` instead of `innerHTML`
- Fix race condition on `_connected` flag — use `threading.Event`
- Fix `$RST` servo_reverse default to match compile-time `false`
- Increase float precision in `$GET` response to 4 decimal places

### Robustness (P2)
- Reduce tachometer debounce from 500 us to 200 us for better accuracy
- Add flush and error handling for CSV recording
- Add logging for WebSocket broadcast/handler exceptions
- Add parameter range validation on settings file load
- Make ROS2 `LaserScan`/`Range` dynamic for 4 or 6 sensors
- Make plots and telemetry store support up to 6 sensors

### Quality (P3)
- Add Serial feedback during IMU gyro calibration
- Replace magic number 800 with `WALL_FOLLOW_BIAS` constant
- Add firmware version string (`FW_VERSION`) to boot and `$GET`
- Add `threading.Lock` to protocol `_detected_sensor_count`
- Add input validation before `$SET` in settings panel
- Add microseconds to recording filename to prevent collision
- Make Makefile arduino-cli path cross-platform
- Add missing dashboard/sim module imports to CI
- Pin minimum versions in `dashboard/requirements.txt`

### Sensor Configuration
- Add modular sensor configuration via `sensor_config.h` with compile-time `SENSOR_CONFIG` flag
- Support 4x TF-Luna LiDAR (SerialPIO UART) and 6x VL53L0X ToF (I2C) configurations
- Steering logic uses role indices (`IDX_LEFT`, `IDX_FRONT_LEFT`, etc.) instead of hardcoded array positions
- Dynamic telemetry output loops over `SENSOR_COUNT`
- EEPROM v6 with `sensor_config` field; rejects mismatched saved settings
- `$GET` reports `SNS` (sensor count) and `SMX` (max range) as read-only
- Dashboard/simulator auto-detect sensor count from telemetry header
- Web UI auto-adapts grid layout (2 or 3 columns) based on sensor count

### Build Fixes
- Fix compile error: include `sensor_config.h` early in `.ino` so `DEFAULT_FOD`/`SOD`/`ACD`/`CFD` macros are defined before use in global initializers
- Install `Adafruit_VL53L0X` library in CI firmware build step
- Document external library dependency in `CLAUDE.md`

### Documentation
- Add `docs/CHANGELOG.md` with full project history
- Update `docs/architecture.md`: new sensor configuration section, VL53L0X driver docs, role indices, dynamic telemetry format, EEPROM v6
- Update `docs/tuning.md`: per-config default thresholds table, VL53L0X sensor notes
- Update `docs/dashboard.md`: multi-sensor plots/geometry, `SNS`/`SMX` parameters, `--sensors` sim flag
- Update `docs/regulations-notes.md`: threshold references for both sensor configs

---

## [0.6.0] — 2026-03-16

### Fixed
- Recalculate PID gains (Tyreus-Luyben) for microsecond ESC scale (KP=4.18, KI=2.93, KD=0.43)
- Improve PID responsiveness with updated derivative filtering

---

## [0.5.0] — 2026-03-15

### Added
- Battery monitoring via ADC on GP26 with resistor divider (R1=18k, R2=10k)
- Low-voltage safety cutoff (default 6.0V, 10s sustained -> emergency stop)
- `$BAT` command to read battery voltage
- Battery UI panel in ESP web dashboard with enable/disable toggle
- IMU calibration feature (bias calibration on boot, ~1s, 200 samples)
- IMU rotate flag (`IMR`) for 180-degree-rotated IMU
- Servo reverse flag (`SVR`) for reversed servo direction
- ESC calibration wizard (max 2000 -> min 1000 -> neutral 1500 us, saved to EEPROM)
- ESC switched to `writeMicroseconds()` — forward [1540-1700], reverse [1460-1000], neutral 1500 us
- Light mode toggle in ESP web dashboard
- Grouped configuration display in web UI (checkboxes for IMU/servo settings)

### Changed
- Increase command buffer sizes for WiFi debug and parsing functions
- Refactor web UI styles for improved readability and consistency

### Documentation
- Update docs for IMU filtering, battery monitoring, and ESP web UI changes

---

## [0.4.0] — 2026-03-14

### Added
- ESP32-S3 hardware abstraction layer (`hw_esp32s3.h`)
- Platform auto-detection header — firmware compiles for RP2350 or ESP32-S3
- Standalone hardware test sketch for ESP32-S3
- ESP32-S3 build job in CI pipeline
- Live track map in ESP web dashboard (canvas-based, real-time position)
- `$DRVEN`/`$DRVOFF` commands for manual drive without `$START`
- STA/AP dual WiFi mode for ESP bridge
- Hardware test WiFi sketch with telemetry and I2C scanner

### Fixed
- UART pin assignment — GP16 is TX, GP17 is RX (was swapped)
- Default WiFi mode changed from STA to AP for hotspot reliability
- Volatile type mismatch in `get_speed()` for ESP32 build

---

## [0.3.0] — 2026-03-13

### Added
- ROS2 Humble bridge (Docker) with standard message types
  - Publishers: range, IMU, odometry, laser scan, status (25 Hz)
  - Subscribers: `cmd_vel`, throttle, steering
  - Services: connect, disconnect, ping, start, stop, save, load, reset
  - TF tree: static sensor frames + dynamic odometry
- `$DRV` manual drive command (steer + speed, 500ms timeout)
- GitHub Actions CI pipeline: firmware build, Python lint, import check, ROS2 Docker build
- Code style review job (reviewdog + ruff + cppcheck) for pull requests
- Web dashboard on ESP bridge (HTTP :80, WebSocket :81, TCP :23)
- Start/stop control, remote hardware tests, auto-tune UI, mobile-friendly layout
- Rotation controls in Track Map
- Dashboard demo GIF

### Changed
- Wemos D1 Mini replaces generic ESP8266 references throughout docs and code
- Build scripts updated with firmware compilation features (`make compile`, `make upload`)
- README rewritten with full project overview, command protocol, and architecture

---

## [0.2.0] — 2026-03-13

### Added
- Runtime configuration system — `#define` constants converted to `cfg_*` globals
- EEPROM persistence with magic number `0x554D4252` ("UMBR")
- ASCII command protocol over TCP (`$GET`, `$SET`, `$SAVE`, `$LOAD`, `$RST`, `$PING`)
- Python dashboard — web UI (aiohttp + vanilla JS) and tkinter desktop version
- Simulator bridge mode (`--bridge`, port 8023) for dashboard testing
- Makefile and make.bat for common tasks (`make web`, `make sim`, `make dashboard-tk`, etc.)

---

## [0.1.0] — 2026-03-12

### Added
- IMU support for MPU-6050 gyro (I2C) with wrong-direction detection
- PID speed control with auto-tuning and CSV logging for analysis
- Optical encoder tachometer (62 holes/rev, 60 mm wheel) with debounce
- Hardware test suite in `tests.h` (lidar, servo, taho, esc, speed, autotune, reactive, cal)
- 2D kinematic bicycle simulator with LiDAR ray casting
- Track builder with rounded corners and chicane barriers

---

## [0.0.1] — 2026-03-10

### Added
- Initial firmware for RP2350 (Pico 2) with 4x TF-Luna LiDAR
- 40 ms control loop: read sensors -> steering logic -> speed logic -> telemetry
- SerialPIO-based LiDAR polling
- Servo and ESC PWM control
- Architecture and tuning documentation
