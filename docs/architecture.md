# Architecture

## Overview

The firmware supports two hardware platforms from a single codebase, auto-detected at compile time:

### Platform A: RP2350 + ESP8266 (two-chip)

```
Pico 2 Firmware ──UART1──▶ Wemos D1 Mini WiFi Bridge ──┬─ TCP:23 ──▶ Dashboard / ROS2
   (luna_car.h)     GP17 TX → RX    (wifi_debug.ino)    ├─ HTTP:80 ─▶ Built-in Web UI
                    GP16 RX ← TX                        └─ WS:81 ──▶ Web UI (real-time)
```

### Platform B: ESP32-S3 (single-chip)

```
ESP32-S3 Firmware ──┬─ TCP:23 ──▶ Dashboard / ROS2
   (hw_esp32s3.h)   ├─ HTTP:80 ─▶ Built-in Web UI
   WiFi built-in    └─ WS:81 ──▶ Web UI (real-time)
```

| File | Role |
|---|---|
| `hw_config.h` | Platform auto-detection (`PLATFORM_RP2350` / `PLATFORM_ESP32S3`) |
| `luna_car.h` | RP2350 hardware layer — SerialPIO LiDAR, Servo, PID, IMU |
| `hw_esp32s3.h` | ESP32-S3 hardware layer — 6× VL53L0X I2C, WiFi, ESP32Servo, PID, IMU |
| `Umbreon_roborace.ino` | Control logic, runtime config, EEPROM, command protocol (shared) |
| `wifi_debug/wifi_debug.ino` | Wemos D1 Mini firmware — WiFi AP + UART↔TCP bridge (RP2350 only) |
| `dashboard/` | Python app — live plots, track map, remote settings editor |

### Platform auto-detection (`hw_config.h`)

```cpp
#if defined(ARDUINO_ARCH_RP2040)        // arduino-pico (covers RP2350)
  #define PLATFORM_RP2350   1
  #define PLATFORM_ESP32S3  0
#elif defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)
  #define PLATFORM_RP2350   0
  #define PLATFORM_ESP32S3  1
#endif
```

### Telemetry abstraction

Both platforms use `telem` for telemetry output. On RP2350, `telem` is `Serial1` (UART to ESP8266). On ESP32-S3, `telem` is `_telem_stream` — a `TelemetryStream` object that line-buffers output and broadcasts to TCP + WebSocket clients.

The unified `HAS_TELEM` flag replaces `USE_WIFI_DEBUG`:
- RP2350 with `USE_WIFI_DEBUG=1` → `HAS_TELEM=1`
- ESP32-S3 → always `HAS_TELEM=1`
- RP2350 without WiFi → `HAS_TELEM=0`

---

## Hardware Layers

Both platforms expose the same `Car` class interface (`init`, `poll_lidars`, `read_sensors`, `write_speed`, `write_steer`, `pid_control_motor`, `bat_update`, `imu_init`, `imu_update`), allowing `Umbreon_roborace.ino` to work unchanged.

---

## luna_car.h (RP2350)

### TF-Luna LiDAR driver (UART)

Each of the 4 sensors is read over a dedicated **SerialPIO** port (software UART, RX-only).

**Frame format (9 bytes):**
```
[0]  0x59        header byte 1
[1]  0x59        header byte 2
[2]  Dist_L      distance low byte  (cm)
[3]  Dist_H      distance high byte (cm)
[4]  Str_L       signal strength low
[5]  Str_H       signal strength high
[6]  Temp_L      chip temperature low
[7]  Temp_H      chip temperature high
[8]  Checksum    sum of bytes 0–7, truncated to uint8
```

`poll_lidars()` drains all pending bytes from each SerialPIO and feeds them into a per-sensor state machine (`_process_byte`). It must be called **every loop iteration** to prevent UART buffer overflow.

`read_sensors()` returns a static `int[4]` array of distances in **cm×10**:
- `s[0]` Left (GP2)
- `s[1]` Front-Left (GP3)
- `s[2]` Front-Right (GP4)
- `s[3]` Right (GP5)

Using cm×10 keeps the same numeric range as the original Sharp IR sensors, so all threshold constants remain unchanged.

If a sensor has not yet received a valid packet, its slot returns `9999` (treated as "very far").

---

### Tachometer & speed

An **optical encoder** on the central rod (GP13) fires a RISING interrupt for each of the **62 holes** in the disc.

`get_speed()` computes speed in **m/s**:

```
speed = (π × wheel_diameter) / (ENCODER_HOLES × pulse_period_s)
      = (π × 0.063) / (62 × elapsed_s)
```

- Wheel diameter: 60 mm
- Encoder holes: 62 per revolution → 62 pulses per wheel revolution (measured)
- At 3 m/s: ~935 pulses/s, one pulse every ~1.1 ms
- Debounce: pulses closer than **1 ms** are ignored
- Stopped detection: if `elapsed > 3 × last_period` or no pulse ever seen, returns 0

---

### Motor PID

`pid_control_motor()` runs a **PD controller** (no integrator) each tick:

```
error   = target_speed - get_speed()
output  = error × P  +  (error - last_error) × D
```

| Gain | Value |
|---|---|
| P | 200 |
| D | 110 |

Output is clamped to `[0, 100]` — only forward correction, braking is handled by `write_speed(0)`.

---

### Steering & ESC mapping

Both the steering servo and motor ESC use the Servo library (PWM).

**Steering** (`write_steer(s)`, s ∈ [-1000, 1000]):
- Maps to servo angle [MNP, XNP], neutral NTP (default 40°–140°, neutral 90°)
- Input is inverted when `SVR=1` (configurable servo reverse)

**ESC** (`write_speed(s)`, s ∈ [-1000, 1000]):
- Uses `writeMicroseconds()` — range 1000–2000 µs, neutral 1500 µs
- Forward: mapped to [MSP, XSP] µs (default 1540–1700)
- Reverse: mapped to [1000, BSP] µs (default 1000–1460)
- Neutral/brake: 1500 µs

---

### IMU — MPU-6050 gyro Z (optional)

Enabled by `#define USE_IMU 1` in `Umbreon_roborace.ino` (before the `#include`).

**Wiring:**

| MPU-6050 pin | Pico 2 pin |
|---|---|
| VCC | 3.3 V |
| GND | GND |
| SDA | GP0 |
| SCL | GP1 |

I2C1 at 400 kHz. Only the **gyro Z-axis** is read (registers `0x47`–`0x48`).

**Configuration:**
- Full-scale range: ±500°/s (sensitivity 65.5 LSB/°/s)
- Digital Low-Pass Filter: ~42 Hz bandwidth (DLPF_CFG = 3)

**Functions:**

| Function | Description |
|---|---|
| `imu_init()` | Wakes sensor, sets gyro range & DLPF. Returns `false` if sensor not found (I2C NACK) — all IMU calls become no-ops. |
| `imu_calibrate()` | Samples 200 gyro Z readings over ~1s while stationary, computes average bias offset. Called once at boot after `imu_init()`. |
| `imu_update()` | Reads gyro Z, subtracts bias, applies EMA filter (α=0.3), applies dead zone (0.4°/s), integrates into `heading`. Called every control tick (25 Hz). |
| `reset_heading()` | Zeroes `heading` accumulator. Called after every recovery manoeuvre. |

**Signal pipeline:** `raw → bias subtract → EMA low-pass → dead zone → integrate`

**Members:**
- `yaw_rate` — filtered gyro Z reading in °/s (positive = counter-clockwise). Negated when `IMR=1` (IMU mounted 180° rotated)
- `heading` — cumulative heading change in degrees since last reset
- `gyro_bias` — calibrated Z-axis offset (°/s), subtracted from every reading

All IMU code is wrapped in `#if USE_IMU` — setting it to `0` strips the IMU entirely from the binary (no `Wire.h` overhead).

---

## hw_esp32s3.h (ESP32-S3)

On the ESP32-S3, sensors, WiFi, and the web server are all handled on one chip — no separate WiFi bridge needed.

### VL53L0X ToF sensors (I2C, XSHUT address assignment)

6× VL53L0X time-of-flight sensors share one **I2C bus** (GPIO 8 SDA, GPIO 9 SCL). Each sensor has a dedicated XSHUT pin for boot-time address assignment:

| Index | Position | XSHUT GPIO | I2C Address |
|---|---|---|---|
| s[0] | Left | GPIO 4 | 0x30 |
| s[1] | Front-Left | GPIO 5 | 0x31 |
| s[2] | Front-Right | GPIO 6 | 0x32 |
| s[3] | Right | GPIO 7 | 0x33 |
| s[4] | Front | GPIO 12 | 0x34 |
| s[5] | Rear | GPIO 14 | 0x35 |

**Address assignment procedure** (automatic at boot):
1. All XSHUT pins held LOW (sensors in hardware standby)
2. Release one at a time, assign unique I2C address via `setAddress()`
3. Start continuous ranging with 33ms timing budget

No manual pre-configuration needed — all sensors ship at default address 0x29 and are configured automatically.

`poll_lidars()` calls `readRangeContinuousMillimeters()` on each sensor. VL53L0X reports in mm, which equals cm×10 (the firmware's native distance unit).

`read_sensors()` returns `int[6]` — s[0]–s[3] keep original roles for backward compatibility, s[4] (Front) and s[5] (Rear) are additive. Sensors without valid readings return `9999`.

### Built-in WiFi server

The ESP32-S3 runs its own WiFi AP and web server (same AP name "Umbreon", password "12345678"):

- **Port 80**: HTTP — serves the built-in web dashboard (`web_ui.h` PROGMEM)
- **Port 81**: WebSocket — real-time bidirectional relay
- **Port 23**: Raw TCP — backward compat with Python dashboard / ROS2 bridge

WiFi also supports **STA mode** (join existing network) with AP fallback.

### TelemetryStream

A custom `Stream` subclass that replaces `Serial1`:

- **Output**: line-buffered; when `\n` is written, the complete line is broadcast to all connected TCP and WebSocket clients
- **Input**: ring buffer fed by TCP/WebSocket receive handlers; firmware reads commands via `telem.available()` / `telem.read()` as usual

This allows all `telem.print()` calls in `Umbreon_roborace.ino` to work identically on both platforms.

### Pin layout

| Signal | GPIO |
|---|---|
| I2C SDA (VL53L0X + IMU) | GPIO 8 |
| I2C SCL (VL53L0X + IMU) | GPIO 9 |
| Steering servo | GPIO 10 |
| Motor ESC | GPIO 11 |
| VL53L0X XSHUT [0]–[3] | GPIO 4, 5, 6, 7 |
| VL53L0X XSHUT [4]–[5] | GPIO 12, 14 |
| Tachometer | GPIO 13 |
| Battery ADC | GPIO 26 |
| Status LED | GPIO 2 |

### ESP32-specific notes

- **ESP32Servo**: Uses LEDC PWM channels (same API as standard Servo library)
- **IRAM_ATTR**: Required on tachometer ISR (ESP32 runs ISRs from IRAM, not flash)
- **PROGMEM**: Memory-mapped on ESP32, so `server.send()` works directly (no `send_P` needed)
- **IMU**: Same MPU-6050 setup, shared I2C bus with VL53L0X sensors (skips `Wire.begin()` since `Car::init()` already called it)
- **Battery monitoring**: Same ADC + resistor divider as RP2350 (GPIO 26, EMA filter, 500ms self-throttle)

---

## Umbreon_roborace.ino — Control Logic

### Main loop

```
loop()
├── poll_lidars()          // always — keep UART buffer empty
├── bat_update()           // battery ADC (if BEN=1, self-throttles to 500ms)
└── every 40 ms → work()
    ├── poll_lidars()      // fresh data before decisions
    ├── imu_update()       // (if USE_IMU) — bias, EMA, dead zone, integrate
    ├── read_sensors()
    ├── steering
    ├── speed
    ├── pid_control_motor()
    ├── WiFi telemetry        // (if HAS_TELEM)
    ├── stuck check
    ├── wrong-direction / dead-end check
    └── low-voltage cutoff    // (if BEN=1, >10s below BLV → emergency stop)
```

### Steering logic

```
if both sides > SIDE_OPEN_DIST:
    diff = +800            // open corridor — bias right wall
else:
    diff = s[3] - s[0]     // track mid-channel between walls

if all 4 sensors < ALL_CLOSE_DIST:
    diff = +800            // boxed in — hard right turn to escape
```

`diff` is then multiplied by `coef` (0.3 clear / 0.7 blocked) before `write_steer()`.

### Speed logic

| Front sensors | coef | speed |
|---|---|---|
| Both clear (< 120 cm) | 0.3 | 2.7 m/s |
| One or both blocked | 0.7 | 0.8 m/s |

### Stuck detection

If `s[1] < 20 cm` OR `s[2] < 20 cm` OR `speed < 0.1 m/s` for more than **25 consecutive ticks** (~1 s):

1. `write_steer(0)` — straighten wheels
2. `go_back()` — short reverse manoeuvre
3. Resume forward at 2 m/s

### Wrong-direction / dead-end detection

A single accumulator `turns` detects when the car is going the wrong way. The data source depends on `USE_IMU`:

| Mode | Source | Trigger |
|---|---|---|
| `USE_IMU = 1` | Gyro Z yaw rate (°/s × dt) | `turns > WRONG_DIR_DEG` (CW) or `turns < -WRONG_DIR_DEG` (CCW) |
| `USE_IMU = 0` | Heuristic `diff × speed / -1000` | `turns < -18` |

**With IMU:** correct-direction accumulation decays ×0.97/tick so normal laps don't false-trigger. Recovery steers toward the correct race direction based on `RACE_CW`.

**Without IMU:** same heuristic as before — sustained one-direction steering at speed builds up `turns` until it crosses the threshold. Recovery always steers hard right then left.

When triggered:
1. Stop
2. Steer hard toward correct direction (or hard right without IMU)
3. `go_back_long()` — long reverse
4. Steer opposite, drive forward at 2 m/s for 900 ms
5. Reset `turns = 0` (and `heading` if IMU)

---

## Runtime Configuration & EEPROM

All tuning parameters (obstacle thresholds, PID gains, ESC/steering limits, speed coefficients, etc.) are stored as `cfg_*` global variables with compile-time defaults. They can be changed at runtime via the command protocol and persisted to EEPROM.

`luna_car.h` declares them as `extern`; `Umbreon_roborace.ino` defines and initialises them.

### EEPROM layout

A packed `CarSettings` struct at address 0 (~60 bytes):
- Magic `0x554D4252` ("UMBR"), version 5, all parameters, trailing checksum
- `load_settings()` in `setup()` validates magic + version + checksum before applying
- `save_settings()` populates struct, computes checksum, writes via `EEPROM.put()` + `commit()`

### Command protocol

ASCII over the existing WiFi TCP bridge. Processed in `loop()` via `process_commands()`.

| Command | Response | Description |
|---|---|---|
| `$PING` | `$PONG` | Connection test |
| `$GET` | `$CFG:FOD=1200,...` | Read all parameters |
| `$SET:KP=5.0` | `$ACK` / `$NAK:reason` | Set parameters |
| `$SAVE` | `$ACK` | Persist to EEPROM |
| `$LOAD` | `$ACK` / `$NAK` | Restore from EEPROM |
| `$RST` | `$ACK` | Reset to compile-time defaults |
| `$SRV:<angle>` | *(none)* | Direct servo write (0–180°) for calibration |
| `$ESC:<us>` | *(none)* | Direct ESC write (1000–2000 µs) for calibration |
| `$BAT` | `$BAT:<voltage>` | Read battery voltage (requires BEN=1) |
| `$TEST:cal` | `$T:CAL,...` / `$TDONE:cal` | Run ESC calibration (max→min→neutral) |

See [dashboard.md](dashboard.md) for the full key table and dashboard usage.

---

## Battery Monitoring

Optional feature on GP26 (ADC0). Disabled by default (`BEN=0`).

**Hardware:** resistor divider — R1=18kΩ (bat→GP26), R2=10kΩ (GP26→GND). Multiplier `BML=(R1+R2)/R2=2.8`.

**Software:**
- `bat_update()` reads 12-bit ADC every 500ms (self-throttled), applies multiplier, EMA filter (α=0.05)
- `$BAT` command returns `$BAT:7.42`
- Low-voltage cutoff: if `bat_voltage < BLV` for >10s → emergency stop, sends `$T:BAT,phase=LOW_VOLTAGE_CUTOFF`
- Web UI header shows voltage with color coding (green >7V, yellow 6.2–7V, red <6.2V), polled every 5s

---

## wifi_debug — Wemos D1 Mini WiFi Telemetry (RP2350 only)

Separate firmware for the Wemos D1 Mini (ESP8266). Flashed independently via Arduino IDE. **Not needed on ESP32-S3** — WiFi is built into `hw_esp32s3.h`.

### What it does

Creates WiFi AP **"Umbreon"** (password `12345678`) and runs three servers:
- **Port 80**: HTTP — built-in web dashboard (`web_ui.h`)
- **Port 81**: WebSocket — real-time bidirectional relay for web UI
- **Port 23**: Raw TCP — backward compat with Python dashboard / ROS2

**Built-in web UI** (`web_ui.h` PROGMEM, ~20KB):
- Live telemetry (4 LiDARs, speed, steer, IMU, battery voltage)
- Track map with normal/light/collapsed modes
- Settings in grouped categories (⚠ Obstacles, ⏱ Speed, ⚙ PID, ⇄ Steering, ⏲ Loop, ⚸ Encoder, ☑ Flags, ⚡ Battery) with checkbox rendering for bool params
- Servo calibration wizard (step-by-step min/max/neutral)
- ESC min-speed slider
- Manual drive with steer/speed sliders
- Hardware tests with confirm dialogs for motor tests

### Wiring

| D1 Mini | Pico 2 |
|---|---|
| RX | GP17 (UART1 TX) |
| TX | GP16 (UART1 RX) |
| 3V3 | 3.3 V (or power via USB) |
| GND | GND |

### Car-side telemetry (`HAS_TELEM = 1`)

Each control tick (25 Hz) the car sends a CSV line over UART1:

```
ms,s0,s1,s2,s3,steer,speed,target[,yaw,heading]              (RP2350, 4 sensors)
ms,s0,s1,s2,s3,s4,s5,steer,speed,target[,yaw,heading]        (ESP32-S3, 6 sensors)
```

| Field | Unit | Description |
|---|---|---|
| ms | ms | `millis()` timestamp |
| s0–s3 | cm×10 | Sensor distances (Left, FL, FR, Right) |
| s4–s5 | cm×10 | ESP32-S3 only: Front, Rear |
| steer | — | Steering command sent (`diff × coef`) |
| speed | m/s | Measured wheel speed |
| target | m/s | Target speed |
| yaw | °/s | Gyro Z rate (only if `USE_IMU`) |
| heading | ° | Accumulated heading (only if `USE_IMU`) |

A CSV header line is printed once at startup.

### Connecting

1. Connect to WiFi AP **Umbreon**
2. Open TCP client to `192.168.4.1:23` (PuTTY Raw, `nc`, telnet, etc.)
3. CSV telemetry streams in at 25 lines/sec

### Flashing the Wemos D1 Mini

1. Install **ESP8266** board package in Arduino IDE
2. Select board: **LOLIN(WEMOS) D1 mini**
3. Connect via USB — no manual download-mode wiring needed
4. Upload `wifi_debug/wifi_debug.ino`
