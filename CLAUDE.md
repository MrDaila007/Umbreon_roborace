# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Umbreon is an autonomous roborace car built on a Raspberry Pi Pico 2 (RP2350). It uses 4× TF-Luna LiDAR sensors for obstacle detection, an optical encoder for speed measurement, an optional MPU-6050 gyro for heading, and a Wemos D1 Mini WiFi bridge for wireless telemetry and remote tuning. A Python web dashboard and 2D simulator complete the stack.

## Build & Run Commands

**Windows** uses `make.bat`, **Linux/macOS** uses `Makefile`. Both provide identical targets.

| Command | What it does |
|---------|-------------|
| `make install` | `pip install -r dashboard/requirements.txt` |
| `make web` | Start web dashboard at http://localhost:8080 |
| `make dashboard-tk` | Start desktop tkinter dashboard |
| `make sim` | Run simulator with live matplotlib animation |
| `make sim-fast` | Run simulator with pre-computed plot |
| `make bridge` | Start TCP bridge only (port 8023) for dashboard testing |
| `make demo` | Launch sim bridge + web dashboard together |
| `make compile` | Compile firmware via arduino-cli |
| `make upload` | Compile & upload firmware (set `SERIAL=COM4` to pick port) |
| `make clean` | Remove Python `__pycache__` dirs |
| `make ros2-build` | Build ROS2 Docker image (ros:humble) |
| `make ros2-run` | Run ROS2 bridge against car (192.168.4.1:23) |
| `make ros2-sim` | Run ROS2 bridge against simulator (localhost:8023) |
| `make ros2-shell` | Open a shell in the ROS2 container |

**Firmware notes:**
- FQBN: `rp2040:rp2040:rpipico2` (board package: [arduino-pico](https://github.com/earlephilhower/arduino-pico) v5.5.1)
- arduino-cli is bundled with Arduino IDE at `%LOCALAPPDATA%\Programs\arduino-ide\resources\app\lib\backend\resources\arduino-cli`
- Libraries (bundled with board package): Servo(rp2040), Wire, EEPROM
- WiFi bridge: FQBN `esp8266:esp8266:d1_mini` (Wemos D1 Mini / ESP8266), upload `wifi_debug/wifi_debug.ino`
- WiFi bridge external library: "WebSockets" by Markus Sattler (Arduino Library Manager)

## Linting & CI

CI runs on push/PR via `.github/workflows/ci.yml`:
- **Python lint**: `ruff check dashboard/ simulation/` (config in `ruff.toml`)
- **C++ static analysis**: `cppcheck` on firmware files
- **Firmware compile**: arduino-cli build for `rp2040:rp2040:rpipico2`
- **Python imports**: verifies all dashboard/sim modules import cleanly
- **ROS2 build**: Docker image build with colcon

No automated test suite — diagnostics are in `tests.h` (8 WiFi-accessible hardware tests: lidar, servo, taho, esc, speed, autotune, reactive, cal).

## Architecture

### Three-layer system

```
Pico 2 Firmware ──UART1──▶ Wemos D1 Mini WiFi Bridge ──┬─ TCP:23 ──▶ Python Dashboard / ROS2
   (C++)            GP17 TX → RX    (ESP8266)           ├─ HTTP:80 ─▶ Built-in Web UI (phone)
                    GP16 RX ← TX                        └─ WS:81 ──▶ Built-in Web UI (real-time)
```

### Firmware (`Umbreon_roborace.ino` + `luna_car.h`)

- **40 ms control loop** in `work()`: read LiDARs → IMU → steering logic → speed logic → PID → telemetry output → stuck/wrong-direction detection
- `luna_car.h` is the hardware abstraction layer: LiDAR polling (SerialPIO), servo/ESC PWM, PID, tachometer ISR, IMU I2C
- `tests.h` provides diagnostic routines (included by default; `#define COMPETITION_MODE` for auto-start)
- **Start/Stop**: car boots stopped by default; `$START`/`$STOP` control driving; idle telemetry streams sensor data when stopped
- **Remote tests**: 8 WiFi-accessible tests via `$TEST:<name>` (output `$T:` progress, `$TR:` results, `$TDONE:` completion)
- All distances are in **cm×10** units (e.g., 1200 = 120.0 cm)
- ESC: `writeMicroseconds()` — forward [1540–1700], reverse [1460–1000], neutral 1500 µs
- **Calibration**: ESC auto-calibrates on first boot (max 2000→min 1000→neutral 1500 µs, saved to EEPROM). Servo calibration is manual via web UI wizard (step-by-step min/max/neutral with +/- buttons). ESC min-speed calibration via live slider in web UI.
- **Hardware flags**: `IMR` (negate yaw for 180°-rotated IMU), `SVR` (reverse servo direction), `CAL` (ESC calibrated flag)
- 28 runtime-configurable parameters via `$SET`/`$GET` commands, persisted to EEPROM with `$SAVE`

### WiFi Bridge (`wifi_debug/wifi_debug.ino` + `web_ui.h`) — Wemos D1 Mini

- Creates AP "Umbreon" (password `12345678`)
- **Port 80**: HTTP server — built-in web dashboard (`web_ui.h` PROGMEM), open `http://192.168.4.1` from phone/laptop
- **Port 81**: WebSocket server — real-time bidirectional relay for web dashboard
- **Port 23**: Raw TCP server — backward compat with Python dashboard / ROS2 bridge
- Line-buffered protocol-aware relay (UART lines broadcast to both TCP and WebSocket)
- LED status: slow blink (no clients), fast blink (client, no data), solid (data flowing)

### Dashboard (`dashboard/`)

Two implementations sharing `protocol.py` and `car_config.py`:
- **Web (recommended)**: `server.py` (aiohttp) bridges WebSocket clients to car's TCP socket. UI in `static/index.html` (vanilla JS, canvas charts, track map).
- **Desktop**: `app.py` (tkinter + matplotlib). Supporting modules: `connection.py`, `telemetry.py`, `plots.py`, `track_map.py`, `settings_panel.py`, `settings_file.py`.

### Simulator (`simulation/sim.py`)

- 2D kinematic bicycle model with LiDAR ray casting
- Mirrors the car's control algorithm for offline testing
- Optional TCP bridge mode (port 8023) emits identical CSV telemetry for dashboard testing

## Command Protocol (ASCII over TCP)

```
$PING           → $PONG
$GET            → $CFG:FOD=1200,SOD=1000,...
$SET:KP=5.0    → $ACK  (or $NAK:reason)
$SAVE           → $ACK
$LOAD           → $ACK  (or $NAK:no_saved_config)
$RST            → $ACK
$START          → $ACK              (begin autonomous driving)
$STOP           → $ACK              (halt motors, stay idle)
$STATUS         → $STS:RUN|STOP     (query running state)
$DRV:<s>,<v>   → (none)             (manual drive: steer -1000..+1000, speed m/s; fire-and-forget, 500ms timeout)
$SRV:<angle>   → (none)             (direct servo write 0-180° for calibration)
$ESC:<us>      → (none)             (direct ESC write 1000-2000 µs for calibration)
$TEST:<name>    → $T:<TAG>,k=v,...   (progress lines)
                   $TR:<method>,K=V  (results, autotune only)
                   $TDONE:<name>     (completion)
```

Test names: `lidar`, `servo`, `taho`, `esc`, `speed`, `autotune`, `reactive`, `cal`.
Motor tests (esc, speed, autotune, cal) skip serial confirmation — dashboard provides confirm dialog.
Tests auto-stop the car. Send `$STOP` to abort a running test.

## Telemetry Format (CSV, 25 Hz)

```
#ms,s0,s1,s2,s3,steer,speed,target[,yaw,heading]
```

8 fields without IMU, 10 with IMU. Sensors s0–s3 are in cm×10.

## Key Constants

- EEPROM magic: `0x554D4252` ("UMBR"), version 3
- Encoder: 62 holes/rev, 60 mm wheel diameter
- PID defaults (Tyreus-Luyben): KP=4.18, KI=2.93, KD=0.43
- Servo range: 40°–140°, neutral 90° (configurable via calibration wizard)
- ESC range: 1540–1700 µs forward, 1460 µs min reverse, 1500 µs neutral (configurable)
- Compile flag: `#pragma GCC optimize("Ofast")`
- Conditional compilation: `USE_WIFI_DEBUG`, `USE_IMU` (defined at top of .ino)

## ROS2 Bridge (`ros2/`)

Docker-based ROS2 Humble bridge node. Uses standard message types only (no custom msgs).

### Architecture

```
ros2/
├── Dockerfile              ros:humble base, copies protocol.py + car_config.py
├── docker-compose.yml      umbreon-bridge (car) + umbreon-bridge-sim (simulator)
└── src/umbreon_bridge/     ament_python package
    ├── umbreon_bridge/
    │   ├── bridge_node.py  Main ROS2 node
    │   ├── tcp_client.py   Thread-based TCP client
    │   ├── protocol.py     Copied from dashboard/ at build time
    │   └── car_config.py   Copied from dashboard/ at build time
    └── launch/             Launch file with host/port args
```

### Topics Published (25 Hz)

| Topic | Type | Source |
|-------|------|--------|
| `/umbreon/range/{left,front_left,front_right,right}` | `sensor_msgs/Range` | LiDARs s0–s3 |
| `/umbreon/imu` | `sensor_msgs/Imu` | yaw rate + heading |
| `/umbreon/odom` | `nav_msgs/Odometry` | Dead-reckoning |
| `/umbreon/scan` | `sensor_msgs/LaserScan` | 4 LiDARs combined |
| `/umbreon/status` | `std_msgs/String` | RUN / STOP / DISCONNECTED |

### Topics Subscribed

| Topic | Type | Effect |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Manual teleop via `$DRV` |
| `/umbreon/throttle` | `std_msgs/Float32` | Direct speed override (m/s) |
| `/umbreon/steering` | `std_msgs/Float32` | Direct steer override (-1.0..+1.0) |

### Services (all `std_srvs/Trigger`)

`/umbreon/{connect,disconnect,ping,start,stop,save,load,reset}`

### TF Tree

- **Static**: `base_link` → `lidar_{left,front_left,front_right,right}`, `base_scan`
- **Dynamic**: `odom` → `base_link` (dead-reckoning)

### Node Parameters

`host`, `port`, `auto_connect` + all 23 writable car config keys (changes → `$SET`)
