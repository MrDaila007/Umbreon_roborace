
# Umbreon -- Roborace Car Firmware

Autonomous roborace car firmware for **Raspberry Pi Pico 2 (RP2350)** using **4x TF-Luna LiDAR** sensors, with a **live web dashboard** for telemetry, track mapping, and remote tuning.

<p align="center">
  <img src="docs/Logo.png" alt="Umbreon Team Logo" width="400">
</p>

## Dashboard Demo
![Dashboard Demo](docs/Demo.gif)

## Hardware

Two platform options — same firmware codebase, auto-detected at compile time:

### Platform A: RP2350 + ESP8266 (two-chip, current)

| Component | Model | Notes |
|---|---|---|
| Microcontroller | RP2350 (Pico 2) | Arduino-Pico framework |
| Distance sensors | 4x Benewake TF-Luna | UART, 115200 baud (SerialPIO) |
| IMU | MPU-6050 | I2C, gyro Z only (optional) |
| WiFi bridge | Wemos D1 Mini (ESP8266) | 3-port server: HTTP, WebSocket, TCP |
| Steering | Servo motor | PWM |
| Drive | Brushless motor + ESC | PWM |
| Speed feedback | Optical encoder (62 holes) | RISING interrupt |

**Pin Layout (RP2350):**

| Signal | GPIO |
|---|---|
| LiDAR 0 -- Left | GP2 |
| LiDAR 1 -- Front-Left | GP3 |
| LiDAR 2 -- Front-Right | GP4 |
| LiDAR 3 -- Right | GP5 |
| WiFi TX (UART0) | GP16 |
| WiFi RX (UART0) | GP17 |
| IMU SDA (I2C) | GP0 |
| IMU SCL (I2C) | GP1 |
| Steering servo | GP10 |
| Motor ESC | GP11 |
| Tachometer | GP13 |

### Platform B: ESP32-S3 (single-chip, planned)

| Component | Model | Notes |
|---|---|---|
| Microcontroller | ESP32-S3-DevKitC-1 | WiFi built-in, no bridge needed |
| Distance sensors | 4x Benewake TF-Luna | **I2C** (addresses 0x10–0x13) |
| IMU | MPU-6050 | I2C, shared bus with LiDARs |
| Steering | Servo motor | PWM (LEDC) |
| Drive | Brushless motor + ESC | PWM (LEDC) |
| Speed feedback | Optical encoder (62 holes) | RISING interrupt |

**Pin Layout (ESP32-S3):**

| Signal | GPIO |
|---|---|
| I2C SDA (LiDAR + IMU) | GPIO 8 |
| I2C SCL (LiDAR + IMU) | GPIO 9 |
| Steering servo | GPIO 10 |
| Motor ESC | GPIO 11 |
| Tachometer | GPIO 13 |
| Status LED | GPIO 2 |

> **TF-Luna I2C setup**: Each sensor must be pre-configured for I2C mode with a unique address (factory default is UART mode, address 0x10). See `hw_esp32s3.h` header comments for instructions.

## File Structure

```
Umbreon_roborace/
|-- Umbreon_roborace.ino      Main sketch -- control logic, config, EEPROM, commands
|-- hw_config.h               Platform auto-detection (RP2350 vs ESP32-S3)
|-- luna_car.h                RP2350 hardware layer (SerialPIO LiDAR, Servo, PID, IMU)
|-- hw_esp32s3.h              ESP32-S3 hardware layer (I2C LiDAR, WiFi, ESP32Servo)
|-- tests.h                   Bench testing utilities
|-- wifi_debug/
|   |-- wifi_debug.ino        Wemos D1 Mini WiFi bridge firmware (ESP8266)
|   +-- web_ui.h              Built-in web dashboard (PROGMEM HTML/JS)
|-- hardware_test_wifi/
|   |-- hardware_test_wifi.ino  Standalone hardware test for RP2350 + WiFi
|   +-- i2c_scanner/            I2C bus scanner for Umbreon pin config
|-- hardware_test_esp32s3/
|   +-- hardware_test_esp32s3.ino  Standalone hardware test for ESP32-S3
|-- dashboard/
|   |-- server.py             Web dashboard server (aiohttp)
|   |-- static/index.html     Web UI -- charts, track map, settings
|   |-- app.py                Desktop dashboard (tkinter, alternative)
|   |-- protocol.py           Telemetry parser + command protocol
|   |-- car_config.py         Constants, key mappings, defaults
|   |-- connection.py         TCP client (for tkinter version)
|   |-- telemetry.py          Ring buffers (for tkinter version)
|   |-- plots.py              Matplotlib plots (for tkinter version)
|   |-- track_map.py          Canvas track map (for tkinter version)
|   |-- settings_panel.py     Settings UI (for tkinter version)
|   +-- settings_file.py      JSON settings file I/O
|-- simulation/
|   +-- sim.py                2D simulator with --bridge mode for dashboard testing
|-- docs/
|   |-- architecture.md       Software design overview
|   |-- dashboard.md          Dashboard & command protocol reference
|   +-- tuning.md             Tuning guide for all parameters
|-- ros2/
|   |-- Dockerfile            ROS2 Humble Docker build
|   |-- docker-compose.yml    Bridge for car + simulator profiles
|   +-- src/umbreon_bridge/   ROS2 Python package (bridge node)
|-- Makefile                  Build/run targets (Linux/macOS)
+-- make.bat                  Build/run targets (Windows)
```

## Quick Start

### 1. Flash the firmware

1. Install the [arduino-pico](https://github.com/earlephilhower/arduino-pico) board package in Arduino IDE.
2. Select **Tools -> Board -> Raspberry Pi Pico 2**.
3. Open `Umbreon_roborace.ino`.
4. Connect the Pico 2 via USB while holding BOOTSEL, then click **Upload**.
5. After `setup()` the firmware waits **3.7 s** for the ESC to arm before moving.

### 2. Flash the WiFi bridge (RP2350 only)

1. Install the **ESP8266** board package and **WebSockets** library (by Markus Sattler) in Arduino IDE.
2. Select board: **LOLIN(WEMOS) D1 mini**.
3. Open `wifi_debug/wifi_debug.ino` and upload to the Wemos D1 Mini via USB.
4. The bridge supports **STA mode** (join your WiFi) and **AP mode** (create hotspot). Set `WIFI_MODE`, `STA_SSID`, `STA_PASS` at the top of the file. Falls back to AP if STA connection fails.

### 2b. Flash ESP32-S3 (alternative single-chip)

1. Install the **ESP32** board package and **ESP32Servo** + **WebSockets** (by Markus Sattler) libraries.
2. Pre-configure each TF-Luna sensor for **I2C mode** with unique addresses (0x10–0x13). See `hw_esp32s3.h` for instructions.
3. Select board: **ESP32-S3 Dev Module**.
4. Open `Umbreon_roborace.ino` — platform is auto-detected via `hw_config.h`.
5. Upload. WiFi AP "Umbreon" starts automatically — no separate bridge needed.
6. Use `hardware_test_esp32s3/hardware_test_esp32s3.ino` for initial hardware validation (includes I2C scan).

### 3. Run the dashboard

```bash
# Install dependencies
make install        # or: pip install -r dashboard/requirements.txt

# Start the web dashboard
make web            # or: cd dashboard && python server.py
```

Open **http://localhost:8080** in a browser. Connect to WiFi AP **Umbreon** (password `12345678`), then click Connect (default `192.168.4.1:23`).

> **Built-in web dashboard**: You can also open **http://192.168.4.1** directly from a phone/laptop connected to the Umbreon AP — no Python server needed. The ESP serves a full dashboard with telemetry, track map, settings, hardware tests, and manual drive.

### 4. Test with the simulator (no hardware needed)

```bash
# Terminal 1: start simulator with TCP bridge
make bridge         # or: cd simulation && python sim.py --bridge --headless

# Terminal 2: start web dashboard
make web            # or: cd dashboard && python server.py
```

Open **http://localhost:8080**, enter host `localhost`, port `8023`, click Connect.

## Dashboards

### Built-in ESP Web Dashboard (phone-friendly)

Open **http://192.168.4.1** from any device on the Umbreon WiFi — no server needed:

- **Live telemetry** -- 4 LiDAR distances, speed, steer, IMU heading
- **Track map** -- dead-reckoning trajectory with LiDAR wall points, pan/zoom
- **Manual drive** -- steer/speed sliders with enable checkbox (no $START required)
- **Remote settings** -- read/write all 25 parameters, save/load EEPROM
- **Hardware tests** -- LiDAR, servo, tacho, ESC, speed, autotune, reactive

### Python Web Dashboard (full-featured)

Run `make web` and open **http://localhost:8080**:

- **4 real-time charts** -- LiDAR distances, speed, steering, IMU
- **Track map** -- dead-reckoning from speed + gyro, color-coded wall points
- **Remote settings** -- read/write all 25 parameters, save to EEPROM
- **Recording** -- save telemetry to CSV

See [docs/dashboard.md](docs/dashboard.md) for full reference.

## Control Algorithm

```
loop (every 40 ms)
|
|-- poll_lidars()          read all pending UART bytes from 4x TF-Luna
|-- process_commands()     handle dashboard commands via WiFi
|
+-- work()
    |-- read_sensors()     get distances [Left, FL, FR, Right] in cm*10
    |-- imu_update()       read gyro Z, integrate heading
    |-- steering logic     wall-balancing + obstacle avoidance
    |-- speed + PID        target m/s tracked with PID controller
    |-- telemetry          send CSV line over WiFi
    |-- stuck detection    reverse if blocked for >25 ticks (~1 s)
    +-- wrong-dir detect   gyro-based U-turn detection + recovery
```

See [docs/architecture.md](docs/architecture.md) for full details.

## Runtime Configuration

All tuning parameters are runtime-configurable via the dashboard and persistable to EEPROM. Key parameters:

| Key | Default | Description |
|---|---|---|
| `FOD` | 1200 | Front obstacle distance (cm*10) |
| `SOD` | 1000 | Side open distance |
| `KP` | 4.18 | PID proportional gain |
| `KI` | 2.93 | PID integral gain |
| `KD` | 0.43 | PID derivative gain |
| `SPD1` | 2.7 | Speed when clear (m/s) |
| `SPD2` | 0.8 | Speed when blocked (m/s) |
| `RCW` | 1 | Race clockwise (1) or CCW (0) |

See [docs/tuning.md](docs/tuning.md) for the full 25-parameter tuning guide.

## Command Protocol

ASCII commands over WiFi (TCP:23, WebSocket:81, or built-in web UI on HTTP:80):

| Command | Response | Description |
|---|---|---|
| `$PING` | `$PONG` | Connection test |
| `$GET` | `$CFG:FOD=1200,...` | Read all parameters |
| `$SET:KP=5.0` | `$ACK` | Set parameters at runtime |
| `$SAVE` | `$ACK` | Persist to EEPROM |
| `$LOAD` | `$ACK` | Restore from EEPROM |
| `$RST` | `$ACK` | Reset to compile-time defaults |
| `$DRV:<s>,<v>` | *(none)* | Manual drive (steer, speed m/s) — 500ms timeout |
| `$DRVEN` | `$ACK` | Enable manual drive (works without $START) |
| `$DRVOFF` | `$ACK` | Disable manual drive, stop motors |
| `$START` | `$ACK` | Begin autonomous driving |
| `$STOP` | `$ACK` | Halt motors, disable drive mode |
| `$STATUS` | `$STS:RUN\|STOP` | Query running state |
| `$TEST:<name>` | `$T:...` / `$TDONE:` | Run hardware test (lidar, servo, taho, esc, speed, autotune, reactive) |

## ROS2 Bridge (Docker)

The ROS2 Humble bridge runs in Docker and publishes Umbreon telemetry as standard ROS2 topics. Requires Docker.

```bash
# Build the image
make ros2-build

# Run against the real car (connect to WiFi AP "Umbreon" first)
make ros2-run

# Run against the simulator
make bridge              # terminal 1: start sim
make ros2-sim            # terminal 2: start ROS2 bridge

# Inside the container
ros2 topic list
ros2 service call /umbreon/start std_srvs/srv/Trigger
ros2 topic echo /umbreon/odom
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.3}}"
```

**Published topics**: `/umbreon/range/*`, `/umbreon/imu`, `/umbreon/odom`, `/umbreon/scan`, `/umbreon/status`
**Subscribed**: `/cmd_vel` (teleop), `/umbreon/throttle`, `/umbreon/steering`
**Services**: `/umbreon/{connect,disconnect,ping,start,stop,save,load,reset}`
**TF**: `odom` → `base_link` → `lidar_*` frames

## Make Targets

| Command | Description |
|---|---|
| `make install` | Install Python dependencies |
| `make web` | Start web dashboard (localhost:8080) |
| `make bridge` | Run simulator with TCP bridge |
| `make demo` | Start sim + dashboard together |
| `make sim` | Run live simulation (matplotlib) |
| `make sim-fast` | Pre-computed simulation plot |
| `make clean` | Remove Python caches |
| `make ros2-build` | Build ROS2 Docker image |
| `make ros2-run` | Run ROS2 bridge (car) |
| `make ros2-sim` | Run ROS2 bridge (simulator) |
| `make ros2-shell` | Open ROS2 container shell |

On Windows, use `make.bat` (same targets).

## Dependencies

### RP2350 Firmware (Arduino IDE)

- **Board**: `Raspberry Pi Pico 2` via [arduino-pico](https://github.com/earlephilhower/arduino-pico)
- **Libraries**: `Servo`, `Wire`, `EEPROM` (bundled with arduino-pico)
- `SerialPIO` is part of the arduino-pico core

### ESP32-S3 Firmware (Arduino IDE)

- **Board**: `ESP32-S3 Dev Module` via [arduino-esp32](https://github.com/espressif/arduino-esp32)
- **Libraries**: `ESP32Servo`, `WebSockets` by Markus Sattler (Arduino Library Manager)
- `WiFi`, `WebServer`, `Wire`, `EEPROM` are bundled with arduino-esp32

### WiFi Bridge (Arduino IDE, RP2350 only)

- **Board**: `LOLIN(WEMOS) D1 mini` via ESP8266 core
- **Library**: `WebSockets` by Markus Sattler (Arduino Library Manager)

### Dashboard (Python)

- `numpy`, `matplotlib`, `aiohttp`
- Install: `pip install -r dashboard/requirements.txt`
