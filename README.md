# Umbreon -- Roborace Car Firmware

Autonomous roborace car firmware for **Raspberry Pi Pico 2 (RP2350)** using **4x TF-Luna LiDAR** sensors, with a **live web dashboard** for telemetry, track mapping, and remote tuning.

![Dashboard Demo](docs/Demo.gif)

## Hardware

| Component | Model | Notes |
|---|---|---|
| Microcontroller | RP2350 (Pico 2) | Arduino-Pico framework |
| Distance sensors | 4x Benewake TF-Luna | UART, 115200 baud |
| IMU | MPU-6050 | I2C, gyro Z only (optional) |
| WiFi bridge | DT-06 (ESP-M2 / ESP8285) | Custom firmware, TCP bridge |
| Steering | Servo motor | PWM |
| Drive | Brushless motor + ESC | PWM |
| Speed feedback | Optical encoder (62 holes) | RISING interrupt |

### Pin Layout

| Signal | GPIO |
|---|---|
| LiDAR 0 -- Left | GP2 |
| LiDAR 1 -- Front-Left | GP3 |
| LiDAR 2 -- Front-Right | GP4 |
| LiDAR 3 -- Right | GP5 |
| WiFi TX (UART1) | GP6 |
| WiFi RX (UART1) | GP7 |
| IMU SDA (I2C) | GP0 |
| IMU SCL (I2C) | GP1 |
| Steering servo | GP10 |
| Motor ESC | GP11 |
| Tachometer | GP13 |

## File Structure

```
Umbreon_roborace/
|-- Umbreon_roborace.ino      Main sketch -- control logic, config, EEPROM, commands
|-- luna_car.h                Car hardware abstraction (LiDAR, ESC, servo, PID, IMU)
|-- tests.h                   Bench testing utilities
|-- wifi_debug/
|   +-- wifi_debug.ino        DT-06 WiFi bridge firmware (ESP8285)
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

### 2. Flash the WiFi bridge

1. Install the **ESP8266** board package in Arduino IDE.
2. Select board: **Generic ESP8285 Module**.
3. Open `wifi_debug/wifi_debug.ino` and upload to the DT-06 module.

### 3. Run the dashboard

```bash
# Install dependencies
make install        # or: pip install -r dashboard/requirements.txt

# Start the web dashboard
make web            # or: cd dashboard && python server.py
```

Open **http://localhost:8080** in a browser. Connect to WiFi AP **Umbreon** (password `12345678`), then click Connect (default `192.168.4.1:23`).

### 4. Test with the simulator (no hardware needed)

```bash
# Terminal 1: start simulator with TCP bridge
make bridge         # or: cd simulation && python sim.py --bridge --headless

# Terminal 2: start web dashboard
make web            # or: cd dashboard && python server.py
```

Open **http://localhost:8080**, enter host `localhost`, port `8023`, click Connect.

## Dashboard

Live web dashboard with dark theme, zero external JS dependencies:

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

ASCII commands over WiFi TCP (port 23):

| Command | Response | Description |
|---|---|---|
| `$PING` | `$PONG` | Connection test |
| `$GET` | `$CFG:FOD=1200,...` | Read all parameters |
| `$SET:KP=5.0` | `$ACK` | Set parameters at runtime |
| `$SAVE` | `$ACK` | Persist to EEPROM |
| `$LOAD` | `$ACK` | Restore from EEPROM |
| `$RST` | `$ACK` | Reset to compile-time defaults |

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

On Windows, use `make.bat` (same targets).

## Dependencies

### Firmware (Arduino IDE)

- **Board**: `Raspberry Pi Pico 2` via [arduino-pico](https://github.com/earlephilhower/arduino-pico)
- **Libraries**: `Servo`, `Wire`, `EEPROM` (bundled with arduino-pico)
- `SerialPIO` is part of the arduino-pico core

### WiFi Bridge (Arduino IDE)

- **Board**: `Generic ESP8285 Module` via ESP8266 core

### Dashboard (Python)

- `numpy`, `matplotlib`, `aiohttp`
- Install: `pip install -r dashboard/requirements.txt`
