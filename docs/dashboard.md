# Dashboard

Live telemetry viewer, track mapper, and remote settings editor for Umbreon.

Two interfaces available:

| Version | Command | Notes |
|---|---|---|
| **Web** (recommended) | `cd dashboard && python server.py` | Open `http://localhost:8080` in browser |
| Desktop (tkinter) | `cd dashboard && python app.py` | Standalone window, no browser needed |

---

## Requirements

- Python 3.8+
- `numpy`, `matplotlib`, `aiohttp`

```
pip install -r dashboard/requirements.txt
```

---

## Quick Start

### With the real car

1. Power up Umbreon
2. Connect laptop to WiFi AP **Umbreon** (password `12345678`)
3. Run `cd dashboard && python server.py`
4. Open `http://localhost:8080` in browser
5. Click **Connect** (default `192.168.4.1:23`)
6. Live plots and track map appear as telemetry streams in

### Testing with the simulator

1. In one terminal: `cd simulation && python sim.py --bridge`
2. In another terminal: `cd dashboard && python server.py`
3. Open `http://localhost:8080` in browser
4. Enter host `localhost`, port `8023`, click **Connect**
5. Sim telemetry (with IMU) streams to the dashboard live

---

## Layout (Web)

```
┌─ Toolbar ──────────────────────────────────────────────────┐
│ UMBREON │ Host Port [Connect] │ ● Status │ Ping Rec │ N   │
├──────────────────────────────────┬─────────────────────────┤
│  Charts (2×2 grid)               │  Settings panel         │
│  ┌──────────┬──────────┐        │  [Read][Write][SaveEE]  │
│  │ LiDAR    │ Speed    │        │  ┌─ Obstacle Dist ────┐ │
│  │ (4 lines)│ (2 lines)│        │  │ FOD  SOD  ACD  CFD │ │
│  ├──────────┼──────────┤        │  ├─ PID Tuning ───────┤ │
│  │ Steering │ IMU      │        │  │ KP   KI   KD       │ │
│  │          │ (2 lines)│        │  ├─ ...               │ │
│  └──────────┴──────────┘        │  └─────────────────────┘ │
├──────────────────────────────────┴─────────────────────────┤
│  Track Map  [Clear] [Center]                               │
│  ● car + trail + wall points (pan/zoom with mouse)        │
└────────────────────────────────────────────────────────────┘
```

Dark theme. Charts use lightweight Canvas API (no external libraries). All zero-dependency — works offline.

---

## Files

### Web dashboard (recommended)

| File | Role |
|---|---|
| `server.py` | aiohttp web server — serves HTML, bridges WebSocket ↔ TCP |
| `static/index.html` | Complete web UI — dark theme, canvas charts, track map, settings |

### Shared modules

| File | Role |
|---|---|
| `protocol.py` | CSV telemetry parser (8-field no-IMU, 10-field with-IMU). Command encoder/decoder |
| `car_config.py` | Constants, sensor geometry, key mappings, default values |
| `settings_file.py` | JSON load/save for PC-side settings profiles |

### Desktop (tkinter) version

| File | Role |
|---|---|
| `app.py` | Main window, toolbar, event loop. Polls `rx_queue` every 20 ms |
| `connection.py` | TCP client with background I/O thread. Partial-recv buffering |
| `telemetry.py` | Thread-safe ring buffers (`deque(maxlen=2500)`) for all channels |
| `plots.py` | 4 stacked matplotlib subplots, 10 Hz refresh, 250-sample rolling window |
| `track_map.py` | Canvas-based dead-reckoning map with wall points, car trail, pan/zoom |
| `settings_panel.py` | Scrollable grouped widgets for all 25 parameters. Car and file I/O buttons |

---

## Plots

Four stacked subplots, updated at ~10 Hz with a 250-sample rolling window (~10 seconds at 25 Hz):

| Subplot | Channels | Y range |
|---|---|---|
| LiDAR distances | s0 (L), s1 (FL), s2 (FR), s3 (R) | 0–3000 cm×10 |
| Speed | actual (m/s), target (m/s) | 0–4.0 |
| Steering | steer command | ±800 |
| IMU | yaw rate (°/s), heading (°) | ±200 |

---

## Track Map

Builds a 2D map in real time using dead reckoning:

- **Heading**: from IMU gyro yaw rate (preferred) or kinematic bicycle model
- **Position**: integrated from `speed × dt`
- **Wall points**: LiDAR ray endpoints projected from sensor positions, color-coded per sensor
- **Car trail**: yellow polyline of all visited positions

**Sensor geometry** (from real car measurements):

```
Wheelbase: 173 mm    Sensor forward offset: 253 mm (from rear axle)
Max steering angle: 28°

Sensor    Angle   Lateral offset
s0 L-Out   +45°     +90 mm (left)
s1 L-Fwd     0°     +40 mm
s2 R-Fwd     0°     -40 mm
s3 R-Out   -45°     -90 mm (right)
```

**Controls:**
- **Click + drag**: pan
- **Scroll wheel**: zoom
- Grid consolidation: 20 mm cells, deduplicates wall points for performance

---

## Settings Panel

### Parameter Groups

| Group | Keys |
|---|---|
| Obstacle Distances | `FOD` `SOD` `ACD` `CFD` |
| PID Tuning | `KP` `KI` `KD` |
| Speed / ESC | `MSP` `XSP` `BSP` |
| Steering | `MNP` `XNP` `NTP` |
| Tachometer | `ENH` `WDM` |
| Control Loop | `LMS` `SPD1` `SPD2` `COE1` `COE2` |
| Navigation | `WDD` `RCW` `STK` |
| Flags (read-only) | `IMU` `DBG` |

### Buttons

| Button | Action |
|---|---|
| **Read** | Sends `$GET` — car replies with all current values |
| **Write** | Sends `$SET:KP=5.0,KI=3.0,...` — updates car's live parameters |
| **Save EE** | Sends `$SAVE` — persists current settings to EEPROM |
| **Load EE** | Sends `$LOAD` — restores settings from EEPROM |
| **Reset** | Sends `$RST` — resets all parameters to compile-time defaults |
| **Load File** | Loads a `.json` settings profile from disk into the panel |
| **Save File** | Saves current panel values to a `.json` file |

---

## Command Protocol

ASCII commands over the existing TCP bridge. Telemetry lines start with digits, commands start with `$`, headers start with `#` — no ambiguity.

| Command | Response | Description |
|---|---|---|
| `$PING` | `$PONG` | Connection test |
| `$GET` | `$CFG:FOD=1200,SOD=1000,...` | Read all parameters |
| `$SET:KP=5.0,KI=3.0` | `$ACK` or `$NAK:reason` | Set one or more parameters |
| `$SAVE` | `$ACK` | Save to EEPROM |
| `$LOAD` | `$ACK` or `$NAK:no_saved_config` | Load from EEPROM |
| `$RST` | `$ACK` | Reset to compile-time defaults |

### Key Abbreviations

| Key | Parameter | Type |
|---|---|---|
| `FOD` | Front Obstacle Dist (cm×10) | int |
| `SOD` | Side Open Dist | int |
| `ACD` | All Close Dist | int |
| `CFD` | Close Front Dist | int |
| `KP` | PID Kp | float |
| `KI` | PID Ki | float |
| `KD` | PID Kd | float |
| `MSP` | Min Speed (ESC) | int |
| `XSP` | Max Speed (ESC) | int |
| `BSP` | Min Reverse Speed | int |
| `MNP` | Min Steer Point | int |
| `XNP` | Max Steer Point | int |
| `NTP` | Neutral Steer Point | int |
| `ENH` | Encoder Holes | int |
| `WDM` | Wheel Diameter (m) | float |
| `LMS` | Loop Period (ms) | int |
| `SPD1` | Speed Clear (m/s) | float |
| `SPD2` | Speed Blocked (m/s) | float |
| `COE1` | Steer Coef Clear | float |
| `COE2` | Steer Coef Blocked | float |
| `WDD` | Wrong Dir Degrees | float |
| `RCW` | Race Clockwise (0/1) | int |
| `STK` | Stuck Threshold (ticks) | int |
| `IMU` | IMU Enabled (read-only) | int |
| `DBG` | WiFi Debug Enabled (read-only) | int |

---

## EEPROM Storage

Settings are stored as a packed `CarSettings` struct at EEPROM address 0:

- **Magic**: `0x554D4252` ("UMBR")
- **Version**: 1
- **Checksum**: sum of all preceding bytes (uint8)
- **Size**: ~42 bytes

On boot, `load_settings()` reads EEPROM and validates magic + version + checksum. If valid, globals are populated from the stored values. If invalid (first boot, or corrupted), compile-time defaults are used.

---

## Recording

Click **Record** to save telemetry to a timestamped CSV file (`umbreon_YYYYMMDD_HHMMSS.csv`) in the dashboard directory. The file has the same format as the live telemetry stream, with a `#` header line.

---

## Simulator Bridge

`sim.py --bridge` runs the simulation with a TCP server on port 8023, mimicking the DT-06 WiFi bridge. This lets you test the dashboard without hardware.

```
cd simulation && python sim.py --bridge
```

The bridge:
- Sends CSV telemetry (with simulated IMU yaw/heading) at 25 Hz
- Responds to `$PING`, `$GET`, `$SET`, `$SAVE`, `$LOAD`, `$RST`
- Accepts multiple dashboard clients simultaneously
- Runs the matplotlib sim visualization alongside

The sim window shows the car running the track. The dashboard shows live plots and builds a track map from the sim's LiDAR + dead reckoning.

---

## Toolbar Reference

| Control | Description |
|---|---|
| **Host** | IP address (default `192.168.4.1`) |
| **Port** | TCP port (default `23`) |
| **Connect / Disconnect** | Toggle TCP connection |
| **Record / Stop Rec** | Toggle CSV recording |
| **Ping** | Send `$PING`, expect `$PONG` |
| **Frames: N** | Total telemetry frames received |
