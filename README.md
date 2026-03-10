# Umbreon — Roborace Car Firmware

Autonomous roborace car firmware for **Raspberry Pi Pico 2 (RP2350)** using **4× TF-Luna LiDAR** sensors instead of traditional IR sensors.

## Hardware

| Component | Model | Notes |
|---|---|---|
| Microcontroller | RP2350 (Pico 2) | Arduino-Pico framework |
| Distance sensors | 4× Benewake TF-Luna | UART, 115200 baud |
| Steering | Servo motor | PWM |
| Drive | Brushless motor + ESC | PWM |
| Speed feedback | Hall-effect tachometer | RISING interrupt |

### Pin Layout

| Signal | GPIO |
|---|---|
| LiDAR 0 — Left | GP2 |
| LiDAR 1 — Front-Left | GP3 |
| LiDAR 2 — Front-Right | GP4 |
| LiDAR 3 — Right | GP5 |
| Steering servo | GP10 |
| Motor ESC | GP11 |
| Tachometer | GP13 |

## File Structure

```
Umbreon_roborace/
├── Umbreon_roborace.ino   # Main sketch — control logic
├── luna_car.h             # Car hardware abstraction (LiDAR, ESC, servo, PID)
└── docs/
    ├── architecture.md    # Software design overview
    └── tuning.md          # Tuning guide for thresholds and PID
```

## Dependencies (Arduino IDE)

- **Board**: `Raspberry Pi Pico 2` via [arduino-pico](https://github.com/earlephilhower/arduino-pico)
- **Library**: `Servo` (bundled with arduino-pico)
- `SerialPIO` is part of the arduino-pico core — no separate install needed

## Quick Start

1. Install the arduino-pico board package in Arduino IDE.
2. Select **Tools → Board → Raspberry Pi Pico 2**.
3. Open `Umbreon_roborace.ino`.
4. Connect the Pico 2 via USB while holding BOOTSEL, then click **Upload**.
5. After `setup()` the firmware waits **3.7 s** for the ESC to arm before moving.

## Control Algorithm

```
loop (every 40 ms)
│
├─ poll_lidars()          read all pending UART bytes from 4× TF-Luna
│
└─ work()
   ├─ read_sensors()      get distances [Left, FL, FR, Right] in cm×10
   ├─ steering logic      wall-balancing + obstacle avoidance
   ├─ speed + PID         target m/s tracked with PD controller
   ├─ stuck detection     reverse if blocked for >25 ticks (~1 s)
   └─ U-turn detection    reverse + counter-steer on dead ends
```

See [docs/architecture.md](docs/architecture.md) for full details.

## Tuning

Key constants in `Umbreon_roborace.ino`:

| Constant | Default | Meaning |
|---|---|---|
| `FRONT_OBSTACLE_DIST` | 1200 | 120 cm — start steering |
| `SIDE_OPEN_DIST` | 1000 | 100 cm — side counted as open |
| `ALL_CLOSE_DIST` | 800 | 80 cm — surrounded, force turn |
| `CLOSE_FRONT_DIST` | 201 | 20 cm — emergency reverse |
| `LOOP_MS` | 40 | Control loop period (ms) |

See [docs/tuning.md](docs/tuning.md) for a full tuning guide.
