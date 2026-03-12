# Architecture

## Overview

The firmware is split into two layers:

| File | Role |
|---|---|
| `luna_car.h` | Hardware abstraction — sensors, actuators, PID |
| `Umbreon_roborace.ino` | Control logic — steering, speed, stuck/U-turn recovery |

---

## luna_car.h

### TF-Luna LiDAR driver

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
- Maps to servo angle [40°, 140°], neutral 90°
- Input is inverted so positive = right

**ESC** (`write_speed(s)`, s ∈ [-1000, 1000]):
- Forward: mapped to [96°, 110°] PWM angle
- Reverse: mapped to [0°, 85°]
- Neutral/brake: 90°

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
| `imu_update()` | Reads gyro Z, integrates into `heading` (degrees). Called every control tick (25 Hz). Rejects stale samples (dt > 0.5 s). |
| `reset_heading()` | Zeroes `heading` accumulator. Called after every recovery manoeuvre. |

**Members:**
- `yaw_rate` — latest gyro Z reading in °/s (positive = counter-clockwise)
- `heading` — cumulative heading change in degrees since last reset

All IMU code is wrapped in `#if USE_IMU` — setting it to `0` strips the IMU entirely from the binary (no `Wire.h` overhead).

---

## Umbreon_roborace.ino — Control Logic

### Main loop

```
loop()
├── poll_lidars()          // always — keep UART buffer empty
└── every 40 ms → work()
    ├── poll_lidars()      // fresh data before decisions
    ├── imu_update()       // (if USE_IMU)
    ├── read_sensors()
    ├── steering
    ├── speed
    ├── pid_control_motor()
    ├── stuck check
    ├── U-turn check
    └── wrong-direction check  // (if USE_IMU)
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

### U-turn / dead-end detection

An accumulator `turns` integrates `diff × speed / -1000` each tick. It is clamped to [-1500, +50].

When `turns < -18`:
1. Stop
2. Steer hard right (1000) for 20 ms
3. `go_back_long()` — longer reverse
4. Steer left (-700), drive forward at 2 m/s for 900 ms
5. Reset `turns = 0`

### Wrong-direction detection (IMU)

*Only active when `USE_IMU = 1`.*

Uses the gyro Z yaw rate to detect when the car is travelling opposite to the race direction.

An accumulator `wd` integrates `yaw_rate × dt` each tick:
- **Clockwise race** (`RACE_CW = true`): normal driving produces negative yaw (right turns). Positive accumulation = wrong direction.
- **Counter-clockwise** (`RACE_CW = false`): the opposite.

Correct-direction accumulation decays by ×0.97 per tick, preventing false triggers during normal laps. Wrong-direction heading builds freely until it crosses `WRONG_DIR_DEG` (default 120°).

When triggered:
1. Stop
2. Steer hard toward the correct race direction
3. `go_back_long()` — long reverse
4. Steer opposite, drive forward at 2 m/s for 900 ms
5. Reset accumulator and `heading`

The heading accumulator is also reset after stuck and U-turn recoveries to avoid stale data influencing detection.
