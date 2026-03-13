# Tuning Guide

All distances are in **cm×10** (e.g. `1200` = 120 cm).
Speed values are in **m/s**.

Parameters can be changed at runtime via the **dashboard** (see [dashboard.md](dashboard.md)) or by editing the compile-time defaults in `Umbreon_roborace.ino`. Runtime changes can be persisted to EEPROM with `$SAVE`.

---

## Obstacle Thresholds

| Key | Variable | Default | What to change |
|---|---|---|---|
| `FOD` | `cfg_front_obstacle_dist` | 1200 | Distance at which the car starts steering around a front obstacle. Increase for earlier reaction; decrease to get closer to walls before turning. |
| `SOD` | `cfg_side_open_dist` | 1000 | If both side sensors read above this, the corridor is treated as open and the car biases toward the right wall. |
| `ACD` | `cfg_all_close_dist` | 800 | If all 4 sensors read below this, the car forces a hard right turn. Increase if getting stuck in wide but tight spaces. |
| `CFD` | `cfg_close_front_dist` | 201 | Emergency threshold — triggers stuck counter. Set to ~2× the minimum reliable TF-Luna range (~20 cm). |

---

## Speed & Steering Coefficients

| Key | Variable | Default | Effect |
|---|---|---|---|
| `SPD1` | `cfg_spd_clear` | 2.7 | Top straight-line speed (m/s). Reduce if the car overshoots corners. |
| `SPD2` | `cfg_spd_blocked` | 0.8 | Cornering speed. Increase if the car is too slow around bends. |
| `COE1` | `cfg_coe_clear` | 0.3 | Steering gain when path is free. Higher = more aggressive wall-following. |
| `COE2` | `cfg_coe_blocked` | 0.7 | Steering gain while avoiding obstacle. Higher = sharper turns. |

---

## Stuck Recovery

| Key | Variable | Default | Effect |
|---|---|---|---|
| `STK` | `cfg_stuck_thresh` | 25 | Ticks before triggering reversal (25 × 40 ms = 1 s). Reduce to react faster; increase to tolerate brief slowdowns. |

`go_back()` timing:
```cpp
car.write_speed(-150); delay(200);   // first nudge
car.write_speed(-150); delay(700);   // main reverse
```
Increase the second `delay` if the car needs more room to clear an obstacle.

---

## Wrong-Direction / Dead-End Detection (`turns`)

A single `turns` accumulator feeds from either IMU gyro or steering heuristic.

**With IMU (`USE_IMU = 1`):** `turns` accumulates real gyro heading (°). See the IMU tuning section below for thresholds.

**Without IMU (`USE_IMU = 0`):** `turns` accumulates `diff × speed / -1000` each tick.

- **Threshold `-18.0`** — lower magnitude = triggers faster (e.g. `-10`); higher = less sensitive.
- **Clamp `[-1500, +50]`** — the negative clamp prevents runaway accumulation in long corridors.

`go_back_long()` reverse duration:
```cpp
car.write_speed(-150); delay(1000);
car.write_speed(-150); delay(1800);
```
Increase delays on longer cars or tighter dead ends.

---

## Wrong-Direction Detection — IMU (`USE_IMU = 1`)

| Key | Variable | Default | What to change |
|---|---|---|---|
| `IMU` | `USE_IMU` | `1` | Compile-time only. Set to `0` to disable IMU entirely (no `Wire.h` linked). |
| `RCW` | `cfg_race_cw` | `1` | Set to `0` for counter-clockwise races. Changeable at runtime. |
| `WDD` | `cfg_wrong_dir_deg` | `120.0` | Degrees of wrong-direction heading before recovery triggers. Lower = more sensitive (may false-trigger on tight hairpins). Higher = slower reaction. |

**Decay factor** (hardcoded `0.97`): correct-direction heading decays by 3 % per tick. At 25 Hz this halves in ~0.9 s. Increase toward `1.0` if you want the detector to remember longer history; decrease if tight corners cause false triggers.

**Typical behaviour:**
- At 90°/s sustained wrong-direction yaw, triggers after ~1.3 s
- Normal 90° right corner at speed contributes negative heading → decays quickly, never triggers

> **Tip:** If the MPU-6050 is not connected, `imu_init()` returns `false` and all IMU reads become no-ops — the car runs normally without the sensor.

---

## ESC / Servo Limits

| Key | Variable | Default | Effect |
|---|---|---|---|
| `MSP` | `cfg_min_speed` | 96 | Minimum ESC PWM for forward (too low = no movement) |
| `XSP` | `cfg_max_speed` | 110 | Maximum ESC PWM for forward (increase for more top speed) |
| `BSP` | `cfg_min_bspeed` | 85 | Minimum ESC PWM for reverse |
| `MNP` | `cfg_min_point` | 40 | Maximum left steering angle (degrees) |
| `XNP` | `cfg_max_point` | 140 | Maximum right steering angle (degrees) |
| `NTP` | `cfg_neutral_point` | 90 | Straight-ahead servo angle |

> Always verify ESC neutral (90°) matches your ESC calibration before driving.

---

## PID Speed Controller

| Key | Variable | Default | Effect |
|---|---|---|---|
| `KP` | `cfg_pid_kp` | 4.18 | Proportional — higher = faster speed correction, risk of oscillation |
| `KI` | `cfg_pid_ki` | 2.93 | Integral — eliminates steady-state error; too high = overshoot |
| `KD` | `cfg_pid_kd` | 0.43 | Derivative — damps oscillation; increase if speed hunts |

Defaults were auto-tuned (Tyreus-Luyben method, Ku=9.20, Tu=0.648s).

Typical manual tuning:
1. Set `KI = 0`, `KD = 0`, increase `KP` until speed tracks well but oscillates.
2. Increase `KD` until oscillation damps out.
3. Add small `KI` to eliminate steady-state error.

---

## Loop Period

| Key | Variable | Default | Effect |
|---|---|---|---|
| `LMS` | `cfg_loop_ms` | 40 | Control loop period in ms (25 Hz). Decrease for faster response at high speed. |

`poll_lidars()` is also called outside the timed block, so LiDAR data is always fresh regardless of loop period.

---

## TF-Luna Sensor Notes

- Reliable range: **20 cm – 800 cm** (200–8000 in cm×10 units).
- Readings below 20 cm may be unreliable — keep `CLOSE_FRONT_DIST` ≥ 200.
- Default output rate from the factory: **100 Hz** (one frame every 10 ms).
  At 115200 baud, 9-byte frames arrive faster than the 40 ms control loop — no data loss expected.
- If a sensor returns all zeros, check wiring (TX of TF-Luna → RX GPIO, 5 V power, GND).
