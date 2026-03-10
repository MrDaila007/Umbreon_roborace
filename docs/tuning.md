# Tuning Guide

All distances are in **cm×10** (e.g. `1200` = 120 cm).
Speed values are in **m/s**.

---

## Obstacle Thresholds (`Umbreon_roborace.ino`)

| Constant | Default | What to change |
|---|---|---|
| `FRONT_OBSTACLE_DIST` | 1200 | Distance at which the car starts steering around a front obstacle. Increase for earlier reaction; decrease to get closer to walls before turning. |
| `SIDE_OPEN_DIST` | 1000 | If both side sensors read above this, the corridor is treated as open and the car biases toward the right wall. |
| `ALL_CLOSE_DIST` | 800 | If all 4 sensors read below this, the car forces a hard right turn. Increase if getting stuck in wide but tight spaces. |
| `CLOSE_FRONT_DIST` | 201 | Emergency threshold — triggers stuck counter. Set to ~2× the minimum reliable TF-Luna range (~20 cm). |

---

## Speed & Steering Coefficients (`work()`)

```cpp
case 0:                         // path clear
    coef = 0.3f; spd = 2.7f;
    break;
default:                        // obstacle ahead
    coef = 0.7f; spd = 0.8f;
```

- **`spd` (clear)** — top straight-line speed. Reduce if the car overshoots corners.
- **`spd` (blocked)** — cornering speed. Increase if the car is too slow around bends.
- **`coef` (clear)** — steering gain when path is free. Higher = more aggressive wall-following.
- **`coef` (blocked)** — steering gain while avoiding obstacle. Higher = sharper turns.

---

## Stuck Recovery (`stuck_time > 25`)

- **25 ticks** at 40 ms/tick = **1 second** before triggering reversal.
- Reduce the threshold (e.g. `15`) to react faster; increase to tolerate brief slowdowns.

`go_back()` timing:
```cpp
car.write_speed(-150); delay(200);   // first nudge
car.write_speed(-150); delay(700);   // main reverse
```
Increase the second `delay` if the car needs more room to clear an obstacle.

---

## U-Turn Detection (`turns < -18.0`)

`turns` accumulates `diff × speed / -1000` each tick.

- **Threshold `-18.0`** — lower magnitude = triggers faster (e.g. `-10`); higher = less sensitive.
- **Clamp `[-1500, +50]`** — the negative clamp prevents runaway accumulation in long corridors.

`go_back_long()` reverse duration:
```cpp
car.write_speed(-150); delay(1000);
car.write_speed(-150); delay(1800);
```
Increase delays on longer cars or tighter dead ends.

---

## ESC / Servo Limits (`luna_car.h`)

| Constant | Default | Effect |
|---|---|---|
| `MIN_SPEED` | 96 | Minimum ESC PWM for forward (too low = no movement) |
| `MAX_SPEED` | 110 | Maximum ESC PWM for forward (increase for more top speed) |
| `MIN_BSPEED` | 85 | Minimum ESC PWM for reverse |
| `MIN_POINT` | 40 | Maximum left steering angle (degrees) |
| `MAX_POINT` | 140 | Maximum right steering angle (degrees) |
| `NEUTRAL_POINT` | 90 | Straight-ahead servo angle |

> Always verify ESC neutral (90°) matches your ESC calibration before driving.

---

## PID Speed Controller (`luna_car.h`)

| Gain | Default | Effect |
|---|---|---|
| `MOTOR_P` | 200 | Proportional — higher = faster speed correction, risk of oscillation |
| `MOTOR_D` | 110 | Derivative — damps oscillation; increase if speed hunts |

Typical tuning procedure:
1. Set `MOTOR_D = 0`, increase `MOTOR_P` until speed tracks well but oscillates.
2. Increase `MOTOR_D` until oscillation damps out.

---

## Loop Period (`LOOP_MS`)

Default: **40 ms** (25 Hz).

- Decrease (e.g. `20`) for faster response at high speed.
- `poll_lidars()` is also called outside the timed block, so LiDAR data is always fresh regardless of loop period.

---

## TF-Luna Sensor Notes

- Reliable range: **20 cm – 800 cm** (200–8000 in cm×10 units).
- Readings below 20 cm may be unreliable — keep `CLOSE_FRONT_DIST` ≥ 200.
- Default output rate from the factory: **100 Hz** (one frame every 10 ms).
  At 115200 baud, 9-byte frames arrive faster than the 40 ms control loop — no data loss expected.
- If a sensor returns all zeros, check wiring (TX of TF-Luna → RX GPIO, 5 V power, GND).
