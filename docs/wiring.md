# Wiring

Pin assignments for the Raspberry Pi Pico 2 (RP2350). Select sensor configuration at compile time via `SENSOR_CONFIG` in `Umbreon_roborace.ino`.

---

## Config 1 — 4x TF-Luna (`SENSOR_4X_LUNA`)

| Pico 2 Pin | Function | Connected to |
|---|---|---|
| GP0 | I2C0 SDA | MPU-6050 SDA (optional, `USE_IMU=1`) |
| GP1 | I2C0 SCL | MPU-6050 SCL (optional, `USE_IMU=1`) |
| GP2 | SerialPIO RX | TF-Luna s[0] Left — TX |
| GP3 | SerialPIO RX | TF-Luna s[1] Front-Left — TX |
| GP4 | SerialPIO RX | TF-Luna s[2] Front-Right — TX |
| GP5 | SerialPIO RX | TF-Luna s[3] Right — TX |
| GP10 | PWM | Steering servo signal |
| GP11 | PWM | Motor ESC signal |
| GP13 | Digital input (IRQ) | Optical encoder (62 holes, RISING) |
| GP16 | UART1 TX | Wemos D1 Mini RX |
| GP17 | UART1 RX | Wemos D1 Mini TX |
| GP26 | ADC0 | Battery divider (optional, `BEN=1`) |
| 3V3 | Power | MPU-6050 VCC, TF-Luna VCC |
| 5V / VBUS | Power | TF-Luna VCC (5V required) |
| GND | Ground | Common ground |

**TF-Luna wiring (per sensor):**

| TF-Luna pin | Connect to |
|---|---|
| TX (green) | Pico GP2/GP3/GP4/GP5 (RX only, 115200 baud) |
| RX | Not connected |
| 5V (red) | 5V supply |
| GND (black) | GND |

---

## Config 2 — 6x VL53L0X (`SENSOR_6X_VL53L0X`)

| Pico 2 Pin | Function | Connected to |
|---|---|---|
| GP0 | I2C0 SDA | MPU-6050 SDA (optional, `USE_IMU=1`) |
| GP1 | I2C0 SCL | MPU-6050 SCL (optional, `USE_IMU=1`) |
| GP6 | Digital output | VL53L0X s[0] Hard-Left — XSHUT |
| GP7 | Digital output | VL53L0X s[1] Left — XSHUT |
| GP8 | Digital output | VL53L0X s[2] Front-Left — XSHUT |
| GP9 | Digital output | VL53L0X s[3] Front-Right — XSHUT |
| GP10 | PWM | Steering servo signal |
| GP11 | PWM | Motor ESC signal |
| GP13 | Digital input (IRQ) | Optical encoder (62 holes, RISING) |
| GP14 | Digital output | VL53L0X s[4] Right — XSHUT |
| GP15 | Digital output | VL53L0X s[5] Hard-Right — XSHUT |
| GP16 | UART1 TX | Wemos D1 Mini RX |
| GP17 | UART1 RX | Wemos D1 Mini TX |
| GP20 | I2C1 SDA | VL53L0X SDA (all 6 on shared bus) |
| GP21 | I2C1 SCL | VL53L0X SCL (all 6 on shared bus) |
| GP26 | ADC0 | Battery divider (optional, `BEN=1`) |
| 3V3 | Power | MPU-6050 VCC, VL53L0X VCC |
| GND | Ground | Common ground |

**VL53L0X wiring (6 sensors on shared I2C1 bus):**

| VL53L0X pin | Connect to |
|---|---|
| SDA | GP20 (shared bus, Wire1) |
| SCL | GP21 (shared bus, Wire1) |
| XSHUT | Individual pin per sensor (GP6-GP9, GP14-GP15) |
| VCC | 3.3V |
| GND | GND |

I2C addresses are assigned at boot (0x30-0x35) via XSHUT pin sequencing — all sensors start in reset, then are enabled one at a time.

---

## WiFi Bridge — Wemos D1 Mini (ESP8266)

Same wiring for both sensor configs.

| D1 Mini pin | Pico 2 pin |
|---|---|
| RX | GP16 (UART1 TX) |
| TX | GP17 (UART1 RX) |
| 5V | Separate USB (recommended) or shared 5V |
| GND | GND |

---

## Battery Monitoring (optional)

Disabled by default (`BEN=0`). Enable via `$SET:BEN=1`.

```
Battery+ ──[18k R1]──┬──[10k R2]── GND
                      └── GP26 (ADC0)
```

Multiplier: `BML = (R1 + R2) / R2 = 2.8` (default). Max safe input: ~9.4V with these resistors (3.3V ADC limit).

---

## Pin Map Summary

```
         ┌─────────────────────┐
         │    Pico 2 (RP2350)  │
         │                     │
  IMU ── │ GP0  SDA      GP26 │ ── Battery ADC
  IMU ── │ GP1  SCL      GP22 │
         │ GP2  Luna[0]  GP21 │ ── VL53 SCL
         │ GP3  Luna[1]  GP20 │ ── VL53 SDA
         │ GP4  Luna[2]  GP19 │
         │ GP5  Luna[3]  GP18 │
  VL53 ──│ GP6  XSHUT[0] GP17 │ ── D1 Mini TX
  VL53 ──│ GP7  XSHUT[1] GP16 │ ── D1 Mini RX
  VL53 ──│ GP8  XSHUT[2] GP15 │ ── VL53 XSHUT[5]
  VL53 ──│ GP9  XSHUT[3] GP14 │ ── VL53 XSHUT[4]
 Servo ──│ GP10 PWM       GP13 │ ── Tachometer
   ESC ──│ GP11 PWM       GP12 │
         │                     │
         └─────────────────────┘

  4x TF-Luna uses:  GP2-GP5 (UART RX)
  6x VL53L0X uses:  GP6-GP9, GP14-GP15 (XSHUT) + GP20-GP21 (I2C1)
  Both configs use:  GP0-GP1 (IMU), GP10-GP11 (servo/ESC), GP13 (tacho),
                     GP16-GP17 (WiFi UART), GP26 (battery)
```
